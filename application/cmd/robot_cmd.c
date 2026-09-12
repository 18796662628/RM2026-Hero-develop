#include "robot_cmd.h"

#include "message_center.h"
#include "remote_control.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "math.h"

#define ARM_HOLD_TICKS 500U
#define YAW_ALIGN_ANGLE_DEG \
    ((float)YAW_CHASSIS_ALIGN_ECD * (360.0f / 8192.0f))

static Publisher_t *gimbal_cmd_pub;
static Publisher_t *chassis_cmd_pub;
static Subscriber_t *gimbal_feed_sub;

static RC_ctrl_t *remote_control;
static Gimbal_Ctrl_Cmd_s gimbal_cmd;
static Chassis_Ctrl_Cmd_s chassis_cmd;
static Gimbal_Upload_Data_s gimbal_feedback;
static Robot_Safety_State_e safety_state = ROBOT_SAFETY_ESTOP;
static uint16_t arm_hold_ticks;
static uint8_t remote_frame_seen;
static uint8_t yaw_target_initialized;
static uint32_t yaw_control_tick;
static float yaw_target_angle;
static float yaw_rate_filtered;
static float yaw_rate_command;
static uint8_t pitch_target_initialized;
static uint32_t pitch_control_tick;
static float pitch_target_angle;
static float pitch_soft_limit_min;
static float pitch_soft_limit_max;
static uint8_t chassis_rotate_initialized;
static uint32_t chassis_rotate_control_tick;
static float chassis_rotate_filtered;
static float chassis_rotate_command;
static Chassis_Mode_e chassis_mode_last = CHASSIS_ZERO_FORCE;

/* Keep a directly addressable symbol for Ozone; this does not affect control. */
volatile float yaw_angle_trace = 0.0f;
volatile Chassis_Frame_Debug_s chassis_frame_debug;
static uint8_t chassis_frame_trace_initialized;
static float chassis_frame_imu_reference;
static float chassis_frame_motor_reference;

static uint8_t IsSmallGyroMode(Chassis_Mode_e mode)
{
    return mode == CHASSIS_ROTATE || mode == CHASSIS_REVERSE_ROTATE;
}

static void ResetChassisRotationOffset(void)
{
    chassis_frame_debug.chassis_offset_command_deg = 0.0f;
    chassis_frame_debug.chassis_field_frame_active = 0U;
}

/*
 * Original chassis-board method: use the current relative Yaw encoder angle
 * against the mechanically verified chassis-alignment encoder value.  In
 * small-gyro mode the gimbal is gyro-stabilized, so this relative angle gives
 * the rotation needed to express the requested motion in the chassis frame.
 */
static float GetOriginalChassisOffsetAngle(Chassis_Mode_e mode)
{
    float offset_angle;

    if (!IsSmallGyroMode(mode)
        || gimbal_feedback.gimbal_imu_data == NULL
        || !gimbal_feedback.yaw_feedback_online) {
        ResetChassisRotationOffset();
        return 0.0f;
    }

    offset_angle = -(gimbal_feedback.yaw_motor_single_round_angle
        - YAW_ALIGN_ANGLE_DEG);
    while (offset_angle > 180.0f) {
        offset_angle -= 360.0f;
    }
    while (offset_angle < -180.0f) {
        offset_angle += 360.0f;
    }

    chassis_frame_debug.chassis_offset_command_deg = offset_angle;
    chassis_frame_debug.chassis_field_frame_active = 1U;
    return offset_angle;
}

static void UpdateChassisFrameDebug(void)
{
    float imu_yaw;
    float motor_yaw;

    chassis_frame_debug.safety_armed = safety_state == ROBOT_SAFETY_ARMED;
    if (gimbal_feedback.gimbal_imu_data == NULL) {
        chassis_frame_debug.yaw_feedback_online = 0U;
        chassis_frame_debug.imu_yaw_delta_deg = 0.0f;
        chassis_frame_debug.yaw_motor_delta_deg = 0.0f;
        chassis_frame_debug.chassis_delta_imu_minus_motor_deg = 0.0f;
        chassis_frame_debug.chassis_delta_imu_plus_motor_deg = 0.0f;
        chassis_frame_debug.chassis_offset_command_deg = 0.0f;
        chassis_frame_debug.chassis_field_frame_active = 0U;
        chassis_frame_trace_initialized = 0U;
        return;
    }

    imu_yaw = gimbal_feedback.gimbal_imu_data->YawTotalAngle;
    motor_yaw = gimbal_feedback.yaw_motor_total_angle;
    chassis_frame_debug.imu_yaw_total_deg = imu_yaw;
    chassis_frame_debug.yaw_motor_total_deg = motor_yaw;
    chassis_frame_debug.yaw_motor_ecd = gimbal_feedback.yaw_ecd;
    chassis_frame_debug.yaw_feedback_online = gimbal_feedback.yaw_feedback_online;

    if (safety_state == ROBOT_SAFETY_ESTOP || !gimbal_feedback.yaw_feedback_online) {
        chassis_frame_trace_initialized = 0U;
        chassis_frame_debug.imu_yaw_delta_deg = 0.0f;
        chassis_frame_debug.yaw_motor_delta_deg = 0.0f;
        chassis_frame_debug.chassis_delta_imu_minus_motor_deg = 0.0f;
        chassis_frame_debug.chassis_delta_imu_plus_motor_deg = 0.0f;
        chassis_frame_debug.chassis_offset_command_deg = 0.0f;
        chassis_frame_debug.chassis_field_frame_active = 0U;
        return;
    }

    if (!chassis_frame_trace_initialized) {
        chassis_frame_imu_reference = imu_yaw;
        chassis_frame_motor_reference = motor_yaw;
        chassis_frame_trace_initialized = 1U;
    }

    chassis_frame_debug.imu_yaw_delta_deg = imu_yaw - chassis_frame_imu_reference;
    chassis_frame_debug.yaw_motor_delta_deg = motor_yaw - chassis_frame_motor_reference;
    chassis_frame_debug.chassis_delta_imu_minus_motor_deg =
        chassis_frame_debug.imu_yaw_delta_deg - chassis_frame_debug.yaw_motor_delta_deg;
    chassis_frame_debug.chassis_delta_imu_plus_motor_deg =
        chassis_frame_debug.imu_yaw_delta_deg + chassis_frame_debug.yaw_motor_delta_deg;
}

static uint8_t IsValidSwitchState(uint8_t switch_state)
{
    return switch_is_up(switch_state)
        || switch_is_mid(switch_state)
        || switch_is_down(switch_state);
}

static uint8_t IsEmergencyStopRequested(void)
{
    if (!remote_frame_seen || !RemoteControlIsOnline()) {
        return 1U;
    }

    if (!IsValidSwitchState(remote_control[TEMP].rc.switch_left)
        || !IsValidSwitchState(remote_control[TEMP].rc.switch_right)) {
        return 1U;
    }

    return switch_is_down(remote_control[TEMP].rc.switch_left)
        && switch_is_down(remote_control[TEMP].rc.switch_right);
}

static uint8_t AreSwitchesCentered(void)
{
    return switch_is_mid(remote_control[TEMP].rc.switch_left)
        && switch_is_mid(remote_control[TEMP].rc.switch_right);
}

static void SetSafeCommands(void)
{
    chassis_cmd.vx = 0.0f;
    chassis_cmd.vy = 0.0f;
    chassis_cmd.wz = 0.0f;
    chassis_cmd.offset_angle = 0.0f;
    chassis_cmd.chassis_mode = CHASSIS_ZERO_FORCE;
    chassis_cmd.supercap_flag = SUPERCAP_UNUSE;
    chassis_cmd.power_buffer = 0U;
    chassis_cmd.power_limit = 0U;
    chassis_cmd.track_wheel_mode = TRACK_WHEEL_OFF;
    chassis_cmd.putter_offset = 0.0f;
    chassis_cmd.is_power_on = 0U;
    chassis_cmd.robot_enabled = 0U;

    gimbal_cmd.yaw_target_angle = 0.0f;
    gimbal_cmd.yaw_target_speed = 0.0f;
    gimbal_cmd.yaw_target_acc = 0.0f;
    gimbal_cmd.pitch_target_angle = 0.0f;
    gimbal_cmd.pitch_target_speed = 0.0f;
    gimbal_cmd.pitch_target_acc = 0.0f;
    gimbal_cmd.auto_aim_mode = AUTO_AIM_OFF;
    gimbal_cmd.gimbal_mode = GIMBAL_ZERO_FORCE;
    gimbal_cmd.robot_enabled = 0U;
}

static float MapChassisStick(int16_t stick, float max_value)
{
    float value = (float)stick;

    if (fabsf(value) < CHASSIS_RC_DEADBAND) {
        return 0.0f;
    }

    return value * max_value / RC_STICK_FULL_SCALE;
}

static const char *ChassisModeName(Chassis_Mode_e mode)
{
    switch (mode) {
        case CHASSIS_ROTATE:
            return "auto rotate forward";
        case CHASSIS_NO_FOLLOW:
            return "manual omni";
        case CHASSIS_REVERSE_ROTATE:
            return "auto rotate reverse";
        default:
            return "zero force";
    }
}

static float ShapeChassisRotateCommand(float rotate_raw)
{
    float dt;
    float max_delta;

    if (!chassis_rotate_initialized) {
        DWT_GetDeltaT(&chassis_rotate_control_tick);
        chassis_rotate_initialized = 1U;
        return 0.0f;
    }

    dt = DWT_GetDeltaT(&chassis_rotate_control_tick);
    if (dt > CHASSIS_CONTROL_DT_MAX_S) {
        dt = CHASSIS_CONTROL_DT_MAX_S;
    }

    chassis_rotate_filtered += (rotate_raw - chassis_rotate_filtered)
        * dt / (CHASSIS_ROTATE_FILTER_TAU_S + dt);
    max_delta = CHASSIS_ROTATE_ACCEL_LIMIT_PER_S2 * dt;
    if (chassis_rotate_filtered - chassis_rotate_command > max_delta) {
        chassis_rotate_command += max_delta;
    } else if (chassis_rotate_command - chassis_rotate_filtered > max_delta) {
        chassis_rotate_command -= max_delta;
    } else {
        chassis_rotate_command = chassis_rotate_filtered;
    }

    return chassis_rotate_command;
}

static void BuildArmedChassisCommand(void)
{
    float rotate_raw;
    uint8_t small_gyro_mode;

    chassis_cmd.vx = MapChassisStick(remote_control[TEMP].rc.rocker_r1,
        CHASSIS_RC_MAX_SPEED);
    chassis_cmd.vy = MapChassisStick(remote_control[TEMP].rc.rocker_r_,
        CHASSIS_RC_MAX_SPEED);
    chassis_cmd.offset_angle = 0.0f;

    if (switch_is_up(remote_control[TEMP].rc.switch_right)) {
        chassis_cmd.chassis_mode = CHASSIS_ROTATE;
        rotate_raw = CHASSIS_AUTO_ROTATE_SPEED;
    } else if (switch_is_down(remote_control[TEMP].rc.switch_right)) {
        chassis_cmd.chassis_mode = CHASSIS_REVERSE_ROTATE;
        rotate_raw = -CHASSIS_AUTO_ROTATE_SPEED;
    } else {
        chassis_cmd.chassis_mode = CHASSIS_NO_FOLLOW;
        rotate_raw = MapChassisStick(remote_control[TEMP].rc.dial,
            CHASSIS_RC_MAX_ROTATE);
    }

    small_gyro_mode = IsSmallGyroMode(chassis_cmd.chassis_mode);
    if (small_gyro_mode && !gimbal_feedback.yaw_feedback_online) {
        chassis_cmd.vx = 0.0f;
        chassis_cmd.vy = 0.0f;
        rotate_raw = 0.0f;
    }
    chassis_cmd.offset_angle = GetOriginalChassisOffsetAngle(chassis_cmd.chassis_mode);
    chassis_frame_debug.chassis_command_vx = chassis_cmd.vx;
    chassis_frame_debug.chassis_command_vy = chassis_cmd.vy;
    chassis_frame_debug.chassis_command_mode = (uint8_t)chassis_cmd.chassis_mode;
    if (small_gyro_mode && !gimbal_feedback.yaw_feedback_online) {
        /* Do not let a stale filtered command keep the chassis spinning. */
        chassis_rotate_initialized = 0U;
        chassis_rotate_filtered = 0.0f;
        chassis_rotate_command = 0.0f;
        chassis_cmd.wz = 0.0f;
    } else {
        chassis_cmd.wz = ShapeChassisRotateCommand(rotate_raw);
    }
    chassis_frame_debug.chassis_command_wz = chassis_cmd.wz;

    if (chassis_cmd.chassis_mode != chassis_mode_last) {
        LOGINFO("[chassis] mode=%s", ChassisModeName(chassis_cmd.chassis_mode));
        chassis_mode_last = chassis_cmd.chassis_mode;
    }

    chassis_cmd.supercap_flag = SUPERCAP_UNUSE;
    chassis_cmd.power_buffer = 0U;
    chassis_cmd.power_limit = 0U;
    chassis_cmd.track_wheel_mode = TRACK_WHEEL_OFF;
    chassis_cmd.putter_offset = 0.0f;
    chassis_cmd.is_power_on = 1U;
    chassis_cmd.robot_enabled = 1U;
}

static void BuildArmedGimbalCommand(void)
{
    float yaw_rate_raw;
    float yaw_rate_target;
    float max_rate_delta;
    float pitch_rate;
    float dt;

    if (gimbal_feedback.gimbal_imu_data == NULL) {
        return;
    }

    if (!yaw_target_initialized) {
        yaw_target_angle = gimbal_feedback.gimbal_imu_data->YawTotalAngle;
        DWT_GetDeltaT(&yaw_control_tick);
        yaw_target_initialized = 1U;
    } else {
        dt = DWT_GetDeltaT(&yaw_control_tick);
        if (dt > YAW_CONTROL_DT_MAX_S) {
            dt = YAW_CONTROL_DT_MAX_S;
        }

        yaw_rate_raw = (float)remote_control[TEMP].rc.rocker_l_;
        if (fabsf(yaw_rate_raw) < YAW_RC_DEADBAND) {
            yaw_rate_raw = 0.0f;
        }
        yaw_rate_raw = YAW_RC_DIRECTION * yaw_rate_raw
            * YAW_RC_MAX_RATE_DEG_PER_S / RC_STICK_FULL_SCALE;

        /* Filter stick noise, then limit command acceleration for smooth starts/stops. */
        yaw_rate_filtered += (yaw_rate_raw - yaw_rate_filtered)
            * dt / (YAW_RC_FILTER_TAU_S + dt);
        max_rate_delta = YAW_RC_ACCEL_LIMIT_DEG_PER_S2 * dt;
        yaw_rate_target = yaw_rate_filtered;
        if (yaw_rate_target - yaw_rate_command > max_rate_delta) {
            yaw_rate_command += max_rate_delta;
        } else if (yaw_rate_command - yaw_rate_target > max_rate_delta) {
            yaw_rate_command -= max_rate_delta;
        } else {
            yaw_rate_command = yaw_rate_target;
        }
        yaw_target_angle += yaw_rate_command * dt;
    }

    if (!pitch_target_initialized) {
        pitch_target_angle = gimbal_feedback.gimbal_imu_data->Pitch;
        pitch_soft_limit_min = pitch_target_angle - PITCH_SOFT_LIMIT_DOWN_FROM_ARM_DEG;
        pitch_soft_limit_max = pitch_target_angle + PITCH_SOFT_LIMIT_UP_FROM_ARM_DEG;
        DWT_GetDeltaT(&pitch_control_tick);
        pitch_target_initialized = 1U;
    } else {
        dt = DWT_GetDeltaT(&pitch_control_tick);
        if (dt > PITCH_CONTROL_DT_MAX_S) {
            dt = PITCH_CONTROL_DT_MAX_S;
        }

        pitch_rate = PITCH_RC_DIRECTION * (float)remote_control[TEMP].rc.rocker_l1
            * PITCH_RC_MAX_RATE_DEG_PER_S / RC_STICK_FULL_SCALE;
        pitch_target_angle += pitch_rate * dt;
        if (pitch_target_angle > pitch_soft_limit_max) {
            pitch_target_angle = pitch_soft_limit_max;
        } else if (pitch_target_angle < pitch_soft_limit_min) {
            pitch_target_angle = pitch_soft_limit_min;
        }
    }

    gimbal_cmd.yaw_actual_angle = gimbal_feedback.gimbal_imu_data->YawTotalAngle;
    gimbal_cmd.yaw_actual_speed = gimbal_feedback.gimbal_imu_data->Gyro[INS_YAW_ADDRESS_OFFSET];
    gimbal_cmd.yaw_target_angle = yaw_target_angle;
    gimbal_cmd.pitch_target_angle = pitch_target_angle;
    gimbal_cmd.gimbal_mode = GIMBAL_GYRO_MODE;
    gimbal_cmd.robot_enabled = 1U;
}

static void SetSafetyState(Robot_Safety_State_e next_state)
{
    if (safety_state == next_state) {
        return;
    }

    safety_state = next_state;
    if (next_state == ROBOT_SAFETY_ESTOP) {
        LOGWARNING("[safety] emergency stop");
    } else if (next_state == ROBOT_SAFETY_DISARMED) {
        LOGINFO("[safety] disarmed; hold both switches in the middle to arm");
    } else {
        LOGINFO("[safety] armed");
    }
}

static void UpdateSafetyState(void)
{
    if (IsEmergencyStopRequested()) {
        arm_hold_ticks = 0U;
        SetSafetyState(ROBOT_SAFETY_ESTOP);
        return;
    }

    if (safety_state == ROBOT_SAFETY_ESTOP) {
        arm_hold_ticks = 0U;
        SetSafetyState(ROBOT_SAFETY_DISARMED);
    }

    if (safety_state != ROBOT_SAFETY_DISARMED) {
        return;
    }

    if (!AreSwitchesCentered()) {
        arm_hold_ticks = 0U;
        return;
    }

    if (arm_hold_ticks < ARM_HOLD_TICKS) {
        arm_hold_ticks++;
    }

    if (arm_hold_ticks == ARM_HOLD_TICKS) {
        SetSafetyState(ROBOT_SAFETY_ARMED);
    }
}

void RobotCMDInit(void)
{
    gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    remote_control = RemoteControlInit(&huart3);
    SetSafeCommands();
    LOGWARNING("[safety] booted locked");
}

void RobotCMDTask(void)
{
    SubGetMessage(gimbal_feed_sub, &gimbal_feedback);

    if (gimbal_feedback.gimbal_imu_data != NULL) {
        yaw_angle_trace = gimbal_feedback.gimbal_imu_data->YawTotalAngle;
    } else {
        yaw_angle_trace = 0.0f;
    }

    if (remote_control[TEMP].rc_update_flag) {
        if (!remote_frame_seen) {
            LOGINFO("[safety] remote frame received; left=%u right=%u",
                remote_control[TEMP].rc.switch_left,
                remote_control[TEMP].rc.switch_right);
        }
        remote_frame_seen = 1U;
    }

    UpdateSafetyState();
    UpdateChassisFrameDebug();
    SetSafeCommands();
    if (safety_state == ROBOT_SAFETY_ARMED) {
        BuildArmedGimbalCommand();
        BuildArmedChassisCommand();
    } else {
        yaw_target_initialized = 0U;
        yaw_rate_filtered = 0.0f;
        yaw_rate_command = 0.0f;
        pitch_target_initialized = 0U;
        chassis_rotate_initialized = 0U;
        chassis_rotate_filtered = 0.0f;
        chassis_rotate_command = 0.0f;
        chassis_mode_last = CHASSIS_ZERO_FORCE;
        ResetChassisRotationOffset();
    }

    PubPushMessage(gimbal_cmd_pub, &gimbal_cmd);
    PubPushMessage(chassis_cmd_pub, &chassis_cmd);
}

Robot_Safety_State_e RobotCMDGetSafetyState(void)
{
    return safety_state;
}
