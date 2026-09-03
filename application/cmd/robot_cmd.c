#include "robot_cmd.h"

#include "message_center.h"
#include "remote_control.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "math.h"

#define ARM_HOLD_TICKS 500U

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
    chassis_cmd.vx = MapChassisStick(remote_control[TEMP].rc.rocker_r1,
        CHASSIS_RC_MAX_SPEED);
    chassis_cmd.vy = MapChassisStick(remote_control[TEMP].rc.rocker_r_,
        CHASSIS_RC_MAX_SPEED);
    chassis_cmd.wz = ShapeChassisRotateCommand(
        MapChassisStick(remote_control[TEMP].rc.dial, CHASSIS_RC_MAX_ROTATE));
    chassis_cmd.offset_angle = 0.0f;
    chassis_cmd.chassis_mode = CHASSIS_NO_FOLLOW;
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
        pitch_soft_limit_min = pitch_target_angle - PITCH_SOFT_LIMIT_FROM_ARM_DEG;
        pitch_soft_limit_max = pitch_target_angle + PITCH_SOFT_LIMIT_FROM_ARM_DEG;
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

    if (remote_control[TEMP].rc_update_flag) {
        if (!remote_frame_seen) {
            LOGINFO("[safety] remote frame received; left=%u right=%u",
                remote_control[TEMP].rc.switch_left,
                remote_control[TEMP].rc.switch_right);
        }
        remote_frame_seen = 1U;
    }

    UpdateSafetyState();
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
    }

    PubPushMessage(gimbal_cmd_pub, &gimbal_cmd);
    PubPushMessage(chassis_cmd_pub, &chassis_cmd);
}

Robot_Safety_State_e RobotCMDGetSafetyState(void)
{
    return safety_state;
}
