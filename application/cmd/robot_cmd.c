#include "robot_cmd.h"

#include "message_center.h"
#include "remote_control.h"
#include "bsp_dwt.h"
#include "bsp_log.h"

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

static void BuildArmedGimbalCommand(void)
{
    float yaw_rate;
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

        yaw_rate = YAW_RC_DIRECTION * (float)remote_control[TEMP].rc.rocker_l_
            * YAW_RC_MAX_RATE_DEG_PER_S / RC_STICK_FULL_SCALE;
        yaw_target_angle += yaw_rate * dt;
    }

    gimbal_cmd.yaw_actual_angle = gimbal_feedback.gimbal_imu_data->YawTotalAngle;
    gimbal_cmd.yaw_actual_speed = gimbal_feedback.gimbal_imu_data->Gyro[INS_YAW_ADDRESS_OFFSET];
    gimbal_cmd.yaw_target_angle = yaw_target_angle;
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
        remote_frame_seen = 1U;
    }

    UpdateSafetyState();
    SetSafeCommands();
    if (safety_state == ROBOT_SAFETY_ARMED) {
        BuildArmedGimbalCommand();
    } else {
        yaw_target_initialized = 0U;
    }

    PubPushMessage(gimbal_cmd_pub, &gimbal_cmd);
    PubPushMessage(chassis_cmd_pub, &chassis_cmd);
}

Robot_Safety_State_e RobotCMDGetSafetyState(void)
{
    return safety_state;
}
