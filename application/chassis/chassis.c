#include "chassis.h"

#include "can.h"
#include "dji_motor.h"
#include "message_center.h"
#include "robot_def.h"

#ifdef CHASSIS_BOARD

static Subscriber_t *chassis_cmd_sub;

static DJIMotorInstance *wheel_lf_drive;
static DJIMotorInstance *wheel_rf_drive;
static DJIMotorInstance *wheel_rb_drive;
static DJIMotorInstance *wheel_lb_drive;
static DJIMotorInstance *wheel_lf_steer;
static DJIMotorInstance *wheel_rf_steer;
static DJIMotorInstance *wheel_rb_steer;
static DJIMotorInstance *wheel_lb_steer;

static DJIMotorInstance *RegisterSpeedMotor(CAN_HandleTypeDef *can_handle,
                                            uint16_t tx_id,
                                            Motor_Type_e motor_type)
{
    Motor_Init_Config_s config = {
        .can_init_config.can_handle = can_handle,
        .can_init_config.tx_id = tx_id,
        .controller_param_init_config.speed_PID = {
            .Kp = 1.0f,
            .Ki = 0.0f,
            .Kd = 0.0f,
            .IntegralLimit = 3000.0f,
            .MaxOut = 8000.0f,
            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit |
                       PID_Derivative_On_Measurement,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = motor_type,
    };

    return DJIMotorInit(&config);
}

static void StopAllChassisMotors(void)
{
    DJIMotorStop(wheel_lf_drive);
    DJIMotorStop(wheel_rf_drive);
    DJIMotorStop(wheel_rb_drive);
    DJIMotorStop(wheel_lb_drive);
    DJIMotorStop(wheel_lf_steer);
    DJIMotorStop(wheel_rf_steer);
    DJIMotorStop(wheel_rb_steer);
    DJIMotorStop(wheel_lb_steer);
}

static void EnableAllChassisMotors(void)
{
    DJIMotorEnable(wheel_lf_drive);
    DJIMotorEnable(wheel_rf_drive);
    DJIMotorEnable(wheel_rb_drive);
    DJIMotorEnable(wheel_lb_drive);
    DJIMotorEnable(wheel_lf_steer);
    DJIMotorEnable(wheel_rf_steer);
    DJIMotorEnable(wheel_rb_steer);
    DJIMotorEnable(wheel_lb_steer);
}

static void SetAllChassisRefsToZero(void)
{
    DJIMotorSetRef(wheel_lf_drive, 0.0f);
    DJIMotorSetRef(wheel_rf_drive, 0.0f);
    DJIMotorSetRef(wheel_rb_drive, 0.0f);
    DJIMotorSetRef(wheel_lb_drive, 0.0f);
    DJIMotorSetRef(wheel_lf_steer, 0.0f);
    DJIMotorSetRef(wheel_rf_steer, 0.0f);
    DJIMotorSetRef(wheel_rb_steer, 0.0f);
    DJIMotorSetRef(wheel_lb_steer, 0.0f);
}

void ChassisInit(void)
{
    /* The CAN bus and IDs are taken from the verified Sentinel configuration. */
    wheel_lf_drive = RegisterSpeedMotor(&hcan2, 2U, M3508);
    wheel_rf_drive = RegisterSpeedMotor(&hcan2, 3U, M3508);
    wheel_rb_drive = RegisterSpeedMotor(&hcan1, 2U, M3508);
    wheel_lb_drive = RegisterSpeedMotor(&hcan1, 1U, M3508);

    wheel_lf_steer = RegisterSpeedMotor(&hcan2, 1U, GM6020);
    wheel_rf_steer = RegisterSpeedMotor(&hcan2, 4U, GM6020);
    wheel_rb_steer = RegisterSpeedMotor(&hcan1, 3U, GM6020);
    wheel_lb_steer = RegisterSpeedMotor(&hcan1, 2U, GM6020);

    chassis_cmd_sub = SubRegister("sentinel_chassis_cmd", sizeof(Sentinel_Chassis_Cmd_s));
    SetAllChassisRefsToZero();
    StopAllChassisMotors();
}

void ChassisTask(void)
{
    Sentinel_Chassis_Cmd_s command = {
        .emergency_stop = 1U,
        .chassis_mode = CHASSIS_ZERO_FORCE,
    };

    SubGetMessage(chassis_cmd_sub, &command);

    if (command.emergency_stop || command.chassis_mode == CHASSIS_ZERO_FORCE)
    {
        SetAllChassisRefsToZero();
        StopAllChassisMotors();
        return;
    }

    EnableAllChassisMotors();

    /* Four-steer kinematics and steering zero calibration are added next. */
    SetAllChassisRefsToZero();
}

#else

void ChassisInit(void) {}
void ChassisTask(void) {}

#endif
