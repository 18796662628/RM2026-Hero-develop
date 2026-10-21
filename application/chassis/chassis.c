#include "chassis.h"

#include <math.h>

#include "bsp_log.h"
#include "can.h"
#include "daemon.h"
#include "dji_motor.h"
#include "message_center.h"
#include "robot_def.h"
#include "sentinel_chassis_config.h"
#include "swerve_kinematics.h"

#ifdef CHASSIS_BOARD

#define CHASSIS_PI 3.14159265358979323846f
#define RAD_TO_DEGREE (180.0f / CHASSIS_PI)
#define DRIVE_MPS_TO_OUTPUT_DEGREE_PER_SECOND \
    (360.0f * SENTINEL_DRIVE_OUTPUT_TO_WHEEL_RATIO / \
     (2.0f * CHASSIS_PI * SENTINEL_WHEEL_RADIUS_M))

typedef enum
{
    WHEEL_LF = 0,
    WHEEL_RF,
    WHEEL_RB,
    WHEEL_LB,
    WHEEL_COUNT,
} Wheel_Index_e;

typedef struct
{
    DJIMotorInstance *drive_motor;
    DJIMotorInstance *steer_motor;
    Swerve_Module_Position_s position;
    float steer_direction;
    float drive_direction;
    float steering_zero_total_angle;
} Wheel_Module_s;

static Subscriber_t *chassis_cmd_sub;
static Wheel_Module_s wheel_modules[WHEEL_COUNT] = {
    [WHEEL_LF] = {
        .position = {+SENTINEL_HALF_MODULE_SPACING_M, +SENTINEL_HALF_MODULE_SPACING_M},
        .steer_direction = SENTINEL_LF_STEER_DIRECTION,
        .drive_direction = SENTINEL_LF_DRIVE_DIRECTION,
    },
    [WHEEL_RF] = {
        .position = {+SENTINEL_HALF_MODULE_SPACING_M, -SENTINEL_HALF_MODULE_SPACING_M},
        .steer_direction = SENTINEL_RF_STEER_DIRECTION,
        .drive_direction = SENTINEL_RF_DRIVE_DIRECTION,
    },
    [WHEEL_RB] = {
        .position = {-SENTINEL_HALF_MODULE_SPACING_M, -SENTINEL_HALF_MODULE_SPACING_M},
        .steer_direction = SENTINEL_RB_STEER_DIRECTION,
        .drive_direction = SENTINEL_RB_DRIVE_DIRECTION,
    },
    [WHEEL_LB] = {
        .position = {-SENTINEL_HALF_MODULE_SPACING_M, +SENTINEL_HALF_MODULE_SPACING_M},
        .steer_direction = SENTINEL_LB_STEER_DIRECTION,
        .drive_direction = SENTINEL_LB_DRIVE_DIRECTION,
    },
};
static uint16_t steering_feedback_wait_ticks;
static uint8_t steering_zero_captured;

static DJIMotorInstance *RegisterDriveMotor(CAN_HandleTypeDef *can_handle, uint16_t tx_id)
{
    Motor_Init_Config_s config = {
        .can_init_config = {.can_handle = can_handle, .tx_id = tx_id},
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 1.0f,
                .Ki = 0.0f,
                .Kd = 0.0f,
                .IntegralLimit = 1000.0f,
                .MaxOut = 3000.0f,
                .Improve = PID_Integral_Limit | PID_Derivative_On_Measurement,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = M3508,
    };

    return DJIMotorInit(&config);
}

static DJIMotorInstance *RegisterSteerMotor(CAN_HandleTypeDef *can_handle, uint16_t tx_id)
{
    Motor_Init_Config_s config = {
        .can_init_config = {.can_handle = can_handle, .tx_id = tx_id},
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 2.0f,
                .Ki = 0.0f,
                .Kd = 0.0f,
                .DeadBand = 0.5f,
                .IntegralLimit = 0.0f,
                .MaxOut = 250.0f,
            },
            .speed_PID = {
                .Kp = 8.0f,
                .Ki = 0.0f,
                .Kd = 0.0f,
                .IntegralLimit = 0.0f,
                .MaxOut = 2500.0f,
                .Improve = PID_Derivative_On_Measurement,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = GM6020,
    };

    return DJIMotorInit(&config);
}

static void StopAllChassisMotors(void)
{
    for (uint8_t index = 0U; index < WHEEL_COUNT; index++)
    {
        DJIMotorSetRef(wheel_modules[index].drive_motor, 0.0f);
        DJIMotorStop(wheel_modules[index].drive_motor);
        DJIMotorStop(wheel_modules[index].steer_motor);
    }
}

static void EnableAllChassisMotors(void)
{
    for (uint8_t index = 0U; index < WHEEL_COUNT; index++)
    {
        DJIMotorEnable(wheel_modules[index].drive_motor);
        DJIMotorEnable(wheel_modules[index].steer_motor);
    }
}

static uint8_t SteeringFeedbackReady(void)
{
    for (uint8_t index = 0U; index < WHEEL_COUNT; index++)
    {
        if (!DaemonIsOnline(wheel_modules[index].steer_motor->daemon))
            return 0U;
    }
    return 1U;
}

static void CaptureSteeringZero(void)
{
    for (uint8_t index = 0U; index < WHEEL_COUNT; index++)
        wheel_modules[index].steering_zero_total_angle =
            wheel_modules[index].steer_motor->measure.total_angle;

    steering_zero_captured = 1U;
    LOGINFO("[chassis] steering zero captured: LF=%d RF=%d RB=%d LB=%d",
            wheel_modules[WHEEL_LF].steer_motor->measure.ecd,
            wheel_modules[WHEEL_RF].steer_motor->measure.ecd,
            wheel_modules[WHEEL_RB].steer_motor->measure.ecd,
            wheel_modules[WHEEL_LB].steer_motor->measure.ecd);
}

static float GetSteeringAngle(const Wheel_Module_s *module)
{
    float relative_degree = module->steer_direction *
                            (module->steer_motor->measure.total_angle -
                             module->steering_zero_total_angle);
    return SwerveWrapAngle(relative_degree / RAD_TO_DEGREE);
}

static void SetWheelTarget(Wheel_Module_s *module, Swerve_Module_State_s target)
{
    float current_angle = GetSteeringAngle(module);

    if (fabsf(target.drive_speed) < SENTINEL_STATIONARY_SPEED_MPS)
    {
        DJIMotorSetRef(module->drive_motor, 0.0f);
        DJIMotorSetRef(module->steer_motor, module->steer_motor->measure.total_angle);
        return;
    }

    SwerveOptimizeState(&target, current_angle);
    float steering_delta = SwerveWrapAngle(target.steering_angle_rad - current_angle);
    float steering_target = module->steer_motor->measure.total_angle +
                            module->steer_direction * steering_delta * RAD_TO_DEGREE;
    float drive_target = target.drive_speed * DRIVE_MPS_TO_OUTPUT_DEGREE_PER_SECOND *
                         module->drive_direction;

    DJIMotorSetRef(module->steer_motor, steering_target);
    DJIMotorSetRef(module->drive_motor, drive_target);
}

static void SetChassisVelocity(float vx, float vy, float wz)
{
    Swerve_Module_Position_s positions[WHEEL_COUNT];
    Swerve_Module_State_s targets[WHEEL_COUNT];

    for (uint8_t index = 0U; index < WHEEL_COUNT; index++)
        positions[index] = wheel_modules[index].position;

    SwerveCalculate(positions, vx, vy, wz, targets);
    for (uint8_t index = 0U; index < WHEEL_COUNT; index++)
        SetWheelTarget(&wheel_modules[index], targets[index]);
}

void ChassisInit(void)
{
    /* CAN mapping verified from the original Sentinel configuration. */
    wheel_modules[WHEEL_LF].drive_motor = RegisterDriveMotor(&hcan2, 2U);
    wheel_modules[WHEEL_RF].drive_motor = RegisterDriveMotor(&hcan2, 3U);
    wheel_modules[WHEEL_RB].drive_motor = RegisterDriveMotor(&hcan1, 2U);
    wheel_modules[WHEEL_LB].drive_motor = RegisterDriveMotor(&hcan1, 1U);

    wheel_modules[WHEEL_LF].steer_motor = RegisterSteerMotor(&hcan2, 1U);
    wheel_modules[WHEEL_RF].steer_motor = RegisterSteerMotor(&hcan2, 4U);
    wheel_modules[WHEEL_RB].steer_motor = RegisterSteerMotor(&hcan1, 3U);
    wheel_modules[WHEEL_LB].steer_motor = RegisterSteerMotor(&hcan1, 2U);

    chassis_cmd_sub = SubRegister("sentinel_chassis_cmd", sizeof(Sentinel_Chassis_Cmd_s));
    StopAllChassisMotors();
}

void ChassisTask(void)
{
    Sentinel_Chassis_Cmd_s command = {
        .emergency_stop = 1U,
        .chassis_mode = CHASSIS_ZERO_FORCE,
    };

    SubGetMessage(chassis_cmd_sub, &command);

    /* Wait for real CAN feedback before capturing the manually aligned zero. */
    if (!steering_zero_captured)
    {
        StopAllChassisMotors();
        if (steering_feedback_wait_ticks < 100U)
            steering_feedback_wait_ticks++;
        else if (SteeringFeedbackReady())
            CaptureSteeringZero();
        return;
    }

    /* First moving build permits only the manual, non-following test mode. */
    if (command.emergency_stop || command.chassis_mode != CHASSIS_NO_FOLLOW)
    {
        StopAllChassisMotors();
        return;
    }

    EnableAllChassisMotors();
    SetChassisVelocity(command.vx * SENTINEL_TEST_MAX_LINEAR_SPEED_MPS,
                       command.vy * SENTINEL_TEST_MAX_LINEAR_SPEED_MPS,
                       0.0f);
}

#else

void ChassisInit(void) {}
void ChassisTask(void) {}

#endif
