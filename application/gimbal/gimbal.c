#include "gimbal.h"

#include "robot_def.h"
#include "dji_motor.h"
#include "ins_task.h"
#include "message_center.h"
#include "bsp_log.h"

static Publisher_t *gimbal_pub;
static Subscriber_t *gimbal_sub;
static Gimbal_Upload_Data_s gimbal_feedback_data;
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;
static attitude_t *gimbal_imu_data;

#if defined(ONE_BOARD)
static DJIMotorInstance *yaw_motor;
static DJIMotorInstance *pitch_motor;

static void ResetPIDState(PIDInstance *pid, float measure, float ref)
{
    pid->Measure = measure;
    pid->Last_Measure = measure;
    pid->Err = 0.0f;
    pid->Last_Err = 0.0f;
    pid->Last_ITerm = 0.0f;
    pid->Pout = 0.0f;
    pid->Iout = 0.0f;
    pid->Dout = 0.0f;
    pid->ITerm = 0.0f;
    pid->Output = 0.0f;
    pid->Last_Output = 0.0f;
    pid->Last_Dout = 0.0f;
    pid->Ref = ref;
    pid->ERRORHandler.ERRORCount = 0U;
    pid->ERRORHandler.ERRORType = PID_ERROR_NONE;
}

static void ResetYawController(void)
{
    ResetPIDState(&yaw_motor->motor_controller.angle_PID,
        gimbal_imu_data->YawTotalAngle,
        gimbal_imu_data->YawTotalAngle);
    ResetPIDState(&yaw_motor->motor_controller.speed_PID,
        gimbal_imu_data->Gyro[INS_YAW_ADDRESS_OFFSET],
        0.0f);
    yaw_motor->motor_controller.pid_ref = 0.0f;
}

static void ResetPitchController(void)
{
    ResetPIDState(&pitch_motor->motor_controller.angle_PID,
        gimbal_imu_data->Pitch,
        gimbal_imu_data->Pitch);
    ResetPIDState(&pitch_motor->motor_controller.speed_PID,
        gimbal_imu_data->Gyro[INS_PITCH_ADDRESS_OFFSET],
        0.0f);
    pitch_motor->motor_controller.pid_ref = 0.0f;
}
#endif

void GimbalInit(void)
{
    gimbal_imu_data = INS_Init();
    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));

#if defined(ONE_BOARD)
    Motor_Init_Config_s yaw_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 1U,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = YAW_ANGLE_PID_KP,
                .Ki = 0.0f,
                .Kd = 0.02f,
                .DeadBand = 0.0f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit
                    | PID_Derivative_On_Measurement | PID_MeasureFiliter,
                .IntegralLimit = 5.0f,
                .MaxOut = YAW_ANGLE_PID_MAX_OUT_DEG_PER_S,
                .Measure_LPF_RC = 0.5f,
            },
            .speed_PID = {
                .Kp = 3000.0f,
                .Ki = 600.0f,
                .Kd = 0.0f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit
                    | PID_Derivative_On_Measurement | PID_MeasureFiliter,
                .IntegralLimit = 1200.0f,
                .MaxOut = YAW_SPEED_PID_MAX_OUT,
                .CoefA = 0.2f,
                .CoefB = 2.0f,
                .Measure_LPF_RC = 0.5f,
            },
            .other_angle_feedback_ptr = &gimbal_imu_data->YawTotalAngle,
            .other_speed_feedback_ptr = &gimbal_imu_data->Gyro[INS_YAW_ADDRESS_OFFSET],
            .pid_struct_type = Cascade_PID,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = GM6020,
    };

    yaw_motor = DJIMotorInit(&yaw_config);
    LOGINFO("[gimbal] yaw registered on CAN1, id 1");

    Motor_Init_Config_s pitch_config = {
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id = 2U,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 0.6f,
                .Ki = 0.05f,
                .Kd = 0.00001f,
                .DeadBand = 0.0f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit
                    | PID_Derivative_On_Measurement | PID_MeasureFiliter,
                .IntegralLimit = 2.0f,
                .MaxOut = PITCH_ANGLE_PID_MAX_OUT_DEG_PER_S,
                .Measure_LPF_RC = 0.5f,
            },
            .speed_PID = {
                .Kp = -9000.0f,
                .Ki = -500.0f,
                .Kd = -4.0f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit
                    | PID_Derivative_On_Measurement | PID_MeasureFiliter,
                .IntegralLimit = 1000.0f,
                .MaxOut = PITCH_SPEED_PID_MAX_OUT,
                .Measure_LPF_RC = 0.5f,
            },
            .other_angle_feedback_ptr = &gimbal_imu_data->Pitch,
            .other_speed_feedback_ptr = &gimbal_imu_data->Gyro[INS_PITCH_ADDRESS_OFFSET],
            .pid_struct_type = Cascade_PID,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = GM6020,
    };

    pitch_motor = DJIMotorInit(&pitch_config);
    LOGINFO("[gimbal] pitch registered on CAN2, id 2");
#endif
}

void GimbalTask(void)
{
    static uint8_t yaw_active;
    static uint8_t yaw_online_last = 2U;
    static uint8_t pitch_active;
    static uint8_t pitch_online_last = 2U;
    uint8_t yaw_online;
    uint8_t pitch_online;

    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);

#if defined(ONE_BOARD)
    yaw_online = DaemonIsOnline(yaw_motor->daemon);
    if (yaw_online != yaw_online_last) {
        LOGINFO("[gimbal] yaw CAN feedback %s", yaw_online ? "online" : "offline");
        yaw_online_last = yaw_online;
    }
    pitch_online = DaemonIsOnline(pitch_motor->daemon);
    if (pitch_online != pitch_online_last) {
        LOGINFO("[gimbal] pitch CAN feedback %s", pitch_online ? "online" : "offline");
        pitch_online_last = pitch_online;
    }

    if (!gimbal_cmd_recv.robot_enabled
        || gimbal_cmd_recv.gimbal_mode != GIMBAL_GYRO_MODE
        || !yaw_online) {
        DJIMotorStop(yaw_motor);
        ResetYawController();
        yaw_active = 0U;
    } else {
        DJIMotorEnable(yaw_motor);
        DJIMotorChangeFeed(yaw_motor, ANGLE_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(yaw_motor, SPEED_LOOP, OTHER_FEED);
        DJIMotorOuterLoop(yaw_motor, ANGLE_LOOP);
        DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw_target_angle);

        if (!yaw_active) {
            LOGINFO("[gimbal] yaw gyro control enabled");
            yaw_active = 1U;
        }
    }

    if (!gimbal_cmd_recv.robot_enabled
        || gimbal_cmd_recv.gimbal_mode != GIMBAL_GYRO_MODE
        || !pitch_online) {
        DJIMotorStop(pitch_motor);
        ResetPitchController();
        pitch_active = 0U;
    } else {
        DJIMotorEnable(pitch_motor);
        DJIMotorChangeFeed(pitch_motor, ANGLE_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(pitch_motor, SPEED_LOOP, OTHER_FEED);
        DJIMotorOuterLoop(pitch_motor, ANGLE_LOOP);
        DJIMotorSetRef(pitch_motor, gimbal_cmd_recv.pitch_target_angle);

        if (!pitch_active) {
            LOGINFO("[gimbal] pitch gyro control enabled");
            pitch_active = 1U;
        }
    }

    gimbal_feedback_data.yaw_ecd = yaw_motor->measure.ecd;
    gimbal_feedback_data.yaw_motor_single_round_angle = yaw_motor->measure.angle_single_round;
    gimbal_feedback_data.pitch_ecd = pitch_motor->measure.ecd;
#endif

    gimbal_feedback_data.gimbal_imu_data = gimbal_imu_data;
    PubPushMessage(gimbal_pub, &gimbal_feedback_data);
}
