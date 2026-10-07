#include "shoot.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "message_center.h"
#include "bsp_dwt.h"
#include "general_def.h"
#include "tool.h"
#include "referee_UI.h"
#include "DMmotor.h"
#include "bsp_log.h"
#include "math.h"
#include "stdlib.h"

#define SHOOT_FRICTION_TARGET_SPEED 2000.0f
#define SHOOT_LOADER_FEED_TIMEOUT_MS 1000.0f
#define SHOOT_LOADER_JAM_CONFIRM_MS 150.0f
#define SHOOT_LOADER_POSITION_TOLERANCE_RAD 0.08f
#define SHOOT_LOADER_LOW_SPEED_RAD_S 0.30f
#define SHOOT_LOADER_HIGH_TORQUE 4.0f
#define SHOOT_FRICTION_HIGH_CURRENT 6000
#define SHOOT_FRICTION_LOW_SPEED_DEG_S 250.0f

static Publisher_t *shoot_pub;
static Subscriber_t *shoot_sub;
static Shoot_Ctrl_Cmd_s shoot_cmd_recv;         
static Shoot_Upload_Data_s shoot_feedback_data; 

/* Directly addressable traces for Ozone and RTT-assisted jam diagnosis. */
volatile uint8_t shoot_loader_feed_active;
volatile uint8_t shoot_loader_jam_fault;
volatile float shoot_loader_target_position;
volatile float shoot_loader_position_error;
volatile float shoot_loader_speed;
volatile float shoot_loader_torque;

#if defined(CHASSIS_BOARD) || defined(ONE_BOARD)
static DMMotorInstance *loader;
static float loader_offset_angle = 24.55;
#endif

#if defined(GIMBAL_BOARD) || defined(ONE_BOARD)
static DJIMotorInstance *friction_l, *friction_r;
static float friction_feedforwardl = 100, friction_feedforwardr = -100;
static float fricl_speed = 0, limit_speed = 0, fricr_speed = 0;
const static float shoot_speed_target = SHOOT_FRICTION_TARGET_SPEED;
const static float shoot2_speed_target = SHOOT_FRICTION_TARGET_SPEED;
const static float limit_speed_target = 0;
#endif

#if defined(CHASSIS_BOARD) || defined(ONE_BOARD)
static float CalculateNextAngle(float current_angle)
{
    const float angle_step = PI / 3.0f;
    float step_index;
    float target_angle;

    /* Select the next physical feed index in the negative direction.
     * floorf keeps this valid after the loader has accumulated many turns. */
    step_index = floorf((loader_offset_angle - current_angle + 0.2f) / angle_step)
        + 1.0f;
    target_angle = loader_offset_angle - step_index * angle_step;
    while (target_angle >= current_angle - 0.02f) {
        target_angle -= angle_step;
    }
    return target_angle;
}
#endif

void ShootInit()
{
#if defined(CHASSIS_BOARD) || defined(ONE_BOARD)
    Motor_Init_Config_s loader_motor_config = {
        .can_init_config = {
            .can_handle =
#if defined(ONE_BOARD)
                &hcan1,
#else
                &hcan2,
#endif
            .tx_id = 0x101,
            .rx_id = 0x11,
        },
        .motor_type = DM_4310,
        .controller_setting_init_config = {
            .control_range = {
                .P_max = 12.5,
                .V_max = 30,
                .T_max = 10,
            },
        },
        .motor_contro_type = ANGLE_LOOP_CONTRO,
    };
    loader = DMMotorInit(&loader_motor_config);
    DMMotorStop(loader);
    LOGINFO("[shoot] loader registered on CAN%d, tx=0x101 rx=0x11",
        loader_motor_config.can_init_config.can_handle == &hcan1 ? 1 : 2);
#endif

#if defined(GIMBAL_BOARD) || defined(ONE_BOARD)
    Motor_Init_Config_s friction_config = {
        .can_init_config = {
            .can_handle = &hcan2,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp            = 1.0,
                .Ki            = 0,
                .Kd            = 0,
                .Improve       = PID_Integral_Limit,
                .IntegralLimit = 15000,
                .MaxOut        = 10000,
            },

        },

        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED, .speed_feedback_source = MOTOR_FEED,

            .outer_loop_type    = SPEED_LOOP,
            .close_loop_type    = SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
            .feedforward_flag   = CURRENT_FEEDFORWARD,
        },
        .motor_type = M3508};

    friction_config.can_init_config.tx_id                                = 2; // 左摩擦轮,改txid和方向就行
    friction_config.controller_setting_init_config.motor_reverse_flag    = MOTOR_DIRECTION_REVERSE;
    friction_config.controller_param_init_config.current_feedforward_ptr = &friction_feedforwardr;
    friction_l                                                           = DJIMotorInit(&friction_config);
    // 三摩擦轮外加电机
    friction_config.can_init_config.tx_id                             = 1;
    friction_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    friction_config.controller_param_init_config.current_feedforward_ptr = &friction_feedforwardl;
    friction_r                                                      = DJIMotorInit(&friction_config);
    DJIMotorStop(friction_l);
    DJIMotorStop(friction_r);
    LOGINFO("[shoot] friction target=%.0f", SHOOT_FRICTION_TARGET_SPEED);
#endif
    shoot_cmd_recv.shoot_mode = SHOOT_OFF;

    shoot_pub = PubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
    shoot_sub = SubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));

}

static ramp_t fricl_on_ramp, fricr_on_ramp;
static ramp_t fricl_off_ramp, fricr_off_ramp;
static Loader_Mode_e last_mode;
#if defined(CHASSIS_BOARD) || defined(ONE_BOARD)
static uint8_t loader_position_initialized;
static uint8_t loader_online_last = 2U;
static float loader_feed_start_ms;
static float loader_jam_start_ms;
#endif

#if defined(CHASSIS_BOARD) || defined(ONE_BOARD)
static uint8_t FrictionSpeedReady(void)
{
#if defined(ONE_BOARD)
    float left_speed = fabsf(friction_l->measure.speed_aps);
    float right_speed = fabsf(friction_r->measure.speed_aps);
    float target_speed = SHOOT_FRICTION_TARGET_SPEED;
    float tolerance = target_speed * 0.35f;

    return fabsf(left_speed - target_speed) <= tolerance
        && fabsf(right_speed - target_speed) <= tolerance;
#else
    return 0U;
#endif
}

static uint8_t LoaderJamSignature(void)
{
    uint8_t dm_jam = fabsf(loader->measure.tor) >= SHOOT_LOADER_HIGH_TORQUE
        && fabsf(loader->measure.vel) <= SHOOT_LOADER_LOW_SPEED_RAD_S;
#if defined(ONE_BOARD)
    uint8_t friction_jam = FrictionSpeedReady()
        && (abs(friction_l->measure.real_current) >= SHOOT_FRICTION_HIGH_CURRENT
            || abs(friction_r->measure.real_current) >= SHOOT_FRICTION_HIGH_CURRENT)
        && fabsf(loader->measure.vel) <= SHOOT_LOADER_LOW_SPEED_RAD_S;
#else
    uint8_t friction_jam = 0U;
#endif

    return dm_jam || friction_jam;
}

static void LatchLoaderJam(const char *reason)
{
    shoot_loader_jam_fault = 1U;
    shoot_loader_feed_active = 0U;
    loader_jam_start_ms = 0.0f;
    DMMotorStop(loader);
    LOGERROR("[shoot] loader jam: %s, target=%.3f actual=%.3f torque=%.3f speed=%.3f",
        reason, shoot_loader_target_position, loader->measure.total_position,
        loader->measure.tor, loader->measure.vel);
}

static void MonitorLoaderFeed(void)
{
    float now_ms;
    float elapsed_ms;
    uint8_t jam_signature;

    if (!shoot_loader_feed_active || shoot_loader_jam_fault
        || !loader_position_initialized) {
        return;
    }

    now_ms = DWT_GetTimeline_ms();
    elapsed_ms = now_ms - loader_feed_start_ms;
    shoot_loader_position_error = fabsf(
        shoot_loader_target_position - loader->measure.total_position);
    shoot_loader_speed = loader->measure.vel;
    shoot_loader_torque = loader->measure.tor;

    if (shoot_loader_position_error <= SHOOT_LOADER_POSITION_TOLERANCE_RAD) {
        shoot_loader_feed_active = 0U;
        loader_jam_start_ms = 0.0f;
        LOGINFO("[shoot] loader feed complete, error=%.3f rad",
            shoot_loader_position_error);
        return;
    }

    if (elapsed_ms >= SHOOT_LOADER_FEED_TIMEOUT_MS) {
        LatchLoaderJam("position timeout");
        return;
    }

    jam_signature = LoaderJamSignature();
    if (jam_signature) {
        if (loader_jam_start_ms <= 0.0f) {
            loader_jam_start_ms = now_ms;
        } else if (now_ms - loader_jam_start_ms >= SHOOT_LOADER_JAM_CONFIRM_MS) {
            LatchLoaderJam("high load and low speed");
        }
    } else {
        loader_jam_start_ms = 0.0f;
    }
}
#endif

void ShootTask()
{
    SubGetMessage(shoot_sub, &shoot_cmd_recv);

#if defined(CHASSIS_BOARD) || defined(ONE_BOARD)
    uint8_t loader_online = DMMotorIsOnline(loader);

    if (loader_online != loader_online_last) {
        LOGINFO("[shoot] loader CAN feedback %s", loader_online ? "online" : "offline");
        loader_online_last = loader_online;
    }

    if (shoot_cmd_recv.shoot_mode == SHOOT_OFF
        || shoot_cmd_recv.friction_mode == FRICTION_OFF) {
        DMMotorStop(loader);
        loader_position_initialized = 0U;
        shoot_loader_feed_active = 0U;
        shoot_loader_jam_fault = 0U;
        loader_feed_start_ms = 0.0f;
        loader_jam_start_ms = 0.0f;
        shoot_loader_position_error = 0.0f;
    } else {
        if (!loader_online || shoot_loader_jam_fault) {
            loader_position_initialized = 0U;
        }
        if (!shoot_loader_jam_fault) {
            DMMotorEnable1(loader);
        }
        if (loader_online && !loader_position_initialized
            && !shoot_loader_jam_fault) {
            /* First valid feedback defines the stationary loader hold point. */
            DMMotorSetPos(loader, loader->measure.total_position, 0.0f);
            loader_position_initialized = 1U;
            LOGINFO("[shoot] loader position anchored at %.3f rad",
                loader->measure.total_position);
        }
    }

    float current_angle;
    switch (shoot_cmd_recv.load_mode) {
        case LOAD_STOP:
            last_mode = LOAD_STOP;
            break;
        case LOAD_1_BULLET:

            if (shoot_cmd_recv.friction_mode == FRICTION_OFF) break;

            if (loader_position_initialized && last_mode == LOAD_STOP
                && shoot_cmd_recv.shooter_referee_heat < 100)
            {
                current_angle = loader->measure.total_position;
                float target_angle = CalculateNextAngle(current_angle);
                DMMotorSetPos(loader, target_angle, 5.0f);
                shoot_loader_target_position = target_angle;
                shoot_loader_feed_active = 1U;
                shoot_loader_position_error = fabsf(target_angle - current_angle);
                shoot_loader_speed = loader->measure.vel;
                shoot_loader_torque = loader->measure.tor;
                loader_feed_start_ms = DWT_GetTimeline_ms();
                loader_jam_start_ms = 0.0f;
                LOGINFO("[shoot] loader step %.3f -> %.3f rad",
                    current_angle, target_angle);
            } else if (!loader_position_initialized) {
                LOGWARNING("[shoot] single shot ignored: loader feedback is offline");
            }
            last_mode = LOAD_1_BULLET;
            break;
        case LOAD_BURSTFIRE:
        default:
            /* Burst fire is intentionally not enabled for this training task. */
            last_mode = LOAD_STOP;
            break;
    }
    MonitorLoaderFeed();
#endif

#if defined(GIMBAL_BOARD) || defined(ONE_BOARD)
    if (shoot_cmd_recv.shoot_mode == SHOOT_OFF
        || shoot_cmd_recv.friction_mode == FRICTION_OFF
        || shoot_loader_jam_fault) {
        DJIMotorStop(friction_l);
        DJIMotorStop(friction_r);
        friction_feedforwardl = 0.0f;
        friction_feedforwardr = 0.0f;
    } else {
        DJIMotorEnable(friction_l);
        DJIMotorEnable(friction_r);
        friction_feedforwardl = 100.0f;
        friction_feedforwardr = -100.0f;
    }

    static float shoot_speed = 0, shoot2_speed = 0, limit_shoot_speed;
    if (shoot_cmd_recv.friction_mode == FRICTION_ON) {
        // 根据收到的弹速设置设定摩擦轮电机参考值,需实测后填入
        fricl_speed  = (shoot_speed + (shoot_speed_target - shoot_speed) * ramp_calc(&fricl_on_ramp));
        fricr_speed = (shoot2_speed + (shoot2_speed_target - shoot2_speed) * ramp_calc(&fricr_on_ramp));
        ramp_init(&fricl_off_ramp, 3000);
        ramp_init(&fricr_off_ramp, 3000);
    } else if (shoot_cmd_recv.friction_mode == FRICTION_OFF) {
        fricl_speed  = (shoot_speed + (0 - shoot_speed) * ramp_calc(&fricl_off_ramp));
        fricr_speed = (shoot2_speed + (0 - shoot2_speed) * ramp_calc(&fricr_off_ramp));
        ramp_init(&fricl_on_ramp, 1000);
        ramp_init(&fricr_on_ramp, 1000);
    }
    DJIMotorSetRef(friction_l, fricl_speed);
    DJIMotorSetRef(friction_r, fricr_speed);
#endif
    PubPushMessage(shoot_pub, (void *)&shoot_feedback_data);

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* USER CODE BEGIN Callback 0 */

    /* USER CODE END Callback 0 */
    if (htim->Instance == TIM14) {
        HAL_IncTick();
    }
}
