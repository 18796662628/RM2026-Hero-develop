#ifndef ROBOT_CMD_H
#define ROBOT_CMD_H

#include "robot_def.h"

/**
 * @brief 机器人核心控制任务初始化,会被RobotInit()调用
 * 
 */
void RobotCMDInit();

/**
 * @brief 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率)
 * 
 */
void RobotCMDTask();

Robot_Safety_State_e RobotCMDGetSafetyState(void);

/* Actual continuous Yaw angle in degrees, exported for Ozone data sampling. */
extern volatile float yaw_angle_trace;

typedef struct
{
    float imu_yaw_total_deg;
    float yaw_motor_total_deg;
    float imu_yaw_delta_deg;
    float yaw_motor_delta_deg;
    float chassis_delta_imu_minus_motor_deg;
    float chassis_delta_imu_plus_motor_deg;
    uint16_t yaw_motor_ecd;
    uint8_t yaw_feedback_online;
    uint8_t safety_armed;
    float chassis_offset_command_deg;
    uint8_t chassis_field_frame_active;
    float chassis_command_vx;
    float chassis_command_vy;
    float chassis_body_vx;
    float chassis_body_vy;
    float chassis_wheel_lf_ref;
    float chassis_wheel_rf_ref;
    float chassis_wheel_lb_ref;
    float chassis_wheel_rb_ref;
    float chassis_wheel_scale;
    float chassis_command_wz;
    uint8_t chassis_wheels_online;
    uint8_t chassis_command_mode;
} Chassis_Frame_Debug_s;

/* Directly addressable diagnostic data for the first coordinate-frame test. */
extern volatile Chassis_Frame_Debug_s chassis_frame_debug;

#endif // !ROBOT_CMD_H
