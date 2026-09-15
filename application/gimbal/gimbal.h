#ifndef GIMBAL_H
#define GIMBAL_H

#include "stdint.h"

typedef struct
{
    float actual_angle_deg;
    float target_angle_deg;
    float imu_speed_deg_per_s;
    float motor_speed_deg_per_s;
    float angle_pid_iout;
    float speed_pid_iout;
    float speed_pid_output;
    float gravity_feedforward;
    int16_t motor_current;
    uint8_t feedback_online;
    uint8_t control_active;
} Pitch_Feedforward_Debug_s;

/* Read-only runtime mirror for Ozone data sampling. */
extern volatile Pitch_Feedforward_Debug_s pitch_ff_debug;

/**
 * @brief 初始化云台,会被RobotInit()调用
 * 
 */
void GimbalInit();

/**
 * @brief 云台任务
 * 
 */
void GimbalTask();



#endif // GIMBAL_H
