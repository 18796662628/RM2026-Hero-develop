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

#endif // !ROBOT_CMD_H
