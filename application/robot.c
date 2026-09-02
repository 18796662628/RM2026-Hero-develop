/**
 * @Author: HDC h2019dc@outlook.com
 * @Date: 2023-09-08 16:47:43
 * @LastEditors: HDC h2019dc@outlook.com
 * @LastEditTime: 2023-10-26 21:51:44
 * @FilePath: \2024_Control_New_Framework_Base-dev-all\application\robot.c
 * @Description:
 *
 * Copyright (c) 2023 by Alliance-EC, All Rights Reserved.
 */
#include "bsp_init.h"
#include "robot.h"
#include "robot_def.h"
#include "robot_task.h"
#include "chassis.h"
#include "gimbal.h"
#include "robot_cmd.h"

void RobotInit()
{
    __disable_irq();

    BSPInit();
    RobotCMDInit();
    GimbalInit();
    ChassisInit();

    __enable_irq();
}

void RobotTask()
{
    RobotCMDTask();
    GimbalTask();
    ChassisTask();
}
