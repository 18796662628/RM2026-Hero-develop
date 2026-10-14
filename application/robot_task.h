#pragma once

#include "cmsis_os.h"
#include "daemon.h"
#include "motor_task.h"
#include "robot_cmd.h"
#include "robot_def.h"
#ifdef CHASSIS_BOARD
#include "chassis.h"
#endif

__attribute__((noreturn)) void _RobotCMDTask(void *argument)
{
    (void)argument;
    for (;;)
    {
        RobotCMDTask();
        osDelay(1);
    }
}

#ifdef CHASSIS_BOARD
__attribute__((noreturn)) void _ChassisTask(void *argument)
{
    (void)argument;
    for (;;)
    {
        ChassisTask();
        osDelay(1);
    }
}
#endif

__attribute__((noreturn)) void motorControlTask(void *argument)
{
    (void)argument;
    for (;;)
    {
        MotorControlTask();
        osDelay(1);
    }
}

__attribute__((noreturn)) void _DaemonTask(void *argument)
{
    (void)argument;
    for (;;)
    {
        DaemonTask();
        osDelay(1);
    }
}
