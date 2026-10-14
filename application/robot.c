#include "robot.h"

#include "bsp_init.h"
#include "buzzer.h"
#include "chassis.h"
#include "robot_cmd.h"
#include "robot_def.h"

void RobotInit(void)
{
    __disable_irq();

    BSPInit();
    RobotCMDInit();

#ifdef CHASSIS_BOARD
    ChassisInit();
#endif

    buzzer_one_note(Do_freq, 0.1f);
    __enable_irq();
}

void RobotTask(void)
{
}
