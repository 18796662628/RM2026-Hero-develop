#ifndef SENTINEL_ROBOT_CMD_H
#define SENTINEL_ROBOT_CMD_H

#include "robot_def.h"
#include "sentinel_remote.h"

void RobotCMDInit(void);
void RobotCMDTask(void);

void SentinelBuildChassisPacket(const Sentinel_Remote_Input_s *remote,
                                uint8_t remote_online,
                                Sentinel_Chassis_Packet_s *packet);

#endif
