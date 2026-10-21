#include "robot_cmd.h"

#include <string.h>

#include "master_process.h"
#include "message_center.h"
#include "robot_def.h"
#include "sentinel_remote.h"
#include "usart.h"

#define SENTINEL_PACKET_SEQUENCE_MASK 0xFFU

static float NormalizeRemoteAxis(int16_t axis)
{
    if (axis > SENTINEL_REMOTE_AXIS_MAX)
        axis = SENTINEL_REMOTE_AXIS_MAX;
    else if (axis < -SENTINEL_REMOTE_AXIS_MAX)
        axis = -SENTINEL_REMOTE_AXIS_MAX;

    return (float)axis / (float)SENTINEL_REMOTE_AXIS_MAX;
}

void SentinelBuildChassisPacket(const Sentinel_Remote_Input_s *remote,
                                uint8_t remote_online,
                                Sentinel_Chassis_Packet_s *packet)
{
    static uint8_t sequence;

    packet->magic = SENTINEL_CHASSIS_PACKET_MAGIC;
    packet->sequence = sequence++ & SENTINEL_PACKET_SEQUENCE_MASK;
    packet->emergency_stop = 1U;
    packet->chassis_mode = CHASSIS_ZERO_FORCE;
    packet->vx = 0.0f;
    packet->vy = 0.0f;
    packet->wz = 0.0f;

    if (!remote_online || remote == NULL || remote->sa == SENTINEL_SWITCH_DOWN)
        return;

    if (remote->sd != SENTINEL_SWITCH_DOWN)
    {
        /* Autonomous and upper-computer modes are introduced after navigation. */
        return;
    }

    packet->emergency_stop = 0U;
    packet->vx = NormalizeRemoteAxis(remote->j2);
    packet->vy = NormalizeRemoteAxis(remote->j1);

    switch (remote->sc)
    {
        case SENTINEL_SWITCH_DOWN:
            packet->chassis_mode = CHASSIS_NO_FOLLOW;
            break;
        case SENTINEL_SWITCH_MID:
            packet->chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
            break;
        case SENTINEL_SWITCH_UP:
            packet->chassis_mode = CHASSIS_ROTATE;
            packet->wz = 1.0f;
            break;
        default:
            packet->emergency_stop = 1U;
            packet->chassis_mode = CHASSIS_ZERO_FORCE;
            packet->vx = 0.0f;
            packet->vy = 0.0f;
            break;
    }
}

#if defined(CHASSIS_BOARD)

static Publisher_t *chassis_cmd_pub;
static HostInstance *board_link;
static Sentinel_Chassis_Packet_s received_packet = {
    .magic = SENTINEL_CHASSIS_PACKET_MAGIC,
    .emergency_stop = 1U,
    .chassis_mode = CHASSIS_ZERO_FORCE,
};
static volatile uint8_t board_link_offline = 1U;

static void BoardLinkOfflineCallback(void *instance)
{
    (void)instance;
    board_link_offline = 1U;
}

static void BoardLinkReceiveCallback(void)
{
    Sentinel_Chassis_Packet_s packet;

    /* comm_instance points to USARTInstance; the received bytes are in recv_buff. */
    memcpy(&packet, ((USARTInstance *)board_link->comm_instance)->recv_buff, sizeof(packet));
    if (packet.magic != SENTINEL_CHASSIS_PACKET_MAGIC)
        return;

    received_packet = packet;
    board_link_offline = 0U;
    DaemonReload(board_link->daemon);
}

void RobotCMDInit(void)
{
    chassis_cmd_pub = PubRegister("sentinel_chassis_cmd", sizeof(Sentinel_Chassis_Cmd_s));

    HostInstanceConf link_config = {
        .usart_handle = &huart1,
        .callback = BoardLinkReceiveCallback,
        .comm_mode = HOST_USART,
        .RECV_SIZE = sizeof(Sentinel_Chassis_Packet_s),
        .daemon_conf = {
            .reload_count = 100U,
            .callback = BoardLinkOfflineCallback,
        },
    };
    board_link = HostInit(&link_config);
}

void RobotCMDTask(void)
{
    Sentinel_Chassis_Cmd_s command = {
        .emergency_stop = 1U,
        .chassis_mode = CHASSIS_ZERO_FORCE,
        .vx = 0.0f,
        .vy = 0.0f,
        .wz = 0.0f,
    };

    if (!board_link_offline)
    {
        command.emergency_stop = received_packet.emergency_stop;
        command.chassis_mode = received_packet.chassis_mode;
        command.vx = received_packet.vx;
        command.vy = received_packet.vy;
        command.wz = received_packet.wz;
    }

    PubPushMessage(chassis_cmd_pub, &command);
}

#elif defined(GIMBAL_BOARD)

static HostInstance *board_link;
static Sentinel_Chassis_Packet_s outgoing_packet;

void RobotCMDInit(void)
{
    HostInstanceConf link_config = {
        .usart_handle = &huart1,
        .comm_mode = HOST_USART,
        .RECV_SIZE = 1U,
    };
    board_link = HostInit(&link_config);
}

void RobotCMDTask(void)
{
    Sentinel_Remote_Input_s remote;
    uint8_t remote_online = SentinelRemoteGet(&remote);

    SentinelBuildChassisPacket(&remote, remote_online, &outgoing_packet);
    HostSend(board_link, (uint8_t *)&outgoing_packet, sizeof(outgoing_packet));
}

#else

void RobotCMDInit(void) {}
void RobotCMDTask(void) {}

#endif
