#include "robot_cmd.h"

#include <string.h>

#include "master_process.h"
#include "message_center.h"
#include "robot_def.h"
#include "usart.h"

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

    memcpy(&packet, board_link->comm_instance, sizeof(packet));
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
static Sentinel_Chassis_Packet_s outgoing_packet = {
    .magic = SENTINEL_CHASSIS_PACKET_MAGIC,
    .emergency_stop = 1U,
    .chassis_mode = CHASSIS_ZERO_FORCE,
};

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
    /* W.BUS parsing is not guessed. Before it is implemented, stay safe. */
    HostSend(board_link, (uint8_t *)&outgoing_packet, sizeof(outgoing_packet));
}

#else

void RobotCMDInit(void) {}
void RobotCMDTask(void) {}

#endif
