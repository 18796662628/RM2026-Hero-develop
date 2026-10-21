#include "sentinel_remote.h"

#include <string.h>

#include "bsp_log.h"
#include "bsp_usart.h"
#include "daemon.h"

#define WFLY_SBUS_CENTER 1024
#define WFLY_SBUS_SWITCH_DEADBAND 200
#define WFLY_SBUS_FLAG_FRAME_LOST (1U << 2)
#define WFLY_SBUS_FLAG_FAILSAFE (1U << 3)
#define WFLY_SBUS_OFFLINE_TICKS 100U

static USARTInstance *wfly_usart;
static DaemonInstance *wfly_daemon;
static Sentinel_Remote_Input_s latest_input;
static volatile uint8_t input_online;

static int16_t DecodeChannel(const uint8_t *frame, uint8_t index)
{
    uint32_t bit_offset = (uint32_t)index * 11U;
    uint8_t byte_index = 1U + (uint8_t)(bit_offset / 8U);
    uint8_t shift = (uint8_t)(bit_offset % 8U);
    uint32_t packed = (uint32_t)frame[byte_index] |
                      ((uint32_t)frame[byte_index + 1U] << 8U) |
                      ((uint32_t)frame[byte_index + 2U] << 16U);

    return (int16_t)((packed >> shift) & 0x07FFU) - WFLY_SBUS_CENTER;
}

static Sentinel_Switch_Position_e DecodeTwoPositionSwitch(int16_t value)
{
    return value >= 0 ? SENTINEL_SWITCH_DOWN : SENTINEL_SWITCH_UP;
}

static Sentinel_Switch_Position_e DecodeThreePositionSwitch(int16_t value)
{
    if (value > WFLY_SBUS_SWITCH_DEADBAND)
        return SENTINEL_SWITCH_DOWN;
    if (value < -WFLY_SBUS_SWITCH_DEADBAND)
        return SENTINEL_SWITCH_UP;
    return SENTINEL_SWITCH_MID;
}

static uint8_t IsValidFrame(const uint8_t frame[WFLY_SBUS_FRAME_SIZE])
{
    uint8_t flags = frame[23U];
    uint8_t end = frame[24U];

    if (frame[0U] != WFLY_SBUS_HEADER)
        return 0U;

    /* 0x00 is standard S.BUS; the alternatives are used by S.BUS2 variants. */
    if (end != WFLY_SBUS_END && end != 0x04U && end != 0x14U &&
        end != 0x24U && end != 0x34U)
        return 0U;

    if ((flags & (WFLY_SBUS_FLAG_FRAME_LOST | WFLY_SBUS_FLAG_FAILSAFE)) != 0U)
        return 0U;

    return 1U;
}

static void WflyReceiverOfflineCallback(void *instance)
{
    (void)instance;
    SentinelRemoteMarkOffline();
}

static void WflyReceiverRxCallback(void)
{
    const uint8_t *frame = wfly_usart->recv_buff;
    Sentinel_Remote_Input_s input;

    if (!IsValidFrame(frame))
    {
        SentinelRemoteMarkOffline();
        return;
    }

    input.j1 = DecodeChannel(frame, 0U); // CH1
    input.j2 = DecodeChannel(frame, 1U); // CH2
    input.j3 = DecodeChannel(frame, 2U); // CH3
    input.j4 = DecodeChannel(frame, 3U); // CH4
    input.sa = DecodeTwoPositionSwitch(DecodeChannel(frame, 4U));
    input.sb = DecodeThreePositionSwitch(DecodeChannel(frame, 5U));
    input.sc = DecodeThreePositionSwitch(DecodeChannel(frame, 6U));
    input.sd = DecodeTwoPositionSwitch(DecodeChannel(frame, 7U));

    __disable_irq();
    latest_input = input;
    input_online = 1U;
    __enable_irq();
    DaemonReload(wfly_daemon);
}

void SentinelRemoteInit(UART_HandleTypeDef *uart_handle)
{
    USART_Init_Config_s usart_config = {
        .recv_buff_size = WFLY_SBUS_FRAME_SIZE,
        .usart_handle = uart_handle,
        .module_callback = WflyReceiverRxCallback,
    };
    Daemon_Init_Config_s daemon_config = {
        .reload_count = WFLY_SBUS_OFFLINE_TICKS,
        .callback = WflyReceiverOfflineCallback,
    };

    SentinelRemoteMarkOffline();
    wfly_usart = USARTRegister(&usart_config);
    wfly_daemon = DaemonRegister(&daemon_config);
    LOGINFO("[wfly] RF206S W.BUS receiver initialized");
}

uint8_t SentinelRemoteGet(Sentinel_Remote_Input_s *input)
{
    uint8_t online;

    if (input == NULL)
        return 0U;

    __disable_irq();
    *input = latest_input;
    online = input_online;
    __enable_irq();
    return online;
}

void SentinelRemoteMarkOffline(void)
{
    __disable_irq();
    memset(&latest_input, 0, sizeof(latest_input));
    input_online = 0U;
    __enable_irq();
}
