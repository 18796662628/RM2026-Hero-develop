#ifndef SENTINEL_REMOTE_H
#define SENTINEL_REMOTE_H

#include <stdint.h>

#include "main.h"

/* All joystick channels are normalized to [-1000, 1000] by the W.BUS decoder. */
#define SENTINEL_REMOTE_AXIS_MAX 1000

typedef enum
{
    SENTINEL_SWITCH_DOWN = 0,
    SENTINEL_SWITCH_MID = 1,
    SENTINEL_SWITCH_UP = 2,
} Sentinel_Switch_Position_e;

typedef struct
{
    int16_t j1;
    int16_t j2;
    int16_t j3;
    int16_t j4;
    Sentinel_Switch_Position_e sa;
    Sentinel_Switch_Position_e sb;
    Sentinel_Switch_Position_e sc;
    Sentinel_Switch_Position_e sd;
} Sentinel_Remote_Input_s;

/* RF206S W.BUS output is configured as an S.BUS-compatible serial stream. */
#define WFLY_SBUS_FRAME_SIZE 25U
#define WFLY_SBUS_HEADER 0x0FU
#define WFLY_SBUS_END 0x00U

void SentinelRemoteInit(UART_HandleTypeDef *uart_handle);
uint8_t SentinelRemoteGet(Sentinel_Remote_Input_s *input);
void SentinelRemoteMarkOffline(void);

#endif
