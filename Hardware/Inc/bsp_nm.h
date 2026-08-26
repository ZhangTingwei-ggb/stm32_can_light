#ifndef __BSP_NM_H
#define __BSP_NM_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

#define NM_HEARTBEAT_ID      0x700
#define NM_LOCAL_NODE_ID     0x01
#define NM_HEARTBEAT_PERIOD  200
#define NM_TIMEOUT_MS        1000
#define NM_LOCAL_NODE_ID     0x01

void Bsp_Nm_Init(void);
void Bsp_Nm_Update(void);
void Bsp_Nm_OnHeartbeat(const uint8_t *data);

uint8_t  Bsp_Nm_PeerOnline(void);
uint8_t  Bsp_Nm_GetPeerState(void);
uint16_t Bsp_Nm_GetPeerUptime(void);
uint32_t Bsp_Nm_GetPeerAlive(void);
uint32_t Bsp_Nm_GetTxCount(void);
uint32_t Bsp_Nm_GetRxCount(void);

#endif