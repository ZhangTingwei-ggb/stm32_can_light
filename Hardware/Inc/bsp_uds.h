#ifndef __BSP_UDS_H
#define __BSP_UDS_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

#define UDS_REQ_ID    0x7DF
#define UDS_RESP_ID   0x7E7

void Bsp_Uds_Init(void);
void Bsp_Uds_TpFeed(const uint8_t *data, uint8_t dlc);
void Bsp_Uds_SessionTimer_Check(void);
void Bsp_Uds_SelfTest_Update(void);

void Bsp_Uds_SetDTC(uint16_t code, uint8_t active);
void Bsp_Uds_ClearAllDTC(void);

#endif
