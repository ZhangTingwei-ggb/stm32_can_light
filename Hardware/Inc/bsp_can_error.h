#ifndef __BSP_CAN_ERROR_H
#define __BSP_CAN_ERROR_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

typedef enum {
    CAN_ERR_NONE        = 0x00,
    CAN_ERR_ACK         = 0x01,
    CAN_ERR_STUFF       = 0x02,
    CAN_ERR_FORM        = 0x04,
    CAN_ERR_CRC         = 0x08,
    CAN_ERR_BUS_OFF     = 0x10,
    CAN_ERR_PASSIVE     = 0x20,
    CAN_ERR_ROLLING_CNT = 0x40,
    CAN_ERR_CSUM        = 0x80,
} CanErrorFlag;

typedef enum {
    CAN_STATE_ACTIVE    = 0,
    CAN_STATE_WARNING   = 1,
    CAN_STATE_PASSIVE   = 2,
    CAN_STATE_BUS_OFF   = 3,
} CanBusState;

void Bsp_CanError_Init(void);
void Bsp_CanError_Check(void);
void Bsp_CanError_ReportFrameError(uint8_t flags);

CanBusState Bsp_CanError_GetState(void);
uint8_t     Bsp_CanError_GetFlags(void);
void        Bsp_CanError_ClearFlags(void);
uint32_t    Bsp_CanError_GetLastErrorTick(void);
uint32_t    Bsp_CanError_GetErrorCount(void);
uint32_t    Bsp_CanError_GetRecoverCount(void);

#endif
