#ifndef __MYCAN_H
#define __MYCAN_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

#define CAN_ID_LOW_BEAM_ON    0x100
#define CAN_ID_LOW_BEAM_OFF   0x101
#define CAN_ID_HIGH_BEAM_ON   0x200
#define CAN_ID_HIGH_BEAM_OFF  0x201
#define CAN_ID_LEFT_TURN_ON   0x300
#define CAN_ID_LEFT_TURN_OFF  0x301
#define CAN_ID_RIGHT_TURN_ON  0x400
#define CAN_ID_RIGHT_TURN_OFF 0x401
#define CAN_ID_HAZARD_ON      0x500
#define CAN_ID_HAZARD_OFF     0x501
#define CAN_ID_AFS_ANGLE      0x600
#define CAN_ID_NM_HEARTBEAT   0x700

#define CAN_SECURE_LEN  8
#define CAN_CNT_POS     6
#define CAN_CSUM_POS    7

typedef enum {
    CAN_RX_OK = 0,
    CAN_RX_CRC_FAIL,
    CAN_RX_DUPLICATE,
    CAN_RX_CC_ERROR,
} CanRxStatus;

void Bsp_Can_Init(void);

uint8_t Bsp_Can_SendSecure(uint16_t stdId, const uint8_t *data, uint8_t len);
uint8_t Bsp_Can_SendRaw(uint16_t stdId, const uint8_t *data, uint8_t dlc);
void    Bsp_Can_Transmit(CAN_TxHeaderTypeDef *TxHeader, uint8_t *Data);

uint8_t      Bsp_Can_ReceiveFlag(void);
CanRxStatus  Bsp_Can_Receive(CAN_RxHeaderTypeDef *RxHeader, uint8_t *Data);
uint8_t      Bsp_Can_RxPopFrame(CAN_RxHeaderTypeDef *RxHeader, uint8_t *Data);

#endif