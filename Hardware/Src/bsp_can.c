#include "bsp_can.h"
#include "bsp_can_error.h"
#include "bsp_uds.h"
#include <string.h>

extern CAN_HandleTypeDef hcan;

/* ===================== CRC8 (AUTOSAR SAE J1850 poly 0x1D) ===================== */
static uint8_t Can_Crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0xFF;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (uint8_t)((crc << 1) ^ 0x1D);
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

/* ===================== 每 ID RollingCounter 计数表 ===================== */
typedef struct {
    uint16_t id;
    uint8_t  cnt;
} CntEntry;

#define CNT_TABLE_SIZE 16
static CntEntry txCnt[CNT_TABLE_SIZE];
static uint8_t txCntUsed = 0;
static CntEntry rxCnt[CNT_TABLE_SIZE];
static uint8_t rxCntUsed = 0;

static uint8_t Cnt_Next(uint16_t id, CntEntry *table, uint8_t *used)
{
    for (uint8_t i = 0; i < *used; i++) {
        if (table[i].id == id) {
            uint8_t c = table[i].cnt;
            table[i].cnt = (uint8_t)((c + 1) & 0x0F);
            return c;
        }
    }
    if (*used < CNT_TABLE_SIZE) {
        table[*used].id = id;
        table[*used].cnt = 1;
        (*used)++;
    }
    return 0;
}

static int8_t Cnt_Find(uint16_t id, CntEntry *table, uint8_t used)
{
    for (uint8_t i = 0; i < used; i++) {
        if (table[i].id == id) return (int8_t)i;
    }
    return -1;
}

/* ===================== 接收环形缓冲 ===================== */
#define RX_BUF_SIZE 8
static struct {
    CAN_RxHeaderTypeDef Header;
    uint8_t Data[8];
} RxBuf[RX_BUF_SIZE];
static volatile uint8_t RxHead = 0;
static volatile uint8_t RxTail = 0;
static volatile uint8_t RxCount = 0;
static volatile uint32_t RxOverflow = 0;

/* ===================== 发送 ===================== */
static uint8_t Can_Tx(const CAN_TxHeaderTypeDef *hdr, const uint8_t *data)
{
    uint32_t mailbox;
    if (HAL_CAN_AddTxMessage(&hcan, (CAN_TxHeaderTypeDef *)hdr, (uint8_t *)data, &mailbox) != HAL_OK) {
        return 1;
    }

    uint32_t start = HAL_GetTick();
    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) != 3) {
        if (HAL_GetTick() - start > 100) {
            HAL_CAN_AbortTxRequest(&hcan, mailbox);
            return 1;
        }
    }
    return 0;
}

void Bsp_Can_Transmit(CAN_TxHeaderTypeDef *TxHeader, uint8_t *Data)
{
    Can_Tx(TxHeader, Data);
}

uint8_t Bsp_Can_SendSecure(uint16_t stdId, const uint8_t *data, uint8_t len)
{
    uint8_t frame[CAN_SECURE_LEN];
    for (uint8_t i = 0; i < 6; i++) {
        frame[i] = (i < len) ? data[i] : 0;
    }
    frame[CAN_CNT_POS] = Cnt_Next(stdId, txCnt, &txCntUsed);
    frame[CAN_CSUM_POS] = Can_Crc8(frame, 7);

    CAN_TxHeaderTypeDef hdr;
    hdr.StdId = stdId;
    hdr.ExtId = 0;
    hdr.IDE = CAN_ID_STD;
    hdr.RTR = CAN_RTR_DATA;
    hdr.DLC = CAN_SECURE_LEN;
    hdr.TransmitGlobalTime = DISABLE;

    return Can_Tx(&hdr, frame);
}

uint8_t Bsp_Can_SendRaw(uint16_t stdId, const uint8_t *data, uint8_t dlc)
{
    CAN_TxHeaderTypeDef hdr;
    hdr.StdId = stdId;
    hdr.ExtId = 0;
    hdr.IDE = CAN_ID_STD;
    hdr.RTR = CAN_RTR_DATA;
    hdr.DLC = dlc;
    hdr.TransmitGlobalTime = DISABLE;

    return Can_Tx(&hdr, data);
}

void Bsp_Can_Init(void)
{
    CAN_FilterTypeDef filter;
    filter.FilterBank = 0;
    filter.FilterIdHigh = 0x0000;
    filter.FilterIdLow = 0x0000;
    filter.FilterMaskIdHigh = 0x0000;
    filter.FilterMaskIdLow = 0x0000;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation = ENABLE;
    HAL_CAN_ConfigFilter(&hcan, &filter);

    HAL_CAN_Start(&hcan);
    HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

    txCntUsed = 0;
    rxCntUsed = 0;
    RxHead = 0;
    RxTail = 0;
    RxCount = 0;
    RxOverflow = 0;
}

/* ===================== 接收 ===================== */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef hdr;
    uint8_t data[8];

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &hdr, data) == HAL_OK) {
        if (RxCount < RX_BUF_SIZE) {
            RxBuf[RxTail].Header = hdr;
            memcpy(RxBuf[RxTail].Data, data, 8);
            RxTail = (RxTail + 1) % RX_BUF_SIZE;
            RxCount++;
        } else {
            RxOverflow++;
        }
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    }
}

uint8_t Bsp_Can_RxPopFrame(CAN_RxHeaderTypeDef *RxHeader, uint8_t *Data)
{
    if (RxCount == 0) return 0;

    *RxHeader = RxBuf[RxHead].Header;
    memcpy(Data, RxBuf[RxHead].Data, 8);
    RxHead = (RxHead + 1) % RX_BUF_SIZE;
    RxCount--;
    return 1;
}

static CanRxStatus Can_ValidateFrame(const CAN_RxHeaderTypeDef *hdr, const uint8_t *data)
{
    uint8_t cnt = data[CAN_CNT_POS] & 0x0F;

    if (hdr->DLC < CAN_SECURE_LEN)
        return CAN_RX_CRC_FAIL;

    if (Can_Crc8(data, 7) != data[CAN_CSUM_POS]) {
        Bsp_CanError_ReportFrameError(CAN_ERR_CSUM);
        return CAN_RX_CRC_FAIL;
    }

    int8_t idx = Cnt_Find(hdr->StdId, rxCnt, rxCntUsed);
    if (idx < 0) {
        if (rxCntUsed < CNT_TABLE_SIZE) {
            rxCnt[rxCntUsed].id = hdr->StdId;
            rxCnt[rxCntUsed].cnt = cnt;
            rxCntUsed++;
        }
        return CAN_RX_OK;
    }

    uint8_t last = rxCnt[idx].cnt;
    rxCnt[idx].cnt = cnt;

    if (cnt == last) {
        Bsp_CanError_ReportFrameError(CAN_ERR_ROLLING_CNT);
        return CAN_RX_DUPLICATE;
    }
    if (cnt == ((last + 1) & 0x0F)) {
        return CAN_RX_OK;
    }

    Bsp_CanError_ReportFrameError(CAN_ERR_ROLLING_CNT);
    return CAN_RX_CC_ERROR;
}

uint8_t Bsp_Can_ReceiveFlag(void)
{
    return (RxCount > 0);
}

CanRxStatus Bsp_Can_Receive(CAN_RxHeaderTypeDef *RxHeader, uint8_t *Data)
{
    if (!Bsp_Can_RxPopFrame(RxHeader, Data)) {
        return CAN_RX_CRC_FAIL;
    }

    if (RxHeader->StdId == UDS_REQ_ID) {
        return CAN_RX_OK;
    }

    return Can_ValidateFrame(RxHeader, Data);
}