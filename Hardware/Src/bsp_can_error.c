#include "bsp_can_error.h"
#include "bsp_uds.h"

extern CAN_HandleTypeDef hcan;

static struct {
    CanBusState state;
    uint8_t     flags;
    uint32_t    lastErrTick;
    uint32_t    errCount;
    uint32_t    recoverCount;
} CanErr = {0};

static uint8_t lastTec = 0;
static uint8_t lastRec = 0;

static void Bsp_CanError_BusOffRecovery(void);

void Bsp_CanError_Init(void)
{
    CanErr.state = CAN_STATE_ACTIVE;
    CanErr.flags = 0;
    CanErr.lastErrTick = 0;
    CanErr.errCount = 0;
    CanErr.recoverCount = 0;
    lastTec = 0;
    lastRec = 0;
}

void Bsp_CanError_ReportFrameError(uint8_t flags)
{
    CanErr.flags |= (flags & (CAN_ERR_ROLLING_CNT | CAN_ERR_CSUM));
    CanErr.lastErrTick = HAL_GetTick();
    CanErr.errCount++;
}

void Bsp_CanError_Check(void)
{
    uint32_t halErr = HAL_CAN_GetError(&hcan);
    uint32_t tick = HAL_GetTick();

    if (halErr & HAL_CAN_ERROR_ACK) {
        CanErr.flags |= CAN_ERR_ACK;
        CanErr.lastErrTick = tick;
        CanErr.errCount++;
        Bsp_Uds_SetDTC(0x1002, 1);
    }
    if (halErr & HAL_CAN_ERROR_STF) {
        CanErr.flags |= CAN_ERR_STUFF;
        CanErr.lastErrTick = tick;
        CanErr.errCount++;
        Bsp_Uds_SetDTC(0x1004, 1);
    }
    if (halErr & HAL_CAN_ERROR_FOR) {
        CanErr.flags |= CAN_ERR_FORM;
        CanErr.lastErrTick = tick;
        CanErr.errCount++;
    }
    if (halErr & HAL_CAN_ERROR_CRC) {
        CanErr.flags |= CAN_ERR_CRC;
        CanErr.lastErrTick = tick;
        CanErr.errCount++;
        Bsp_Uds_SetDTC(0x1003, 1);
    }

    if (__HAL_CAN_GET_FLAG(&hcan, CAN_FLAG_BOF)) {
        CanErr.flags |= CAN_ERR_BUS_OFF;
        CanErr.state = CAN_STATE_BUS_OFF;
        CanErr.lastErrTick = tick;
        Bsp_Uds_SetDTC(0x1001, 1);
        Bsp_CanError_BusOffRecovery();
    }

    uint32_t esr = hcan.Instance->ESR;
    uint8_t tec = (esr >> 16) & 0xFF;
    uint8_t rec = (esr >> 8) & 0xFF;

    if (tec > 127) {
        CanErr.flags |= CAN_ERR_PASSIVE;
        CanErr.state = CAN_STATE_PASSIVE;
    } else if (tec > 96 || rec > 96) {
        CanErr.state = CAN_STATE_WARNING;
    } else {
        CanErr.state = CAN_STATE_ACTIVE;
    }

    lastTec = tec;
    lastRec = rec;
}

static void Bsp_CanError_BusOffRecovery(void)
{
    HAL_CAN_Stop(&hcan);

    uint32_t waitStart = HAL_GetTick();
    while (HAL_GetTick() - waitStart < 10) {
    }

    if (HAL_CAN_Start(&hcan) == HAL_OK) {
        HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
        CanErr.state = CAN_STATE_ACTIVE;
        CanErr.recoverCount++;
        Bsp_Uds_SetDTC(0x1001, 0);
    }
}

CanBusState Bsp_CanError_GetState(void)
{
    return CanErr.state;
}

uint8_t Bsp_CanError_GetFlags(void)
{
    return CanErr.flags;
}

void Bsp_CanError_ClearFlags(void)
{
    CanErr.flags = 0;
}

uint32_t Bsp_CanError_GetLastErrorTick(void)
{
    return CanErr.lastErrTick;
}

uint32_t Bsp_CanError_GetErrorCount(void)
{
    return CanErr.errCount;
}

uint32_t Bsp_CanError_GetRecoverCount(void)
{
    return CanErr.recoverCount;
}
