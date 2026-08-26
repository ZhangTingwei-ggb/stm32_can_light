#include "bsp_nm.h"
#include "bsp_can.h"
#include "bsp_uds.h"

typedef enum {
    NM_STATE_BOOT    = 0x00,
    NM_STATE_NORMAL  = 0x01,
    NM_STATE_ERROR   = 0x02,
    NM_STATE_OFFLINE = 0x03,
} NmNodeState;

typedef struct {
    uint8_t  peerOnline;
    uint8_t  peerState;
    uint16_t peerUptime;
    uint32_t peerAlive;
    uint32_t lastRxTick;
    uint32_t txCount;
    uint32_t rxCount;
    uint32_t sendTimer;
} NmStatus;

static NmStatus nm = {0};
static uint32_t bootTick = 0;

void Bsp_Nm_Init(void)
{
    bootTick = HAL_GetTick();
    nm.peerOnline = 0;
    nm.peerState = NM_STATE_OFFLINE;
    nm.peerUptime = 0;
    nm.peerAlive = 0;
    nm.lastRxTick = 0;
    nm.txCount = 0;
    nm.rxCount = 0;
    nm.sendTimer = 0;
}

void Bsp_Nm_OnHeartbeat(const uint8_t *data)
{
    if (data[0] != NM_LOCAL_NODE_ID) return;

    nm.lastRxTick = HAL_GetTick();
    nm.peerAlive = data[2];
    nm.peerUptime = (uint16_t)((data[3] << 8) | data[4]);
    nm.peerState = data[1];
    nm.peerOnline = 1;
    nm.rxCount++;

    Bsp_Uds_SetDTC(0x1005, 0);
}

void Bsp_Nm_Update(void)
{
    uint32_t tick = HAL_GetTick();

    if (tick - nm.sendTimer >= NM_HEARTBEAT_PERIOD) {
        nm.sendTimer = tick;
        uint8_t payload[6];
        payload[0] = NM_LOCAL_NODE_ID;
        payload[1] = NM_STATE_NORMAL;
        payload[2] = (uint8_t)(tick / NM_HEARTBEAT_PERIOD);
        uint16_t uptime = (uint16_t)((tick - bootTick) / 1000);
        payload[3] = (uptime >> 8) & 0xFF;
        payload[4] = uptime & 0xFF;
        payload[5] = 0x00;
        if (Bsp_Can_SendSecure(NM_HEARTBEAT_ID, payload, 6) == 0) {
            nm.txCount++;
        }
    }

    if (nm.peerOnline) {
        if (tick - nm.lastRxTick > NM_TIMEOUT_MS) {
            nm.peerOnline = 0;
            nm.peerState = NM_STATE_OFFLINE;
            Bsp_Uds_SetDTC(0x1005, 1);
        }
    }
}

uint8_t Bsp_Nm_PeerOnline(void)
{
    return nm.peerOnline;
}

uint8_t Bsp_Nm_GetPeerState(void)
{
    return nm.peerState;
}

uint16_t Bsp_Nm_GetPeerUptime(void)
{
    return nm.peerUptime;
}

uint32_t Bsp_Nm_GetPeerAlive(void)
{
    return nm.peerAlive;
}

uint32_t Bsp_Nm_GetTxCount(void)
{
    return nm.txCount;
}

uint32_t Bsp_Nm_GetRxCount(void)
{
    return nm.rxCount;
}