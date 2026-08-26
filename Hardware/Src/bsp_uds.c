#include "bsp_uds.h"
#include "bsp_can.h"
#include "bsp_can_error.h"
#include "bsp_nm.h"
#include "bsp_light.h"
#include <string.h>

extern LightState currentLightState;

/* ===================== NRC ===================== */
#define NRC_SNS          0x11
#define NRC_SFNS         0x12
#define NRC_IMLOIF       0x13
#define NRC_CNC          0x22
#define NRC_ROOR         0x31
#define NRC_SAD          0x33
#define NRC_IK           0x35
#define NRC_RSE          0x37

#define DID_SW_VERSION      0xF190
#define DID_HW_VERSION      0xF191
#define DID_VENDOR_CODE     0xF192
#define DID_VIN_NUMBER      0xF193
#define DID_LIGHT_STATUS    0xF194
#define DID_CAN_ERROR_INFO  0xF195
#define DID_SERIAL_NUMBER   0xF196
#define DID_NM_STATUS       0xF197

#define ROUTINE_LIGHT_SELFTEST 0x1000

/* ===================== 会话管理 ===================== */
static uint8_t currentSession = 0x01;
static uint32_t sessionTimeout = 0;
static uint8_t securityUnlocked = 0;
static uint16_t securitySeed = 0;
static uint8_t seedRequested = 0;
static uint8_t secFailCount = 0;
static uint32_t secLockUntil = 0;

/* ===================== DTC 表 ===================== */
typedef struct {
    uint16_t code;
    uint8_t  status;
} DtcEntry;

#define DTC_TABLE_SIZE 16
static DtcEntry dtcTable[DTC_TABLE_SIZE] = {
    {0x1001, 0}, {0x1002, 0}, {0x1003, 0}, {0x1004, 0},
    {0x1005, 0}, {0x2001, 0}, {0x2002, 0}, {0x2003, 0},
};
#define DTC_COUNT 8

/* ===================== 灯光自检状态机 ===================== */
typedef enum {
    SELFTEST_IDLE = 0,
    SELFTEST_LOW_BEAM,
    SELFTEST_HIGH_BEAM,
    SELFTEST_LEFT_TURN,
    SELFTEST_RIGHT_TURN,
    SELFTEST_HAZARD,
    SELFTEST_DONE
} SelfTestState;

static SelfTestState selfTestState = SELFTEST_IDLE;
static uint32_t selfTestTimer = 0;

/* ===================== ISO-TP 接收状态 ===================== */
typedef struct {
    uint8_t  inProgress;
    uint8_t  buf[256];
    uint16_t totalLen;
    uint16_t received;
    uint8_t  expectSeq;
} UdsTpRx;

static UdsTpRx tpRx = {0};

/* ===================== 内部函数声明 ===================== */
static uint8_t Uds_BuildNegResponse(uint8_t *resp, uint8_t sid, uint8_t nrc);
static uint8_t Uds_HandleFrame(const uint8_t *req, uint8_t len, uint8_t *resp);
static void    Uds_TpSend(uint8_t *buf, uint16_t len);
static uint8_t Uds_WaitFlowControl(uint32_t timeoutMs);
static uint16_t Uds_FindDtc(uint16_t code);
static uint8_t *Uds_MemCpyU16(uint8_t *dst, uint16_t v);
static uint8_t Uds_HandleReadDTC_Sub_0A(const uint8_t *req, uint8_t len, uint8_t *resp);

/* ===================== DTC 管理 ===================== */
void Bsp_Uds_SetDTC(uint16_t code, uint8_t active)
{
    uint16_t i = Uds_FindDtc(code);
    if (i >= DTC_COUNT) return;

    if (active) {
        dtcTable[i].status |= 0x09;          /* testFailed + confirmed */
    } else {
        dtcTable[i].status &= ~0x09;
    }
}

void Bsp_Uds_ClearAllDTC(void)
{
    for (uint8_t i = 0; i < DTC_COUNT; i++) {
        dtcTable[i].status = 0;
    }
}

static uint16_t Uds_FindDtc(uint16_t code)
{
    for (uint16_t i = 0; i < DTC_COUNT; i++) {
        if (dtcTable[i].code == code) return i;
    }
    return DTC_TABLE_SIZE;
}

static uint8_t *Uds_MemCpyU16(uint8_t *dst, uint16_t v)
{
    dst[0] = (v >> 8) & 0xFF;
    dst[1] = v & 0xFF;
    return dst + 2;
}

/* ===================== 0x10 会话控制 ===================== */
static uint8_t Uds_HandleSessionControl(const uint8_t *req, uint8_t len, uint8_t *resp)
{
    if (len < 2) return Uds_BuildNegResponse(resp, req[0], NRC_IMLOIF);

    uint8_t sub = req[1];
    if (sub != 0x01 && sub != 0x02 && sub != 0x03)
        return Uds_BuildNegResponse(resp, req[0], NRC_SFNS);

    currentSession = sub;
    sessionTimeout = HAL_GetTick() + 5000;
    if (sub == 0x01) {
        securityUnlocked = 0;
        seedRequested = 0;
    }

    resp[0] = 0x50;
    resp[1] = sub;
    resp[2] = 0x32;
    resp[3] = 0xE8;
    return 4;
}

/* ===================== 0x22 / 0x2E DID ===================== */
static uint8_t Uds_HandleReadData(const uint8_t *req, uint8_t len, uint8_t *resp)
{
    if (len < 3) return Uds_BuildNegResponse(resp, req[0], NRC_IMLOIF);

    uint16_t did = (uint16_t)((req[1] << 8) | req[2]);
    resp[0] = 0x62;
    resp[1] = req[1];
    resp[2] = req[2];

    switch (did) {
        case DID_SW_VERSION:
            memcpy(&resp[3], "V1.0", 4);
            return 7;
        case DID_HW_VERSION:
            memcpy(&resp[3], "HW-1", 4);
            return 7;
        case DID_VENDOR_CODE:
            memcpy(&resp[3], "STM32", 5);
            return 8;
        case DID_VIN_NUMBER:
            memcpy(&resp[3], "LVZZC3A14HC123456", 17);
            return 20;
        case DID_LIGHT_STATUS:
            resp[3] = (uint8_t)currentLightState;
            return 4;
        case DID_CAN_ERROR_INFO:
            resp[3] = (uint8_t)Bsp_CanError_GetState();
            resp[4] = Bsp_CanError_GetFlags();
            Uds_MemCpyU16(&resp[5], (uint16_t)Bsp_CanError_GetErrorCount());
            resp[7] = (HAL_GetTick() - Bsp_CanError_GetLastErrorTick()) & 0xFF;
            return 8;
        case DID_SERIAL_NUMBER:
            memcpy(&resp[3], "FLC20260707", 11);
            return 14;
        case DID_NM_STATUS:
            resp[3] = Bsp_Nm_PeerOnline() ? 0x01 : 0x00;
            Uds_MemCpyU16(&resp[4], Bsp_Nm_GetPeerUptime());
            return 6;
        default:
            return Uds_BuildNegResponse(resp, req[0], NRC_ROOR);
    }
}

static uint8_t Uds_HandleWriteData(const uint8_t *req, uint8_t len, uint8_t *resp)
{
    if (len < 3) return Uds_BuildNegResponse(resp, req[0], NRC_IMLOIF);

    if (!securityUnlocked)
        return Uds_BuildNegResponse(resp, req[0], NRC_SAD);

    uint16_t did = (uint16_t)((req[1] << 8) | req[2]);

    switch (did) {
        case DID_VIN_NUMBER:
            if (len - 3 != 17)
                return Uds_BuildNegResponse(resp, req[0], NRC_IMLOIF);
            break;
        default:
            return Uds_BuildNegResponse(resp, req[0], NRC_ROOR);
    }

    resp[0] = 0x6E;
    resp[1] = req[1];
    resp[2] = req[2];
    memcpy(&resp[3], &req[3], len - 3);
    return len;
}

/* ===================== 0x27 安全访问 ===================== */
static uint8_t Uds_HandleSecurityAccess(const uint8_t *req, uint8_t len, uint8_t *resp)
{
    if (len < 2) return Uds_BuildNegResponse(resp, req[0], NRC_IMLOIF);

    uint8_t sub = req[1];
    if (sub != 0x01 && sub != 0x02)
        return Uds_BuildNegResponse(resp, req[0], NRC_SFNS);

    resp[0] = 0x67;
    resp[1] = sub;

    if (sub == 0x01) {
        if (HAL_GetTick() < secLockUntil)
            return Uds_BuildNegResponse(resp, req[0], NRC_RSE);
        securitySeed = (uint16_t)(HAL_GetTick() & 0xFFFF);
        seedRequested = 1;
        securityUnlocked = 0;
        resp[2] = (securitySeed >> 8) & 0xFF;
        resp[3] = securitySeed & 0xFF;
        return 4;
    }

    /* sub == 0x02: 发送密钥 */
    if (!seedRequested)
        return Uds_BuildNegResponse(resp, req[0], NRC_RSE);
    if (len < 4) return Uds_BuildNegResponse(resp, req[0], NRC_IMLOIF);

    uint16_t key = (uint16_t)((req[2] << 8) | req[3]);
    seedRequested = 0;

    if (key == (securitySeed ^ 0x5A5A)) {
        securityUnlocked = 1;
        secFailCount = 0;
        return 2;
    }

    secFailCount++;
    if (secFailCount >= 3) {
        secLockUntil = HAL_GetTick() + 5000;
        secFailCount = 0;
    }
    return Uds_BuildNegResponse(resp, req[0], NRC_IK);
}

/* ===================== 0x31 例行控制 ===================== */
static uint8_t Uds_HandleRoutineControl(const uint8_t *req, uint8_t len, uint8_t *resp)
{
    if (len < 4) return Uds_BuildNegResponse(resp, req[0], NRC_IMLOIF);

    uint8_t sub = req[1];
    uint16_t routineID = (uint16_t)((req[2] << 8) | req[3]);

    if (sub != 0x01 && sub != 0x02 && sub != 0x03)
        return Uds_BuildNegResponse(resp, req[0], NRC_SFNS);

    resp[0] = 0x71;
    resp[1] = sub;
    resp[2] = req[2];
    resp[3] = req[3];

    if (routineID != ROUTINE_LIGHT_SELFTEST)
        return Uds_BuildNegResponse(resp, req[0], NRC_ROOR);

    switch (sub) {
        case 0x01:
            if (selfTestState == SELFTEST_IDLE || selfTestState == SELFTEST_DONE) {
                selfTestState = SELFTEST_LOW_BEAM;
                selfTestTimer = HAL_GetTick();
                return 4;
            }
            return Uds_BuildNegResponse(resp, req[0], NRC_CNC);
        case 0x02:
            if (selfTestState != SELFTEST_IDLE) {
                Bsp_Light_Hazard_Off();
                Bsp_Light_LowBeam_Off();
                selfTestState = SELFTEST_IDLE;
            }
            return 4;
        case 0x03:
            resp[4] = (selfTestState == SELFTEST_DONE) ? 0x00 : 0x01;
            return 5;
        default:
            return Uds_BuildNegResponse(resp, req[0], NRC_SFNS);
    }
}

/* ===================== 0x19 / 0x14 DTC ===================== */
static uint8_t Uds_HandleReadDTC(const uint8_t *req, uint8_t len, uint8_t *resp)
{
    if (len < 2) return Uds_BuildNegResponse(resp, req[0], NRC_IMLOIF);

    uint8_t sub = req[1];
    resp[0] = 0x59;
    resp[1] = sub;

    switch (sub) {
        case 0x01: {
            if (len < 3) return Uds_BuildNegResponse(resp, req[0], NRC_IMLOIF);
            uint8_t mask = req[2];
            uint8_t count = 0;
            for (uint8_t i = 0; i < DTC_COUNT; i++) {
                if (dtcTable[i].status & mask) count++;
            }
            resp[2] = 0;
            resp[3] = count;
            return 4;
        }
        case 0x02: {
            if (len < 3) return Uds_BuildNegResponse(resp, req[0], NRC_IMLOIF);
            uint8_t mask = req[2];
            uint8_t *p = &resp[2];
            for (uint8_t i = 0; i < DTC_COUNT; i++) {
                if (dtcTable[i].status & mask) {
                    p = Uds_MemCpyU16(p, dtcTable[i].code);
                    *p++ = dtcTable[i].status;
                }
            }
            return (uint8_t)(p - resp);
        }
        case 0x0A:
            return Uds_HandleReadDTC_Sub_0A(req, len, resp);
        default:
            return Uds_BuildNegResponse(resp, req[0], NRC_SFNS);
    }
}

static uint8_t Uds_HandleReadDTC_Sub_0A(const uint8_t *req, uint8_t len, uint8_t *resp)
{
    (void)req;
    (void)len;
    uint8_t *p = &resp[2];
    uint8_t found = 0;
    for (uint8_t i = 0; i < DTC_COUNT; i++) {
        if (dtcTable[i].status & 0x09) {
            p = Uds_MemCpyU16(p, dtcTable[i].code);
            *p++ = dtcTable[i].status;
            *p++ = 0x00;
            *p++ = 0x00;
            found = 1;
            break;
        }
    }
    if (!found) {
        p = Uds_MemCpyU16(p, 0x0000);
        *p++ = 0x00;
        *p++ = 0x00;
        *p++ = 0x00;
    }
    return (uint8_t)(p - resp);
}

static uint8_t Uds_HandleClearDTC(const uint8_t *req, uint8_t len, uint8_t *resp)
{
    if (len < 3) return Uds_BuildNegResponse(resp, req[0], NRC_IMLOIF);

    uint16_t group = (uint16_t)((req[1] << 8) | req[2]);
    if (group == 0xFFFF || group == 0x0000) {
        Bsp_Uds_ClearAllDTC();
    } else {
        return Uds_BuildNegResponse(resp, req[0], NRC_ROOR);
    }
    resp[0] = 0x54;
    return 1;
}

/* ===================== 0x3E 保持连接 ===================== */
static uint8_t Uds_HandleTesterPresent(const uint8_t *req, uint8_t len, uint8_t *resp)
{
    if (len < 2 || req[1] != 0x00)
        return Uds_BuildNegResponse(resp, req[0], NRC_SFNS);

    if (currentSession != 0x01)
        sessionTimeout = HAL_GetTick() + 5000;

    resp[0] = 0x7E;
    resp[1] = 0x00;
    return 2;
}

/* ===================== 否定响应 ===================== */
static uint8_t Uds_BuildNegResponse(uint8_t *resp, uint8_t sid, uint8_t nrc)
{
    resp[0] = 0x7F;
    resp[1] = sid;
    resp[2] = nrc;
    return 3;
}

/* ===================== UDS 分发 ===================== */
static uint8_t Uds_HandleFrame(const uint8_t *req, uint8_t len, uint8_t *resp)
{
    if (len == 0) return 0;

    uint8_t service = req[0];
    switch (service) {
        case 0x10: return Uds_HandleSessionControl(req, len, resp);
        case 0x22: return Uds_HandleReadData(req, len, resp);
        case 0x2E: return Uds_HandleWriteData(req, len, resp);
        case 0x27: return Uds_HandleSecurityAccess(req, len, resp);
        case 0x31: return Uds_HandleRoutineControl(req, len, resp);
        case 0x19: return Uds_HandleReadDTC(req, len, resp);
        case 0x14: return Uds_HandleClearDTC(req, len, resp);
        case 0x3E: return Uds_HandleTesterPresent(req, len, resp);
        default:   return Uds_BuildNegResponse(resp, req[0], NRC_SNS);
    }
}

/* ===================== ISO-TP 发送 ===================== */
static void Uds_TpSend(uint8_t *buf, uint16_t len)
{
    uint8_t frame[8];

    if (len <= 7) {
        frame[0] = (0x0 << 4) | (uint8_t)len;
        memcpy(&frame[1], buf, len);
        Bsp_Can_SendRaw(UDS_RESP_ID, frame, (uint8_t)(len + 1));
        return;
    }

    frame[0] = (0x1 << 4) | ((len >> 8) & 0x0F);
    frame[1] = len & 0xFF;
    memcpy(&frame[2], buf, 6);
    Bsp_Can_SendRaw(UDS_RESP_ID, frame, 8);

    if (!Uds_WaitFlowControl(200)) return;

    uint16_t idx = 6;
    uint8_t seq = 1;
    while (idx < len) {
        uint8_t n = (len - idx > 7) ? 7 : (uint8_t)(len - idx);
        frame[0] = (0x2 << 4) | (seq & 0x0F);
        memcpy(&frame[1], &buf[idx], n);
        Bsp_Can_SendRaw(UDS_RESP_ID, frame, (uint8_t)(n + 1));
        seq++;
        idx += n;
    }
}

static uint8_t Uds_WaitFlowControl(uint32_t timeoutMs)
{
    uint32_t start = HAL_GetTick();
    CAN_RxHeaderTypeDef hdr;
    uint8_t data[8];

    while (HAL_GetTick() - start < timeoutMs) {
        if (Bsp_Can_RxPopFrame(&hdr, data)) {
            if (hdr.StdId == UDS_REQ_ID && ((data[0] & 0xF0) == 0x30)) {
                return 1;
            }
        }
    }
    return 0;
}

/* ===================== ISO-TP 接收 ===================== */
void Bsp_Uds_TpFeed(const uint8_t *data, uint8_t dlc)
{
    uint8_t pci = data[0];
    uint8_t type = pci >> 4;
    uint8_t response[256];

    if (type == 0x0) {
        uint8_t len = pci & 0x0F;
        if (len > dlc - 1) len = dlc - 1;
        if (len == 0) return;
        uint16_t respLen = Uds_HandleFrame(&data[1], len, response);
        if (respLen > 0) Uds_TpSend(response, respLen);
    }
    else if (type == 0x1) {
        uint16_t total = (uint16_t)(((pci & 0x0F) << 8) | data[1]);
        if (total == 0 || total > sizeof(tpRx.buf)) return;
        tpRx.inProgress = 1;
        tpRx.totalLen = total;
        tpRx.received = 0;
        tpRx.expectSeq = 1;
        uint8_t n = (total > 6) ? 6 : (uint8_t)total;
        memcpy(&tpRx.buf[0], &data[2], n);
        tpRx.received = n;

        uint8_t fc[8] = {0x30, 0x00, 0x00};
        Bsp_Can_SendRaw(UDS_RESP_ID, fc, 3);
    }
    else if (type == 0x2) {
        if (!tpRx.inProgress) return;
        uint8_t seq = pci & 0x0F;
        if (seq != (tpRx.expectSeq & 0x0F)) {
            tpRx.inProgress = 0;
            return;
        }
        uint16_t remain = tpRx.totalLen - tpRx.received;
        uint8_t n = (remain > 7) ? 7 : (uint8_t)remain;
        memcpy(&tpRx.buf[tpRx.received], &data[1], n);
        tpRx.received += n;
        tpRx.expectSeq++;

        if (tpRx.received >= tpRx.totalLen) {
            tpRx.inProgress = 0;
            uint16_t respLen = Uds_HandleFrame(tpRx.buf, tpRx.received, response);
            if (respLen > 0) Uds_TpSend(response, respLen);
        }
    }
}

/* ===================== 会话超时 ===================== */
void Bsp_Uds_SessionTimer_Check(void)
{
    if (currentSession != 0x01) {
        if (HAL_GetTick() > sessionTimeout) {
            currentSession = 0x01;
            securityUnlocked = 0;
            seedRequested = 0;
        }
    }
}

/* ===================== 灯光自检 ===================== */
void Bsp_Uds_SelfTest_Update(void)
{
    if (selfTestState == SELFTEST_IDLE || selfTestState == SELFTEST_DONE)
        return;

    if (HAL_GetTick() - selfTestTimer < 300)
        return;

    selfTestTimer = HAL_GetTick();

    switch (selfTestState) {
        case SELFTEST_LOW_BEAM:
            Bsp_Light_LowBeam_On();
            selfTestState = SELFTEST_HIGH_BEAM;
            break;
        case SELFTEST_HIGH_BEAM:
            Bsp_Light_LowBeam_Off();
            Bsp_Light_HighBeam_On();
            selfTestState = SELFTEST_LEFT_TURN;
            break;
        case SELFTEST_LEFT_TURN:
            Bsp_Light_HighBeam_Off();
            Bsp_Light_LeftTurn_On();
            selfTestState = SELFTEST_RIGHT_TURN;
            break;
        case SELFTEST_RIGHT_TURN:
            Bsp_Light_LeftTurn_Off();
            Bsp_Light_RightTurn_On();
            selfTestState = SELFTEST_HAZARD;
            break;
        case SELFTEST_HAZARD:
            Bsp_Light_RightTurn_Off();
            Bsp_Light_Hazard_On();
            selfTestState = SELFTEST_DONE;
            break;
        default:
            selfTestState = SELFTEST_IDLE;
            break;
    }
}

void Bsp_Uds_Init(void)
{
    currentSession = 0x01;
    securityUnlocked = 0;
    seedRequested = 0;
    selfTestState = SELFTEST_IDLE;
    tpRx.inProgress = 0;
}
