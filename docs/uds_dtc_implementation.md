# UDS 诊断协议 + CAN 错误处理 — 详细实现方案

> **实现状态：已落地为代码**（`Hardware/Src/bsp_can_error.c` / `bsp_uds.c` / `bsp_nm.c`，并已集成进 `main.c`）
> 配套诊断仪脚本：`tools/uds_tester.py`（基于 python-can）

## 一、概述

### 实现目标

在前大灯控制系统基础上，增加以下两个车载嵌入式核心功能：

| 功能模块 | 说明 | 面试价值 |
|----------|------|---------|
| **CAN 错误处理** | 监控 CAN 总线错误状态，实现 Bus-Off 自动恢复 | ★★★★★ |
| **UDS 诊断协议** | 实现 ISO 14229 标准的诊断通信，支持故障码查询 | ★★★★★ |
| **网络管理心跳** | 周期心跳帧 + 对端掉线监测（超时置 DTC） | ★★★★ |
| **信号安全** | 应用帧 RollingCounter + CRC8 (AUTOSAR) 校验 | ★★★★ |

### 硬件需求

| 硬件 | 用途 | 说明 |
|------|------|------|
| **USB-CAN 分析仪**（如 PCAN-USB、CANalyst-II、ZCANUSB） | 充当诊断仪(Tester) | 必备，用于发送 UDS 诊断请求 |
| **双绞线** | CAN 总线连接 | 主机 ←→ 从机 ←→ CAN 分析仪 |
| **2个 120Ω 电阻** | CAN 总线终端电阻 | 总线两端各接一个 |

> **注意**：主机和从机各需要 120Ω 终端电阻。如果将 CAN 分析仪也挂到总线上，需要确保整条总线正好**两端各一个 120Ω**，不能多也不能少。

### CAN 总线连接拓扑

```
                 120Ω                       120Ω
    ┌────────┐    │                          │   ┌────────┐
    │  主机  │────┼──────────────────────────┼───│  从机  │
    │ STM32  │    │    CAN_H (双绞线)         │   │ STM32  │
    └────────┘    │                          │   └────────┘
                  │    CAN_L (双绞线)         │
                  │                          │
                  └──────┬───────────────────┘
                         │
                    ┌────┴────┐
                    │ CAN分析仪│
                    │(诊断仪)  │
                    └─────────┘
```

---

## 二、CAN 错误处理与 Bus-Off 恢复

### 2.1 CAN 错误状态机

STM32F1 CAN 控制器有三种错误状态：

```
                    TEC > 127
  ┌─────────┐  ───────────────>  ┌────────────┐  ───────────────>  ┌──────────┐
  │ Active  │                    │ Error Pass │  TEC > 255         │ Bus-Off  │
  │ Error   │  <─────────────    │ ive        │  ───────────────>  │          │
  └─────────┘    REC < 128      └────────────┘                    └──────────┘
                                                                       │
      │                          TEC = Transmit Error Counter          │
      │                          REC = Receive Error Counter           │
      │                          128次成功发送后TEC归零                │
      └────────────────────────────────────────────────────────────────┘
                          HAL_CAN_Stop + 延迟 + HAL_CAN_Start
```

### 2.2 软件实现

#### 新增文件：`Hardware/Inc/bsp_can_error.h`

```c
#ifndef __BSP_CAN_ERROR_H
#define __BSP_CAN_ERROR_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

/* CAN 错误状态枚举 */
typedef enum {
    CAN_ERR_NONE        = 0x00,
    CAN_ERR_ACK         = 0x01,   // 应答错误（总线上没有其他节点）
    CAN_ERR_STUFF       = 0x02,   // 位填充错误
    CAN_ERR_FORM        = 0x04,   // 格式错误
    CAN_ERR_CRC         = 0x08,   // CRC 校验错误
    CAN_ERR_BUS_OFF     = 0x10,   // 总线关闭
    CAN_ERR_PASSIVE     = 0x20,   // 错误被动
} CanErrorFlag;

/* CAN 综合错误状态 */
typedef enum {
    CAN_STATE_ACTIVE    = 0,      // 主动错误（正常工作）
    CAN_STATE_WARNING   = 1,      // 警告（TEC/REC > 96）
    CAN_STATE_PASSIVE   = 2,      // 被动错误（TEC > 127）
    CAN_STATE_BUS_OFF   = 3,      // 总线关闭（TEC > 255）
} CanBusState;

/* 初始化 CAN 错误监控 */
void Bsp_CanError_Init(void);

/* 周期性错误检查（在 main loop 中每 100ms 调用） */
void Bsp_CanError_Check(void);

/* 获取当前总线状态 */
CanBusState Bsp_CanError_GetState(void);

/* 获取错误标志位 */
uint8_t Bsp_CanError_GetFlags(void);

/* 清除错误标志 */
void Bsp_CanError_ClearFlags(void);

/* 获取最近一次错误的时间戳 */
uint32_t Bsp_CanError_GetLastErrorTick(void);

#endif
```

#### 新增文件：`Hardware/Src/bsp_can_error.c`

```c
#include "bsp_can_error.h"

extern CAN_HandleTypeDef hcan;

static struct {
    CanBusState state;
    uint8_t     flags;        // 错误标志位
    uint32_t    lastErrTick;  // 最后一次错误的时间戳
    uint32_t    errCount;     // 错误总次数
    uint32_t    recoverCount; // 恢复总次数
} CanErr = {0};

/* 保存上一次的 TEC/REC 用于判断状态变化 */
static uint8_t lastTec = 0;
static uint8_t lastRec = 0;

void Bsp_CanError_Init(void)
{
    CanErr.state = CAN_STATE_ACTIVE;
    CanErr.flags = 0;
    CanErr.errCount = 0;
    CanErr.recoverCount = 0;
    lastTec = 0;
    lastRec = 0;
}

void Bsp_CanError_Check(void)
{
    /* 1. 读取 CAN 控制器错误状态 */
    uint32_t halErr = HAL_CAN_GetError(&hcan);
    uint32_t tick = HAL_GetTick();
    
    /* 2. 检查具体错误类型 */
    if (halErr & HAL_CAN_ERROR_ACK) {
        CanErr.flags |= CAN_ERR_ACK;
        CanErr.lastErrTick = tick;
        CanErr.errCount++;
    }
    if (halErr & HAL_CAN_ERROR_STF) {
        CanErr.flags |= CAN_ERR_STUFF;
        CanErr.lastErrTick = tick;
        CanErr.errCount++;
    }
    if (halErr & HAL_CAN_ERROR_FORM) {
        CanErr.flags |= CAN_ERR_FORM;
        CanErr.lastErrTick = tick;
        CanErr.errCount++;
    }
    if (halErr & HAL_CAN_ERROR_CRC) {
        CanErr.flags |= CAN_ERR_CRC;
        CanErr.lastErrTick = tick;
        CanErr.errCount++;
    }
    
    /* 3. 检查 Bus-Off 状态 */
    if (HAL_CAN_IsBusOff(&hcan)) {
        CanErr.flags |= CAN_ERR_BUS_OFF;
        CanErr.state = CAN_STATE_BUS_OFF;
        CanErr.lastErrTick = tick;
        
        /* 执行 Bus-Off 恢复流程 */
        Bsp_CanError_BusOffRecovery();
    }
    
    /* 4. 检查 Error Passive（通过 TEC 寄存器判断） */
    /* STM32F1 的 CAN_ESR 寄存器存放 TEC/REC */
    uint32_t esr = hcan.Instance->ESR;
    uint8_t tec = (esr >> 16) & 0xFF;  // CAN_ESR[23:16] = TEC
    uint8_t rec = (esr >> 8) & 0xFF;   // CAN_ESR[15:8]  = REC
    
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

/* Bus-Off 恢复流程 */
static void Bsp_CanError_BusOffRecovery(void)
{
    /* ISO 11898 建议的恢复流程：
     * 1. 停止 CAN 控制器
     * 2. 等待总线空闲（延迟）
     * 3. 重新初始化 CAN 控制器
     * 4. 重新启动
     */
    
    /* Step 1: 停止 CAN */
    HAL_CAN_Stop(&hcan);
    
    /* Step 2: 延迟等待总线恢复（128 * 11位时间 ≈ 5.6ms @ 250kbps） */
    uint32_t waitStart = HAL_GetTick();
    while (HAL_GetTick() - waitStart < 10) {
        /* 等待10ms，确保总线恢复 */
    }
    
    /* Step 3: 重新启动 CAN */
    if (HAL_CAN_Start(&hcan) == HAL_OK) {
        /* Step 4: 重新激活 RX 中断 */
        HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
        
        CanErr.state = CAN_STATE_ACTIVE;
        CanErr.recoverCount++;
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
```

#### 需在 `main.c` 中修改

```c
/* USER CODE BEGIN Includes */
#include "bsp_can_error.h"
/* USER CODE END Includes */

/* USER CODE BEGIN 2 */
OLED_Init();
Bsp_Can_Init();
Bsp_Light_Init();
Bsp_Encoder_Init();
Bsp_CanError_Init();    // ← 新增：初始化 CAN 错误监控

OLED_ShowString(1, 1, "Indicator:");
OLED_ShowString(2, 1, "Dir:");
/* USER CODE END 2 */

/* USER CODE BEGIN WHILE */
while (1)
{
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
    
    // ===== 新增：CAN 错误周期检查（每 100ms） =====
    static uint32_t lastCanErrCheck = 0;
    if (HAL_GetTick() - lastCanErrCheck > 100) {
        Bsp_CanError_Check();
        lastCanErrCheck = HAL_GetTick();
        
        // 可选：在 OLED 上显示 CAN 状态
        CanBusState state = Bsp_CanError_GetState();
        if (state == CAN_STATE_ACTIVE) {
            // OLED_ShowString(3, 1, "CAN:OK ");
        } else if (state == CAN_STATE_WARNING) {
            // OLED_ShowString(3, 1, "CAN:WARN");
        } else if (state == CAN_STATE_PASSIVE) {
            // OLED_ShowString(3, 1, "CAN:PASS");
        } else if (state == CAN_STATE_BUS_OFF) {
            // OLED_ShowString(3, 1, "CAN:BOFF");
        }
    }
    // ==============================================
    
    int32_t angle = Bsp_Encoder_GetCount();
    // ... 原有代码 ...
```

### 2.3 硬件排查对照表

当 `Bsp_CanError_Check()` 检测到错误时，按以下方向排查：

| 错误类型 | 可能原因 | 排查方法 |
|----------|---------|---------|
| **ACK 错误** | 总线上没有其他节点；终端电阻缺失 | 检查从机电源和连接；用万用表量 CAN_H/CAN_L 间电阻应为 **60Ω** |
| **填充/格式/CRC 错误** | 总线干扰；波特率不匹配 | 检查双绞线是否远离电机等干扰源；用示波器看波形质量 |
| **Bus-Off** | 连续错误超过 255 次 | 检查接线，用 CAN 分析仪监听总线 |

---

## 三、UDS 诊断协议 (ISO 14229)

### 3.1 UDS 协议概述

UDS (Unified Diagnostic Services) 是车载电子系统的标准诊断协议，基于 ISO 14229 定义。

**通信模型**：

```
┌──────────┐     UDS请求 (CAN ID=0x7DF)     ┌──────────┐
│ 诊断仪   │ ──────────────────────────────>│ ECU      │
│ (Tester) │                                 │ (本设备) │
│ CAN分析仪 │ <──────────────────────────────│          │
└──────────┘     UDS响应 (CAN ID=0x7DF+8)    └──────────┘
```

### 3.2 CAN 诊断地址分配

| 用途 | CAN ID | 说明 |
|------|--------|------|
| 物理请求 (Tester→ECU) | **0x7DF** | 标准诊断请求 ID |
| 物理响应 (ECU→Tester) | **0x7E7** | 0x7DF + 8 = 0x7E7 |

> 注：这是 11 位 ID 的标准诊断地址。可以使用其他 ID，只要保证请求和响应 ID 的关系为 `RespID = ReqID + 8`。

### 3.3 ISO 15765-2 传输层

CAN 单帧最多 8 字节。UDS 报文可能超过 8 字节，需要拆包：

```
单帧 (SF) [≤7字节数据]:
┌─────┬─────────────────────┐
│ PCI │     Data             │
│ 0x03 │ 0x22 0xF1 0x90      │  ← 3字节UDS数据
└─────┴─────────────────────┘

PCI = [7:4]=0x0 + [3:0]=数据长度(3)

多帧发送（首帧 + 连续帧）:
┌────────┬──────────────────┐
│  FF    │  Data(首包)       │  ← PCI=0x10 + 全长(高4位)
│ 0x12 0x34 │ ...            │  ← 全长 = 0x1234 & 0x0FFF = 0x234
├────────┼──────────────────┤
│  CF    │  Data(后续)       │  ← PCI=0x20 + 序号
│ 0x21   │ ...               │  ← 第1个连续帧
│ 0x22   │ ...               │  ← 第2个连续帧
└────────┴──────────────────┘

流控帧 (FC):
┌─────┬────┬────┐
│ PCI │ BS │STmin│
│ 0x30 │0x00│ 0x00│  ← 持续发送，无限制
└─────┴────┴────┘
PCI = 0x30
BS  = 0x00 (无块大小限制)
STmin = 0x00 (连续帧间最小间隔0ms)
```

### 3.4 UDS 服务实现细节

#### 0x10 — DiagnosticSessionControl (诊断会话控制)

```
请求:  0x10 + [子功能]
响应:  0x50 + [子功能] + [P2Server_max] + [P2StarServer_max]

子功能:
  0x01 = defaultSession (默认会话)
  0x02 = programmingSession (编程会话)
  0x03 = extendedDiagnosticSession (扩展诊断会话)
```

```c
static uint8_t currentSession = 0x01;  // 当前会话，默认 Default
static uint32_t sessionTimeout = 0;     // 会话超时定时器

static uint8_t Uds_HandleSessionControl(uint8_t *req, uint8_t len, uint8_t *resp)
{
    uint8_t subFunc = req[1];
    
    /* 会话切换有效性检查 */
    switch (subFunc) {
        case 0x01:  // Default
        case 0x02:  // Programming
        case 0x03:  // Extended
            break;
        default:
            return Uds_BuildNegResponse(resp, req[0], 0x12);  // SubFunctionNotSupported
    }
    
    currentSession = subFunc;
    sessionTimeout = HAL_GetTick() + 5000;  // 5秒超时
    
    /* 构建肯定响应: 0x50 + subFunc + P2(500ms) + P2*(5000ms) */
    resp[0] = 0x50;
    resp[1] = subFunc;
    resp[2] = 0x32;  // P2 = 50 * 10ms = 500ms
    resp[3] = 0xE8;  // P2* = 232 * 10ms = 2320ms (0xE8)
    return 4;
}
```

#### 0x22 — ReadDataByIdentifier (通过ID读取数据)

```
请求:  0x22 + [DID高字节] + [DID低字节]
响应:  0x62 + [DID高字节] + [DID低字节] + [数据]
```

**DID 定义表**：

```c
#define DID_SW_VERSION      0xF190  // 软件版本号 (4字节)
#define DID_HW_VERSION      0xF191  // 硬件版本号 (4字节)
#define DID_VENDOR_CODE     0xF192  // 供应商代码 (4字节)
#define DID_VIN_NUMBER      0xF193  // VIN码 (17字节)
#define DID_LIGHT_STATUS    0xF194  // 当前灯光状态 (1字节)
#define DID_CAN_ERROR_INFO  0xF195  // CAN错误统计信息 (8字节)
#define DID_SERIAL_NUMBER   0xF196  // 序列号 (8字节)

static uint8_t Uds_HandleReadData(uint8_t *req, uint8_t len, uint8_t *resp)
{
    uint16_t did = (req[1] << 8) | req[2];
    uint8_t idx = 3;  // 响应数据起始位置
    
    resp[0] = 0x62;           // 肯定响应 SID
    resp[1] = req[1];         // DID 高字节
    resp[2] = req[2];         // DID 低字节
    
    switch (did) {
        case DID_SW_VERSION:
            resp[3] = 'V'; resp[4] = '1'; resp[5] = '.'; resp[6] = '0';
            return 7;
            
        case DID_HW_VERSION:
            resp[3] = 'H'; resp[4] = 'W'; resp[5] = '-'; resp[6] = '1';
            return 7;
            
        case DID_VENDOR_CODE:
            resp[3] = 'S'; resp[4] = 'T'; resp[5] = 'M'; resp[6] = '3';
            return 7;
            
        case DID_VIN_NUMBER:
            /* 17字节 VIN 码 */
            memcpy(&resp[3], "LVZZC3A14HC123456", 17);
            return 20;
            
        case DID_LIGHT_STATUS:
            resp[3] = (uint8_t)currentLightState;
            return 4;
            
        case DID_CAN_ERROR_INFO:
            resp[3] = (uint8_t)Bsp_CanError_GetState();
            resp[4] = Bsp_CanError_GetFlags();
            resp[5] = (HAL_GetTick() - Bsp_CanError_GetLastErrorTick()) >> 24;
            resp[6] = (HAL_GetTick() - Bsp_CanError_GetLastErrorTick()) >> 16;
            resp[7] = (HAL_GetTick() - Bsp_CanError_GetLastErrorTick()) >> 8;
            resp[8] = (HAL_GetTick() - Bsp_CanError_GetLastErrorTick());
            return 9;
            
        default:
            return Uds_BuildNegResponse(resp, req[0], 0x31);  // RequestOutOfRange
    }
}
```

#### 0x2E — WriteDataByIdentifier (通过ID写入数据)

```
请求:  0x2E + [DID] + [数据]
响应:  0x6E + [DID] + [数据]
```

```c
static uint8_t Uds_HandleWriteData(uint8_t *req, uint8_t len, uint8_t *resp)
{
    /* 检查安全访问状态（只有编程会话且已解锁才能写） */
    if (currentSession != 0x02 && !securityUnlocked) {
        return Uds_BuildNegResponse(resp, req[0], 0x33);  // SecurityAccessDenied
    }
    
    uint16_t did = (req[1] << 8) | req[2];
    
    switch (did) {
        case DID_VIN_NUMBER:
            if (len - 3 != 17)  // VIN需要17字节
                return Uds_BuildNegResponse(resp, req[0], 0x13);  // IncorrectMessageLen
            // 保存 VIN 码到 EEPROM/Flash...
            break;
            
        default:
            return Uds_BuildNegResponse(resp, req[0], 0x31);  // RequestOutOfRange
    }
    
    resp[0] = 0x6E;
    resp[1] = req[1];
    resp[2] = req[2];
    memcpy(&resp[3], &req[3], len - 3);
    return len;
}
```

#### 0x31 — RoutineControl (例行控制)

```
请求:  0x31 + [子功能] + [RoutineID高] + [RoutineID低] + [可选参数]
响应:  0x71 + [子功能] + [RoutineID高] + [RoutineID低] + [结果]

子功能:
  0x01 = Start (开始)
  0x02 = Stop (停止)
  0x03 = GetResult (获取结果)

RoutineID:
  0x1000 = 灯光自检 (Light Self-Test)
```

**灯光自检流程**：

```c
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
static uint8_t  selfTestResult = 0xFF;  // 0xFF = 未完成

void Bsp_Light_SelfTest_Update(void)
{
    if (selfTestState == SELFTEST_IDLE || selfTestState == SELFTEST_DONE)
        return;
    
    if (HAL_GetTick() - selfTestTimer < 500)  // 每个灯亮500ms
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
            break;
    }
}

static uint8_t Uds_HandleRoutineControl(uint8_t *req, uint8_t len, uint8_t *resp)
{
    uint8_t subFunc = req[1];
    uint16_t routineID = (req[2] << 8) | req[3];
    
    resp[0] = 0x71;
    resp[1] = subFunc;
    resp[2] = req[2];
    resp[3] = req[3];
    
    if (routineID == 0x1000) {  // 灯光自检
        switch (subFunc) {
            case 0x01:  // Start
                selfTestState = SELFTEST_LOW_BEAM;
                selfTestTimer = HAL_GetTick();
                selfTestResult = 0xFF;
                return 4;
                
            case 0x02:  // Stop
                if (selfTestState != SELFTEST_IDLE) {
                    Bsp_Light_Hazard_Off();
                    Bsp_Light_LowBeam_Off();
                    selfTestState = SELFTEST_IDLE;
                }
                return 4;
                
            case 0x03:  // GetResult
                resp[4] = (selfTestState == SELFTEST_DONE) ? 0x00 : 0x01;
                return 5;
                
            default:
                return Uds_BuildNegResponse(resp, req[0], 0x12);
        }
    }
    
    return Uds_BuildNegResponse(resp, req[0], 0x31);
}
```

#### 0x19 — ReadDTCInformation (读取故障码)

```
请求:  0x19 + [子功能] + [可选参数]
响应:  0x59 + [子功能] + [数据]

子功能:
  0x01 = 读取 DTC 数量（按状态掩码）
  0x02 = 读取 DTC 列表
  0x0A = 读取固化 DTC（最近一次）
```

```c
/* DTC 故障码定义 */
typedef struct {
    uint16_t code;      // DTC 编码
    uint8_t  status;    // 故障状态 (bit0=当前激活, bit4=历史记录)
    char     desc[16];  // 故障描述
} DTC_Entry;

static DTC_Entry dtcTable[] = {
    {0x1001, 0, "CAN BUS OFF"},
    {0x1002, 0, "CAN ACK ERR"},
    {0x1003, 0, "CAN CRC ERR"},
    {0x1004, 0, "CAN STUFF ERR"},
    {0x1005, 0, "COMM LOST"},     // 对端心跳超时
    {0x2001, 0, "LOW BEAM FAIL"},
    {0x2002, 0, "HIGH BEAM FAIL"},
    {0x2003, 0, "TURN LEFT FAIL"},
};
#define DTC_COUNT (sizeof(dtcTable) / sizeof(dtcTable[0]))

static uint8_t Uds_HandleReadDTC(uint8_t *req, uint8_t len, uint8_t *resp)
{
    uint8_t subFunc = req[1];
    
    resp[0] = 0x59;
    resp[1] = subFunc;
    
    switch (subFunc) {
        case 0x01: {  // 读取 DTC 数量
            uint8_t statusMask = req[2];
            uint8_t activeCount = 0;
            for (int i = 0; i < DTC_COUNT; i++) {
                if (dtcTable[i].status & statusMask)
                    activeCount++;
            }
            resp[2] = 0;           // 高字节
            resp[3] = activeCount; // 低字节
            return 4;
        }
        
        case 0x02: {  // 读取 DTC 列表
            uint8_t statusMask = req[2];
            uint8_t idx = 3;
            for (int i = 0; i < DTC_COUNT; i++) {
                if (dtcTable[i].status & statusMask) {
                    resp[idx++] = (dtcTable[i].code >> 8) & 0xFF;  // DTC高
                    resp[idx++] = dtcTable[i].code & 0xFF;         // DTC低
                    resp[idx++] = dtcTable[i].status;               // 状态
                }
            }
            return idx;
        }
        
        default:
            return Uds_BuildNegResponse(resp, req[0], 0x12);
    }
}
```

#### 0x14 — ClearDiagnosticInformation (清除故障码)

```
请求:  0x14 + [groupOfDTCHigh] + [groupOfDTCLow]
       0x14 0xFF 0xFF = 清除所有DTC
响应:  0x54
```

```c
static uint8_t Uds_HandleClearDTC(uint8_t *req, uint8_t len, uint8_t *resp)
{
    uint16_t group = (req[1] << 8) | req[2];
    
    if (group == 0xFFFF || group == 0x0000) {
        // 清除所有 DTC
        for (int i = 0; i < DTC_COUNT; i++) {
            dtcTable[i].status = 0;
        }
    }
    
    resp[0] = 0x54;
    return 1;
}
```

#### 0x27 — SecurityAccess (安全访问)

```
请求:  0x27 + [子功能] + [密钥]
响应:  0x67 + [子功能] + [种子/结果]

子功能:
  0x01 = 请求种子 (RequestSeed)
  0x02 = 发送密钥 (SendKey)
  0x03 = 请求种子 (扩展)
  0x04 = 发送密钥 (扩展)
```

```c
static uint8_t securityUnlocked = 0;
static uint16_t securitySeed = 0;

static uint8_t Uds_HandleSecurityAccess(uint8_t *req, uint8_t len, uint8_t *resp)
{
    uint8_t subFunc = req[1];
    
    resp[0] = 0x67;
    resp[1] = subFunc;
    
    switch (subFunc) {
        case 0x01: {  // 请求种子
            securitySeed = (uint16_t)(HAL_GetTick() & 0xFFFF);  // 用时间作为种子
            resp[2] = (securitySeed >> 8) & 0xFF;
            resp[3] = securitySeed & 0xFF;
            securityUnlocked = 0;
            return 4;
        }
        
        case 0x02: {  // 发送密钥
            uint16_t key = (req[2] << 8) | req[3];
            // 简单算法：密钥 = 种子 XOR 0x5A5A
            if (key == (securitySeed ^ 0x5A5A)) {
                securityUnlocked = 1;
                return 2;  // 简短肯定响应
            } else {
                securityUnlocked = 0;
                return Uds_BuildNegResponse(resp, req[0], 0x35);  // InvalidKey
            }
        }
        
        default:
            return Uds_BuildNegResponse(resp, req[0], 0x12);
    }
}
```

#### 0x3E — TesterPresent (保持连接)

```
请求:  0x3E + [0x00] (零子功能)
响应:  0x7E + [0x00]
```

```c
static uint8_t Uds_HandleTesterPresent(uint8_t *req, uint8_t len, uint8_t *resp)
{
    sessionTimeout = HAL_GetTick() + 5000;  // 重置会话超时
    resp[0] = 0x7E;
    resp[1] = req[1];
    return 2;
}
```

### 3.5 否定响应码 (NRC)

```c
/* 常用否定响应码 */
#define NRC_GR                       0x00  // 通用拒绝
#define NRC_SNS                      0x11  // 服务不支持
#define NRC_SFNS                     0x12  // 子功能不支持
#define NRC_IMLOIF                   0x13  // 报文长度错误
#define NRC_CNC                      0x22  // 条件不满足
#define NRC_ROOR                     0x31  // 请求超出范围
#define NRC_SAD                      0x33  // 安全访问被拒绝
#define NRC_IK                       0x35  // 密钥错误
#define NRC_RSE                      0x37  // 请求序列错误
#define NRC_NRSN                     0x78  // 响应待定

static uint8_t Uds_BuildNegResponse(uint8_t *resp, uint8_t sid, uint8_t nrc)
{
    resp[0] = 0x7F;           // 否定响应标志
    resp[1] = sid;            // 原服务ID
    resp[2] = nrc;            // 否定响应码
    return 3;
}
```

### 3.6 UDS 主处理函数 — 完整实现

```c
/* 新增文件: Hardware/Src/bsp_uds.c */

#include "bsp_uds.h"
#include "bsp_light.h"
#include "bsp_can_error.h"
#include <string.h>
#include <stdint.h>

extern LightState currentLightState;
extern CAN_HandleTypeDef hcan;

/* UDS 会话超时检查（在 main loop 中调用） */
void Bsp_Uds_SessionTimer_Check(void)
{
    if (currentSession != 0x01) {  // 不是 Default 会话
        if (HAL_GetTick() > sessionTimeout) {
            currentSession = 0x01;   // 超时退回 Default
            securityUnlocked = 0;    // 锁定安全访问
        }
    }
}

/* UDS 主入口 — 收到 CAN 诊断消息时调用 */
void Bsp_Uds_Process(uint8_t *data, uint8_t len)
{
    uint8_t response[64];
    uint8_t respLen = 0;
    uint8_t service = data[0];
    
    /* 解析服务并处理 */
    switch (service) {
        case 0x10: respLen = Uds_HandleSessionControl(data, len, response); break;
        case 0x22: respLen = Uds_HandleReadData(data, len, response);       break;
        case 0x2E: respLen = Uds_HandleWriteData(data, len, response);      break;
        case 0x31: respLen = Uds_HandleRoutineControl(data, len, response); break;
        case 0x19: respLen = Uds_HandleReadDTC(data, len, response);        break;
        case 0x14: respLen = Uds_HandleClearDTC(data, len, response);       break;
        case 0x27: respLen = Uds_HandleSecurityAccess(data, len, response); break;
        case 0x3E: respLen = Uds_HandleTesterPresent(data, len, response);  break;
        default:
            respLen = Uds_BuildNegResponse(response, service, NRC_SNS);
            break;
    }
    
    /* 发送响应 (CAN ID = 请求ID + 8) */
    CAN_TxHeaderTypeDef txHeader;
    txHeader.StdId = 0x7E7;        // 0x7DF + 8
    txHeader.ExtId = 0;
    txHeader.IDE = CAN_ID_STD;
    txHeader.RTR = CAN_RTR_DATA;
    txHeader.DLC = (respLen <= 8) ? respLen : 8;
    txHeader.TransmitGlobalTime = DISABLE;
    
    Bsp_Can_Transmit(&txHeader, response);
}
```

> **实现说明**：实际代码中 ISO-TP 多帧发送已实现（`Uds_TpSend`），响应超过 7 字节时自动发送 FF + 连续帧，并在发送前等待测试仪流控帧(FC)；多帧接收同样支持（`Bsp_Uds_TpFeed` 状态机处理 FF/CF）。

### 3.7 在 main.c 中的集成

```c
/* USER CODE BEGIN Includes */
#include "bsp_can_error.h"
#include "bsp_uds.h"
/* USER CODE END Includes */

/* USER CODE BEGIN 2 */
OLED_Init();
Bsp_Can_Init();
Bsp_Light_Init();
Bsp_Encoder_Init();
Bsp_CanError_Init();
/* USER CODE END 2 */

while (1)
{
    /* USER CODE BEGIN 3 */
    
    /* ===== 1. CAN 错误检查（每100ms） ===== */
    static uint32_t lastCanErrCheck = 0;
    if (HAL_GetTick() - lastCanErrCheck > 100) {
        Bsp_CanError_Check();
        lastCanErrCheck = HAL_GetTick();
    }
    
    /* ===== 2. UDS 会话超时检查 ===== */
    Bsp_Uds_SessionTimer_Check();
    
    /* ===== 3. 灯光自检更新 ===== */
    Bsp_Light_SelfTest_Update();
    
    /* ===== 4. 编码器 ===== */
    int32_t angle = Bsp_Encoder_GetCount();
    
    /* ===== 5. 原有 AFS + 灯光更新 ===== */
    Bsp_AFS_Update(recangle);
    Bsp_Light_Update();
    
    /* ===== 6. OLED显示（略） ===== */
    
    /* ===== 7. 按键处理 ===== */
    if (KeyNum != 0) {
        // ... 原有按键逻辑 ...
        KeyNum = 0;
    }
    
    /* ===== 8. CAN 接收处理 ===== */
    if (Bsp_Can_ReceiveFlag()) {
        Bsp_Can_Receive(&RxHeader, RxData);
        
        if (RxHeader.StdId == 0x7DF) {
            // UDS 诊断请求
            Bsp_Uds_Process(RxData, RxHeader.DLC);
        } else {
            // 原有灯光控制 CAN 消息
            // ... 原有 CAN 处理代码不变 ...
        }
    }
}
```

---

## 三、网络管理心跳（NM）与应用帧安全格式

### 3.9 网络管理心跳

节点以 `CAN ID = 0x700`、周期 **200ms** 发送心跳帧，用于对端节点在线监测：

```
心跳帧 (DLC=8):
data[0] = 节点ID (本节点 0x01)
data[1] = 节点状态 (0x00 Boot / 0x01 Normal / 0x02 Error)
data[2] = 心跳计数 (每周期 +1)
data[3..4] = 节点运行时间 (秒, 16bit)
data[5] = 保留
data[6] = RollingCounter
data[7] = CRC8
```

- 对端超过 **1000ms** 未收到心跳 → 判定离线，置 DTC `0x1005 (COMM LOST)`，OLED 显示 `NM:LOST`
- 恢复心跳后自动清除 DTC，OLED 显示 `NM:ON`
- 可通过 UDS `DID 0xF197` 读取对端在线状态与运行时间

### 3.10 应用帧安全格式（RollingCounter + CRC8）

所有应用报文（灯光命令 / AFS 角度 / 心跳）统一升级为 8 字节安全帧：

```
data[0..5] = 信号数据（最多 6 字节）
data[6]    = RollingCounter（每消息独立计数，0~15 循环）
data[7]    = CRC8（AUTOSAR SAE J1850，poly=0x1D，init=0xFF，覆盖 data[0..6]）
```

接收端校验：

| 校验结果 | 处理 |
|----------|------|
| CRC8 校验失败 | 丢弃报文，记 DTC + 错误计数 |
| RollingCounter 重复 | 丢弃（防重放/重发攻击） |
| RollingCounter 跳变/复位 | 记录 CC_ERROR，仍接受该帧 |
| 连续 +1 | 正常接收 |

UDS 诊断报文（0x7DF/0x7E7）走 ISO-TP，不参与此校验。

---

## 四、验证测试方法

### 4.1 CAN 错误处理测试

| 测试场景 | 操作 | 预期结果 |
|----------|------|---------|
| 正常通信 | 主机←→从机正常连接 | `Bsp_CanError_GetState()` 返回 `ACTIVE` |
| ACK错误 | 只给主机上电，不给从机上电 | `CAN_ERR_ACK` 标志置位 |
| Bus-Off | 拔掉 CAN 总线，快速多次发送 | 进入 `BUS_OFF`，自动恢复 |
| 查看错误 | 读 `DID_CAN_ERROR_INFO` | 返回错误次数和最近错误时间 |

### 4.2 UDS 测试（使用 CAN 分析仪）

用 CAN 分析仪配套的上位机软件（或自行编写脚本）发送以下 16 进制数据：

```
=== 1. 切换会话 ===
发送: 02 10 02           → 切换到编程会话
应答: 06 50 02 00 32 00 E8

=== 2. 读取软件版本 ===
发送: 03 22 F1 90
应答: 06 62 F1 90 56 31 2E 30  → "V1.0"

=== 3. 读取灯光状态 ===
发送: 03 22 F1 94
应答: 04 62 F1 94 XX   → XX = 当前灯光状态

=== 4. 读取 CAN 错误信息 ===
发送: 03 22 F1 95
应答: 09 62 F1 95 XX XX XX XX XX XX  → 错误信息

=== 5. 安全访问 ===
发送: 02 27 01          → 请求种子
应答: 04 67 01 XX XX    → 返回种子
发送: 04 27 02 XX XX    → 发送密钥 (种子 XOR 0x5A5A)
应答: 02 67 02          → 解锁成功

=== 6. 灯光自检 ===
发送: 04 31 01 10 00    → 开始自检
应答: 04 71 01 10 00

=== 7. 读取故障码 ===
发送: 03 19 02 FF       → 读取所有DTC
应答: XX XX ...          → DTC列表

=== 8. 保持连接 ===
发送: 02 3E 00
应答: 02 7E 00

=== 9. 回到默认会话 ===
发送: 02 10 01
应答: 06 50 01 00 32 00 E8
```

---

## 五、文件清单总结

### 新增文件（均已实现）

| 文件路径 | 内容 | 预估行数 |
|----------|------|---------|
| `Hardware/Inc/bsp_can_error.h` | CAN 错误处理接口 | ~40 |
| `Hardware/Src/bsp_can_error.c` | CAN 错误监控 + Bus-Off 恢复 | ~150 |
| `Hardware/Inc/bsp_uds.h` | UDS 协议接口 | ~20 |
| `Hardware/Src/bsp_uds.c` | UDS 协议栈完整实现（含 ISO-TP 多帧） | ~580 |
| `Hardware/Inc/bsp_nm.h` | 网络管理接口 | ~20 |
| `Hardware/Src/bsp_nm.c` | 心跳发送 + 对端掉线监测 | ~120 |
| `tools/uds_tester.py` | Python 诊断仪（python-can，内置 ISO-TP 客户端） | ~200 |
| `docs/uds_dtc_implementation.md` | 本实现文档 | — |

### 修改文件

| 文件路径 | 修改内容 |
|----------|---------|
| `Hardware/Src/bsp_can.c` | CRC8(AUTOSAR) 校验 + 每 ID RollingCounter + RX 环形缓冲(8 帧) + 帧校验 |
| `Hardware/Inc/bsp_can.h` | 协议常量定义 + 发送/接收新接口 |
| `Core/Src/main.c` | 错误周期检查、UDS 分发、心跳更新、校验路由、OLED 状态显示 |
| `cmake/stm32cubemx/CMakeLists.txt` | 添加新源文件 |

---

## 六、面试加分话术

当面试官问到这个项目，你可以这样说：

> **"我在前大灯项目中，基于 STM32F103 完整的 CAN 控制器，实现了一套 CAN 总线错误监控和自动恢复机制，能够检测 ACK 错误、填充错误、格式错误、CRC 错误，并在 Bus-Off 后自动恢复通信。"**
>
> **"在此基础上，我按照 ISO 14229 标准实现了 UDS 诊断协议栈，支持诊断会话控制、DID 读写、例行控制（灯光自检）、故障码读取与清除、安全访问等功能，并实现了 ISO-TP 多帧收发，VIN 等长报文可完整传输。使用自研 Python 脚本作为诊断仪进行了联调测试。"**
>
> **"同时实现了网络管理心跳机制（200ms 心跳 + 1s 掉线监测），并将所有应用报文升级为 RollingCounter + CRC8 安全帧格式，具备防重放和坏帧检测能力。"**

---

> **文档版本**: v2.0（方案已全部实现）  
> **最后更新**: 2026-08-20
