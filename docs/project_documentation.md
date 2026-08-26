# 车载前大灯控制系统 — 嵌入式项目文档

## 项目概述

基于 **STM32F103C8T6** 微控制器，通过 **CAN 总线**通信的前大灯控制系统，支持近光灯、远光灯、左右转向流水灯、危险警示灯以及 AFS 弯道辅助照明功能。

---

## 硬件架构

### MCU
- **型号**：STM32F103C8T6 (Cortex-M3, 72MHz, 64KB Flash, 20KB RAM)

### 外设资源分配

| 外设 | 功能 | 引脚 |
|------|------|------|
| TIM2 | PWM 灯光输出（4通道） | PA0~PA3 |
| TIM3 | 编码器输入（AFS角度检测） | PA6, PA7 |
| CAN1 | CAN 总线通信 | PB8(RX), PB9(TX) |
| EXTI | 按键中断 | PB12~PB15 |
| SPI/I2C | OLED 显示屏 | PB8, PB9等相关引脚 |
| GPIO | 板载指示灯 | PC13 |

### 按键映射

| 引脚 | 按键 | 功能 |
|------|------|------|
| PB12 | Key1 | 第1次→近光灯 / 第2次→远光灯 / 第3次→关闭，循环 |
| PB13 | Key2 | 左转向灯 开关切换 |
| PB14 | Key3 | 右转向灯 开关切换 |
| PB15 | Key4 | 双闪(危险警示灯) 开关切换 |

### PWM 灯控参数

| 灯光模式 | PWM 占空比 | 说明 |
|----------|-----------|------|
| 近光灯 | 1% (10/1000) | AFS 弯道辅助时动态调节 |
| 远光灯 | 100% (1000/1000) | 全亮 |
| 转向灯 | 流水效果 | 4步循环（0%→40%→100%→40%） |
| 关闭 | 0% (0/1000) | 所有通道归零 |

---

## 软件架构

```
┌──────────────────────────────────────────────────┐
│                   Main Loop                       │
├──────────────────────────────────────────────────┤
│  ┌──────────┐  ┌──────────┐  ┌────────────────┐  │
│  │按键处理模块│  │CAN通信模块│  │ 灯光控制模块   │  │
│  │ bsp_key  │  │ bsp_can  │  │ bsp_light     │  │
│  └──────────┘  └──────────┘  └────────────────┘  │
│  ┌──────────┐  ┌──────────┐  ┌────────────────┐  │
│  │ 编码器模块│  │ OLED显示 │  │ AFS弯道辅助    │  │
│  │bsp_encoder│  │  OLED    │  │ bsp_light_afs  │  │
│  └──────────┘  └──────────┘  └────────────────┘  │
├──────────────────────────────────────────────────┤
│                   BSP 层                          │
│          STM32 HAL + CubeMX 配置                  │
└──────────────────────────────────────────────────┘
```

### 项目文件清单

```
Core/
├── Inc/
│   ├── main.h
│   ├── stm32f1xx_hal_conf.h
│   ├── stm32f1xx_it.h
├── Src/
│   ├── main.c                   // 主循环与初始化
│   ├── stm32f1xx_hal_msp.c      // HAL底层初始化（GPIO复用配置等）
│   ├── stm32f1xx_it.c           // 中断服务函数
│   ├── syscalls.c / sysmem.c    // 系统调用
│   └── system_stm32f1xx.c       // 系统时钟配置

Hardware/
├── Inc/
│   ├── bsp_can.h                // CAN 接口 + 安全帧/校验常量
│   ├── bsp_can_error.h          // CAN 错误监控接口
│   ├── bsp_uds.h                // UDS 诊断协议接口
│   ├── bsp_nm.h                 // 网络管理心跳接口
│   ├── bsp_key.h                // 按键定义
│   ├── bsp_light.h              // 灯光控制接口
│   ├── bsp_encoder.h            // 编码器接口
│   ├── OLED.h / OLED_Font.h     // OLED 显示
│   └── Delay.h                  // 延时函数
├── Src/
│   ├── bsp_can.c                // CAN收发 + CRC8 + RollingCounter + RX环形缓冲
│   ├── bsp_can_error.c          // 错误监控 + Bus-Off 自动恢复
│   ├── bsp_uds.c                // UDS 协议栈（含 ISO-TP 多帧）
│   ├── bsp_nm.c                 // 心跳发送 + 对端掉线监测
│   ├── bsp_key.c                // 按键中断处理
│   ├── bsp_light.c              // 灯光PWM控制 + 流水灯
│   ├── bsp_encoder.c            // 编码器读取
│   ├── OLED.c                   // OLED驱动
│   └── Delay.c                  // 延时实现

tools/
└── uds_tester.py                // Python 诊断仪（python-can）

Drivers/
├── CMSIS/                       // ARM CMSIS 标准库
└── STM32F1xx_HAL_Driver/        // STM32 HAL 驱动库
```

---

## CAN 通信协议

### CAN 配置参数

| 参数 | 值 |
|------|-----|
| 波特率 | 250 kbps |
| 时钟源 | APB1 (36MHz) |
| 预分频器 | 9 |
| BS1 | 11 TQ |
| BS2 | 4 TQ |
| SJW | 4 TQ |
| 滤波器 | 接收所有 ID（Mask = 0x0000） |

### CAN 消息 ID 定义

所有应用报文采用 **8 字节安全帧**：`data[0..5]=信号`、`data[6]=RollingCounter`、`data[7]=CRC8(AUTOSAR)`。

| ID | 数据 | 功能 |
|----|------|------|
| 0x100 | {0x10,_,_,_,_,_,CNT,CRC} | 近光灯开启 |
| 0x101 | {0x11,_,_,_,_,_,CNT,CRC} | 近光灯关闭 |
| 0x200 | {0x20,_,_,_,_,_,CNT,CRC} | 远光灯开启 |
| 0x201 | {0x21,_,_,_,_,_,CNT,CRC} | 远光灯关闭 |
| 0x300 | {0x30,_,_,_,_,_,CNT,CRC} | 左转向灯开启 |
| 0x301 | {0x31,_,_,_,_,_,CNT,CRC} | 左转向灯关闭 |
| 0x400 | {0x40,_,_,_,_,_,CNT,CRC} | 右转向灯开启 |
| 0x401 | {0x41,_,_,_,_,_,CNT,CRC} | 右转向灯关闭 |
| 0x500 | {0x50,_,_,_,_,_,CNT,CRC} | 双闪(危险警示灯)开启 |
| 0x501 | {0x51,_,_,_,_,_,CNT,CRC} | 双闪(危险警示灯)关闭 |
| 0x600 | {angle[0..3],_,_,CNT,CRC} | AFS 角度数据（int32_t） |
| 0x700 | {ID,ST,Alive,UpTimeHi,UpTimeLo,_,CNT,CRC} | 网络管理心跳帧（200ms） |
| 0x7DF | ISO-TP 单/多帧 | UDS 诊断请求（Tester→ECU） |
| 0x7E7 | ISO-TP 单/多帧 | UDS 诊断响应（ECU→Tester） |

### CAN 通信流程

```
主机:                         从机:
  │                            │
  ├─ 按键触发 EXT中断 ──→ KeyNum flag 置位
  │                            │
  ├─ 主循环检测到 KeyNum
  ├─ 调用 Bsp_Light_*()       │
  ├─ CAN发送消息 ──────────────→ HAL_CAN_RxFifo0MsgPendingCallback
  │                            ├─ 读取 RxData
  │                            ├─ 解析 StdId
  │                            └─ 调用对应 Bsp_Light_*()
  │                            │
```

---

## 灯光控制逻辑

### 流水灯效果

转向灯和双闪采用 4 步流水循环：

```
Step 0: 所有通道关闭
Step 1: 内侧通道 40% 亮度
Step 2: 中间通道 100% 亮度
Step 3: 外侧通道 100% 亮度 / 内侧 40%
```

每步间隔 **150ms**，形成流水视觉效果。

### AFS 弯道辅助照明

当近光灯开启且编码器检测到方向盘转动时：

```
角度 > 0（左转）:
  CH1=10(1%), CH2=10(1%), CH3=角度映射值, CH4=角度映射值
  
角度 < 0（右转）:
  CH1=角度映射值, CH2=角度映射值, CH3=10(1%), CH4=10(1%)

角度 = 0（直行）:
  所有通道 = 10(1%)
```

角度映射范围：0~2160 对应 PWM 0~1000

---

## 关键实现细节

### 按键消抖

```c
static uint32_t lastKeyTick = 0;
static const uint32_t keyDebounceMs = 300;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    uint32_t tick = HAL_GetTick();
    if (tick - lastKeyTick < keyDebounceMs) return;  // 300ms 消抖
    lastKeyTick = tick;
    // ... 按键处理
}
```

### CAN 发送超时保护

```c
void Bsp_Can_Transmit(CAN_TxHeaderTypeDef* TxHeader, uint8_t* Data)
{
    uint32_t TxMailbox;
    if (HAL_CAN_AddTxMessage(&hcan, TxHeader, Data, &TxMailbox) != HAL_OK)
        return;
    
    uint32_t startTick = HAL_GetTick();
    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) != 3) {
        if (HAL_GetTick() - startTick > 100) break;  // 100ms 超时
    }
}
```

### PWM 初始化

```c
void Bsp_Light_Init(void)
{
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);  // PA0
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);  // PA1
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);  // PA2
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);  // PA3
}
```

---

## 测试方法

### 功能测试

| 测试项 | 操作 | 预期结果 |
|--------|------|---------|
| 近光灯 | 按 Key1 第1次 | OLED 显示 "LB"，PWM=1% |
| 远光灯 | 按 Key1 第2次 | OLED 显示 "HB"，PWM=100% |
| 关闭 | 按 Key1 第3次 | OLED 显示 "OFF"，PWM=0% |
| 左转向 | 按 Key2 | OLED 显示 "TL"，流水灯 |
| 右转向 | 按 Key3 | OLED 显示 "TR"，流水灯 |
| 双闪 | 按 Key4 | OLED 显示 "HAZ"，全流水 |

### CAN 通信测试

1. 按下任意按键，观察从机 PC13 LED 是否闪烁
2. 使用 CAN 分析仪（如 PCAN-USB、CANalyst-II）监听总线报文

### 编码器/AFS 测试

旋转编码器，观察 OLED 显示的 "L/R + 角度值"，同时近光灯 PWM 输出应随角度变化。

---

## 开发环境

| 工具 | 版本 |
|------|------|
| MCU | STM32F103C8T6 |
| IDE | STM32CubeMX + VS Code |
| 编译器 | ARM GCC (gcc-arm-none-eabi) |
| 构建系统 | CMake + Ninja |
| HAL 库 | STM32F1xx HAL Driver |

---

## 面试可能问题与回答

### Q1: CAN 总线错误如何处理？
- 监控 `HAL_CAN_GetError()` 返回值
- ACK 错误 → 检查总线终端电阻和连接
- CRC/填充/格式错误 → 检查总线干扰
- Bus-Off → 停止 CAN、等待 100ms、重新 Start

### Q2: 为什么用 250kbps 波特率？
- 车载 CAN 常用速率：125kbps(低速)、250kbps(中速)、500kbps(高速)
- 250kbps 在抗干扰和传输速率间取得平衡，适合灯光控制场景

### Q3: 流水灯如何实现的？
- 4 步状态机，每步设置不同通道的 PWM 占空比
- 使用 `HAL_GetTick()` 实现 150ms 非阻塞定时
- `Bsp_Light_Update()` 在 main loop 中周期调用

### Q4: 按键消抖为什么用 300ms？
- 机械按键抖动通常 < 20ms
- 300ms 为安全值，防止用户误触和快速连按
- 可根据实际按键特性调整

### Q5: CAN 滤波怎么配置的？
- 设置为 IDMASK 模式、32 位宽
- Mask 全部为 0 → 接收所有 ID
- 实际车载应用需按需求过滤特定 ID

---

## 扩展方向

1. ~~**UDS 诊断协议 (ISO 14229)**~~ ✅ 已实现 — 支持诊断会话控制、DID 读写、灯光自检、安全访问
2. ~~**DTC 故障码管理**~~ ✅ 已实现 — CAN 通信错误、心跳超时、灯光故障
3. **FreeRTOS** — 多任务调度，分离按键/CAN/灯光/显示任务
4. **CAN FD** — 更高带宽，传输更多诊断数据
5. **Bootloader** — 通过 CAN 远程升级固件（配合 UDS 0x34/0x36/0x37 编程会话）

---

> **文档版本**: v1.2  
> **适用平台**: STM32F103C8T6  
> **最后更新**: 2026-08-20
