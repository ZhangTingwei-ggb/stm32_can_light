# stm32_can_light — 车载前大灯 CAN 控制系统

基于 **STM32F103C8T6** 的前大灯控制系统，通过 **CAN 总线 (250kbps)** 实现灯光控制、AFS 弯道辅助照明、UDS 诊断、网络管理心跳与信号安全校验，是一套贴近车载电子量产要求的完整示例工程。

---

## 一、项目简介

本工程实现了一个典型的车载灯光控制器节点：既可作为**主机**（按键触发指令下发），也可作为**从机**（收到 CAN 报文后驱动 PWM 灯控），同一固件双角色运行。在基础灯控之上，补齐了车载嵌入式开发的三大核心能力：

- **CAN 通信可靠性**：RollingCounter + CRC8 安全帧、RX 环形缓冲、错误状态机与 Bus-Off 自动恢复
- **诊断协议**：ISO 14229 (UDS) 完整子集，含 ISO-TP 多帧收发，可被标准诊断仪读写
- **网络管理**：周期心跳 + 对端掉线监测，掉线自动置故障码

| 灯控 | CAN 可靠层 | 诊断 (UDS) | 网络管理 (NM) |
|------|-----------|-----------|--------------|
| 近光 / 远光 | RollingCounter | 会话控制 0x10 | 200ms 心跳帧 |
| 左/右转向流水灯 | CRC8 (AUTOSAR) | DID 读写 0x22/0x2E | 1s 掉线判定 |
| 危险警示灯 | RX 8 帧环形缓冲 | 安全访问 0x27 | DTC 0x1005 自动置/清 |
| AFS 弯道辅助 | 错误状态机 + Bus-Off 恢复 | 灯光自检 0x31 | OLED NM:ON/LOST |

---

## 二、硬件架构

| 外设 | 功能 | 引脚 |
|------|------|------|
| STM32F103C8T6 | Cortex-M3, 72MHz, 64KB Flash, 20KB RAM | — |
| CAN1 | CAN 总线通信 (250kbps) | PB8(RX), PB9(TX) |
| TIM2 | PWM 灯光输出（4 通道） | PA0~PA3 |
| TIM3 | 编码器输入（AFS 角度检测） | PA6, PA7 |
| EXTI | 按键中断（4 个） | PB12~PB15 |
| OLED | 状态显示 | SPI/I2C |
| GPIO | 板载指示 LED | PC13 |

### 按键映射

| 按键 | 功能 |
|------|------|
| Key1 (PB12) | 近光灯 → 远光灯 → 关闭，循环 |
| Key2 (PB13) | 左转向灯 开关 |
| Key3 (PB14) | 右转向灯 开关 |
| Key4 (PB15) | 双闪（危险警示灯） 开关 |

---

## 三、CAN 协议

### 配置参数

| 参数 | 值 |
|------|-----|
| 波特率 | 250 kbps |
| 时钟源 | APB1 (36MHz)，预分频 9 |
| BS1 / BS2 / SJW | 11 / 4 / 4 TQ |

### 报文 ID 分配

所有应用报文采用 **8 字节安全帧**：`data[0..5]=信号`、`data[6]=RollingCounter`、`data[7]=CRC8`。

| ID | 数据 | 功能 |
|----|------|------|
| 0x100 / 0x101 | {0x10 / 0x11, …, CNT, CRC} | 近光灯 开 / 关 |
| 0x200 / 0x201 | {0x20 / 0x21, …, CNT, CRC} | 远光灯 开 / 关 |
| 0x300 / 0x301 | {0x30 / 0x31, …, CNT, CRC} | 左转向 开 / 关 |
| 0x400 / 0x401 | {0x40 / 0x41, …, CNT, CRC} | 右转向 开 / 关 |
| 0x500 / 0x501 | {0x50 / 0x51, …, CNT, CRC} | 双闪 开 / 关 |
| 0x600 | {angle[0..3], …, CNT, CRC} | AFS 角度 (int32_t) |
| 0x700 | {ID, ST, Alive, UptimeHi, UptimeLo, …, CNT, CRC} | 网络管理心跳（200ms） |
| 0x7DF / 0x7E7 | ISO-TP 帧 | UDS 诊断 请求 / 响应 |

### 安全帧校验

```c
/* CRC8：AUTOSAR SAE J1850，poly=0x1D，init=0xFF，覆盖 data[0..6] */
crc = 0xFF;
for (i = 0; i < 7; i++) { crc ^= data[i]; for (j = 0; j < 8; j++)
    crc = (crc & 0x80) ? (crc << 1) ^ 0x1D : crc << 1; }
```

| 接收校验 | 处理 |
|----------|------|
| CRC8 失败 | 丢弃，计数并置 DTC |
| RollingCounter 重复 | 丢弃（防重放） |
| RollingCounter 跳变 | 记录 CC_ERROR，仍接受 |
| 连续 +1 | 正常接收 |

---

## 四、网络管理 (NM)

- 节点以 **200ms** 周期发送心跳帧 `0x700`，携带节点 ID、状态、心跳计数、运行时间
- 对端 **1000ms** 未收到心跳 → 判定离线：OLED 显示 `NM:LOST`，自动置 DTC `0x1005 (COMM LOST)`
- 心跳恢复后自动清除故障码，OLED 显示 `NM:ON`
- 可通过 UDS `DID 0xF197` 读取对端在线状态与运行时间

---

## 五、UDS 诊断 (ISO 14229)

| 服务 | 说明 |
|------|------|
| 0x10 会话控制 | Default / Programming / Extended，5s 超时回落 |
| 0x22/0x2E DID | 软硬件版本、VIN 码、灯光状态、CAN 错误信息、NM 状态 |
| 0x27 安全访问 | 种子/密钥 (XOR 0x5A5A)，3 次失败锁定 5s |
| 0x31 例行控制 | 灯光自检（各灯依次点亮 300ms） |
| 0x19/0x14 DTC | 故障码数量/列表/快照读取，全清 |
| 0x3E 保持连接 | 会话保持 |

- **ISO-TP 多帧收发**：响应超过 8 字节（如 20 字节 VIN）自动分包发送（FF + 连续帧），请求侧同样支持多帧重组

---

## 六、目录结构

```
Front_Light/
├── Core/                    CubeMX 生成代码 + 主循环 main.c
│   ├── Inc/  Src/
├── Hardware/                BSP 应用层
│   ├── Inc/  Src/
│   │   ├── bsp_can.c         CAN 收发 + CRC8 + RollingCounter + RX 环形缓冲
│   │   ├── bsp_can_error.c   CAN 错误监控 + Bus-Off 自动恢复
│   │   ├── bsp_uds.c         UDS 诊断协议栈（含 ISO-TP 多帧）
│   │   ├── bsp_nm.c          网络管理心跳 + 对端掉线监测
│   │   ├── bsp_key.c         按键中断处理（300ms 消抖）
│   │   ├── bsp_light.c       PWM 灯控 + 流水灯状态机 + AFS
│   │   ├── bsp_encoder.c     编码器角度读取
│   │   └── OLED.c            OLED 驱动
├── Drivers/                 CMSIS + STM32F1xx HAL
├── tools/
│   └── uds_tester.py        Python 诊断仪（python-can）
├── docs/
│   ├── project_documentation.md       项目设计文档
│   └── uds_dtc_implementation.md      UDS/DTC/NM 实现方案
├── cmake/                   构建配置（STM32CubeMX 生成）
├── CMakeLists.txt
└── Front_Light.ioc          CubeMX 工程
```

---

## 七、构建与烧录

```bash
# 环境：VS Code + CMake + Ninja + ARM GCC (gcc-arm-none-eabi)
cmake --preset Debug                 # 配置
cmake --build build/Debug            # 编译，产物 build/Debug/Front_Light.elf
```

- 可用 ST-Link + OpenOCD / CubeProgrammer 烧录 `Front_Light.elf`
- 当前资源占用：**Flash 42.7% / RAM 13.1%**，为后续 Bootloader 预留了空间

---

## 八、联调测试

### 1. 双板 + 按键

主机按键触发灯光指令 → 从机 PWM 输出 + PC13 LED 翻转确认。

### 2. UDS 诊断（USB-CAN 分析仪）

```bash
pip install python-can
python tools/uds_tester.py --interface pcan --channel PCAN_USBBUS1
```

交互菜单支持 15 项测试：

```
 1 读取软件版本 (22 F190)        9  读取DTC数量 (19 01 FF)
 3 读取VIN码 (22 F193)           10 读取DTC列表 (19 02 FF)
 5 读取CAN错误信息 (22 F195)     11 清除DTC (14 FF FF)
 6 读取网络管理状态 (22 F197)    12 切换编程会话 (10 02)
 7 安全访问 (27 01/02)           14 保持连接 (3E 00)
 8 灯光自检 (31 01 1000)         15 连续心跳监控
```

也可一次性发送或监控总线：

```bash
python tools/uds_tester.py --interface pcan --channel PCAN_USBBUS1 --send "02 10 02"
python tools/uds_tester.py --interface socketcan --channel can0 --monitor
```

### 3. 故障注入演示（面试亮点）

| 操作 | 预期现象 |
|------|---------|
| 只给单板上电，另一板断电 | OLED 显示 `NM:LOST`，DTC 0x1005 置位 |
| 拔掉总线后连续发送 | CAN 进入 Bus-Off，自动恢复后重新通信 |
| 恢复对端供电 | 心跳恢复，DTC 自动清除，OLED `NM:ON` |
| 读取 0xF195 | 返回总线状态 / 错误标志 / 错误计数 |

---

## 九、开发环境

| 项 | 值 |
|----|----|
| MCU | STM32F103C8T6 |
| IDE | STM32CubeMX 6.11 + VS Code |
| 工具链 | ARM GCC 14.3 (gcc-arm-none-eabi) + CMake/Ninja |
| HAL | STM32F1xx HAL Driver |
| 诊断工具 | python-can + USB-CAN 分析仪 (PCAN/CANalyst 等) |

---

## 十、面试要点速记

1. **CAN 波特率如何计算**：`波特率 = APB1 / (Prescaler × (BS1 + BS2 + SJW + 1))`，本例 36M / (9 × 16) = 250k
2. **Bus-Off 恢复流程**：Stop → 等待 ≥128 个总线空闲位 → Start → 重开接收中断
3. **安全帧为什么用 RollingCounter + CRC8**：防重放、防重发、检测数据错误，是 AUTOSAR 标准做法
4. **ISO-TP 为什么需要**：CAN 单帧最多 8 字节，长报文（如 VIN 20 字节）需 FF/CF/FC 分包
5. **NM 掉线判定**：连续超时未收到心跳 → 判定节点失效，触发故障存储与降级策略

详细设计见 [docs/project_documentation.md](docs/project_documentation.md) 和 [docs/uds_dtc_implementation.md](docs/uds_dtc_implementation.md)。