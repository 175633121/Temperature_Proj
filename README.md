# STM32F103 测温站采集与 Modbus RTU 转发固件（Temperature_Proj）

> 烟台项目 / 聊城电网测温站板卡固件（PCB 编号 `ORZ_TX_V1.0`）
> MCU：STM32F103C6T6 · Keil μVision 5 + ARMCC · ST 标准外设库 V3.1.0（CMSIS Core CM3）

---

## 1. 项目概述

一块**测温站通信板**：板下挂若干测温模块（经 CD4051 八路模拟开关切换），MCU 通过 **USART1** 轮询/接收模块上报的温度与状态数据，一方面把温度实时转换成 **16 位串行 DAC 码输出**（后端经 V-I 转换送出 4–20 mA 工业电流），另一方面作为 **Modbus RTU 从机**挂 RS485 总线，应答上位机查询。

原始设计目标（源码头部注释）：

- 50 个测温站、共 300 个测温点，RS485 总线，9600 bps
- 主机命令格式 `0xAA 0x55 站地址`
- USART1 收模块数据 19200 bps；USART2 发数到上位机 9600 bps
- 输出量程 −30 ~ 100 ℃，**故障输出电流 3.75 mA**

最后修改时间：2013-08-01（源码注释），状态标注"测试中"。

---

## 2. 工程与构建信息

| 项目 | 值 |
|---|---|
| 工程文件 | `RXHK_Project.uvprojx`（含 `RXHK_Project.uvoptx`） |
| Target | `Target 1` |
| 器件 | `STM32F103C6`（Cortex-M3，Flash 32 KB @0x08000000–0x08007FFF，SRAM 10 KB @0x20000000–0x200027FF） |
| 输出名 | `Rxhk_WWDG_Project`（`output/*.axf`、`.hex`） |
| 编译器 | ARMCC（Keil μVision 5.36 工作区 `G:\keilv5.36_workspace`） |
| 库版本 | STM32F10x_StdPeriph_Driver + CMSIS Core CM3 V3.1.0 |
| 调试/下载 | J-Link SWD（`JLinkSettings.ini`、`DebugConfig/Target_1_STM32F103C6_1.0.0.dbgconf`） |

### 2.1 参与编译的源文件（Keil 工程实际引用）

```
Source/main.c                                      应用主程序（含全部业务逻辑）
Source/drv_wwdg.c / .h                             窗口看门狗驱动
Source/CMSIS/Core/CM3/core_cm3.c                  CMSIS 内核
Source/CMSIS/Core/CM3/system_stm32f10x.c          系统时钟初始化
Source/CMSIS/Core/CM3/startup/arm/startup_stm32f10x_hd.s   启动文件
Source/STM32F10x_StdPeriph_Driver/src/misc.c      NVIC 
Source/STM32F10x_StdPeriph_Driver/src/stm32f10x_can.c
Source/STM32F10x_StdPeriph_Driver/src/stm32f10x_gpio.c
Source/STM32F10x_StdPeriph_Driver/src/stm32f10x_rcc.c
Source/STM32F10x_StdPeriph_Driver/src/stm32f10x_tim.c
Source/STM32F10x_StdPeriph_Driver/src/stm32f10x_usart.c
Source/STM32F10x_StdPeriph_Driver/src/stm32f10x_wwdg.c
```

### 2.2 编译体积（`Rxhk_WWDG_Project.map`）

| 指标 | 大小 |
|---|---|
| Total RO Size（Code + RO Data） | **10 040 B（9.80 KB）** |
| Total RW Size（RW + ZI） | **2 744 B（2.68 KB）** |
| Total ROM Size | **10 356 B（10.11 KB）** |
| Heap / Stack | 各 1 024 B |

Flash 占用约 31%，RAM 占用约 27%，资源充裕。

### 2.3 构建步骤

1. Keil μVision 打开 `RXHK_Project.uvprojx`；
2. 选中 `Target 1` → **Rebuild**；
3. 产物位于 `output/Rxhk_WWDG_Project.axf` 与 `.hex`；
4. J-Link SWD 下载，或直接用 `output/Rxhk_WWDG_Project.hex` 烧录。

> 注意：仓库已用 `.gitignore` 排除 `output/`、`Lis/`、`*.uvguix.*`、`JLinkLog.txt` 等构建产物与个人工作区文件。

---

## 3. 硬件与引脚定义

| 引脚 | 方向 | 功能 | 代码位置 |
|---|---|---|---|
| PA9 / PA10 | AF_PP / 浮空输入 | USART1 TX / RX，接测温模块（19200 bps，**实际只用 RX**） | `GPIO_Configuration()` |
| PA2 / PA3 | AF_PP / 浮空输入 | USART2 TX / RX，接 MAX1483（RS485，9600 bps） | 同上 |
| **PA4** | 推挽输出 | **DATA**，串行 DAC 数据线 | `DATAH/DATAL` |
| **PA5** | 推挽输出 | **CLK**，串行 DAC 时钟线 | `CLKH/CLKL` |
| **PA6** | 推挽输出 | **LAT**，串行 DAC 锁存线 | `LATH/LATL` |
| **PB12** | 推挽输出 | RS485 收发方向控制（DE/RE），`1`=发送 `0`=接收 | `send485 / rec485` |
| **PB13** | 推挽输出 | **状态/故障指示灯**，低电平点亮 | `STATUSON / STATUSOFF` |
| PB14 / PB15 | 推挽输出 | 预留（源码注释标为 CD4051 地址选择线） | — |
| PB0 / PB1、PA0 / PA1、PA7、PB10 / PB11 | 浮空输入 | 预留（拨码开关 / 模块状态），当前代码未使用 | — |
| PC13 / PC14 / PC15 | 推挽输出 | 预留，当前代码未使用 | — |

板上器件：CD4051（八路模拟开关）、6N137（高速光耦）、B0505（隔离电源）、MAX1483（RS485 收发器）。

### 3.1 三线串行 DAC 时序（`dianliu()`）

```
LAT = 0
for i = 0..15:            // MSB first
    DATA = (code >> (15-i)) & 1
    CLK  = 0 → 1          // 上升沿移入
LAT = 0 → 1               // 上升沿锁存输出
```

---

## 4. 时钟与关键外设配置

### 4.1 系统时钟

```
HSE = 10 MHz（代码注释；uvprojx 中 Cpu 声明 CLOCK(8000000)，两者不一致，见 §8）
PLL : HSE/2 × 2  → SYSCLK = 10 MHz
HCLK = SYSCLK/1 = 10 MHz
PCLK2 = HCLK/1 = 10 MHz，PCLK1 = HCLK/1 = 10 MHz
```

外设时钟：GPIOA/B/C、AFIO、USART1（APB2）；USART2、TIM2（APB1）。

### 4.2 TIM2 — 故障监测节拍

```
Prescaler = 36000-1 → 计数频率 10 MHz / 36000 ≈ 277.78 Hz
Period    = 61      → 溢出周期 (61+1) / 277.78 ≈ 223 ms
```

### 4.3 窗口看门狗 WWDG

```c
WWDG_Init(0x7F, 0x5F, WWDG_Prescaler_8);
```

- `F_wwdg = PCLK1 / (4096 × 8) = 10 MHz / 32768 ≈ 305 Hz`
- 超时时间 `(0x7F − 0x3F + 1) / 305 ≈ 213 ms`
- 窗口值 `0x5F`：计数器值高于 0x5F 时写计数器会立刻复位（防止过早喂狗）
- **喂狗在 `WWDG_IRQHandler()` 中完成**（提前唤醒中断 EWI，计数器到 0x40 时触发），主循环不喂狗

### 4.4 NVIC

TIM2、USART1、USART2 均为抢占 0 / 子优先级 0；WWDG 为抢占 2 / 子优先级 3。**工程未调用 `NVIC_PriorityGroupConfig()`**，使用复位默认分组（见 §8）。

---

## 5. 通信协议

### 5.1 USART1 — 测温模块 → MCU（19200 bps，8N1）

采用 **8 字节滑动窗口**解析（`USART1_IRQHandler()`），不依赖帧头同步，靠特征字节组合判定有效帧：

```c
buffer[7] = USART1->DR;                      // 移位进窗口
if ((buffer[3] > 8) && (buffer[3] < 0x13) &&   // 温度高字节粗筛：0x09~0x12
    (buffer[6] == 0x32) &&                     // '2'
    (buffer[7] == 0x30) &&                     // '0'
    (buffer[4] > 140))                         // PD 数据有效性粗判
{
    sentdata[0] = buffer[2];   // TL 温度低 8 位
    sentdata[1] = buffer[3];   // TH 温度高 8 位
    sentdata[2] = buffer[4];   // PD
    sentdata[3] = buffer[5];   // LED
    gzjc = 0;                  // 收到有效帧，清故障计数
}
```

判定条件解读：温度原始值 `T_raw = TH<<8 | TL`，单位 0.1 K。`TH ∈ [0x09, 0x12]` 即 `T_raw ∈ [2304, 4863]`，换算后约 **−42.8 ℃ ~ +213 ℃**，与该板 −30 ~ 100 ℃ 的量程匹配，因此 `buffer[3]` 的判定实质是**温度值合理性粗筛**，而非点地址。`buffer[6]==0x32('2')` 与 `buffer[7]==0x30('0')` 是模块帧尾特征字节。

### 5.2 USART2 — Modbus RTU 从机（RS485，9600 bps，8N1）

**从机地址** `Id = 0x01`。上位机请求（8 字节）：

| 字节 | 值 | 含义 |
|---|---|---|
| 0 | `0x01` | 设备地址 |
| 1 | `0x04` | 功能码：读输入寄存器 |
| 2–3 | `0x00 0x05` | 起始地址 0x0005 |
| 4–5 | `0x00 0x00` | 寄存器数量 |
| 6–7 | CRC_L / CRC_H | CRC16（低字节在前） |

从机应答（9 字节，`sendmodbus()`）：

| 字节 | 值 | 含义 |
|---|---|---|
| 0 | `0x01` | 设备地址 |
| 1 | `0x04` | 功能码 |
| 2 | `0x04` | 数据域字节数 |
| 3 | `TL` | 温度低 8 位 |
| 4 | `TH` | 温度高 8 位 |
| 5 | `PD` | 光功率/放电量参数 |
| 6 | `LED` | LED 状态 |
| 7–8 | CRC_L / CRC_H | CRC16 |

**CRC16-Modbus**：查表法（`CRCHi[]/CRCLo[]`），多项式 `0xA001`，初值 `0xFFFF`，结果 `(CRCLo | CRCHi<<8)`，先发低字节。

**485 方向控制**：发送前 `send485`（PB12=1），逐字节等 `USART_SR` bit6（TC 发送完成），发完 `rec485`（PB12=0）切回接收；发送期间关闭 USART2 接收中断，发完再打开。

---

## 6. 软件流程

### 6.1 主流程

```
main()
 └ Init_All_Periph()             // RCC → GPIO → NVIC → USART1 → USART2 → TIM2
 └ Delay(100)
 └ dianliu(dlms)                 // dlms=0，上电默认输出码 0x3AE0(15040)
 └ STATUSOFF                     // 熄灭故障灯
 └ 清空 sentdata[] / buffer[]
 └ 使能 USART1、USART2 接收中断
 └ Delay(1500)
 └ WWDG_Init(0x7F, 0x5F, /8)     // 启动窗口看门狗
 └ while(1) { dianliu(dlms); rec485; }   // 持续刷新电流输出并保持接收
```

### 6.2 电流输出模式 `dianliu(m)`

| 模式 | 触发条件 | 输出码 | 说明 |
|---|---|---|---|
| `m = 0` | 上电初始化（`dlms` 初值 0） | 固定 `0x3AE0`（15040），连发 2 次 | 上电默认输出 |
| `m = 1` | 正常（`dlms = 1`） | 温度换算结果（见下） | 每次主循环刷新 |
| `m = 2` | 故障（`dlms = 2`，约 3.2 s 无有效数据） | 固定 `0x3C4B`（15435） | 注释标称 3.75 mA |

**温度 → DAC 码换算**（`m = 1`）：

```c
tem  = sentdata[1] * 256 + sentdata[0];   // 原始温度，单位 0.1 K
tem -= 2732;                              // 转 0.1 ℃（减 273.2 K）
tem3 = 50 * tem + 19 * tem / 46 + 15124;  // ≈ 50.413 × tem + 15124
tem  = tem3 + 0xB7;                       // +183 偏置
if (tem > 65535) tem = 65535;             // 上限钳位
```

反推量程映射（按代码推导，实际电流值以后端 V-I 电路标定为准）：

| 温度 | DAC 码 | 备注 |
|---|---|---|
| −30 ℃ | ≈ 183 | 对应量程下限（4 mA） |
| 0 ℃ | ≈ 15 307 | |
| 100 ℃ | ≈ 65 537 → 钳位 65 535 | 设计上限略低于 100 ℃ |

### 6.3 故障监测（TIM2 中断，约 223 ms）

```
gzjc++                        // 每 223 ms 加 1
if (gzjc > 15)                // ≈ 3.2 s 未收到有效模块数据
{
    dlms = 2;                 // 切故障电流输出
    STATUSON;                 // 点亮故障灯 PB13
    gzjc = 16;                // 防溢出
}
else { STATUSOFF; dlms = 1; } // 正常模式
```

通信中断恢复后，`USART1_IRQHandler()` 收到有效帧会把 `gzjc` 清零，自动回到正常模式。

### 6.4 看门狗

`WWDG_IRQHandler()` 中重装载计数器并清标志 —— 只要中断系统不被长期阻塞，系统就不会复位；一旦主循环跑飞或中断被长时间关闭，WWDG 触发硬件复位。

---

## 7. 目录结构

```
Temperature_Proj/
├── README.md                       ← 本文档
├── .gitignore
├── RXHK_Project.uvprojx            Keil 工程（Target 1 / STM32F103C6）
├── RXHK_Project.uvoptx             工程选项
├── JLinkSettings.ini               J-Link 配置
├── DebugConfig/                    调试配置
├── Source/
│   ├── main.c                      ★ 应用主程序（783 行）
│   ├── drv_wwdg.c / .h             ★ 窗口看门狗驱动
│   ├── 485.c(304) 等历史版本       见下说明
│   ├── App/                        485.c、canId.c、stm32f10x_it.c/.h、stm32f10x_conf.h
│   ├── CAN.c / canmaster.c / canslaver.c    CAN 总线方案版本
│   ├── 182.c 183.c 184.c 19.c 191.c         34 字节帧版本（烟台项目）
│   ├── CMSIS/Core/CM3/             CMSIS 内核 + 启动文件 + system_stm32f10x
│   └── STM32F10x_StdPeriph_Driver/ ST 标准外设库（inc / src）
├── output/                         构建产物（已 gitignore）
└── Lis/                            汇编/映射 listing（已 gitignore）
```

### 7.1 未加入 Keil 工程的历史/备选版本

这些文件物理存在但 **未被 `RXHK_Project.uvprojx` 引用**，属于同一硬件平台的其他通信方案或迭代版本，保留供参考：

| 文件 | 方案 | 要点 |
|---|---|---|
| `Source/App/485.c`（2011-10） | 聊城电网 RS485 版 | 使用 USART3（PB10/PB11），帧头 `0xFF 0xAA + 站地址`，PB0/PB1 控制模块复位 |
| `Source/CAN.c`（2012-03） | 聊城 CAN 总线版 | USART1 采集 + CAN 上传，PB10/PB11 作状态灯 |
| `Source/canmaster.c` | CAN 主机 | 站 ID 0x4001–0x4006，DMA 配置 |
| `Source/canslaver.c` | CAN 从机 | PA4–PA7 / PB0 / PB1 控制模块复位 |
| `Source/App/canId.c` | CAN 完整版 | 11 位标准帧，10 kbps（`Prescaler=100`，BS1=5tq/BS2=6tq），主机 ID `0x05`，测温站 ID `0x0A–0x25`（28 站 / 79 点），拨码开关设置站地址，主机发 8 字节、从机回 6 字节（站地址 / 点地址 / TL / TH / LED / PD） |
| `Source/182.c 183.c 184.c 19.c 191.c` | 烟台项目 34 字节帧版 | `0xAA 0x55 站地址 点地址1 TL TH LED PD … 点地址6 … CRC_L CRC_H`，含 CRC8 余式表 |
| `Source/App/stm32f10x_it.c/.h`、`stm32f10x_conf.h` | ST 官方模板 | 中断向量模板，未被引用 |

---

## 8. 代码分析与改进建议（按优先级）

### 高

1. **中断内长延时**：`USART2_IRQHandler()` 在 CRC 校验通过后调用 `Delay(20)`（内层 2000 次空循环，实测数十毫秒）再发送。中断里阻塞会同时影响 USART1 采样与 TIM2 节拍，极端情况下触发 WWDG 复位。
   → 建议：中断里只置标志位，发送动作移到主循环。
2. **共享数据未保护**：`sentdata[]`、`gzjc`、`dlms` 在中断与主循环间共享，均未声明 `volatile`，`dianliu()` 读取 `sentdata[0]/[1]` 组成 16 位温度时可能被 USART1 中断打断，导致高低字节来自不同帧（数据撕裂）。
   → 建议：加 `volatile`；读取时短暂关闭 USART1 中断再拷贝快照。
3. **器件与启动文件/库宏不匹配**：目标芯片 `STM32F103C6` 属 **Low-density**（32 KB Flash / 10 KB SRAM），但工程使用 `startup_stm32f10x_hd.s`，且 `uvprojx` 的 `Define` 为空 —— `stm32f10x.h` 在未定义 `STM32F10X_LD/MD/HD` 时会默认回退到 **HD**，外设寄存器映射与实际芯片不符；F103C6 也不具备 CAN 等外设（而 `stm32f10x_can.c` 被编译进工程）。
   → 建议：改用 `startup_stm32f10x_ld.s`，并在工程 `Define` 中显式加入 `STM32F10X_LD`；移除未使用的 `stm32f10x_can.c`。
4. **系统时钟两处声明不一致**：源码按 HSE = 10 MHz 配置 PLL（`HSE/2 × 2`），而 `uvprojx` 的 `Cpu` 字段写 `CLOCK(8000000)`。若实际晶振是 8 MHz，则 SYSCLK = 8 MHz，TIM2 节拍变为约 279 ms，波特率也会偏 25%（9600 → 实际 12000）。
   → 建议：核对晶振与 `system_stm32f10x.c` 中 `HSE_VALUE`，保持一致。

### 中

5. **中断优先级分组未显式配置**：工程未调用 `NVIC_PriorityGroupConfig()`，WWDG 设置的"抢占 2 / 子优先级 3"与默认的分组位宽可能不匹配，实际生效优先级与预期不同。
   → 建议：在 `NVIC_Configuration()` 开头显式 `NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2)`。
6. **Modbus 帧解析缺少帧间隔超时**：`reccount` 累加到 8 后靠窗口移位自同步，但一旦总线上出现连续噪声，`reccount` 会一直停在 8 并持续做 CRC 运算；没有"3.5 字符静默"复位机制。
   → 建议：用 TIM 或字符间隔超时重置 `reccount`。
7. **PB13 功能冲突**：源码注释把 PB13/14/15 标为 CD4051 地址选择线，但代码里 PB13 被 `STATUSON/STATUSOFF` 用作故障指示灯。实际 CD4051 地址线在本版本中并未驱动。
   → 建议：确认硬件连接后修正注释或改引脚。
8. **`dianliu()` 中 `LATL; LATH;` 紧跟且中间无延时**，锁存脉冲宽度仅一两个指令周期；部分 DAC 要求最小锁存脉宽。
   → 建议：中间插入微秒级延时（`delayus()`）。
9. **CRC 类型/变量类型**：`crc16()` 用 `char i` 作循环变量、`int` 作累加器，长度参数超过 127 会溢出；当前调用长度 ≤ 7 无风险，但接口不健壮。

### 低 / 工程规范

10. **源码为 GBK 编码**，注释在 UTF-8 工具链和 GitHub 上显示为乱码。建议批量转换为 UTF-8（转换不影响编译）。
11. **大量注释掉的代码**（`crc_8`、旧 `USART2_IRQHandler`、CRC 表等），建议清理或移到 `docs/` 归档。
12. **个人工作区文件**（`*.uvguix.*`、`JLinkLog.txt`）已在本次提交前用 `.gitignore` 排除，避免把本机用户名（15810 / Rxhk / zjy90）带进仓库。
13. **未使用的引脚初始化**（PC13/14/15、PA0/PA1、PB10/11 等）建议清理，或补注释说明预留用途。

---

## 9. 快速验证清单（上电后）

1. RS485 总线 9600 bps 8N1，向地址 `0x01` 发送 `01 04 00 05 00 00 0B E0`；
2. 应收到 9 字节应答 `01 04 04 TL TH PD LED CRC_L CRC_H`；
3. 断开测温模块数据超过约 3.2 s → PB13 故障灯点亮、DAC 输出切到故障码 `0x3C4B`；
4. 恢复模块数据 → 故障灯熄灭，DAC 输出回到温度对应码值；
5. 长时间运行不应出现复位（WWDG 由 EWI 中断喂狗）。

---

## 10. 许可与版权

- 应用层代码（`main.c`、`drv_wwdg.c/h` 及各历史版本）为项目自有代码。
- `Source/CMSIS/` 与 `Source/STM32F10x_StdPeriph_Driver/` 为 STMicroelectronics 提供的 CMSIS / 标准外设库，遵循其自带许可（`Source/CMSIS/License.doc`）。

---

*README 由代码分析自动生成（2026-10-18）。所有数值均从源码与 `Rxhk_WWDG_Project.map` 推导，实际电气量（电流 mA 值）需结合后端 V-I 转换电路标定确认。*
