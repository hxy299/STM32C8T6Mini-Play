# STM32C8T6Mini-Play

基于 **STM32F103C8T6** 的桌面宠物下位机（运动控制）固件。

上位机是跑 openVela / NuttX 的 **Radxa Cubie A7Z**（Allwinner A733），
两侧通过 UART 二进制帧协议通信；本仓库是 **只负责执行** 的下位机部分。

---

## 目录

- [1. 功能概览](#1-功能概览)
- [2. 硬件清单](#2-硬件清单)
- [3. 引脚分配](#3-引脚分配)
- [4. 目录结构](#4-目录结构)
- [5. 编译](#5-编译)
- [6. 通信协议](#6-通信协议)
- [7. 上位机测试工具](#7-上位机测试工具)
- [8. 验证与测试](#8-验证与测试)
- [9. 进度](#9-进度)
- [10. 已知限制](#10-已知限制)
- [11. 踩坑记录](#11-踩坑记录)

---

## 1. 功能概览

| 模块 | 内容 | 状态 |
|---|---|---|
| 时基 | TIM3 1 ms 调度（SysTick 留给 `Delay_us/ms`） | ✅ |
| 看门狗 | IWDG 约 1.6 s | ✅ |
| 链路 | USART1 115200 二进制帧 + CRC16 + ACK/NAK | ✅ |
| 心跳 | 1000 ms 无帧即置故障并停止运动 | ✅ |
| 配置 | Flash page 63 持久化，含 CRC 与版本号 | ✅ |
| 舵机 | PA0 / TIM2_CH1，50 Hz，±45° 缓动 | ✅ |
| 步进 ×2 | 28BYJ-48 + ULN2003，履带差速 | ✅ |
| **TOF ×2** | **VL53L0X：底部前测桌沿 + 前脸测障碍（代码支持 4 颗，当前装 2 颗）** | **✅ 代码完成** |
| **蓝牙遥控** | **USART3 PB10/PB11，A7Z 不在场时可独立控制** | **✅ 代码完成** |
| **TTS 转发** | **USART2 PA2/PA3，9600 8N1，TW-TTS 帧中断发送** | **✅ 代码完成** |

**这是一个"执行器从机"**：不自己做决策，只接受上位机的 `MOVE` / `SERVO` 等命令，
并在超时、过热、故障时自主进入安全状态。

---

## 2. 硬件清单

| 项目 | 型号 | 说明 |
|---|---|---|
| MCU | STM32F103C8T6 | LQFP48，Cortex-M3 @72 MHz，64 KB Flash / 20 KB RAM |
| 下载器 | ST-Link V2 | SWD：PA13 / PA14 |
| 电机 | 2 × 28BYJ-48 | 5 线单极步进，内置 1:64 减速箱 |
| 电机驱动 | 2 × ULN2003 模块 | 8 路 IN 各加 10 kΩ 下拉 |
| 电机电源 | **5 V**，≥2 A | 12V→5V 降压模块 |
| 舵机 | SG90 / MG90S | 5 V，PA0 |
| 测距 | 4 × VL53L0X | I²C，XSHUT 独立控制（待接入） |
| 串口 | USB-TTL | 与上位机 A7Z 通信 |

> **电机必须用 5 V。** 28BYJ-48 线圈额定 5 V，12 V 直驱时单相功率是额定的 5.8 倍，
> ULN2003 温升可达 74~95 °C。

---

## 3. 引脚分配

### 步进（履带差速）

| 侧 | 相 A | 相 B | 相 C | 相 D | 时基 |
|---|---|---|---|---|---|
| **左** | PB1 | PB3 | PB4 | PB8 | TIM4 |
| **右** | PA11 | PA12 | PA15 | PB0 | TIM1 |

> PA15 / PB3 / PB4 是 JTAG 引脚，`Stepper_Init()` 中已用
> `GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE)` 释放，
> PA13/PA14 的 SWD 保留。

### 其他

| 功能 | 引脚 | 备注 |
|---|---|---|
| 舵机 | PA0 | TIM2_CH1，50 Hz |
| 上位机 UART | PA9 / PA10 | USART1，115200 8N1 |
| TTS 模块 | PA2 / PA3 | USART2，**9600** 8N1，中断发送 |
| **蓝牙遥控** | **PB10 / PB11** | USART3，115200（模块需先改波特率） |
| I²C1（TOF） | **PB6 / PB7** | 100 kHz，多颗共用一条总线 |
| TOF0 XSHUT（底部前） | **PA4** | 10 kΩ 下拉，**当前已装** |
| TOF3 XSHUT（前脸） | **PA8** | 10 kΩ 下拉，**当前已装** |
| TOF1 / TOF2 XSHUT | PA5 / PB5 | 预留（底部左/右，**当前未装**） |
| 按键 | PA1 | 上拉输入 |
| 心跳 LED | PC13 | 低电平点亮 |
| SWD | PA13 / PA14 | 调试下载 |

### 定时器分配

| 定时器 | 用途 |
|---|---|
| TIM2 | 舵机 50 Hz |
| **TIM3** | **1 ms 调度时基** |
| TIM4 | 左步进 |
| TIM1_UP | 右步进 |
| SysTick | 仅供 `Delay_us/ms`（会被重写并关闭） |

---

## 4. 目录结构

```
.
├── Project.uvprojx        Keil MDK5 工程
├── Project.uvoptx         Keil 调试配置
├── User/                  应用层
│   ├── app_config.h       ★ 所有可调参数集中在这里
│   ├── main.c             初始化顺序 + 主循环
│   └── stm32f10x_it.c     中断向量
├── BSP/                   板级支持：NVIC / LED / IWDG / 1ms 时基
├── System/                Delay（SysTick 微秒/毫秒延时）
├── Hardware/
│   ├── serial.c           USART1(A7Z) + USART3(蓝牙) 双端口环形缓冲
│   ├── servo.c            舵机 PWM + 缓动
│   ├── servo_math.h       纯函数（可在 PC 上单测）
│   ├── stepper.c          步进驱动 + 梯形加减速 + 热保护
│   ├── stepper.h
│   ├── stepper_math.h     纯函数：相序 / Hz→ARR / 减速步数
│   ├── i2c1.c             I²C1 主机（阻塞 + 超时 + 总线恢复）
│   ├── tof.c              VL53L0X ×4（XSHUT 逐颗改址 / 非阻塞轮询）
│   ├── tof_math.h         纯函数：桌沿 / 障碍判定与回滞
│   ├── tts.c              ★ USART2 TW-TTS 转发（中断发送）
│   ├── tts.h
│   └── tts_math.h         纯函数：环形缓冲 / 帧长 / 原始帧识别
├── Protocol/
│   ├── link_proto.c       组帧 / 拆帧状态机 + CRC16（可实例化）
│   └── link_proto.h
├── Service/
│   ├── config.c           Flash 配置持久化
│   ├── cmd_router.h       纯函数：双命令源仲裁与优先级分类
│   ├── tof_service.c      TOF 轮询调度 / 故障联动 / 自动回退
│   └── link_service.c     命令分发 / 心跳 / 故障上报
├── Start/                 CMSIS + startup + 标准外设库入口
├── Library/               ST 标准外设库 V3.5.0
├── docs/                  各阶段验收说明
│   ├── M0-M2.5_验收说明.md
│   ├── M3_舵机验收说明.md
│   ├── M4-M5_步进验收说明.md
│   ├── M6-M7_TOF验收说明.md
│   ├── M9_TTS转发验收说明.md
│   ├── TOF两传感器配置说明.md
│   └── 蓝牙遥控与双源仲裁.md
└── tools/
    ├── pc_link_test.py    ★ PC 端交互测试工具
    ├── selftest_pc_tool.py
    ├── check_source_sanity.py   源码卫生（转义残留 / BOM / 换行）
    ├── run_host_tests.sh        ★ 一条命令跑全部主机单测
    ├── check_firmware_syntax.sh ★ Keil 之前先做语法筛查
    ├── stub/stm32f10x.h         最小 SPL 桩（给上面那个用）
    └── hosttest/          纯 C 主机单元测试（在 PC 上跑）
```

### 设计约定

- **纯函数与硬件分离**：`*_math.h` 只做算术，不碰寄存器，因此能在 PC 上用 gcc 单测
- **参数全部集中在 `User/app_config.h`**，改行为不用翻源文件
- **`app_config.h` 是唯一配置入口**，且被主机测试用桩文件替换，保证可测

---

## 5. 编译

### 环境

| 项目 | 版本 |
|---|---|
| IDE | Keil MDK5 |
| 编译器 | **ARMCC V5.06 update 5**（AC5，不是 AC6） |
| 器件包 | Keil.STM32F1xx_DFP |
| C 标准 | C99（`<uC99>1</uC99>`） |
| 标准外设库 | V3.5.0（已随仓库提供） |

### 预定义宏（`Project.uvprojx` 里已配好）

```
USE_STDPERIPH_DRIVER, STM32F10X_MD
```

### 头文件搜索路径

```
.\Start;.\User;.\Library;.\System;.\BSP;.\Hardware;.\Protocol;.\Service
```

### 步骤

1. 打开 `Project.uvprojx`
2. 确认 Target 是 `STM32F103C8`
3. `Rebuild`（**必须 Rebuild，不能只 Build**——改了 `app_config.h` 之后增量编译可能失效）
4. `Load` 通过 ST-Link 下载

> 下载后串口（USART1，115200）会打印启动横幅。
> 若看到 `mode=FULL-4` 和 `thermal: run<=60000ms`，说明配置正确。

---

## 6. 通信协议

### 帧格式

```
偏移   长度   字段
 0      1     SOF = 0xA5
 1      1     TYPE       0x01=CMD  0x02=ACK  0x03=EVT
 2      2     SEQ        小端
 4      1     OP
 5      1     LEN        载荷长度 0~64
 6      LEN   PAYLOAD
 6+LEN   2     CRC16      小端，覆盖 SOF..PAYLOAD
```

- CRC：**CRC16-CCITT-FALSE**，多项式 `0x1021`，初值 `0xFFFF`，不反转
- 校验向量：`"123456789"` → `0x29B1`
- 拆帧：逐字节状态机，靠 SOF + CRC 重同步，坏帧不阻塞后续
- 半帧保护：字节间隔 > 100 ms 则丢弃半帧

### 操作码

| OP | 名称 | 方向 | 说明 |
|---|---|---|---|
| 0x01 | MOVE | CMD | `dir(1) + speed(1) + steps(2)` |
| 0x02 | SERVO | CMD | 角度 + 可选时长 |
| 0x03 | TTS | CMD | **TW-TTS 帧透传**（`0xFD` 开头原样转发；否则当 UTF-8 正文自动组帧） |
| 0x04 | TOF_CFG | CMD | `poll_ms(1)` 运行时调整轮询间隔 |
| 0x05 | CLEAR_FAULT | CMD | 清故障 |
| 0x06 | PING | CMD | 心跳 |
| 0x07 | SET_CONFIG | CMD | 写 32 位配置字 |
| 0x08 | GET_CONFIG | CMD | 读回配置 |
| 0x09 | STEPPER_STAT | CMD | **诊断**：读电机实时状态 |
| 0x0A | LINK_STAT | CMD | **诊断**：读链路健康计数器 |
| 0x0B | STEPPER_TEST | CMD | **标定**：只驱动单侧电机 |
| 0x0C | MOTOR_STOP | CMD | **停止**：立即切断两路步进 |
| 0x81 | ACK | 回 | `status(1) [+ 附加数据]` |
| 0x82 | NAK | 回 | `status(1) + op(1)` |
| 0x86 | PONG | 回 | `uptime(4) + fault(1) + config(4) + flags(1) + impl(1)` |
| 0x91 | EVT_EDGE_DETECTED | 上报 | `sensor(1) + dist_mm(2)` 桌沿 |
| 0x92 | EVT_TOF_DATA | 上报 | `mask(1) + d0..d3(2 each)` 周期距离 |
| 0x93 | EVT_FAULT | 上报 | `fault(1)` |
| 0x95 | EVT_OBSTACLE_DETECTED | 上报 | `dist_mm(2)` 障碍 |
| 0x94 | EVT_MOTION_DONE | 上报 | `motor(1) + position(4)` |

### MOVE 方向

| dir | 含义 |
|---|---|
| 0 | 前进 |
| 1 | 后退 |
| 2 | 原地左转 |
| 3 | 原地右转 |
| 4 | 停止 |

### 状态码

| 值 | 名称 |
|---|---|
| 0x00 | OK |
| 0x02 | BUSY |
| 0x03 | FAULT_LATCHED |
| 0x04 | CRC_ERROR |
| 0x05 | UNKNOWN_OP |
| 0x07 | FLASH_FAIL |

### 故障码

| 值 | 名称 | 含义 |
|---|---|---|
| 0x00 | NONE | 正常 |
| 0x01 | HEARTBEAT_LOST | 1000 ms 无有效帧 → 停止运动 |
| 0x02 | EDGE | 桌沿（TOF 阶段） |
| 0x03 | OBSTACLE | 障碍（TOF 阶段） |
| 0x04 | THERMAL | 步进过热 |
| 0x05 | TOF_INIT | TOF 初始化失败 |
| 0x0A | NO_EDGE_PROTECT | 盲走模式（可强制开启） |

### 心跳契约（**上位机必须遵守**）

> **运动期间上位机必须每 ≤500 ms 发一个 `PING`。**
> 否则 1000 ms 后下位机置 `HEARTBEAT_LOST` 并调用 `Stepper_StopAll()`。

这条很容易被忽略：单独发一次 `MOVE 2400` 后如果不再发帧，
电机只会走 **1 秒多一点** 就停住。

---

## 7. 上位机测试工具

依赖：`pyserial`

```bash
pip install pyserial
python tools/pc_link_test.py COM7
```

### 交互命令

| 命令 | 作用 |
|---|---|
| `v` | 查询固件能力（刷没刷固件一看就知） |
| `p` | PING |
| `g` / `s <hex32>` | 读 / 写配置 |
| `sv <deg> [ms]` | 舵机（-45~+45°，ms 省略=立即） |
| `fwd [步数] [档位]` | 前进（默认 2000 步 @档位 2） |
| `bck` / `lft` / `rgt` | 后退 / 原地左转 / 原地右转 |
| **`st`** | **立即停止（独立停止命令，无条件生效）** |
| **`stc`** | 立即停止 + 清除热保护冷却 |
| `mv <dir> [档位] [步数]` | 直接指定 dir |
| **`ml` / `mr` `<步数>`** | **只驱动左 / 右电机**（原始方向，有正负） |
| **`dcal`** | **方向标定：依次跑单侧 4 条** |
| `sst` | 读电机实时状态 |
| `lstat` | 读链路健康计数器 |
| `tick` | 测量 1 ms 时基是否准确（10 秒） |
| `tof <ms>` | 设置 TOF 轮询间隔（10..255） |
| `tofd [秒]` | 监听 TOF，实时打印四路距离 |
| `end [次数] [步数]` | 耐久测试，逐轮核对位置 |
| `c [ack]` | 清除故障 |
| `hb` | 500 ms 心跳 |
| `q` | 退出 |

### CLI 模式

```bash
python tools/pc_link_test.py COM7 --selftest
python tools/pc_link_test.py COM7 --verify-boot
python tools/pc_link_test.py COM7 --move-selftest
python tools/pc_link_test.py COM7 --endurance 10 --steps 2000
python tools/pc_link_test.py COM7 --tick-test
python tools/pc_link_test.py COM7 --dcal
```

---

## 8. 验证与测试

### 主机单元测试（不需要硬件，在 PC 上跑）

一条命令跑全部：

```bash
bash tools/run_host_tests.sh
```

也可以单独跑某一个（以 TTS 为例）：

```bash
gcc -std=c99 -Wall -Wextra -Werror -O2 -I tools/hosttest -I Hardware -I User \
    tools/hosttest/test_tts_math.c -o t && ./t
```

| 测试 | 断言数 | 内容 |
|---|---|---|
| `test_link_proto.c` | 30 | CRC 向量、组帧、拆帧、坏帧重同步 |
| `test_servo_math.c` | 17 | 角度钳位、角度→脉宽 |
| `test_stepper_math.c` | 35 | 相序、Hz→ARR、减速步数、梯形运动模拟 |
| `test_tof_math.c` | 39 | 桌沿/障碍判定、回滞、去抖、无回波语义 |
| `test_cmd_router.c` | 43 | 双命令源仲裁、优先级分类、控制权切换 |
| **`test_tts_math.c`** | **34** | **环形缓冲回绕、整帧原子性、帧长、原始帧识别** |
| **合计** | **198** | |

### 固件语法检查（打开 Keil 之前跑）

```bash
bash tools/check_firmware_syntax.sh
```

用 `tools/stub/stm32f10x.h` 这个最小 SPL 桩，在 PC 上对 13 个固件源文件做
`-fsyntax-only`。能挡掉拼写错误、漏分号、结构体字段写错、函数未声明。

> ⚠️ **它不等价于 Keil 编译。** 不产生目标码、不检查链接、不验证外设行为。
> 真正上板前仍必须在 Keil 里 Build 一次。

### 源码卫生检查（编译前必跑）

```bash
python tools/check_source_sanity.py
```

扫描 `User System BSP Hardware Protocol Service`，检测：

- 字面 `` `r`n `` 之类的转义残留（曾导致 `#include` 被吞、30 个未定义标识符）
- 非法 UTF-8
- UTF-8 BOM
- 混用 CRLF/LF

### 硬件验收流程

```text
1. tick              时基是否准（误差应 < 2%）
2. lstat             链路是否零丢帧
3. dcal              方向标定（单侧驱动，最可靠）
4. fwd 2000 2        正常前进
5. st                立即停止
6. sst               位置精确、remaining=0
7. end 10 2000       耐久：偏差 0.00%、左右差 0、热跳闸 0
```

**`end` 的通过标准：**

| 指标 | 合格线 |
|---|---|
| 累计偏差 | < 0.5% |
| **左右累计差** | **必须为 0** |
| 热跳闸 / 动作截断 | **必须为 0** |

---

## 9. 进度

| 阶段 | 内容 | 状态 |
|---|---|---|
| M0 | 时钟 / 时基 / 看门狗 / LED | ✅ |
| M1 | 串口链路 + CRC + ACK | ✅ |
| M2 | 心跳超时 | ✅ |
| M2.5 | Flash 配置持久化 | ✅ |
| M3 | 舵机 | ✅ |
| M4 | 单电机精确步数 | ✅ |
| M5 | 双履带差速 | ✅ |
| **M6** | **VL53L0X I²C 驱动 + 4 颗 XSHUT 地址分配** | **✅ 代码完成** |
| **M7** | **底部 3 颗桌沿检测（距离变远 = 危险）** | **✅ 代码完成** |
| M8 | 前脸障碍检测（距离变近 = 危险） | ✅ 逻辑已实现，待阈值标定 |
| M8.8 | 盲走模式 + 失联保护 | ⬜ |
| **M9** | **TTS 转发（USART2 / 9600 / 中断发送）** | **✅ 代码完成** |
| M10 | A7Z ↔ MCU 整机联调 | ⬜ |
| M11 | 失效安全验收 | ⬜ |
| M12 | 长跑测试 | ⬜ |

---

## 10. 已知限制

| 限制 | 说明 |
|---|---|
| **无编码器** | `position` 只统计"MCU 发了多少脉冲"，不代表电机真转了。高速失步检测不到 |
| **无桌沿保护** | TOF 未接入时，机器人会直接冲出桌面 |
| **速度上限低** | 28BYJ-48 天生慢；实测 5V 全步 300 Hz ≈ 13.9 mm/s |
| **不可闭环** | 想真正闭环需要编码器或限位开关做参考点 |

### 速度参考（全步 2038 步/圈，30 mm 轮）

| 档位 | Hz | 线速度 |
|---|---|---|
| 0 | 120 | 5.5 mm/s |
| 1 | 200 | 9.2 mm/s |
| **2** | **300** | **13.9 mm/s（标准档）** |
| 3 | 600 | 27.7 mm/s（需实测确认不丢步） |

> ⚠️ 不要用更高步频做长距离运动。实测 600~800 Hz 会让 28BYJ-48
> 在 5V 下 **完全失步**，表现为剧烈抖动或转子被磁场拖着转。

---

## 11. 踩坑记录

这些都是真实踩过的，**改代码前先看一遍**：

| # | 现象 | 根因 | 修法 |
|:--:|---|---|---|
| 1 | 编译 30 个"标识符未定义" | PowerShell 单引号里的 `` `r`n `` 被当字面字符写进源码，`#include` 被吞掉 | 改完源码跑 `check_source_sanity.py` |
| 2 | 动作精确停在 1.29 秒 | **uint32 下溢**：`LinkSvc_Dispatch` 更新的 `s_last_frame_ms` 可能比主循环的 `now` 晚 1 ms，`now - last` 变成 42 亿 → 误报心跳丢失 → 安全联锁停机 | 所有时间差**先比大小再相减** |
| 3 | 电机只走 1 秒就停 | 上位机在动作期间不发心跳 | 动作期间每 ≤500 ms 发 `PING` |
| 4 | 热故障永久锁存 | `s_fault = FAULT_THERMAL` 不清除，联锁杀掉之后每一次 MOVE | 改成"按当前是否冷却"判定，冷却结束自动恢复 |
| 5 | 左右分叉（L=8914 / R=8895） | `Stepper_MoveBoth` 一侧启动失败时不回滚 | 任一侧失败就回滚另一侧 |
| 6 | 左右方向怎么调都不对 | **物理左右标反了**。`DIR_SIGN` 只能修"前进极性"，修不了"哪组引脚是左" | 用 `ml`/`mr` 单侧标定，同时改引脚映射和 `DIR_SIGN` |
| 7 | 舵机乱转 | 舵机电源与 MCU **没有共地** | 所有 GND 星形汇合到电源负极 |
| 8 | `ACK(0x2E)` 之类的怪状态码 | MOVE 的 ACK 把预测距离放在了 `status` 位置 | ACK 统一 `payload[0]=status` |
| 9 | 高步频电机狂转/不动 | 全步 600~800 Hz 相当于半步 1200~1600 Hz，远超 5V 能力 | 标准档固定 300 Hz |

### 三条通用纪律

**① 协议联调阶段必须关掉阻塞调试输出**

```c
#define APP_DEBUG_TEXT        0
#define DEBUG_STATUS_PRINT    0
```

`Serial_Printf` 是**阻塞发送**，且 `handle_move` 里它在 ACK **之前**执行，
会推迟应答并打乱主循环时序。这曾导致动作被反复截断。

**② 方向类问题一律用单侧命令定位**

`fwd` / `lft` 都是双电机复合运动，任一符号错就看不出是哪一处。
`ml` / `mr` 只驱动一侧、且**不经过 `DIR_SIGN` 换算**，
"正数让轮子往哪走"是唯一确定的硬件事实。

**③ 改了 `app_config.h` 必须 Rebuild**

增量编译对配置头文件的依赖不可靠，只 `Build` 会出现"改了没生效"。

---

## License

MIT
