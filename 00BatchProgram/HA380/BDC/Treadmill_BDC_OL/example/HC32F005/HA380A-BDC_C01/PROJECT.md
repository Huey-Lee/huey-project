# HC32F005 H1 — 跑步机直流母控（开环）

本文档描述 **H1** 固件的用途、源码布局、编译方式、运行时架构及与上位机相关的协议与调参要点。与历史 C01/C02/C03 工程并列，H1 针对 **110 V AC / 低档母线** 试制场景，电机控制为 **纯开环**（无速度环 PID）。

---

## 1. 项目概述

| 项 | 说明 |
| --- | --- |
| MCU | HC32F005（小华半导体） |
| 典型时钟 | PCLK 24 MHz（见 `MCU_PCLK_HZ_APP`） |
| 通信 | UART1，**2400 baud**，与显示/上位机私有帧格式 |
| 电机策略 | `motor.c`：速度斜坡 + 继电器与 PWM 时序；`motor_drive.c`：PWM ISR、母线锁存占空换算、电流 IR 前馈（KIV）、过流/过压/飞车与 MOS 偏置保护 |
| 量产特性 | `motor.h` 注明：已无单独产线 FACTORY 编译开关（协议见 `CMD_TEST` 等须在联调阶段约束使用） |

---

## 2. 目录结构

```
H1/
├── MDK/                  # Keil µVision 工程（project.uvprojx）、分散加载、输出 hex/map
├── source/               # 应用源码（核心业务）
│   ├── main.c            # 上电入口、主循环调度
│   ├── motor.h / motor.c # 整机状态机、速度档表 speed_param[]、斜坡与故障封装
│   ├── motor_drive.h / motor_drive.c
│   ├── uart_frame.h / uart_frame.c
│   ├── user_timer.*      # TIM0 继电器、TIM2 ADC 触发、TIM6 PWM(ADTIM6)
│   ├── user_adc.*       # ADC 与母线/电流采样缓冲
│   ├── user_usart.*     # UART1 与外设初始化
│   ├── user_gpio.*
│   ├── queue.*          # UART 收包字节队列（与帧解析配合）
│   ├── ringfifo.*       # 历史环形 FIFO（当前可与 UART 管线并存，按需裁剪）
│   ├── point_fun.*      # MAIN_INIT → ctr_init / ADC init 钩子
│   └── ddl_device.h     # DDL 器件选择宏
└── PROJECT.md           # 本文件
```

---

## 3. 构建与下载

1. 使用 **Keil MDK** 打开 `MDK/project.uvprojx`。
2. 选择目标工程配置（通常为 Release）。
3. 编译生成 `MDK/output/release/project.hex`（路径以当前工程为准）。
4. 使用 J-Link/ST-Link 等按芯片手册连接 SWD，烧录 hex。

若在工程选项中覆盖了预定义宏（如 `MOTOR_PWM_PERIOD_TICKS`、`MOTOR_DISPLAY_VOLT_SCALE_X100`），应以 **Keil 配置为准**，其优先级高于 `motor.h` 默认值。

---

## 4. 主循环调度（`main.c`）

主循环严格按照固定顺序迭代（非 RTOS）：

1. **`_500ms_time_handle`**：主循环节拍计数、`base_time` 通信超时、LED 翻转；超时累计触发 **E01** 软停机。
2. **`motor_speed`**：按 TIM6 ISR 等价计数阈值调用，完成 **STATUS_MT_START / MT_RUN / MT_STOP** 的速度与电压斜坡。
3. **`error_chick`**：主循环侧过流、过压、过速等保护（与 ISR 内保护互补）。
4. **`uart_frame_loop`**：从 `rx_queue` 取字节，状态机组帧并调用 **`cmd_proc`**。
5. **`ctr_proc_loop`**：`ctr.status` 主状态机（上电自检、RUN 门控、合闸、故障处理等）。
6. **`wait_for_motor`**：`waitforamoment` 置位后短延时进入 **`STATUS_TO_RUN`**。
7. **`communication_checkout`**：向上位机回送 ACK / 错误等。

TIM6 中断中执行 **`motor_drive_isr`**，负责占空更新、IR 补偿与部分即时保护逻辑。

---

## 5. 状态机要点

### 5.1 主控 `ctr.status`（节选）

- **STATUS_POWERON**：上电 ADC 自检（与 C03 对齐思路），含 **E02**（M+/M− 折算压差过大）、**E05**（上电阶段双轨过低，见下文与 `ERROR_05` 多义）。
- **STATUS_RUN**：启动前门控自检通过后置 **`waitforamoment`**，再转 **STATUS_TO_RUN**。
- **STATUS_TO_RUN**：合继电器 → **delay 500 ms** → **`motor_drive_on_to_run_capture_bus_average`**（默认 20×1 ms 平均 **M+**）锁存母线 ADC 作为占空 **固定分母** → TIM6 PWM 运行。
- **STATUS_ERROR**：按 `ctr.error_code` 区分断电/软减速等，并 **`uart_frame_tx(CMD_ERROR, code)`**。

### 5.2 电机 `motor.status`

- **MT_START**：爬坡至不低于 `vmin`（由上控 **`CMD_VOLTAGE_MIN`** 刻度换算）；超时 **START_PHASE_STALL_TICKS_MAX** → **E03**。
- **MT_RUN**：开环跟速与电压斜坡，`kiv` 跟踪 `speed_param[motor.index].kiv` 及 **`CMD_KIV_*`**。
- **MT_STOP**：减速与停机斜坡，最终 **STATUS_TO_STOP** 关 PWM、续流、断继电器、`CMD_STOP_OVER`。

---

## 6. UART 帧与命令字

帧格式常量见 **`uart_frame.h`**：**SOF 0x7F**、**EOF 0x7E**，含 `cmd` / `len` / 载荷及校验。

主要 **`cmd_e`**（节选，完整列表见源码）：

| 命令宏 | 典型用途 |
| --- | --- |
| CMD_STATUS | 运行状态 / 设定速度（需注意 `LEN=1` 时初速占位逻辑，见 `uart_frame.c` 注释） |
| CMD_ACK / CMD_ERROR / CMD_RESET / CMD_STOP_OVER | 应答与停机完成 |
| CMD_TREADMILLS_SPEED_MAX | 上位速度上限（载荷 10～100 ↔ 内部 ×10） |
| CMD_VOLTAGE_MAX / MIN | 最高/最低电压刻度（V） |
| CMD_OVER_CURRENT_MAX | 刷写 **`over_current`**（全速过流 ADC 阈值） |
| CMD_KIV_1KM … CMD_KIV_12KM | 各 km/h 档位 IR 系数（运行时以上位为准写入表项） |
| CMD_STA_LEVEL / END_LEVEL / ACCELERATED_LEVEL / DECELERATION_LEVEL | 起步电压、停机判据、加减步 |
| CMD_OVER_VALTAGE_MAX | **`over_voltage`** 兜底（V）；RUN 下过压还以 **vmax×百分比** 为主 |

`ee_param_t` 镜像了部分需在显示侧同步缓存的参数。

---

## 7. 关键宏与标定（`motor.h` / `motor_drive.h`）

| 宏（或变量） | 含义 |
| --- | --- |
| **MOTOR_PWM_PERIOD_TICKS** | ADTIM6 周期刻度；PWM 频率 ≈ **PCLK / (4 × PER)** |
| **MOTOR_DISPLAY_VOLT_SCALE_X100** | 显示伏特标尺 → **`ADC_cmd`** 的全局缩放（默认 **880**，现场可调） |
| **MT_START_PWM** 等 | 由 **LEGACY** 周期比例换算到当前 PER，避免改 PER 后起步占空错位 |
| **OVER_CURRENT_MAX** / **OVER_CURRENT_MAX_LOW_SPEED** | 全速与低速（≤约 2 km/h）过流 ADC 门限 |
| **MOTOR_OC_LOW_ISR_DEBOUNCE_SEC** × `MOTOR_TIM6_ISR_HZ_APPROX` | 慢行滤波过流长跑节拍（默认 **12 s** 量级）；与 **`OVER_CURRENT_MAX_LOW_SPEED`（默认≈15 A ADC）** 配合 |
| **MOTOR_OC_FULLSPEED_TICK_NEED** | 快档仅认 **`I_filter > over_current`** 连续如此多拍（默认 **450**） |
| **MOTOR_OC_ARM_DELAY_SEC** | 过流判定前缓起盲区（默认 **2 s**，可 Define） |
| **MOTOR_OVERVOLT_MAX_VOLTAGE_PERCENT** | RUN **E05**：端压 ≥ **vmax × 该百分比**（默认 **130**）；vmax=0 时用 **`over_voltage`** |
| **motor_vbus_adc** | 合闸后锁存的母线 ADC，作占空分母 **`duty ≈ V_cmd × PER / motor_vbus_adc`** |

初值注意：`ctr_init()` 中 **`motor.adjust_ove_curren = 400`** 与 **`over_current` 初值 `OVER_CURRENT_MAX`** 可能在上位未下发 **CMD_OVER_CURRENT_MAX** 前不一致；以 **`over_current`** 实际参与 ISR/保护为准。

**为什么 C03 堵转更易进 E03，而 H1 曾经很难**

| 维度 | HA380A-BDC\_C03 | H1（改前常见问题） |
| --- | --- | --- |
| 起步离场条件 | **`start_tim>15`** 且 **目标折算伏特 `val < motor端压`**，否则一直停在 START | **仅 `adjust_now_voltage≥vmin`** 就进 RUN，堵转仍可「电压指令到位」→ **误进 RUN** |
| 堵转兜底 | **`checkout_start_stall`：`start_tim>60`** 必 E03（主环节拍） | 有 **55 tick**，但往往 **vmin 先满足并已进 RUN**，堵转兜底用不上 |
| 低速过流采样 | **`valtage_cur` 瞬时** 比较 **`CURRENT_6A0`**，`error_chick` 里 **≈21 拍** | 主要依赖 **重度 IIR 滤波**，阈还曾用 **8 A**，更难超 |
| 控制结构 | PID 闭环 `MT_PID`，电流→占空的耦合与 C03 表参数不同 | **纯开环 + IR**，同样在「假 RUN」下游态异常 |

---

**H1（改后）E03 / 起步**

- **START→RUN**：除 **vmin** 斜坡外，另需 **对齐 C03 的端压条件**（`MT_START_FEEDBACK_MIN_TICKS`≥15 后再判 **`tgt_display_V < now_display_V`**，`now_v=0.126×max(Up−Down,0)`）。
- **慢行带（set/cur ≤600≈≤6 km/h）**：**`OVER_CURRENT_MAX_LOW_SPEED` 默认 CURRENT_15A（≈15 A ADC）**，须滤波电流 **长跑约 `MOTOR_OC_LOW_ISR_DEBOUNCE_SEC`（默认 12 s 量级拍数）** 才 E03；原 8 A 档在长期带载工况下等价于「正常负载也报错」。
- **快档（>600）**：**删掉** 「长期>I_LOW」兜底；仅 **`I滤波 > over_current`** 连续 **`MOTOR_OC_FULLSPEED_TICK_NEED`（默认 450）拍** → E03，避免快跑带载人长期在 15 A 附近被判堵转。
- **START 主环**：瞬时电流（去零偏）>**LOW_SPEED 阈** 持续约 **50×主环拍** 仍可 E03（与慢行阈一致为 **15 A ADC**）。
- **`motor_drive_isr`** 节拍随 **ADC**，不等于 PWM。**`MOTOR_OC_ARM_DELAY_SEC`** 内不计过流。
- **`MT_START_FEEDBACK_MIN_TICKS`、`_DEBOUNCE_SEC`、`_FULLSPEED_TICK_NEED`、`MOTOR_OVER_CURRENT_ADC_*`** 均可 Keil Define 微调。

---

**低速 &lt;1.5 km/h 「打脚底板」**

常见为 **低档 KIV（IR）过强 + PWM 斜坡/滞回** 在低占空节拍下的 **扭矩锯齿**；已与 C03 类似方向将 **前两档 speed_param[].kiv 8→6** 试柔顺。仍可试 **再降 kiv**、略减 **`MOTOR_VOLTAGE_RAMP_ADC_MAX_STEP`** 或微调 **duty hysteresis**（权衡噪声/效率）。

---

## 8. 错误码速查（`error_code_enum`）

| 码 | 含义（摘要） |
| --- | --- |
| E01 | 通信超时 → 软减速停机 |
| E02 | 上电/RUN 门控：**M+/M− 折算电压差** 过大 |
| E03 | 过流或起步堵转超时 |
| E04 | 码表占位（继电器短等对齐老机型；当前主路径少触发） |
| E05 | **RUN**：过压（相对 vmax×%）；**POWERON**：双轨过低（与 RUN 同类硬件条件的 **E08** 区分，上位文案需对齐） |
| E06 | 过速 / 飞车类 |
| E07 | 扩展保留 |
| E08 | **RUN** 门控双低；运行中 ISR **端压显著高于指令+IR** 等（单向判 MOS/高压异常） |

硬故障 **`motor_hard_fault_set`**：关 PWM/继电器并进 **STATUS_ERROR**。软故障 **`motor_soft_fault_set`**：进入停机斜坡。

---

## 9. ADC 与外设映射（摘自 `main.c` / GPIO）

- **P24**：电流 CH0（模拟）
- **P26**：M− 母线 CH1（模拟）
- **P32**：M+ 母线 CH2（模拟）
- **STK_LED**：主循环心跳

具体继电器、PWM、UART 脚位以 **`user_gpio` / `user_timer` / `user_usart`** 初始化为准。

---

## 10. 维护说明

- 协议与档位表变更时，请同时核对 **`motor.c` `speed_param[]`**、`uart_frame.c` **`cmd_proc`** 与显示工程中的 **KIV / 限速** 定义。
- 裁剪工程体积时：`ringfifo`、`user_usart` 内未调用函数等需对照 **MAP 文件**确认无引用后再删，避免遗漏弱符号或中断路径。

---

## 11. 版本与版权声明

MCU 与外设底层驱动沿用华大/小华官方 DDL 版权声明（见各文件头 **`main.c` 等**）。应用层注释以仓库内源码为准。

*文档生成自当前 H1 源码树梳理；若与现场分支不一致，以实际编译分支为准。*
