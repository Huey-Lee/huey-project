/* motor.h  电机参数定义、状态枚举、结构体声明 */
#ifndef USER_MOTOR_H_
#define USER_MOTOR_H_

/*──────────────────── 量产固件 ──────────────────────────────────────────
 * 命令字定义见 uart_frame.h；已无产线独占的 FACTORY_CMD / 固定占空测试编译开关。 */
#include "hc32f005.h"
#include "bt.h"
#include "hardware_config.h"

/* ═══════════════════════════════════════════════════════════════════════════
 * 电网自适应（110V / 220V）：合闸后据母线识别环境，加载阈值包 + 本机最大运行电压帽
 * 显示 V→占空指令仍走 MOTOR_DISPLAY_* / adjust_now_voltage；勿与 voltage_param 混淆
 *（voltage_param = 速度斜坡 dV/d_scale，由 motor_recalc_voltage_param_from_cmd 维护）
 * ═══════════════════════════════════════════════════════════════════════════ */
#ifndef BUS_V_220V_THRESHOLD_V
#define BUS_V_220V_THRESHOLD_V   (200u)   /* 折算母线 V，高于此判 220V 电网 */
#endif
#ifndef SAFE_MIN_VBUS_ADC
#define SAFE_MIN_VBUS_ADC        (1000u) /* 占空分母 ADC 哨兵下限（≈120V 量级） */
#endif
#ifndef MOTOR_VBUS_ADC_FALLBACK_DEN
#define MOTOR_VBUS_ADC_FALLBACK_DEN  (2468u) /* 分母异常低时回退，抑制占空比冲顶 */
#endif

#define E02_LIMIT_110V_V         (70u)    /* M+/M− 估算伏特差 */
#define E05_ABS_VOLT_110V        (220u)   /* M+ 估算绝对过压（V），与比值 E05 并行 */
#define MAX_RUN_V_110V           (100u)   /* 110V 电网下本机 vmax 帽（显示伏特刻度） */

#define E02_LIMIT_220V_V         (140u)
#define E05_ABS_VOLT_220V        (420u)
#define MAX_RUN_V_220V           (180u)

/* 理论：1V 母线（至分压点）≈ ADC 计数增量，仅作文档/ Keil 覆写参考 */
#ifndef GRID_VOLT_STEP_ADC_FLT
#define GRID_VOLT_STEP_ADC_FLT   ((ADC_RESOLUTION) / ((V_REF_ADC) * (V_BUS_DIVIDER_RATIO)))
#endif

extern u16 dynamic_e02_limit_v;
extern u16 dynamic_e05_abs_volt_limit;

/* 1：已完成 STATUS_TO_RUN 母线识别；此前不钳 UART 的 vmax，避免 220V 机上电前被限到 110 帽 */
extern u8 motor_grid_profile_known;
/* 母线识别结果：1=220V 参数包，0=110V 参数包 */
extern u8 motor_grid_env_220;

/* 速度斜坡定时计数（每 N 次主循环执行一次速度步进） */
#define VAR_SPEED_CNT             (3)
#define VAR1_SPEED_CNT            (3)
/* Tim6(ADTIM6) 三角波：f_pwm ≈ PCLK/(4×PER)。默认目标载频 MOTOR_PWM_FREQ_HZ=16 kHz；
 * @24 MHz → PER=375；改 Keil 可覆写 MOTOR_PWM_FREQ_HZ 或 MOTOR_PWM_PERIOD_TICKS。 */
#ifndef MCU_PCLK_HZ_APP
#define MCU_PCLK_HZ_APP                 (24000000u)
#endif
#ifndef MOTOR_PWM_FREQ_HZ
#define MOTOR_PWM_FREQ_HZ               (16000u)
#endif
#ifndef MOTOR_PWM_PERIOD_TICKS
#define MOTOR_PWM_PERIOD_TICKS          ((MCU_PCLK_HZ_APP) / (4u * MOTOR_PWM_FREQ_HZ))
#endif
#ifndef MOTOR_ADTIM6_HALF_PERIOD_TICKS
#define MOTOR_ADTIM6_HALF_PERIOD_TICKS    (MOTOR_PWM_PERIOD_TICKS)
#endif
#ifndef MOTOR_TIM6_ISR_HZ_APPROX
#define MOTOR_TIM6_ISR_HZ_APPROX  ((MCU_PCLK_HZ_APP) / (4u * MOTOR_PWM_PERIOD_TICKS))
#endif
#define MOTOR_SPEED_PERIOD_MT_START_ISR   (((MOTOR_TIM6_ISR_HZ_APPROX) * (100u)) / (1000u))

/* C03 同源：`checkout_start_stall()`（motor_speed START 节拍）；与 C03_START_STALL_MAINLOOP_TICKS 对齐 */
#ifndef C03_START_STALL_MAINLOOP_TICKS
#define C03_START_STALL_MAINLOOP_TICKS     (60u)
#endif
#ifndef START_PHASE_STALL_TICKS_MAX
#define START_PHASE_STALL_TICKS_MAX        C03_START_STALL_MAINLOOP_TICKS
#endif
/* 起步并进 RUN：仅要求 v_ramp ≥ v_run_min（见 motor_speed）；不再 dwell，避免 handshake 卡住 10 V*/
#define START_RUN_MIN_DWELL_TICKS          (0u)

/* 「显示刻度 V」→ 占空链路中的电压指令标尺（ADC_cmd）。
 * duty_ticks ≈ (Vcmd_phys/Vbus_phys)×PER，V 来自 101 分压+5V 参考；vbus 取 ISR 内 valtage_up。
 * 默认 880≈×8.80；Define 覆盖 MOTOR_DISPLAY_VOLT_SCALE_X100（常用约 855~895）。
 * 【注意】低档目标电压还与速度斜坡成正比：未达到 adjust_speed_max 时 Vt < 下发的 vmax 属正常现象。 */
#ifndef MOTOR_DISPLAY_VOLT_SCALE_X100
#define MOTOR_DISPLAY_VOLT_SCALE_X100   (880u)
#endif

#define MOTOR_DISPLAY_VOLT_TF_TO_ADC_CMD_FLT(display_volt_tf) \
    ((float)(display_volt_tf) * (float)(MOTOR_DISPLAY_VOLT_SCALE_X100) * 0.01f)

/* 指令电压（ADC_cmd 标尺）→ 显示伏特物理量；与 GET_PHYSICAL_V 配对做 Vcmd/Vbus 闭环 */
#define MOTOR_ADC_CMD_TO_DISPLAY_VOLT_FLT(adc_cmd_) \
    ((float)(adc_cmd_) * 100.0f / (float)(MOTOR_DISPLAY_VOLT_SCALE_X100))

/* 占空物理映射：母线瞬时电压过低时避免除零/冲顶；占空比上限预留死区 */
#ifndef MOTOR_VBUS_PHYSICAL_MIN_VALID_V
#define MOTOR_VBUS_PHYSICAL_MIN_VALID_V   (50.0f)
#endif
#ifndef MOTOR_VBUS_PHYSICAL_FALLBACK_V
#define MOTOR_VBUS_PHYSICAL_FALLBACK_V     (155.0f)
#endif
#ifndef MOTOR_DUTY_RATIO_CAP
#define MOTOR_DUTY_RATIO_CAP               (0.95f)
#endif

#define MOTOR_MIN_VOLT_DISPLAY_TO_CMD_ADC(DISPLAY_V) \
    ((u16)(((u32)(DISPLAY_V) * (u32)(MOTOR_DISPLAY_VOLT_SCALE_X100) + 50u) / 100u))

extern u16 motor_cmd_min_volt_adc_floor(void);  /* vmin 刻度 × 上面系数（四舍五入），与 RUN 路径一致 */

/*──────── 电压标尺拆分（读写维护时请先对号入座） ────────
 * A) 「显示伏特刻度 / 上位机 B」→ 目标电压：**MOTOR_DISPLAY_VOLT_SCALE_X100** → **ADC_cmd**（占空链路 * duty=Vadc×PER/vbus）。
 * B) 「M+−M− 差分 ADC」（cut_adc / cut_dc）→ **面板对齐的粗监视伏特**：**MOTOR_CUT_ADC_DELTA_TO_DISP_VOLT_***（不参与占空）。
 *    字面 0.126 等价于 adc 计数 ×126/1000；与 A) 的比例尺来源不同，不可混写成「一个 880 走遍天下」。*/
#ifndef MOTOR_CUT_ADC_DELTA_TO_DISP_VOLT_X1000
#define MOTOR_CUT_ADC_DELTA_TO_DISP_VOLT_X1000  (126u)
#endif
#define MOTOR_CUT_ADC_DELTA_TO_DISP_VOLT_FLT(delta_adc_) \
    ((float)(delta_adc_) * (float)(MOTOR_CUT_ADC_DELTA_TO_DISP_VOLT_X1000) * 0.001f)

/* 电机停止时对应的 ADC 阈值（约 50 Hz 任务节拍） */
#define MT_OPEN_ADC               (100)
/* 接近停止时触发软停动作的速度斜坡阈值（4 ≈ 0.2 km/h） */
#define MT_STOP_ACT_SPEED_SCALE   (4)

/* 允许起步爬坡的最低速度斜坡值（20 ≈ 0.2 km/h） */
#define MT_START_SPEED_SCALE      (20)
/* 起步爬坡结束、切换到稳速运行的速度斜坡阈值（120 ≈ 1.2 km/h） */
#define MT_TO_RUN_SPEED_SCALE     (120)

/*────────────────── ZHANKONBI 标尺（必须与 ADTIM6 周期 MOTOR_PWM_PERIOD_TICKS 一致）
 * 注意极性：**ZHANKONBI 越大 → OUT_PUT=PER−ZHANKONBI 越小**；与本板一致为**近关断/弱输出**（自检写 ZN=PER）。
 * 加压指令须 **减小 ZHANKONBI**：先算 dt≈(Vcmd/Vbus)×PER，再 **ZHANKONBI=PER−dt**；IR 仍在 ADC_cmd 域叠加后换算。
 * pwm_publish_duty 上限 UK_PWM_MAX：限制 ZN 不超过 PER−5（与死区一致）；自检/关断可直接写 ZN=PER。
 * 【禁止】用旧的 PWM 周期数（例如 545）去乘电压，否则占空会按 PER 比例错乱→饱和畸变→剧烈震动。
 * MOTOR_PWM_LEGACY_PERIOD_TICKS 仅用于把「旧机 PER 下的名义刻度 25 / 12 tick」线性换到当前 PER，
 * 与上述电压公式无关。 */
#ifndef MOTOR_PWM_LEGACY_PERIOD_TICKS
#define MOTOR_PWM_LEGACY_PERIOD_TICKS   (545u)
#endif
#ifndef MOTOR_PWM_LEGACY_IDLE_TICKS
#define MOTOR_PWM_LEGACY_IDLE_TICKS     (25u)
#endif
#ifndef MOTOR_PWM_LEGACY_START_ABSMIN_TICKS
#define MOTOR_PWM_LEGACY_START_ABSMIN_TICKS  (18u) /* 原 12；与线性占空「非零下限」对齐，覆盖静摩擦嗒嗒区（≈15–20 tick 量级） */
#endif

#define MT_START_PWM \
    ((u16)(((u32)(MOTOR_PWM_LEGACY_IDLE_TICKS)      * (u32)(MOTOR_PWM_PERIOD_TICKS) \
         + ((u32)(MOTOR_PWM_LEGACY_PERIOD_TICKS) / (2u))) \
         / (u32)(MOTOR_PWM_LEGACY_PERIOD_TICKS)))
#define MT_START_ABSMIN_DUTY \
    ((u16)(((u32)(MOTOR_PWM_LEGACY_START_ABSMIN_TICKS) * (u32)(MOTOR_PWM_PERIOD_TICKS) \
         + ((u32)(MOTOR_PWM_LEGACY_PERIOD_TICKS) / (2u))) \
         / (u32)(MOTOR_PWM_LEGACY_PERIOD_TICKS)))
#define MT_START_STEP        (3)
/* 判停速度斜坡阈值：低于此值开始计停机超时 */
#define MT_STOP_SPEED        (5)

/* 速度→档位 index 下拉滞回（内部速度单位）：避免 cur 在 100±几抖时反复换档，adc_base 突变引发 IR 震颤 */
#ifndef MOTOR_INDEX_DOWN_HYST_TICKS
#define MOTOR_INDEX_DOWN_HYST_TICKS  (40u)
#endif
/* RUN 段 IR 前馈单拍补偿上限（ADC_cmd），抑制高频 ADC→motor_drive_isr 节拍下 ΔI 对占空的冲击 */
#ifndef MOTOR_KIV_COMP_MAX_ADC_RUN
#define MOTOR_KIV_COMP_MAX_ADC_RUN   (80)
#endif
/* 低速 Kiv Q12 渐乘：兼顾「足底电流台阶」与「1~3 km/h 扛脚不致被踩停」——
 * 在 END 以下按比例爬升→在 END～FULL 再顶到 4096；（旧版 END 以下为 0 易完全无 IR 补偿→低速无力）。*/
#ifndef MOTOR_KIV_SOFT_BLEND_END_BELOW_SCALE
#define MOTOR_KIV_SOFT_BLEND_END_BELOW_SCALE (300u) /* 内部 100≈1 km/h；以下为「弱低速区」上沿≈显示 3.0 km/h */
#endif
/* cs=END 时渐乘达到的 Q12「台阶」→与 FULL 之间再做线性拉上；默认 2048≈半壁，仍抑脚麻又给负荷留 IR */
#ifndef MOTOR_KIV_SOFT_BLEND_AT_END_Q12
#define MOTOR_KIV_SOFT_BLEND_AT_END_Q12 (2048u)
#endif
#ifndef MOTOR_KIV_SOFT_BLEND_FULL_AT_SCALE
#define MOTOR_KIV_SOFT_BLEND_FULL_AT_SCALE (430u) /* ≥≈4.3 km/h 全额 */
#endif
#ifndef MOTOR_KIV_BOOST_CAP_LOWSPD_ADC
#define MOTOR_KIV_BOOST_CAP_LOWSPD_ADC (165) /* 弱低速 IR 帽；纯 120 时上人易顶不穿 → 易被踩懵 */
#endif
#ifndef MOTOR_KIV_BOOST_CAP_RUN_ADC
#define MOTOR_KIV_BOOST_CAP_RUN_ADC (300)
#endif
/* 单次 motor_speed()（≈100 ms START / ~180 ms RUN）内目标电压 ADC_cmd 最大阶跃：抑制主循环节拍的电压「锤击」 */
#ifndef MOTOR_VOLTAGE_RAMP_ADC_MAX_STEP
#define MOTOR_VOLTAGE_RAMP_ADC_MAX_STEP       (10u)
#endif
/* 主环一次电压阶跃拆到 ADC 节拍：用 Q8 累加跟踪 adjust_now；每拍 |Δ| 上限（Keil 可覆写） */
#ifndef MOTOR_MICRO_V_EXPECT_TICKS_PER_MAIN_STEP
#define MOTOR_MICRO_V_EXPECT_TICKS_PER_MAIN_STEP  MOTOR_SPEED_PERIOD_MT_START_ISR
#endif
#ifndef MOTOR_MICRO_V_Q8_SLEW_CAP
#define MOTOR_MICRO_V_Q8_SLEW_CAP \
  ((((u32)MOTOR_VOLTAGE_RAMP_ADC_MAX_STEP) * 256u + (u32)MOTOR_MICRO_V_EXPECT_TICKS_PER_MAIN_STEP - 1u) \
   / (u32)MOTOR_MICRO_V_EXPECT_TICKS_PER_MAIN_STEP + 1u)
#endif
/* 驱动节拍：**纯线性** — duty = Vadc × mul >> LIN_SHIFT（mul 仅在母线快照更新时做一次除法），无 PWM 滞回、无占空 slew。 */
#ifndef MOTOR_VBUS_DUTY_LIN_SHIFT
#define MOTOR_VBUS_DUTY_LIN_SHIFT  (14u) /* PER<<SHIFT / vbus fits u32；越大中间精度略高 */
#endif

/* 加速步长（稳态：步长间隔见 tim6_change_speed_time） */
#define speed_step_ACC       3   /* 默认加速步长 */
/* 减速步长（停机阶段） */
#define speed_step_END       3

/* 速度换算为目标电压（线性拟合：斜率 A，最低电压 B） */
#define GET_SPEED_VOLTAGE(SPEED_SCALE, A, B)  (A * (float)(SPEED_SCALE - 100.0) + B)

/* 母线电压 (V) → ADC 计数：Vadc = Vbus / V_BUS_DIVIDER_RATIO */
#define CALC_VOLTAGE_2_ADC(VOLTAGE) \
    ((float)(VOLTAGE) * (ADC_RESOLUTION) / ((V_BUS_DIVIDER_RATIO) * (V_REF_ADC)))

/* ADC 计数 → 母线电压 (V) */
#define CALC_ADC_2_VOLTAGE(ADC) \
    ((float)(ADC) * (V_REF_ADC) * (V_BUS_DIVIDER_RATIO) / (ADC_RESOLUTION))

/* ADC 计数 → 电流 (A)：I = V_adc / (R_shunt × A_v) */
#define CALC_ADC_2_CURRENT(ADC) \
    ((((float)(ADC) * (V_REF_ADC)) / (ADC_RESOLUTION)) \
     / ((CURRENT_SENSE_RES) * (CURRENT_AMP_GAIN)))

/* 电流门限（ADC 计数）：采样电阻 220V=4mΩ，110V=8mΩ，对应各电流值 */
#define CURRENT_20A0    (548)
#define CURRENT_15A0    (411)
#define CURRENT_14A0    (384)
#define CURRENT_13A0    (357)
#define CURRENT_12A0    (330)
#define CURRENT_11A0    (303)
#define CURRENT_10A0    (276)
#define CURRENT_9A0     (252)
#define CURRENT_8A0     (225)
#define CURRENT_7A0     (197)
#define CURRENT_6A0     (170)
#define CURRENT_5A0     (144)
#define CURRENT_4A0     (117)
#define CURRENT_3A0     (90)
#define CURRENT_2A0     (63)
#define CURRENT_1A0     (37)

#define OVER_CURRENT_MAX               (CURRENT_20A0)

/* 慢行/起步「堵转过流」阈（ADC）= 上位下发的全速过流 lim ÷2.5。
 * 无除法、无浮点：lim × (410/1024) ≈ lim × 0.40039 ≈ lim/2.5（误差 <0.1%）。 */
#ifndef MOTOR_STALL_OC_Q_BITS
#define MOTOR_STALL_OC_Q_BITS          (10u)
#endif
#ifndef MOTOR_STALL_OC_Q_MUL
#define MOTOR_STALL_OC_Q_MUL           (410u)
#endif
#define MOTOR_STALL_OC_ADC_FROM_FULL(lim_adc_) \
    ((u16)(((u32)(lim_adc_) * (u32)MOTOR_STALL_OC_Q_MUL) >> (u32)MOTOR_STALL_OC_Q_BITS))

/* 默认全速 20A 档对应的堵转阈（仅作常量参考；运行中应随 oc_lim_full_mirror 用上一宏现算） */
#define OVER_CURRENT_MAX_LOW_SPEED     (MOTOR_STALL_OC_ADC_FROM_FULL(OVER_CURRENT_MAX))

/* E03：ISR 全速判据 = oc_lim_full_mirror；主环慢行/START 堵转判据 = MOTOR_STALL_OC_ADC_FROM_FULL(oc_lim_full_mirror)。 */
#ifndef PROT_OC_LIM_FULL_ISR_NEED_TICKS
#define PROT_OC_LIM_FULL_ISR_NEED_TICKS (200u)
#endif
/* 【低速堵转过流】主环 START / 慢行带：连续越限拍数 (>此值 E03)，与 C03 checkout_current_over / _start 的 20 一致。 */
#ifndef PROT_OC_SLOWSTALL_MAIN_NEED_TICKS
#define PROT_OC_SLOWSTALL_MAIN_NEED_TICKS (20u)
#endif

#ifndef C03_ISR_MAX_OVER_FAULT_TICKS
#define C03_ISR_MAX_OVER_FAULT_TICKS PROT_OC_LIM_FULL_ISR_NEED_TICKS
#endif
#ifndef C03_MAINLOOP_OC_DEBOUNCE_TICKS
#define C03_MAINLOOP_OC_DEBOUNCE_TICKS PROT_OC_SLOWSTALL_MAIN_NEED_TICKS
#endif

/* START→RUN：端压反馈 + 最少拍（对齐 C03 思路）*/
#ifndef MT_START_FEEDBACK_MIN_TICKS
#define MT_START_FEEDBACK_MIN_TICKS  (15u)
#endif
/* vmin 已到且已过 MIN_TICKS 后，仍未满足 tgt<now（空载皮带松/端压低）时再等此拍（≈100ms/拍）则直接进 RUN，避免误报堵转 */
#ifndef MT_START_FEEDBACK_FALLBACK_TICKS
#define MT_START_FEEDBACK_FALLBACK_TICKS  (40u)
#endif
/* 握手兜底进 RUN：仅当瞬时负载 adc_load 连续低于等于此刻度并满 MT_START_FB_ESC_NEED_TICKS 拍，
 * 才允许破冰；真低速堵转电流偏高时不放行，另见 `error_chick` 内 `checkout_start_stall`。*/
#ifndef MT_START_FB_ESC_LOAD_ADC
#define MT_START_FB_ESC_LOAD_ADC  (CURRENT_10A0)
#endif
#ifndef MT_START_FB_ESC_NEED_TICKS
#define MT_START_FB_ESC_NEED_TICKS  (3u)
#endif

/* 「慢行带」标尺：≤2.0 km/h；堵转阈随全速过流动：MOTOR_STALL_OC_ADC_FROM_FULL(oc_lim_full_mirror)。 */
#ifndef MOTOR_OC_LOW_BAND_SPEED_SCALE_MAX
#define MOTOR_OC_LOW_BAND_SPEED_SCALE_MAX  (200u)
#endif

/* CMD_OVER_CURRENT_MAX：刷新 adjust_ove_curren / oc_lim_full_mirror；堵转阈由全速值按 MOTOR_STALL_OC_ADC_FROM_FULL 推算。 */
#ifndef MOTOR_OVER_CURRENT_ADC_FLOOR
#define MOTOR_OVER_CURRENT_ADC_FLOOR  (CURRENT_10A0)
#endif
#ifndef MOTOR_OVER_CURRENT_ADC_CEIL
#define MOTOR_OVER_CURRENT_ADC_CEIL   (CURRENT_20A0)
#endif
#define OVER_SPEED_TIMEOUT              (200)

/* 判停电压阈值（V）：电机端电压低于此值认为已停稳 */
#define end_set_val  15

/* 起步爬坡目标电压上限（ADC 计数，约 51 V）
 * 防止皮带卡死时爬坡电压无限上升，配合超时 E03 保护 */
#define START_VOLTAGE_MAX_ADC  (500u)

/* 过压窗口参数（POWERON/CMD 兜底等仍可引用 OVER_VOLTAGE_MAX） */
#define OVER_VOLTAGE_OFFSET         (80.0)   /* 过压判断余量（V）（历史参数） */
#define OVER_VOLTAGE_MAX            (240.0)  /* 上电兜底；运行中由 vmax 刷新 over_voltage，见 motor_drive_refresh_over_voltage_from_vmax */
#define OVER_SPEED_TIMEOUT_VOLTAGE  (50)     /* 过速去抖时间（10 ms 单位） */

#ifndef MOTOR_OVERVOLT_MAX_VOLTAGE_PERCENT
#define MOTOR_OVERVOLT_MAX_VOLTAGE_PERCENT   (130u)
#endif
/* E05 母线：valtage_up 相对合闸锁存 motor_vbus_adc 超 MOTOR_BUS_OV_VS_CAPTURE_NUM/DEN（常用 130%/100） */
#ifndef MOTOR_BUS_OV_VS_CAPTURE_NUM
#define MOTOR_BUS_OV_VS_CAPTURE_NUM   (MOTOR_OVERVOLT_MAX_VOLTAGE_PERCENT)
#endif
#ifndef MOTOR_BUS_OV_VS_CAPTURE_DEN
#define MOTOR_BUS_OV_VS_CAPTURE_DEN   (100u)
#endif
/* E08（仅 RUN）：|cut_adc−V_cmd| 超此 ADC 差（与 E05 分立） */
#ifndef MOTOR_ARMATURE_ABNORMAL_DIFF_ADC
#define MOTOR_ARMATURE_ABNORMAL_DIFF_ADC   (317u)
#endif
#ifndef MOTOR_ARMATURE_ABNORMAL_CNT_MAX
#define MOTOR_ARMATURE_ABNORMAL_CNT_MAX    (20u)
#endif
/* 速度斜坡值上限（120 ≈ 12.0 km/h，0.1 km/h 为单位） */
#define MAX_MT_SPEED_SCALE  (120)

/* 错误码枚举（自检 / 批量 C03 对齐说明见下行内注释）
 * E01 通信中断 → 软停机
 * E02 自检：Up/Down 失衡 >70 V，或门控「双轨过低」
 * E03 过流/堵转
 * E04 保留
 * E05 母线过压：监视 M+ raw（valtage_up），相对合闸快照 motor_vbus_adc 超比例（护整流后电容如 CB7）；待机与运行（继电器吸合有效）
 * E06 过速/飞车
 * E07 保留扩展
 * E08 仅 RUN：电枢端压与指令偏离过大 |ΔV|（MOS 击穿、断线等），与 E05 母线逻辑分立
 */

typedef enum _error_code_enum
{
    ERROR_01 = 1,  /* 通信中断，软停机减速 */
    ERROR_02,      /* 自检：轨间失衡或门控双轨过低 */
    ERROR_03,      /* 过流或堵转 */
    ERROR_04,      /* 保留（老码继电器短等） */
    ERROR_05,      /* 母线过压：valtage_up vs motor_vbus_adc 比值 */
    ERROR_06,
    ERROR_07,      /* H1 扩展保留 */
    ERROR_08,      /* RUN：|端压−指令| 过大（电枢异常） */
    ERROR_09,
    ERROR_0A,
    ERROR_0B,
    ERROR_0C,
} error_code_enum;

/* 主状态机枚举（ctr.status 与 motor.status 共用） */
typedef enum _motor_enum
{
    STATUS_POWERON = 1,   /* 上电 ADC 自检（同 C03：E02 + 双阈）*/
    STATUS_RUN,           /* 启动前门控自检（同 C03）→ waitforamoment */
    STATUS_TO_RUN,        /* C03：合继电器 → delay500 → 直接 tim6run，无 ADC 复检门闩 */
    STATUS_STOP,          /* 进入减速斜坡 */
    STATUS_TO_STOP,       /* PWM 关断 → 续流制动 → 断继电器 */
    STATUS_ERROR,         /* 故障处理 */

    STATUS_MT_START,      /* 电机状态：起步 Kick + 爬坡 */
    STATUS_MT_RUN,        /* 电机状态：开环稳速运行（含 IR 前馈补偿） */
    STATUS_MT_STOP,       /* 电机状态：S 型减速停机 */
    STATUS_MT_IMMED_STOP, /* 电机状态：立即停机（保留） */
} motor_enum;

/* u8 status 清屏哨兵：禁止写 (void*)NULL */
#ifndef MOTOR_CTRL_STATUS_CLEARED
#define MOTOR_CTRL_STATUS_CLEARED  (0u)
#endif
#ifndef CTR_STATUS_CLEARED
#define CTR_STATUS_CLEARED       (0u)
#endif

/* 电机运行参数与状态 */
typedef struct _t_motor
{
    u8  status;                 /* 当前电机状态（motor_enum） */
    u8  en;                     /* 使能标志 */
    u8  index;                  /* 当前档位 0~12，对应 speed_param[0..12]（约 12 km/h 封顶） */
    u8  adjust_max_voltage;     /* 上控下发的最高运行电压（显示单位），见 CMD_VOLTAGE_MAX */
    u8  adjust_min_voltage;     /* 上控下发的最低运行电压（伏特刻度）；转 ADC≈ DISPLAY_V×(MOTOR_DISPLAY_VOLT_SCALE_X100/100) */

    u16 adjust_speed_max;       /* 上控下发的最大允许速度（斜坡值） */
    u16 adjust_ove_curren;       /* 与 oc_lim_full_adc 同步（上位缓存字段名遗留）*/
    u16 oc_lim_full_adc;         /* 【最大电流过流】CMD buf[0]×10→ADC门限（与 oc_lim_full_mirror 同步）*/
    u16 oc_lim_slowstall_adc;    /* 【镜像】OVER_CURRENT_MAX_LOW_SPEED，与 oc_lim_full 无关（对齐 C03） */

    u16 set_speed_scale;        /* 目标速度斜坡值（上控指令） */
    u16 cur_speed_scale;        /* 当前速度斜坡值（由加减速步进） */
    u16 sta_speed_scale;        /* 起步初始速度斜坡值 */
    u16 end_speed_scale;        /* 判停目标电压（V） */
    u16 accelerated;            /* 加速步长 */
    u16 deceleration;           /* 减速步长 */
    float voltage_param;        /* 速度→电压线性斜率（dV/d_scale） */

    u16 start_tim;              /* 起步计时器（用于堵转检测） */
    u16 start_fb_cnt;           /* vmin 就绪后握手失败兜底计数（主环 tick）*/
    u16 start_add_val;          /* 起步电压每步增量（ADC） */
    u16 start_set_val;
    u16 start_cur_val;

    u8  adjust_set_voltage;
    u8  adjust_cou_voltage;
    u16 adjust_kiv_voltage;
    u16 adjust_cur_voltage;
    u16 adjust_sta_voltage;     /* 当前起步目标电压（ADC，随爬坡递增） */
    u16 adjust_now_voltage;     /* 当前运行目标电压（ADC） */

    u16   stop_time_late;       /* 停机超时计数（超过 8 次触发断电） */
    float stop_valtage;         /* 停机阶段电压斜坡当前值 */
    float start_valtage;        /* 起步初始电压目标（ADC，可由上控下发） */
    float kiv;                  /* IR 补偿系数（Kcomp），按速度档位缓慢跟踪 */
    u16   kiv_mul_q12;          /* 主环：(round(kiv×1000)×4096/10000)；ISR：(d*mul+2048)>>12 ≡ 原 (d×kiv_milli)/10000 */
    u16   valtage_cur;          /* 电流采样 ADC 值（含零偏，需减 I_offset） */
    u16   valtage_up;           /* M+ 端 ADC 均值（RC 滤波后） */
    u16   valtage_down;         /* M− 端 ADC 均值（RC 滤波后） */
    float motor_valtage;
    float now_current;
    u16   I_offset;             /* 上电零偏校准值（100 次均值，PWM=0 时采样） */
} motor_t;

/* 速度档位参数表（motor.c：13 档 = 最高 12 km/h） */
typedef struct _t_speed_param
{
    float voltage;              /* 该档目标电压（ADC） */
    u16   pwm_max;              /* 保留（开环不限制，由 UK_PWM_MAX 统一约束） */
    u16   pwm_min;
    u8    kiv;                  /* 该档 IR 补偿系数初始值（Kcomp） */
    u8    adjust_max_voltage;   /* 该档最高允许电压 */
    u8    adjust_cont_voltage;  /* 该档持续运行电压 */
    u8    adc_base_curren;      /* 该档空载电流基准（ADC 计数），IR 补偿参考值 */
} speed_param_t;

/* 控制器状态块 */
typedef struct _t_ctr
{
    u8 status;               /* 主状态机状态（motor_enum） */
    u8 error_code;           /* 当前故障码（error_code_enum） */
    u8 ack;                  /* 待上报应答标志 */
    u8 immed_stop_flag;
    u8 immed_stop_voltage_flag;
} ctr_t;

extern ctr_t                ctr;
extern volatile motor_t     motor;
extern speed_param_t        speed_param[];

void motor_speed(void);
extern void ctr_proc_loop(void);
extern void ctr_init(void);
void motor_recalc_voltage_param_from_cmd(void); /* vmin/vmax/smax→voltage_param，防除零 */
extern void stop_motor(void);
extern void set_motor_speed(u8 scale);

/* 软故障：发起减速斜坡，不立即断电 */
extern void motor_soft_fault_set(u8 err_code);
/* 硬故障（1级）：立即关断 PWM 和继电器 */
extern void motor_hard_fault_set(u8 err_code);
/* 紧急制动（2级）：加快减速率后进入停机斜坡（安全开关/急停键） */
extern void motor_emergency_brake_set(u8 err_code);
/* 受控停机（3级）：维持正常减速，通信中断等非致命故障使用 */
extern void motor_controlled_stop_set(u8 err_code);

#endif /* USER_MOTOR_H_ */
