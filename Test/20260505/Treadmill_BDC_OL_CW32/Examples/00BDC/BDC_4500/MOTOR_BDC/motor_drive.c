/* motor_drive.c — 开环驱动 + IR；E03 与 C03 同类逻辑。
 * 全速过流：oc_lim_full_mirror（≈上位 CMD 全速阈）；慢行/起步堵转：全速 ×(410/1024)≈全速÷2.5。
 * E05 本板为母线比值法 checkout_bus_overvoltage_e05。 */

#include "motor.h"
#include "motor_drive.h"
#include "user_timer.h"
#include "bdc_hal.h"

/* 与旧版/精简 motor.h 共存：缺省时补齐 motor_drive.c 所需宏（若 motor.h 已定义则不受影响）。 */
#ifndef MOTOR_KIV_SOFT_BLEND_END_BELOW_SCALE
#define MOTOR_KIV_SOFT_BLEND_END_BELOW_SCALE (300u)
#endif
#ifndef MOTOR_KIV_SOFT_BLEND_AT_END_Q12
#define MOTOR_KIV_SOFT_BLEND_AT_END_Q12 (2048u)
#endif
#ifndef MOTOR_KIV_SOFT_BLEND_FULL_AT_SCALE
#define MOTOR_KIV_SOFT_BLEND_FULL_AT_SCALE (430u)
#endif
#ifndef MOTOR_KIV_BOOST_CAP_LOWSPD_ADC
#define MOTOR_KIV_BOOST_CAP_LOWSPD_ADC (165)
#endif
#ifndef MOTOR_KIV_BOOST_CAP_RUN_ADC
#define MOTOR_KIV_BOOST_CAP_RUN_ADC (300)
#endif
#ifndef CURRENT_20A0
#define CURRENT_20A0 (548u)
#endif
#ifndef OVER_CURRENT_MAX
#define OVER_CURRENT_MAX (CURRENT_20A0)
#endif
#ifndef MOTOR_STALL_OC_Q_BITS
#define MOTOR_STALL_OC_Q_BITS (10u)
#endif
#ifndef MOTOR_STALL_OC_Q_MUL
#define MOTOR_STALL_OC_Q_MUL (410u)
#endif
#ifndef MOTOR_STALL_OC_ADC_FROM_FULL
#define MOTOR_STALL_OC_ADC_FROM_FULL(lim_) \
    ((u16)(((u32)(lim_) * (u32)MOTOR_STALL_OC_Q_MUL) >> (u32)MOTOR_STALL_OC_Q_BITS))
#endif
#ifndef OVER_CURRENT_MAX_LOW_SPEED
#define OVER_CURRENT_MAX_LOW_SPEED (MOTOR_STALL_OC_ADC_FROM_FULL(OVER_CURRENT_MAX))
#endif
#ifndef PROT_OC_LIM_FULL_ISR_NEED_TICKS
#define PROT_OC_LIM_FULL_ISR_NEED_TICKS (200u)
#endif
#ifndef PROT_OC_SLOWSTALL_MAIN_NEED_TICKS
#define PROT_OC_SLOWSTALL_MAIN_NEED_TICKS (20u)
#endif
#ifndef C03_START_STALL_MAINLOOP_TICKS
#define C03_START_STALL_MAINLOOP_TICKS (60u)
#endif
#ifndef MOTOR_OC_LOW_BAND_SPEED_SCALE_MAX
#define MOTOR_OC_LOW_BAND_SPEED_SCALE_MAX (200u)
#endif

extern uint16_t ZHANKONBI;

u8               valtage;
volatile u16    oc_lim_full_mirror = OVER_CURRENT_MAX; /* 与 C03 `over_current` 同源语义；CMD 后经 uart/boot 刷新 */
volatile u16    over_voltage       = (u16)(OVER_VOLTAGE_MAX + 0.5f);
u32             motor_vbus_adc     = 2468u;

#ifndef MOTOR_VBUS_ADC_LIN_BOOT_DEN
#define MOTOR_VBUS_ADC_LIN_BOOT_DEN (2468u)
#endif

static u32 s_vbus_duty_lin_mul =
    (((u32)MOTOR_PWM_PERIOD_TICKS << MOTOR_VBUS_DUTY_LIN_SHIFT) / MOTOR_VBUS_ADC_LIN_BOOT_DEN);

static void motor_drive_refresh_vbus_duty_lin(void);

/* 主环 E03：用「减零偏」负载电流，与 motor.c mt_start_adc_load 一致；避免空载时 ADC 直流偏置叠在阈值上误报。 */
static u16 motor_adc_load_minus_offset(void)
{
    if (motor.valtage_cur > motor.I_offset)
        return (u16)(motor.valtage_cur - motor.I_offset);
    return 0u;
}

/* 主环 E03（慢行/START）：阈 = 当前全速过流 oc_lim_full_mirror ÷2.5，整数用 (lim×410)>>10 */
static u16 stall_oc_trip_adc(void)
{
    return MOTOR_STALL_OC_ADC_FROM_FULL(oc_lim_full_mirror);
}

static u16 motor_kiv_soft_blend_q12_under3k(void)
{
    u16 cs   = motor.cur_speed_scale;
    u16 end  = MOTOR_KIV_SOFT_BLEND_END_BELOW_SCALE;
    u16 full = MOTOR_KIV_SOFT_BLEND_FULL_AT_SCALE;
    u16 bl   = MOTOR_KIV_SOFT_BLEND_AT_END_Q12;

    if (full <= end || cs == 0u)
        return 0u;
    if (cs < end) {
        return (u16)(((u32)cs * (u32)bl + ((u32)end >> 1u)) / (u32)end);
    }
    if (cs >= full)
        return 4096u;

    {
        u32 den = (u32)(full - end);
        u32 num =
            (u32)(cs - end) * (4096u - (u32)bl) + (den >> 1u);

        return (u16)(bl + num / den);
    }
}

void motor_drive_refresh_over_voltage_from_vmax(void)
{
    u8 vmax = motor.adjust_max_voltage;

    if (vmax != 0u)
        over_voltage = (u16)vmax;
}

void motor_drive_boot_sync_limits(void)
{
    /* 仅用 adjust_ove_curren / oc_lim_full_mirror，避免依赖 motor 结构体中的 oc_lim_* 字段（旧头文件无该成员）。 */
    oc_lim_full_mirror = motor.adjust_ove_curren;
    motor_drive_refresh_over_voltage_from_vmax();
    motor_drive_refresh_vbus_duty_lin();
}

static void motor_drive_refresh_vbus_duty_lin(void)
{
    u32 d = motor_vbus_adc;

    if (d < 800u)
        d = 800u;
    s_vbus_duty_lin_mul =
        (((u32)MOTOR_PWM_PERIOD_TICKS << MOTOR_VBUS_DUTY_LIN_SHIFT) + (d >> 1u)) / d;
}

static u16 voltage_adc_to_pwm_ticks_lin(u32 v_adc)
{
    u32 dv = (v_adc * s_vbus_duty_lin_mul) >> MOTOR_VBUS_DUTY_LIN_SHIFT;

    if (dv > (u32)UK_PWM_MAX)
        dv = UK_PWM_MAX;
    if (dv > 0u && dv < (u32)MT_START_ABSMIN_DUTY)
        dv = (u32)MT_START_ABSMIN_DUTY;
    return (u16)dv;
}

static u32 s_I_lp_acc = 0u;

static u8  s_ov_cnt   = 0u;
static u8  s_ms_cnt   = 0u;
static u16 s_prot_oc_lim_full_isr_cnt = 0u;
static u32 s_v_exec_q8                = 0u;

#define RUN_E06_ARM_DELAY_ISR ((MOTOR_TIM6_ISR_HZ_APPROX) * (2u))

static u16 s_run_ov_arm_isr = RUN_E06_ARM_DELAY_ISR;

static void pwm_publish_duty(u16 tgt)
{
    ZHANKONBI = tgt;
}

void reset_current_lp_filter(void)
{
    u32 vq;

    s_I_lp_acc                 = 0u;
    s_ov_cnt                   = 0u;
    s_ms_cnt                   = 0u;
    s_prot_oc_lim_full_isr_cnt = 0u;
    s_run_ov_arm_isr           = RUN_E06_ARM_DELAY_ISR;
    vq                         = (u32)motor.adjust_now_voltage << 8u;
    s_v_exec_q8                = vq;
}

/* C03：`Current_Max_Over()` — ISR 瞬时 valtage_cur vs `over_current`（此处镜像 oc_lim_full_mirror） */
static void Current_Max_Over(void)
{
    if (motor.valtage_cur > oc_lim_full_mirror) {
        if (++s_prot_oc_lim_full_isr_cnt > PROT_OC_LIM_FULL_ISR_NEED_TICKS) {
            s_prot_oc_lim_full_isr_cnt = 0u;
            motor_hard_fault_set(ERROR_03);
        }
    } else {
        s_prot_oc_lim_full_isr_cnt = 0u;
    }
}

static void checkout_bus_overvoltage_e05(void)
{
    static u8 cnt;
    u32       thr;
    u16       vu;

    if (ctr.error_code != 0u) {
        cnt = 0u;
        return;
    }
    if (bdc_relay_on_readback() == 0u) {
        cnt = 0u;
        return;
    }
    vu = motor.valtage_up;
    if (vu < 800u || vu > 4000u) {
        cnt = 0u;
        return;
    }
    if (motor_vbus_adc < 800u) {
        cnt = 0u;
        return;
    }

    thr =
        (motor_vbus_adc * (u32)MOTOR_BUS_OV_VS_CAPTURE_NUM) / (u32)MOTOR_BUS_OV_VS_CAPTURE_DEN;

    if ((u32)vu > thr)
        cnt++;
    else
        cnt = 0u;
    if (cnt > 15u) {
        cnt = 0u;
        motor_hard_fault_set(ERROR_05);
    }
}

#define VOLT_OVERSHOOT_DIFF (159u)
#define VOLT_OVERSHOOT_CNT  (60u)

static u16 voltage_to_duty(int32_t v_adc)
{
    if (v_adc < 0)
        v_adc = 0;
    return voltage_adc_to_pwm_ticks_lin((u32)v_adc);
}

void motor_drive_on_to_run_capture_bus(u16 valtage_up_adc)
{
    u32 v = 2468u;

    if (valtage_up_adc >= 800u && valtage_up_adc <= 4000u)
        v = valtage_up_adc;

    motor_vbus_adc = v;
    motor_drive_refresh_vbus_duty_lin();
}

#ifndef MOTOR_BUS_CAPTURE_AVG_SAMPLES
#define MOTOR_BUS_CAPTURE_AVG_SAMPLES (20u)
#endif
#ifndef MOTOR_BUS_CAPTURE_AVG_GAP_MS
#define MOTOR_BUS_CAPTURE_AVG_GAP_MS (1u)
#endif

void motor_drive_on_to_run_capture_bus_average(void)
{
    u8  i;
    u32 sum = 0u;
    u8  n   = 0u;

    for (i = 0u; i < (u8)MOTOR_BUS_CAPTURE_AVG_SAMPLES; i++) {
        delay1ms((u32)MOTOR_BUS_CAPTURE_AVG_GAP_MS);
        {
            u16 vu = motor.valtage_up;

            if (vu >= 800u && vu <= 4000u) {
                sum += (u32)vu;
                n++;
            }
        }
    }
    if (n > 0u) {
        u16 avg = (u16)((sum + (u32)n / 2u) / (u32)n);

        motor_drive_on_to_run_capture_bus(avg);
    } else
        motor_drive_on_to_run_capture_bus(0u);
}

static u16 motor_v_exec_apply_microstep_q8(void)
{
    u32     tgt_q8 = (u32)motor.adjust_now_voltage << 8u;
    int32_t sg     = (int32_t)tgt_q8;
    int32_t se     = (int32_t)s_v_exec_q8;
    int32_t err    = sg - se;
    int32_t lim    = (int32_t)MOTOR_MICRO_V_Q8_SLEW_CAP;

    if (err > lim)
        err = lim;
    else if (err < -lim)
        err = -lim;
    se += err;
    if (se < 0)
        se = 0;
    s_v_exec_q8 = (u32)se;
    return (u16)(s_v_exec_q8 >> 8);
}

void motor_drive_isr(void)
{
    int16_t cut_adc;
    u16     I_now;
    u16     v_exec;

    cut_adc = (int16_t)motor.valtage_up - (int16_t)motor.valtage_down;
    if (cut_adc < 0) cut_adc = 0;

    {
        u16 raw = (motor.valtage_cur > motor.I_offset) ?
                  (motor.valtage_cur - motor.I_offset) : 0u;
        s_I_lp_acc = s_I_lp_acc - (s_I_lp_acc >> 12u) + raw;
        I_now      = (u16)(s_I_lp_acc >> 12u);
    }

    v_exec = motor.adjust_now_voltage;
    if (motor.status == STATUS_MT_START || motor.status == STATUS_MT_RUN ||
        motor.status == STATUS_MT_STOP)
        v_exec = motor_v_exec_apply_microstep_q8();

    switch (motor.status) {

    case STATUS_MT_START:
    {
        int32_t duty;
        motor.stop_valtage = motor.adjust_min_voltage;

        duty = (int32_t)voltage_adc_to_pwm_ticks_lin((u32)v_exec);
        if (duty < (int32_t)MT_START_ABSMIN_DUTY) duty = (int32_t)MT_START_ABSMIN_DUTY;
        if (duty > (int32_t)UK_PWM_MAX) duty = (int32_t)UK_PWM_MAX;
        pwm_publish_duty((u16)duty);
        break;
    }

    case STATUS_MT_RUN:
    {
        u16     I_noload = speed_param[motor.index].adc_base_curren;
        int32_t kiv_comp = 0;

        if (I_now > I_noload) {
            int32_t d = (int32_t)I_now - (int32_t)I_noload;

            kiv_comp = (d * (int32_t)motor.kiv_mul_q12 + (1 << 11)) >> 12;
            if (kiv_comp > (int32_t)MOTOR_KIV_COMP_MAX_ADC_RUN)
                kiv_comp = (int32_t)MOTOR_KIV_COMP_MAX_ADC_RUN;
        }
        kiv_comp = (kiv_comp * (int32_t)motor_kiv_soft_blend_q12_under3k()) >> 12;

        int32_t V_corr = (int32_t)v_exec + kiv_comp;
        {
            int32_t vboost = (int32_t)MOTOR_KIV_BOOST_CAP_RUN_ADC;

            if (motor.cur_speed_scale < MOTOR_KIV_SOFT_BLEND_FULL_AT_SCALE)
                vboost = (int32_t)MOTOR_KIV_BOOST_CAP_LOWSPD_ADC;
            if (V_corr > (int32_t)v_exec + vboost)
                V_corr = (int32_t)v_exec + vboost;
        }

        pwm_publish_duty(voltage_to_duty(V_corr));

        Current_Max_Over();

        {
            u16 av  = (u16)cut_adc;
            u16 cmd = (u16)(V_corr < 0 ? 0 : V_corr);
            u32 dev = (av >= cmd) ? (u32)((u32)av - (u32)cmd) : (u32)((u32)cmd - (u32)av);

            if (s_run_ov_arm_isr > 0u) {
                s_run_ov_arm_isr--;
                s_ov_cnt = 0u;
            } else if (av > cmd + VOLT_OVERSHOOT_DIFF) {
                if (++s_ov_cnt >= VOLT_OVERSHOOT_CNT) {
                    s_ov_cnt = 0u;
                    motor_hard_fault_set(ERROR_06);
                }
            } else {
                s_ov_cnt = 0u;
            }

            if (dev > (u32)MOTOR_ARMATURE_ABNORMAL_DIFF_ADC) {
                if (++s_ms_cnt >= MOTOR_ARMATURE_ABNORMAL_CNT_MAX) {
                    s_ms_cnt = 0u;
                    motor_hard_fault_set(ERROR_08);
                }
            } else {
                s_ms_cnt = 0u;
            }
        }

        break;
    }

    case STATUS_MT_STOP:
    {
        /* 下控停机降速：不叠 IR/kiv 电流补偿，按电压斜坡 v_exec 直接出占空，避免减速段被补偿抬高 */
        pwm_publish_duty(voltage_adc_to_pwm_ticks_lin((u32)v_exec));

        valtage = (u8)MOTOR_CUT_ADC_DELTA_TO_DISP_VOLT_FLT(cut_adc);
        if ((motor.end_speed_scale >= valtage) || (motor.stop_time_late > 8u)) {
            motor.stop_time_late = 0u;
            motor.status         = NULL;
            ctr.status           = STATUS_TO_STOP;
            pwm_publish_duty(MT_START_PWM);
        }
        if (motor.cur_speed_scale > 300u)
            Current_Max_Over();
        break;
    }

    default:
        break;
    }
}

#define OVERSHOOT_MARGIN  (200u)
#define OVERSHOOT_CNT_MAX (5u)
static void checkout_speed_overshoot(void)
{
    static u8 cnt;
    if (motor.set_speed_scale > 0u &&
        motor.cur_speed_scale >= (motor.set_speed_scale + OVERSHOOT_MARGIN)) {
        if (++cnt > OVERSHOOT_CNT_MAX) { cnt = 0; motor_hard_fault_set(ERROR_06); }
    } else { cnt = 0; }
}

#define OVER_SPEED_CNT_MAX (10u)
static void checkout_over_speed_scale(void)
{
    static u8 cnt;
    if (motor.cur_speed_scale > (motor.adjust_speed_max + 100u)) {
        if (++cnt > OVER_SPEED_CNT_MAX) { cnt = 0; motor_hard_fault_set(ERROR_06); }
    } else { cnt = 0; }
}

/* 慢行带：cur_speed_scale≤200；堵转阈 = 全速过流÷2.5（整数近似） */
static void checkout_current_over(void)
{
    static u8 low_speed_count;

    if (motor.cur_speed_scale <= MOTOR_OC_LOW_BAND_SPEED_SCALE_MAX) {
        if (motor_adc_load_minus_offset() > stall_oc_trip_adc())
            low_speed_count++;
        else
            low_speed_count = 0u;

        if (low_speed_count > PROT_OC_SLOWSTALL_MAIN_NEED_TICKS) {
            low_speed_count = 0u;
            motor_hard_fault_set(ERROR_03);
        }
    }
}

/* C03：`checkout_current_over_start()` */
static void checkout_current_over_start(void)
{
    static u8 start_oc_cnt;

    if (motor_adc_load_minus_offset() > stall_oc_trip_adc()) {
        if (++start_oc_cnt > PROT_OC_SLOWSTALL_MAIN_NEED_TICKS) {
            start_oc_cnt = 0u;
            motor_hard_fault_set(ERROR_03);
        }
    } else {
        start_oc_cnt = 0u;
    }
}

/* C03：`checkout_start_stall()`，`START_STALL_TIMEOUT`=60 */
static void checkout_start_stall(void)
{
    if (motor.start_tim > C03_START_STALL_MAINLOOP_TICKS)
        motor_hard_fault_set(ERROR_03);
}

/* 与 HA380A-BDC_C03 `motor_speed_pid.c` 中 `error_chick()` 一致；STATUS_MT_PID→STATUS_MT_RUN。 */
void error_chick(void)
{
    if (motor.status == STATUS_MT_START) {
        checkout_current_over_start();
        checkout_start_stall();
    }
    if (motor.status == STATUS_MT_RUN || motor.status == STATUS_MT_STOP) {
        checkout_current_over();
        checkout_bus_overvoltage_e05();
    }
    if (motor.status == STATUS_MT_RUN || motor.status == STATUS_MT_START) {
        checkout_over_speed_scale();
    }
    if (motor.status == STATUS_MT_RUN) {
        checkout_speed_overshoot();
    }
}
