/* motor_speed_pid.c  -  PTFM-1.0  Open-Loop Feedforward Controller
 *
 * DESIGN: "Trajectory Prediction Control"
 *   No voltage PID. Speed scale -> V_target (0.084 slope) -> feedforward PWM
 *   -> Slew-Rate Limiter (+/-1 per ISR tick) -> ZHANKONBI output.
 *
 * Optional soft KIV compensates I*R drop only, hard-limited to <=15% V_target.
 * 110V / 220V platform auto-detected at power-on from DC-bus ADC reading.
 *
 * ISR cadence: ~1.6 kHz  (0.625 ms per tick, TIM6 underflow)
 *
 * Fault coverage (PTFM-1.0 spec):
 *   E08 - inverse-time over-voltage  (score accumulator, threshold 3200)
 *   E03 - leaky-bucket stall / OC    (score accumulator, threshold E03_TRIGGER)
 *   E02 - motor disconnected          (PWM high + no current for 1.0 s)
 *   E06 - saturated + under-voltage  (controlled decel, no fault code)
 *   E05 - absolute over-voltage      (checkout_speed_over in error_chick)
 *   E08 (low-speed) - same as legacy (low_speed_count in error_chick)
 */

#include "pid.h"          /* pid_t / mt_pid still compiled in pid.c; kept for link */
#include "motor.h"
#include "motor_speed_pid.h"
#include "MY_TIM.h"

extern uint16_t ZHANKONBI;

/* ── Globals expected by main.c / motor_speed_pid.h externs ────────────── */
u8  valtage            = 0u;
u16 over_current       = OVER_CURRENT_MAX;
u16 over_voltage       = OVER_VOLTAGE_MAX;
u16 timestest          = 0u;
u16 pwm_delta          = 0u;
u16 voltage_over_error_cnt = 0u;

/* ── Platform state ─────────────────────────────────────────────────────── */
static float   s_vbus_v  = 155.0f;   /* measured DC bus voltage (V)         */
static uint8_t s_is_110v = 1u;       /* 1 = 110 V platform, 0 = 220 V       */

/* ── Slew state ─────────────────────────────────────────────────────────── */
static uint16_t s_slew = MT_START_PWM;  /* current slew output (duty units)  */

/* ── E08 : inverse-time over-voltage accumulator ───────────────────────── */
/* excess ADC units above (V_target + 40 V) accumulated each ISR tick.
 * 40 V = 40 * 7.937 ADC ~ 317 counts.  Trigger at 3200 (~ 10 ms at 40 V). */
static int32_t s_e08_score = 0;
#define E08_TRIGGER   3200

/* ── E03 : leaky-bucket stall / over-current ────────────────────────────── */
/* At 1.6 kHz, 3.0 s light-OC = 4800 ticks * 10 pts / 16 leak ratio ~ 330 pts.
 * Match spec: total bucket = 330 * 16 = 5280.                               */
static int32_t s_e03_score = 0;
#define E03_TRIGGER       (330 * 16)
#define E03_LIGHT_PTS     10
#define E03_HEAVY_PTS     110
#define E03_LEAK          1

/* ── E02 : motor-disconnected watchdog ─────────────────────────────────── */
static uint16_t s_e02_cnt = 0u;
#define E02_PWM_MIN   150u
#define E02_CUR_MAX   8u
#define E02_TICKS     1600u    /* 1.0 s at 1.6 kHz */

/* ═══════════════════════════════════════════════════════════════════════════
 * ptfm_bus_init()
 *   Detect 110 V / 220 V from DC-rail ADC during STATUS_POWERON (motor idle).
 *   Adjusts over_current threshold and sets s_vbus_v for feedforward scaling.
 *   Call once at end of successful POWERON ADC check.
 * ═════════════════════════════════════════════════════════════════════════ */
void ptfm_bus_init(void)
{
    /* valtage_up = positive DC rail, motor PWM off -> direct bus reading */
    float v = CALC_ADC_2_VOLTAGE(motor.valtage_up);

    if (v < 200.0f) {
        /* 110 V system: bus ~ 155 V */
        s_vbus_v  = (v > 50.0f) ? v : 155.0f;
        s_is_110v = 1u;
        /* Lower winding resistance -> higher rated current; raise OC * 1.8 */
        over_current = (u16)((u32)OVER_CURRENT_MAX * 18u / 10u);
        if (over_current > 750u) over_current = 750u;
    } else {
        /* 220 V system: bus ~ 311 V */
        s_vbus_v  = (v > 100.0f) ? v : 311.0f;
        s_is_110v = 0u;
        over_current = OVER_CURRENT_MAX;   /* standard threshold */
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * ptfm_slew_reset()
 *   Reset slew and fault accumulators at every STATUS_TO_RUN entry.
 * ═════════════════════════════════════════════════════════════════════════ */
void ptfm_slew_reset(void)
{
    s_slew      = MT_START_PWM;
    s_e08_score = 0;
    s_e03_score = 0;
    s_e02_cnt   = 0u;
    ZHANKONBI   = MT_START_PWM;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * ptfm_ff_pwm()  -  feedforward duty from speed scale
 *
 *   V_target = 0.084 * (scale - 100) + 18   [V]   (PTFM-1.0 eq.1)
 *   PWM_ff   = V_target / V_bus * UK_PWM_MAX
 *
 *   Clamped to [UK_PWM_MIN, UK_PWM_MAX] and to adjust_max_voltage ceiling.
 * ═════════════════════════════════════════════════════════════════════════ */
static uint16_t ptfm_ff_pwm(uint16_t scale)
{
    float vt;
    uint16_t pwm;

    if (scale < 100u) scale = 100u;
    vt = 0.084f * (float)(scale - 100u) + 18.0f;

    /* Hard ceiling from upper-board parameter (adjust_max_voltage in V) */
    { float vm = (float)motor.adjust_max_voltage; if (vt > vm) vt = vm; }

    pwm = (uint16_t)(vt / s_vbus_v * (float)UK_PWM_MAX + 0.5f);
    if (pwm > (u16)UK_PWM_MAX) pwm = (u16)UK_PWM_MAX;
    if (pwm < (u16)UK_PWM_MIN) pwm = (u16)UK_PWM_MIN;
    return pwm;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * ptfm_kiv_pwm()  -  soft KIV load compensation (extra duty units)
 *
 *   Only compensates I*R winding drop; NOT a speed feedback term.
 *   Dead zone: current must exceed base by > 5 ADC counts.
 *   Hard limit: V_boost <= min(15% of V_target, 10 V).
 *   110 V platform: KIV factor halved (lower-R winding, less compensation needed).
 * ═════════════════════════════════════════════════════════════════════════ */
static uint16_t ptfm_kiv_pwm(motor_t mot, uint16_t scale)
{
    uint8_t  idx = (uint8_t)(scale / 100u);
    int16_t  exc;
    float    kiv, vb, vt, vm;
    uint16_t pb;

    if (idx > 12u) idx = 12u;
    exc = (int16_t)mot.valtage_cur - (int16_t)speed_param[idx].adc_base_curren;
    if (exc <= 5) return 0u;   /* dead zone - no compensation, prevents idle oscillation */

    kiv = (float)speed_param[idx].kiv;
    if (s_is_110v) kiv *= 0.5f;          /* 110 V: halve KIV factor */

    vb = (float)exc * kiv / 10.0f;       /* raw V_boost (V) */

    /* Limit: <= 15% of V_target OR 10 V (whichever is less) */
    vt = 0.084f * (float)(scale - 100u) + 18.0f;
    vm = vt * 0.15f;
    if (vm > 10.0f) vm = 10.0f;
    if (vb > vm) vb = vm;

    pb = (uint16_t)(vb / s_vbus_v * (float)UK_PWM_MAX + 0.5f);
    return pb;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * fault_e08()  -  inverse-time runaway detector
 *   Score accumulates proportionally to (actual - target - 40V).
 *   Decays when within limit.  Trigger: relay cutoff + STATUS_ERROR.
 * ═════════════════════════════════════════════════════════════════════════ */
static void fault_e08(int16_t cut_adc, uint16_t scale)
{
    /* Convert V_target to actual-ADC scale (7.937 counts/V) */
    float   vt  = 0.084f * (float)(scale - 100u) + 18.0f;
    int16_t tgt = (int16_t)(vt * 7.937f);
    /* 40 V margin = 40 * 7.937 = 317 ADC counts */
    int32_t exc = (int32_t)cut_adc - (int32_t)tgt - 317;

    if (exc > 0) {
        s_e08_score += exc;
        if (s_e08_score >= E08_TRIGGER) {
            s_e08_score = 0;
            /* First action: cut PWM + relay (tim0 drives relay on this board) */
            tim0stop();
            tim6stop();
            ctr.error_code = ERROR_08;
            ctr.status     = STATUS_ERROR;
        }
    } else {
        s_e08_score -= 10;
        if (s_e08_score < 0) s_e08_score = 0;
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * fault_e03()  -  leaky-bucket stall / over-current
 *   Light OC (+10/tick), severe OC 1.5x (+110/tick), leaks 1/tick below thresh.
 *   Trigger at E03_TRIGGER => stop + E03.
 * ═════════════════════════════════════════════════════════════════════════ */
static void fault_e03(void)
{
    u16 heavy = (u16)((u32)over_current * 15u / 10u);   /* 1.5x rated */

    if (motor.valtage_cur > heavy) {
        s_e03_score += E03_HEAVY_PTS;
    } else if (motor.valtage_cur > over_current) {
        s_e03_score += E03_LIGHT_PTS;
    } else {
        s_e03_score -= E03_LEAK;
        if (s_e03_score < 0) s_e03_score = 0;
    }

    if (s_e03_score >= E03_TRIGGER) {
        s_e03_score = 0;
        tim0stop();
        tim6stop();
        ctr.error_code = ERROR_03;
        ctr.status     = STATUS_ERROR;
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * fault_e02()  -  motor disconnected (voltage present, no current)
 *   ZHANKONBI > 150 AND current_ADC < 8 for 1.0 s => E02.
 * ═════════════════════════════════════════════════════════════════════════ */
static void fault_e02(void)
{
    if (ZHANKONBI > E02_PWM_MIN && motor.valtage_cur < E02_CUR_MAX) {
        if (++s_e02_cnt >= E02_TICKS) {
            s_e02_cnt = 0u;
            tim0stop();
            tim6stop();
            ctr.error_code = ERROR_02;
            ctr.status     = STATUS_ERROR;
        }
    } else {
        s_e02_cnt = 0u;
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * motor_speed_pid_isr()  -  PTFM-1.0 open-loop slew controller
 *   Called at ~1.6 kHz from TIM6 underflow ISR.
 *   No PID integral, no voltage feedback loop.
 * ═════════════════════════════════════════════════════════════════════════ */
void motor_speed_pid_isr(motor_t mot)
{
    int16_t  cut_adc;
    uint16_t pwm_tgt;

    cut_adc = (int16_t)mot.valtage_up - (int16_t)mot.valtage_down;
    if (cut_adc < 0) cut_adc = 0;

    switch (motor.status)
    {

    /* ── STATUS_MT_START : soft-start ramp until back-EMF detected ─────── */
    case STATUS_MT_START:
        motor.stop_valtage = motor.adjust_min_voltage;

        /* Target = 1 km/h feedforward voltage (18 V) */
        pwm_tgt = ptfm_ff_pwm(100u);

        /* Slew +1 per tick toward start target; never go above it here */
        if (s_slew < pwm_tgt)      s_slew++;
        else if (s_slew > pwm_tgt) s_slew = pwm_tgt;

        ZHANKONBI = s_slew;

        /* Transition to run when slew reached target AND motor is rotating
         * (back-EMF ADC > 20 counts ~ 2.5 V indicates belt moving)          */
        if (s_slew >= pwm_tgt && cut_adc > 20) {
            motor.status     = STATUS_MT_PID;
            motor.start_tim  = 0u;
        }
        break;

    /* ── STATUS_MT_PID : feedforward + soft KIV, slew +/-1 per tick ───── */
    case STATUS_MT_PID:
    {
        uint16_t pff = ptfm_ff_pwm(motor.cur_speed_scale);
        uint16_t pkv = ptfm_kiv_pwm(mot, motor.cur_speed_scale);

        pwm_tgt = pff + pkv;
        if (pwm_tgt > (u16)UK_PWM_MAX) pwm_tgt = (u16)UK_PWM_MAX;
        if (pwm_tgt < (u16)UK_PWM_MIN) pwm_tgt = (u16)UK_PWM_MIN;

        /* Slew +-1 per ISR tick toward computed target */
        if      (s_slew < pwm_tgt) s_slew++;
        else if (s_slew > pwm_tgt) s_slew--;

        ZHANKONBI = s_slew;

        /* E06: PWM saturated AND bus voltage below 94% of target
         * => under-voltage while fully loaded; do controlled deceleration   */
        if (s_slew >= (u16)UK_PWM_MAX) {
            float v_act = CALC_ADC_2_VOLTAGE((u16)cut_adc);
            float v_tgt = 0.084f * (float)(motor.cur_speed_scale - 100u) + 18.0f;
            if (v_act < v_tgt * 0.94f && motor.cur_speed_scale > 100u) {
                motor.cur_speed_scale -= (u16)motor.deceleration;
            }
        }

        /* Fault checks - active only in run phase */
        fault_e08(cut_adc, motor.cur_speed_scale);
        fault_e02();
        fault_e03();
    }
    break;

    /* ── STATUS_MT_STOP : slew down, no KIV, handoff to STATUS_TO_STOP ─── */
    case STATUS_MT_STOP:
    {
        /* Track declining feedforward (stop_valtage shrinks in motor_speed()) */
        pwm_tgt = ptfm_ff_pwm(motor.cur_speed_scale);
        /* Never ramp up during stop phase */
        if (pwm_tgt > s_slew) pwm_tgt = s_slew;

        if (s_slew > pwm_tgt)      s_slew--;
        else if (s_slew > MT_START_PWM) { /* hold minimum */}

        ZHANKONBI = s_slew;

        /* Legacy stop detection: back-EMF below END_speed_scale threshold */
        valtage = (u8)(0.126f * (float)cut_adc);
        if ((motor.END_speed_scale >= valtage) || (motor.stop_time_late > 8u)) {
            motor.stop_time_late = 0u;
            motor.status = NULL;
            ctr.status   = STATUS_TO_STOP;
            s_slew       = MT_START_PWM;
            ZHANKONBI    = MT_START_PWM;
        }
    }
    break;

    default:
        break;
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * error_chick()  -  called every ~91 ms from main loop
 *   Legacy absolute-threshold checks (not ISR-bound).
 * ═════════════════════════════════════════════════════════════════════════ */
void error_chick(void)
{
    static u8 lspd_cnt = 0u;
    static u8 ov_cnt   = 0u;

    /* Low-speed stall check (E08, same as legacy behavior) */
    if ((motor.status == STATUS_MT_PID || motor.status == STATUS_MT_STOP)
        && motor.cur_speed_scale <= 200u)
    {
        if (motor.valtage_cur > OVER_CURRENT_MAX_LOW_SPEED) {
            if (++lspd_cnt > 20u) {
                lspd_cnt = 0u;
                tim0stop();
                tim6stop();
                ctr.error_code = ERROR_08;
                ctr.status     = STATUS_ERROR;
            }
        } else lspd_cnt = 0u;
    } else lspd_cnt = 0u;

    /* Absolute over-voltage / runaway (E05) */
    if (motor.status == STATUS_MT_PID || motor.status == STATUS_MT_STOP) {
        int val = (int)motor.valtage_up - (int)motor.valtage_down;
        if (val < 0) val = 0;
        {
            int val_v = (int)CALC_ADC_2_VOLTAGE(val);
            if (val_v >= (int)over_voltage) ov_cnt++;
            else                             ov_cnt = 0u;
        }
        if (ov_cnt > 15u) {
            ov_cnt = 0u;
            tim0stop();
            tim6stop();
            ctr.error_code = ERROR_05;
            ctr.status     = STATUS_ERROR;
        }
    } else ov_cnt = 0u;
}
