/* motor.c  电机状态机、速度斜坡、故障处理（110 V AC / 低档母线段试制）
 *
 * 全程纯开环（无 PID），四阶段与业务对齐：
 *   A POWERON/RUN ：同 C03 自检
 *   A STATUS_TO_RUN：母线锁存；v_ramp := start_kick；合继电器→500 ms→PWM
 *   B MT_START ：每 ~100 ms 一单步抬高 v_ramp 至不低 vmin×速度轮廓（≈ TM_LOWER_VOLTAGE_MIN@1 km）；禁止 ISR 跟踪母线与 IR
 *   C MT_RUN   ：v_ramp 逼近指令电压；允许 IR；母线分母恒为合闸快照，不做实时母线计算
 *   D MT_STOP  ：v_ramp 按减速档下行，近 v_run_min 步长减半→ISR 触 TO_STOP → TO_STOP 断电
 *   TO_STOP    ：PWM 关 → 续流 → 继电器断
 */
#include "motor.h"
#include "user_timer.h"
#include "user_adc.h"
#include "uart_frame.h"
#include "motor_drive.h"

extern uint16_t ZHANKONBI;  /* 逻辑斩波量：越大越弱输出；ISR 写比较值用 OUT_PUT=PER−ZHANKONBI */
extern u8       waitforamoment;

ctr_t   ctr;    /* 控制器状态机（主状态） */
volatile motor_t motor;  /* volatile：主环与 ADC 节拍 ISR 共享，避免优化掉关键序 */

u16 dynamic_e02_limit_v        = E02_LIMIT_110V_V;
u16 dynamic_e05_abs_volt_limit = E05_ABS_VOLT_110V;
u8  motor_grid_profile_known   = 0u;
u8  motor_grid_env_220         = 0u;

/* MT_START 握手兜底：连续低负载拍数计数（瞬时减零偏，与 motor_drive 口径一致） */
static u8 s_start_fb_esc_ok;

static u16 mt_start_adc_load(void)
{
    if (motor.valtage_cur > motor.I_offset)
        return (u16)(motor.valtage_cur - motor.I_offset);
    return 0u;
}

/* 紧急制动减速速率（较大，让电机快速停下） */
#define FAST_BRAKE_DECEL_LEVEL   (25u)
/* 软停止减速速率（与正常减速相同，平稳停机） */
#define SOFT_STOP_DECEL_LEVEL    (speed_step_END)

/* 各速度档位参数表：13 行 ≈ 1~12 km/h，与 HA380A-BDC_C03 motor.c speed_param 前 13 行一致
 *（GET 截距 18、kiv 2~3）；运行中仍可由 CMD_KIV_xKM 覆盖各档 kiv。 */
speed_param_t  speed_param[13] =
{
    {GET_SPEED_VOLTAGE(100,  0.14f, 18), 1000, 1000, 2, 30, 4, 23},
    {GET_SPEED_VOLTAGE(100,  0.14f, 18), 1000, 1000, 2, 30, 4, 23},
    {GET_SPEED_VOLTAGE(200,  0.14f, 18), 1000, 1000, 3, 30, 4, 23},
    {GET_SPEED_VOLTAGE(300,  0.14f, 18), 1000, 1000, 3, 30, 4, 30},
    {GET_SPEED_VOLTAGE(400,  0.14f, 18), 1000, 1000, 3, 25, 5, 30},
    {GET_SPEED_VOLTAGE(500,  0.14f, 18), 1000, 1000, 3, 25, 6, 30},
    {GET_SPEED_VOLTAGE(600,  0.14f, 18), 1000, 1000, 2, 25, 7, 30},
    {GET_SPEED_VOLTAGE(700,  0.14f, 18), 1000, 1000, 2, 20, 7, 40},
    {GET_SPEED_VOLTAGE(800,  0.14f, 18), 1000, 1000, 2, 20, 7, 40},
    {GET_SPEED_VOLTAGE(900,  0.14f, 18), 1000, 1000, 2, 20, 3, 30},
    {GET_SPEED_VOLTAGE(1000, 0.14f, 18), 1000, 1000, 2, 20, 8, 33},
    {GET_SPEED_VOLTAGE(1100, 0.14f, 18), 1000, 1000, 2, 20, 8, 35},
    {GET_SPEED_VOLTAGE(1200, 0.14f, 18), 1000, 1000, 2, 20, 8, 35},
};

/* 初始化控制器和电机默认参数 */
void ctr_init(void)
{
    ctr.status = STATUS_POWERON;

    motor.stop_time_late     = 0;
    motor.start_valtage      = 10;    /* 起步初始电压目标（ADC） */
    motor.adjust_sta_voltage = 10;
    motor.start_add_val      = 5;     /* START 斜坡步（ADC）；主循环破冰节拍≈MT_START ISR 阈值 */
    motor.sta_speed_scale    = 100;    /* 1.0 km/h（内部=100）；与载荷 10 一致 */
    motor.end_speed_scale    = end_set_val;     /* 判停目标电压（V） */
    motor.accelerated        = speed_step_ACC;  /* 加速步长 */
    motor.deceleration       = speed_step_END;  /* 正常减速步长 */
    motor.adjust_ove_curren  = OVER_CURRENT_MAX;
    motor.adjust_max_voltage = 90;   /* CMD_VOLTAGE_MAX 默认（显示刻度 ·V）*/
    motor.adjust_min_voltage = 20;   /* CMD_VOLTAGE_MIN 默认（V）→ 1 km/h 轮廓截距（与 treadmills TM_LOWER_VOLTAGE_MIN 一致） */
    motor.adjust_speed_max   = 1000; /* 上位「100」载荷=10 km/h → ×10 后为 1000 */
    motor.I_offset           = 0;    /* 上电零偏校准结果（100 次均值） */
    motor_grid_profile_known = 0u;
    motor_grid_env_220       = 0u;
    /* 电压–速度线性：90V@10km − vmin@1km，分母 adjust_speed_max−100 */
    motor_recalc_voltage_param_from_cmd();
    motor_drive_boot_sync_limits();
}

/* vmax、vmin、smax 任一异常时跳过除法，斜率退化为 0（GET_SPEED_VOLTAGE 等价于只用常数项 B）。
 * 避免出现 adjust_speed_max==100→(smax−100)==0 的浮点除零。*/
void motor_recalc_voltage_param_from_cmd(void)
{
    u16 dscale;
    int dv;

    if (motor.adjust_speed_max > 100u)
        dscale = (u16)(motor.adjust_speed_max - 100u);
    else
        dscale = 0u;

    dv = (int)motor.adjust_max_voltage - (int)motor.adjust_min_voltage;

    if (dscale == 0u || dv <= 0) {
        motor.voltage_param = 0.0f;
        return;
    }

    motor.voltage_param = ((float)dv) / (float)dscale;
}

/* kiv 只在主环改写；kiv_mul_q12 供 ADC 节拍用 (d×mul+2048)>>12 等价原 (d×kiv_milli+5000)/10000，无软浮点且无慢除法 */
static void motor_kiv_sync_isr_mul(void)
{
    u32 m;

    if (!(motor.kiv > 0.0f)) {
        motor.kiv_mul_q12 = 0u;
        return;
    }
    m = (u32)(motor.kiv * 1000.0f + 0.5f);
    if (m > 10000u)
        m = 10000u;
    motor.kiv_mul_q12 = (u16)((m * 4096u + 5000u) / 10000u);
}

/* 上控下发的最低伏特刻度 → CMD 标尺 ADC（与 RUN 路径 MOTOR_DISPLAY_VOLT_TF_TO_ADC_CMD_FLT 对齐）*/

u16 motor_cmd_min_volt_adc_floor(void)
{
    u8 vmin = motor.adjust_min_voltage;

    /* adjust_minVoltage==0 时按 vmin 底板（与 ctr_init / uart 默认一致），避免 adc_floor≈0 卡住 */
    if (vmin == 0u)
        vmin = 20u;
    return MOTOR_MIN_VOLT_DISPLAY_TO_CMD_ADC((u32)vmin);
}

/*──────────────────── 故障处理辅助函数 ─────────────────────────*/

/* 软故障：停机减速，不立即断电 */
void motor_soft_fault_set(u8 err_code)
{
    if (err_code) {
        ctr.error_code = err_code;
        ctr.ack        = 1u;
    }
    /* 将电机状态切换为停机斜坡 */
    if (motor.status == STATUS_MT_RUN   ||
        motor.status == STATUS_MT_START ||
        motor.status == STATUS_MT_STOP) {
        motor.set_speed_scale = MT_STOP_SPEED;
        motor.status          = STATUS_MT_STOP;
    }
}

/* 立即关闭 PWM 和继电器（不等待减速） */
static void motor_immediate_pwm_cutoff(void)
{
    tim0stop();        /* 断继电器 */
    tim6stop();        /* 关 PWM */
    ctr.status     = CTR_STATUS_CLEARED;
    motor.status   = MOTOR_CTRL_STATUS_CLEARED;
    motor.en       = 0;
    waitforamoment = 0;
}

/* 硬故障：立即断电并上报错误码 */
void motor_hard_fault_set(u8 err_code)
{
    if (err_code) ctr.error_code = err_code;
    ctr.ack = 1u;
    motor_immediate_pwm_cutoff();
    ctr.status = STATUS_ERROR;
}

/* 紧急制动：加快减速率后进入停机斜坡，同时上报错误 */
void motor_emergency_brake_set(u8 err_code)
{
    if (err_code) ctr.error_code = err_code;
    ctr.ack = 1u;
    if (motor.deceleration < FAST_BRAKE_DECEL_LEVEL)
        motor.deceleration = FAST_BRAKE_DECEL_LEVEL;
    motor.set_speed_scale = 0;
    motor.status          = STATUS_MT_STOP;
    ctr.status            = STATUS_ERROR;
}

/* 受控停机：减速率不变，进入停机斜坡，上报错误 */
void motor_controlled_stop_set(u8 err_code)
{
    if (err_code) ctr.error_code = err_code;
    ctr.ack            = 1u;
    motor.deceleration = SOFT_STOP_DECEL_LEVEL;
    motor_soft_fault_set(0u);
    motor.set_speed_scale = MT_STOP_SPEED;
    motor.en              = 0;
    waitforamoment        = 0;
    ctr.status            = STATUS_ERROR;
}

/* 对照 HA380A-BDC_C03 motor.c；H1 仅多算 POWERON 的 I_offset（不报 E04）。 */

static void motor_wait_adc_batch_u32(u32 expect)
{
    u32 spin = 0u;
    while (adc_batch_seq < expect) {
        if (++spin > 800000u)
            break;
    }
}

void ctr_proc_loop(void)
{
    static u8 er02_code = 0, gate_dual_low_cnt = 0;
    u16       val_up, val_down;

    switch (ctr.status)
    {
    case STATUS_POWERON:
    {
        u32 cur_sum = 0u;
        u32 base;
        er02_code          = 0u;
        gate_dual_low_cnt  = 0u;
        user_adc_init();
        /* 仅要 TIM6 底点 UDF→ADC；OUT_PUT=PER−ZHANKONBI→0，压低自检期开关噪声对 I_offset 的影响 */
        ZHANKONBI = MOTOR_PWM_PERIOD_TICKS;
        tim6run();
        base = adc_batch_seq;
        for (u8 i = 0u; i < 100u; i++) {
            motor_wait_adc_batch_u32(base + (u32)i + 1u);
            val_up   = (u16)CALC_ADC_2_VOLTAGE(motor.valtage_up);
            val_down = (u16)CALC_ADC_2_VOLTAGE(motor.valtage_down);
            if ((val_up > val_down + dynamic_e02_limit_v) || (val_down > val_up + dynamic_e02_limit_v))
                er02_code++;
            if ((val_up < 10u) && (val_down < 10u))
                gate_dual_low_cnt++;
            cur_sum += motor.valtage_cur;
        }
        tim6stop();
        motor.I_offset = (u16)(cur_sum / 100u);

        if (er02_code > 5u) {
            er02_code = 0u;
            ctr.error_code = ERROR_02;
            ctr.status     = STATUS_ERROR;
            break;
        }
        if (gate_dual_low_cnt > 5u) {
            gate_dual_low_cnt = 0u;
            ctr.error_code = ERROR_02;
            ctr.status     = STATUS_ERROR;
            break;
        }
        ctr.status = CTR_STATUS_CLEARED;
        break;
    }

    case STATUS_RUN:
        /* 门控节拍与计数与 POWERON 解耦（同 static 块的干净入口）*/
        er02_code          = 0u;
        gate_dual_low_cnt  = 0u;
        user_adc_init();
        ZHANKONBI = MOTOR_PWM_PERIOD_TICKS;
        tim6run();
        {
            u32 base = adc_batch_seq;
            for (u8 i = 0u; i < 100u; i++) {
                motor_wait_adc_batch_u32(base + (u32)i + 1u);
                val_up   = (u16)CALC_ADC_2_VOLTAGE(motor.valtage_up);
                val_down = (u16)CALC_ADC_2_VOLTAGE(motor.valtage_down);
                if ((val_up > val_down + dynamic_e02_limit_v) || (val_down > val_up + dynamic_e02_limit_v))
                    er02_code++;
                if ((val_up < 10u) && (val_down < 10u))
                    gate_dual_low_cnt++;
            }
        }
        tim6stop();
        if (er02_code > 5u) {
            er02_code = 0u;
            ctr.error_code = ERROR_02;
            ctr.status     = STATUS_ERROR;
            break;
        }
        if (gate_dual_low_cnt > 5u) {
            gate_dual_low_cnt = 0u;
            ctr.error_code = ERROR_02;
            ctr.status     = STATUS_ERROR;
            break;
        }
        ctr.status       = CTR_STATUS_CLEARED;
        waitforamoment = 1u;
        break;

    case STATUS_TO_RUN:
        motor.adjust_set_voltage   = speed_param[0].voltage;
        /* 初始 kick = CMD_STA_LEVEL 的 start_valtage（ADC_cmd）；可低于 vmin，由 MT_START 分段抬到 vmin */
        motor.adjust_now_voltage = (u16)(motor.start_valtage + 0.5f);

        if (M0P_ADTIM6->GCONR_f.START == 0 || M0P_TIM0->CR_f.CTEN == 0) {
            /* 满占空：桥臂关断态/零输出，合闸与充电期避免异常占空 */
            ZHANKONBI = MOTOR_PWM_PERIOD_TICKS;
            motor.status             = STATUS_MT_START;
            motor.cur_speed_scale    = motor.sta_speed_scale;
            motor.start_tim          = 0;
            motor.start_fb_cnt       = 0u;
            s_start_fb_esc_ok        = 0u;
            reset_current_lp_filter();
            clear_adcbuf();

            tim0run();
            delay1ms(500);

            tim6run();
            /* 电容充电与 TIM6 触发 ADC 稳定后再锁母线，减轻分母过小导致的占空冲顶 */
            delay1ms(300);
            /* 母线分母：多次取样平均，躲开合闸浪涌顶点 */
            motor_drive_on_to_run_capture_bus_average();

            {
                u16 v_bus_now = (u16)CALC_ADC_2_VOLTAGE((u16)motor_vbus_adc);

                motor_grid_profile_known = 1u;
                if (v_bus_now > BUS_V_220V_THRESHOLD_V) {
                    motor_grid_env_220           = 1u;
                    dynamic_e02_limit_v        = E02_LIMIT_220V_V;
                    dynamic_e05_abs_volt_limit = E05_ABS_VOLT_220V;
                    if (motor.adjust_max_voltage > MAX_RUN_V_220V)
                        motor.adjust_max_voltage = MAX_RUN_V_220V;
                } else {
                    motor_grid_env_220           = 0u;
                    dynamic_e02_limit_v        = E02_LIMIT_110V_V;
                    dynamic_e05_abs_volt_limit = E05_ABS_VOLT_110V;
                    if (motor.adjust_max_voltage > MAX_RUN_V_110V)
                        motor.adjust_max_voltage = MAX_RUN_V_110V;
                }
            }
            uart_sync_rx_voltage_max_from_motor();
            motor_recalc_voltage_param_from_cmd();
            motor_drive_boot_sync_limits();
            ZHANKONBI = MT_START_PWM;
            /* 占空由 motor_drive_isr 线性写入 ZHANKONBI（电压 Q8 微步 + V×mul>>SHIFT） */
        } else {
            ZHANKONBI = MT_START_PWM;
        }
        ctr.status = CTR_STATUS_CLEARED;
        break;

    /*── 进入受控减速 ────────────────────────────────────────────────*/
    case STATUS_STOP:
        motor.status = STATUS_MT_STOP;
        ctr.status   = CTR_STATUS_CLEARED;
        break;

    /*── 停机：PWM 关断 → 续流制动 → 断继电器 ─────────────────────────*/
    case STATUS_TO_STOP:
        tim6stop();          /* 立即关闭 PWM */
        delay1ms(1500);      /* 等待 1.5 s：D10 续流二极管能耗制动，皮带平稳停止 */
        tim0stop();          /* 断开继电器，切断主电路 */
        uart_frame_tx(CMD_STOP_OVER, 1);  /* 通知上控：已完全停机 */
        ctr.status = CTR_STATUS_CLEARED;
        break;

    /*── 故障处理：按错误等级分类执行 ───────────────────────────────*/
    case STATUS_ERROR:
        switch (ctr.error_code) {
            case ERROR_02:
            case ERROR_03:
            case ERROR_04:
            case ERROR_05:
            case ERROR_06:
            case ERROR_07:
            case ERROR_08:
                motor_immediate_pwm_cutoff();  /* 严重故障：立即断电 */
                break;
            case ERROR_01:
                /* 通信超时：执行软停机减速，不立即断电 */
                motor.deceleration    = SOFT_STOP_DECEL_LEVEL;
                motor.set_speed_scale = MT_STOP_SPEED;
                motor.status          = STATUS_MT_STOP;  /* 通信中断：受控减速，非 PID */
                break;
            default:
                motor_soft_fault_set(0u);
                break;
        }
        if (ctr.error_code) uart_frame_tx(CMD_ERROR, ctr.error_code);  /* 上报错误码 */
        ctr.status = CTR_STATUS_CLEARED;
        break;

    default:
        break;
    }
}

/*──────────────────── 速度 / 电压斜坡（MT_START≈100ms；RUN 步长=tim6_change_speed_time×1/f_ISR）──*/

/* v_run_min：仅按上控下发的「最低电压（V）」换算为 ADC_cmd，不与 STA 曲线混算，
 * 避免握手标尺与底板不一致时在 ~10 V 长期判「未到运行条件」。 */
static u16 mt_run_min_voltage_adc(void)
{
    return motor_cmd_min_volt_adc_floor();
}

void motor_speed(void)
{
    switch (motor.status)
    {

    /*── B 起步阶段：斜坡电压单向爬升到 v_run_min（每 ~100 ms 一小步）；禁止 IR ─*/
    case STATUS_MT_START:
    {
        u16 vmin  = mt_run_min_voltage_adc();
        u16 vacc  = (motor.start_add_val > 0u) ? motor.start_add_val : motor.accelerated;

        /* start_tim 仅在手握阶段（已达 vmin）递增。若在升压爬坡期也递增，60 拍堵转超时会在握手
         * 尚未开始前就被耗光 → 最低档/空载亦误报 E03；与 C03 节拍相似但 H1 升压拍数更多。 */

        if (motor.adjust_now_voltage < vmin) {
            motor.start_set_val = 0; /* dwell 归零 */
            motor.start_fb_cnt  = 0u;
            s_start_fb_esc_ok   = 0u;
            {
                u32 step = (u32)vacc;

                if (step > (u32)MOTOR_VOLTAGE_RAMP_ADC_MAX_STEP)
                    step = (u32)MOTOR_VOLTAGE_RAMP_ADC_MAX_STEP;
                u32 nx = (u32)motor.adjust_now_voltage + step;

                if (nx > (u32)vmin)
                    nx = vmin;
                if (nx > (u32)START_VOLTAGE_MAX_ADC)
                    nx = START_VOLTAGE_MAX_ADC;
                motor.adjust_now_voltage = (u16)nx;
            }
        } else {
            motor.start_tim++;

            /* vmin 已到位仍可能堵转：若立刻进 RUN，开环斜坡「假破冰」后电流/滤波难达 20 A 档。
             * 对齐 C03 START→PID：端压需体现皮带已拉起来（显示器粗伏特级比较，cut_dc→**MOTOR_CUT_ADC_DELTA_TO_DISP_VOLT_FLT**）。 */
            float    vmin_hs = (float)motor.adjust_min_voltage;
            int16_t  cut_dc;
            u16      now_v_u16, tgt_v_u16;

            if (vmin_hs < 2.0f)
                vmin_hs = 20.0f;

            tgt_v_u16 = (u16)(GET_SPEED_VOLTAGE((float)motor.sta_speed_scale,
                                                motor.voltage_param,
                                                vmin_hs) + 0.5f);

            cut_dc = (int16_t)motor.valtage_up - (int16_t)motor.valtage_down;
            if (cut_dc < 0)
                cut_dc = 0;
            now_v_u16 = (u16)MOTOR_CUT_ADC_DELTA_TO_DISP_VOLT_FLT(cut_dc);

            /* 握手：端压明显高于速度表预期 → 皮带已拉出；空载皮带松时常长期 now<tgt。
             * 无条件兜底会把真低速堵转吃进 RUN → 仅当连续多拍瞬时负载≤约 10A 才允许兜底。 */
            if (motor.start_tim <= MT_START_FEEDBACK_MIN_TICKS) {
                motor.start_fb_cnt = 0u;
                s_start_fb_esc_ok  = 0u;
            } else if (tgt_v_u16 < now_v_u16) {
                reset_current_lp_filter();
                motor.status        = STATUS_MT_RUN;
                motor.start_tim     = 0u;
                motor.start_set_val = 0u;
                motor.start_fb_cnt  = 0u;
                s_start_fb_esc_ok   = 0u;
            } else {
                if (motor.start_fb_cnt <= MT_START_FEEDBACK_FALLBACK_TICKS)
                    motor.start_fb_cnt++;

                if (motor.start_fb_cnt > MT_START_FEEDBACK_FALLBACK_TICKS) {
                    if (mt_start_adc_load() <= MT_START_FB_ESC_LOAD_ADC) {
                        if (++s_start_fb_esc_ok >= MT_START_FB_ESC_NEED_TICKS) {
                            reset_current_lp_filter();
                            motor.status        = STATUS_MT_RUN;
                            motor.start_tim     = 0u;
                            motor.start_set_val = 0u;
                            motor.start_fb_cnt  = 0u;
                            s_start_fb_esc_ok   = 0u;
                        }
                    } else {
                        s_start_fb_esc_ok = 0u;
                    }
                }
            }
        }

        motor.cur_speed_scale = motor.sta_speed_scale;

        break;
    }

    /*── C 稳速：cur 斜坡；电压斜坡贴近 V(cur)【与 ISR 档位索引用同一速度，避免volt顶满/IR滞后拍振】--*/
    case STATUS_MT_RUN:
    {

        if (motor.cur_speed_scale < motor.set_speed_scale)
            motor.cur_speed_scale += motor.accelerated;
        else if (motor.cur_speed_scale > motor.set_speed_scale)
            motor.cur_speed_scale -= motor.accelerated;

        {
            float vmin_v = (float)motor.adjust_min_voltage;
            float vtf = GET_SPEED_VOLTAGE((float)motor.cur_speed_scale,
                                          motor.voltage_param,
                                          vmin_v);
            float fvadc = MOTOR_DISPLAY_VOLT_TF_TO_ADC_CMD_FLT(vtf);
            u16   vt;

            /* 饱和：电压参数错乱时 vf×缩放 会溢出 u16 */

            if (fvadc <= 0.0f)
                vt = 0;
            else if (fvadc >= 65504.0f)
                vt = (u16)65535u;
            else
                vt = (u16)(fvadc + 0.5f);

            u16 vminf = motor_cmd_min_volt_adc_floor();

            /* 不得低于上控下发的最低伏特刻度折算（避免长期指令低于 vmin）；stop_val ISR 对齐 adjust_min */
            if (vt < vminf)
                vt = vminf;
            /* 封顶最高允许电压刻度（通常为 90 V） */
            if (motor.adjust_max_voltage > 0u) {
                u16 vmaxadc = MOTOR_MIN_VOLT_DISPLAY_TO_CMD_ADC((u32)motor.adjust_max_voltage);

                if (vt > vmaxadc)
                    vt = vmaxadc;
            }

            {
                u16 adj = motor.adjust_now_voltage;
                u16 vt_cmd = vt;

                if (vt > adj) {
                    u16 dv = (u16)(vt - adj);

                    if (dv > MOTOR_VOLTAGE_RAMP_ADC_MAX_STEP)
                        vt_cmd = adj + MOTOR_VOLTAGE_RAMP_ADC_MAX_STEP;
                } else if (vt < adj) {
                    u16 dv = (u16)(adj - vt);

                    if (dv > MOTOR_VOLTAGE_RAMP_ADC_MAX_STEP)
                        vt_cmd = adj - MOTOR_VOLTAGE_RAMP_ADC_MAX_STEP;
                }

                u16 vacc = (motor.start_add_val > 0u) ? motor.start_add_val : motor.accelerated;
                u16 vdec = motor.deceleration;

                if (vdec < 1u)
                    vdec = 1u;

                if (motor.adjust_now_voltage < vt_cmd) {
                    u16 d = vt_cmd - motor.adjust_now_voltage;

                    if (d <= vacc)
                        motor.adjust_now_voltage = vt_cmd;
                    else
                        motor.adjust_now_voltage = (u16)(motor.adjust_now_voltage + vacc);
                } else if (motor.adjust_now_voltage > vt_cmd) {
                    u16 d = motor.adjust_now_voltage - vt_cmd;

                    if (d <= vdec)
                        motor.adjust_now_voltage = vt_cmd;
                    else
                        motor.adjust_now_voltage =
                            (u16)(motor.adjust_now_voltage - vdec);
                }
            }
        }

        /* Kcomp 缓慢跟踪档位表定义 */
        if      (motor.kiv > speed_param[motor.index].kiv) motor.kiv -= 0.2f;
        else if (motor.kiv < speed_param[motor.index].kiv) motor.kiv += 0.2f;
        motor_kiv_sync_isr_mul();
        break;
    }

    /*── D 停机：v_target=0 等价为电压拉到 0；近 v_run_min 步长减半 + 沿用速度斜坡护保护 ─────────*/
    case STATUS_MT_STOP:
    {
        u16 vmin_run = mt_run_min_voltage_adc();
        u16 cur_decel_speed = motor.deceleration;
        u16 vdec;

        /* 低速段减半速度减步（与旧逻辑一致），保持 index/过流分界合理 */
        if (motor.cur_speed_scale < 200u)
            cur_decel_speed = (motor.deceleration > 1u) ?
                (motor.deceleration / 2u) : 1u;

        if (motor.cur_speed_scale < motor.set_speed_scale) {
            u16 dsp = (u16)(motor.set_speed_scale - motor.cur_speed_scale);

            motor.cur_speed_scale =
                (cur_decel_speed >= dsp) ?
                motor.set_speed_scale :
                (u16)(motor.cur_speed_scale + cur_decel_speed);
        } else if (motor.cur_speed_scale > motor.set_speed_scale) {
            u16 dsm = (u16)(motor.cur_speed_scale - motor.set_speed_scale);

            motor.cur_speed_scale =
                (cur_decel_speed >= dsm) ?
                motor.set_speed_scale :
                (u16)(motor.cur_speed_scale - cur_decel_speed);
        }

        if (motor.cur_speed_scale < MT_STOP_SPEED)
            motor.stop_time_late++;

        vdec = motor.deceleration;
        if (vdec < 1u)
            vdec = 1u;
        /* 电压已低于并网最小轮廓时减半减步（柔和触底）*/
        if (motor.adjust_now_voltage < vmin_run) {
            vdec = (vdec > 1u) ? (vdec / 2u) : 1u;
        }
        if (motor.adjust_now_voltage > vdec)
            motor.adjust_now_voltage -= vdec;
        else
            motor.adjust_now_voltage = 0u;

        if (motor.kiv > 0.1f) motor.kiv -= 0.1f;
        else                   motor.kiv  = 0.1f;
        motor_kiv_sync_isr_mul();
        break;
    }
    } /* switch */

    /* 更新速度档位索引：上升立即跟随，下降滞回后再降档（抑制 IR 表边界震颤）*/
    {
        u8 n_raw = (u8)(motor.cur_speed_scale / 100u);

        if (n_raw > 12u)
            n_raw = 12u;

        if (n_raw >= motor.index) {
            motor.index = n_raw;
        } else {
            u16 hi_bound = (u16)motor.index * 100u;

            if (hi_bound > MOTOR_INDEX_DOWN_HYST_TICKS &&
                motor.cur_speed_scale < (hi_bound - MOTOR_INDEX_DOWN_HYST_TICKS)) {
                motor.index = n_raw;
            }
        }
    }
}
