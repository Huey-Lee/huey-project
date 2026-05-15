/* PB04 安全锁：双向去抖 + 插回后不应期，减轻浮空/噪声导致的 E17↔待机来回跳 */
#include "plat_safetylock.h"
#include "bsp_gpio.h"
#include "plat_aerolink.h"
#include "treadmills.h"

/* 连续 HIGH 达此次数（×10ms）才认「拔锁」；≤1s 内生效，仍略去抖 */
#define SAFETY_HI_CONFIRM_10MS    8u    /* 80ms */
/* 连续 LOW 达此次数才认「插回」——与拔锁不对称，防单点低脉冲误清 E17 */
#define SAFETY_LO_CONFIRM_10MS    28u   /* 280ms */
/* 插回确认后一段时间内不累计 HIGH，抑制插头/线材弹性抖动 */
#define SAFETY_REFRACTORY_10MS    45u   /* 450ms */

/* 保持断开期间周期性补发急停帧 */
#define SAFETY_ESTOP_RESEND_10MS  30u

static uint16_t s_hi_cnt;
static uint16_t s_lo_cnt;
static uint8_t  s_active_unsafe;
static uint16_t s_resend_10ms;
static uint16_t s_refractory_10ms;

void SafetyLock_Poll_10ms(void) {
#if (TM_SAFETY_LOCK_ENABLE == 0u)
    return;
#else
    if (s_refractory_10ms > 0u) {
        s_refractory_10ms--;
        s_hi_cnt = 0u; /* 不应期内不累计「假拔」高电平 */
    }

    /* 高=断开（内部上拉）；低=插头闭合接 GND */
    uint8_t hi = (GPIO_ReadPin(SAFETY_LOCK_PORT, SAFETY_LOCK_PIN) == GPIO_Pin_SET);

    if (hi) {
        s_lo_cnt = 0u;
        if (s_refractory_10ms == 0u && s_hi_cnt < SAFETY_HI_CONFIRM_10MS)
            s_hi_cnt++;
        if (s_hi_cnt >= SAFETY_HI_CONFIRM_10MS) {
            if (!s_active_unsafe) {
                s_active_unsafe = 1u;
                s_resend_10ms   = 0u;
                Treadmill_OnSafetyLockOpened();
            } else if (++s_resend_10ms >= SAFETY_ESTOP_RESEND_10MS) {
                s_resend_10ms = 0u;
                AeroLink_Send(FC_CONTROL, FC_CTRL_SFC_SAFETY_ESTOP, (void*)0);
            }
        }
    } else {
        s_hi_cnt = 0u;
        if (s_lo_cnt < SAFETY_LO_CONFIRM_10MS)
            s_lo_cnt++;
        if (s_lo_cnt >= SAFETY_LO_CONFIRM_10MS) {
            if (s_active_unsafe) {
                s_active_unsafe      = 0u;
                s_resend_10ms        = 0u;
                s_refractory_10ms    = SAFETY_REFRACTORY_10MS;
                Treadmill_OnSafetyLockClosed();
            }
        }
    }
#endif /* TM_SAFETY_LOCK_ENABLE */
}
