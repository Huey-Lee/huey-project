#include "plat_gpio_keys.h"
#include "cw32l010_gpio.h"
#include "cw32l010_sysctrl.h"

#if PLAT_USE_GPIO_KEYS

#define GPIOKEY_DEBOUNCE_TICKS  4u

static uint8_t s_db_count;
static KeyID_t s_db_key;
static KeyID_t s_stable;

static KeyID_t prv_raw(void)
{
    /* 低有效；多键：停 > 启 > 加 > 减
     * 原理图：PB00=速度−，PB01=速度+，PA03=启动，PA04=停止（与 PA05 触摸 A 线无关） */
    if (GPIO_ReadPin(CW_GPIOA, GPIO_PIN_4) == GPIO_Pin_RESET)
        return K_ID_OFF;
    if (GPIO_ReadPin(CW_GPIOA, GPIO_PIN_3) == GPIO_Pin_RESET)
        return K_ID_ON;
    if (GPIO_ReadPin(CW_GPIOB, GPIO_PIN_0) == GPIO_Pin_RESET)
        return K_ID_DN;
    if (GPIO_ReadPin(CW_GPIOB, GPIO_PIN_1) == GPIO_Pin_RESET)
        return K_ID_UP;
    return K_ID_NONE;
}

void GpioKeys_Init(void)
{
    GPIO_InitTypeDef g = {0};

    SYSCTRL_AHBPeriphClk_Enable(SYSCTRL_AHB_PERIPH_GPIOA | SYSCTRL_AHB_PERIPH_GPIOB, ENABLE);

    PA03_DIGTAL_ENABLE();
    PA04_DIGTAL_ENABLE();
    PB00_DIGTAL_ENABLE();
    PB01_DIGTAL_ENABLE();

    g.Pins = GPIO_PIN_3 | GPIO_PIN_4;
    g.Mode = GPIO_MODE_INPUT_PULLUP;
    GPIO_Init(CW_GPIOA, &g);

    g.Pins = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_Init(CW_GPIOB, &g);

    s_db_count  = 0;
    s_db_key    = K_ID_NONE;
    s_stable    = K_ID_NONE;
}

KeyID_t GpioKeys_Handler_10ms(void)
{
    KeyID_t r = prv_raw();

    if (r == s_db_key) {
        if (s_db_count < 255u)
            s_db_count++;
    } else {
        s_db_key   = r;
        s_db_count = 1;
    }
    if (s_db_count >= GPIOKEY_DEBOUNCE_TICKS)
        s_stable = s_db_key;
    return s_stable;
}

#else

void GpioKeys_Init(void) { }

KeyID_t GpioKeys_Handler_10ms(void)
{
    return K_ID_NONE;
}

#endif
