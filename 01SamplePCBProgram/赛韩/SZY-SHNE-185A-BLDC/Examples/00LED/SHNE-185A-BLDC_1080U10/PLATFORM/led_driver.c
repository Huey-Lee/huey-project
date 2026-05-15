/**
 * @file    led_driver.c
 * @brief   TM1620 / TM1640 / TM1652 统一 LED 驱动
 *
 * 在 led_driver.h 中配置 LED_DRIVER_TYPE 选择编译哪一路实现。
 */

#include "led_driver.h"
#include "cw32l010_gpio.h"


/* ================================================================
 *  TM1620（三线：STB + CLK + DIN）
 * ================================================================ */
#if (LED_DRIVER_TYPE == LED_DRIVER_TM1620)

static uint8_t s_buf[TM1620_BUF_SIZE];
static uint8_t s_ctrl = 0x8A;

static void prv_GPIO_Init(void)
{
    GPIO_InitTypeDef init = {0};
    __SYSCTRL_GPIOA_CLK_ENABLE();
    __SYSCTRL_GPIOB_CLK_ENABLE();

    init.Pins = LED_TM1620_PIN_CLK | LED_TM1620_PIN_DIN;
    init.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_Init(LED_TM1620_PORT_A, &init);

    init.Pins = LED_TM1620_PIN_STB;
    GPIO_Init(LED_TM1620_PORT_B, &init);

    LED_TM1620_STB_HIGH();
    LED_TM1620_CLK_HIGH();
    LED_TM1620_DIN_HIGH();
}

static void prv_SendByte(uint8_t data)
{
    for (uint8_t i = 0; i < 8; i++) {
        LED_TM1620_CLK_LOW();
        if (data & 0x01) LED_TM1620_DIN_HIGH();
        else             LED_TM1620_DIN_LOW();
        data >>= 1;
        LED_TM1620_CLK_HIGH();
    }
}

static void prv_Cmd(uint8_t cmd)
{
    LED_TM1620_STB_LOW();
    prv_SendByte(cmd);
    LED_TM1620_STB_HIGH();
}

void LED_Init(void)
{
    prv_GPIO_Init();
    LED_ClearBuf();
    prv_Cmd(0x40);
    LED_Refresh();
    LED_SetBrightness(4);
}

void LED_SetBrightness(uint8_t level)
{
    if (level == 0) { s_ctrl = 0x80; }
    else { if (level > 8) level = 8; s_ctrl = (uint8_t)(0x88 + (level - 1)); }
    prv_Cmd(s_ctrl);
}

void LED_Refresh(void)
{
    prv_Cmd(0x40);
    LED_TM1620_STB_LOW();
    prv_SendByte(0xC0);
    for (uint8_t i = 0; i < TM1620_BUF_SIZE; i++) prv_SendByte(s_buf[i]);
    LED_TM1620_STB_HIGH();
    prv_Cmd(s_ctrl);
}

void LED_WriteBuf(uint8_t grid, uint8_t seg_data)
{
    if (grid < 6) s_buf[grid * 2] = seg_data;
}

void LED_ClearBuf(void)
{
    for (uint8_t i = 0; i < TM1620_BUF_SIZE; i++) s_buf[i] = 0;
}

void LED_FillBuf(uint8_t data)
{
    for (uint8_t i = 0; i < TM1620_BUF_SIZE; i++) s_buf[i] = data;
}


/* ================================================================
 *  TM1640（两线：DIN + CLK，无 STB）
 *
 *  格位数由 led_driver.h 中 TM1640_GRID_NUM 控制（1~16）
 *  硬件最大支持 16 个 Grid；只发送 TM1640_GRID_NUM 字节，减少总线占用
 * ================================================================ */
#elif (LED_DRIVER_TYPE == LED_DRIVER_TM1640)

#if (TM1640_GRID_NUM < 1) || (TM1640_GRID_NUM > TM1640_BUF_SIZE)
  #error "TM1640_GRID_NUM 须在 1~16 之间"
#endif

static uint8_t s_buf[TM1640_BUF_SIZE];
static uint8_t s_ctrl = 0x8A;   /* 默认亮度 level 3 */

/* ---------------------------------------------------------------
 * TM1640 时序延时  @PCLK=48MHz，volatile 循环约 3~4 cycle/次
 *   TM1640_DLY_BIT : CLK 高/低脉宽  目标 ≥ 1 μs  → n=16 ≈ 1.1 μs
 *   TM1640_DLY_BUS : START/STOP 建立保持时间 目标 ≥ 4 μs → n=64 ≈ 4.6 μs
 * --------------------------------------------------------------- */
static void prv_Delay(uint32_t n)
{
    volatile uint32_t d = n;
    while (d--);
}
#define TM1640_DLY_BIT()  prv_Delay(16U)
#define TM1640_DLY_BUS()  prv_Delay(64U)

static void prv_GPIO_Init(void)
{
    GPIO_InitTypeDef init = {0};
    __SYSCTRL_GPIOA_CLK_ENABLE();

    init.Pins = LED_TM1640_PIN_DIN | LED_TM1640_PIN_CLK;
    init.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_Init(LED_TM1640_PORT_A, &init);

    LED_TM1640_CLK_HIGH();
    LED_TM1640_DIN_HIGH();
}

/* START：CLK 高时 DIN 下降沿 */
static void prv_Start(void)
{
    LED_TM1640_CLK_HIGH();
    LED_TM1640_DIN_HIGH();
    TM1640_DLY_BUS();               /* 总线空闲/建立时间 */
    LED_TM1640_DIN_LOW();           /* DIN↓ while CLK=H → START */
    TM1640_DLY_BUS();               /* START 保持时间 */
}

/* STOP：CLK 高时 DIN 上升沿
 * 修复原代码 bug：原代码在 CLK=H、DIN=H 状态下直接拉低 DIN，
 * 会被 TM1640 误识别为新的 START 条件，导致数据帧末尾被污染。
 * 正确做法：先拉低 CLK，再确保 DIN=L，然后 CLK↑，最后 DIN↑。 */
static void prv_Stop(void)
{
    LED_TM1640_CLK_LOW();           /* 先降 CLK，避免 DIN 变化时产生误 START */
    TM1640_DLY_BIT();
    LED_TM1640_DIN_LOW();           /* DIN 拉低（CLK=L 期间，安全） */
    TM1640_DLY_BUS();
    LED_TM1640_CLK_HIGH();
    TM1640_DLY_BUS();               /* STOP 建立时间 */
    LED_TM1640_DIN_HIGH();          /* DIN↑ while CLK=H → STOP */
    TM1640_DLY_BUS();               /* 总线空闲时间 */
}

/* LSB 先发，8 位，上升沿锁存；字节结束后 CLK 保留低电平 */
static void prv_SendByte(uint8_t data)
{
    uint8_t i;
    for (i = 0; i < 8; i++) {
        LED_TM1640_CLK_LOW();
        TM1640_DLY_BIT();           /* CLK 低脉宽 */
        if (data & 0x01) LED_TM1640_DIN_HIGH();
        else             LED_TM1640_DIN_LOW();
        TM1640_DLY_BIT();           /* DIN 建立时间（CLK 上升前） */
        data >>= 1;
        LED_TM1640_CLK_HIGH();      /* ↑ TM1640 在此沿锁存 DIN */
        TM1640_DLY_BIT();           /* CLK 高脉宽（数据保持时间） */
    }
    LED_TM1640_CLK_LOW();           /* 字节末尾拉低 CLK，状态确定 */
    TM1640_DLY_BIT();
}

/* 发送单字节命令（带起止条件）*/
static void prv_Cmd(uint8_t cmd)
{
    prv_Start();
    prv_SendByte(cmd);
    prv_Stop();
}

void LED_Init(void)
{
    prv_GPIO_Init();
    LED_ClearBuf();
    prv_Cmd(0x40);          /* 数据命令：自动地址递增，普通模式 */
    LED_Refresh();
    LED_SetBrightness(8);
}

void LED_SetBrightness(uint8_t level)
{
    if (level == 0) {
        s_ctrl = 0x80;      /* 关屏（Display OFF）*/
    } else {
        if (level > 8) level = 8;
        s_ctrl = (uint8_t)(0x88 + (level - 1));
    }
    prv_Cmd(s_ctrl);
}

void LED_Refresh(void)
{
    uint8_t i;
    prv_Cmd(0x40);          /* 数据命令 */
    prv_Start();
    prv_SendByte(0xC0);     /* 地址从 00H 开始 */
    for (i = 0; i < TM1640_GRID_NUM; i++) prv_SendByte(s_buf[i]);
    prv_Stop();
    prv_Cmd(s_ctrl);        /* 显示控制命令（亮度 + 开屏）*/
}

void LED_WriteBuf(uint8_t grid, uint8_t seg_data)
{
    if (grid < TM1640_GRID_NUM) s_buf[grid] = seg_data;
}

void LED_ClearBuf(void)
{
    uint8_t i;
    for (i = 0; i < TM1640_BUF_SIZE; i++) s_buf[i] = 0;
}

void LED_FillBuf(uint8_t data)
{
    uint8_t i;
    for (i = 0; i < TM1640_GRID_NUM; i++) s_buf[i] = data;
}


/* ================================================================
 *  TM1652（硬件 UART2：PB04，19200 波特率，8O1）
 *
 *  正确协议：
 *  - 帧格式：奇校验 8O1（TM1652 强制要求，无奇偶校验则无响应）
 *  - 数据帧：[0x08][GR1][GR2]...[GRn]，一次突发写所有格位
 *  - 控制帧：[0x18][亮度字节]，亮度查表见 s_brightTable
 *  - 每帧后等待 TXBUSY 清零，再延时 ≥5ms 供 TM1652 锁存
 * ================================================================ */
#elif (LED_DRIVER_TYPE == LED_DRIVER_TM1652)

#include "cw32l010_uart.h"

/* TM1652 亮度查找表（5级，来自可工作驱动） */
static const uint8_t s_brightTable[5] = {
    0x88U,   /* Level 1 最暗 */
    0x44U,   /* Level 2      */
    0x1AU,   /* Level 3 中等 */
    0x36U,   /* Level 4      */
    0xFEU,   /* Level 5 最亮 */
};

static uint8_t s_buf[TM1652_GRID_MAX];
static uint8_t s_ctrl = 0xFEU; /* 默认：最高亮度 */

/* 毫秒延时（帧间锁存用） */
static void prv_DelayMs(uint32_t ms)
{
    volatile uint32_t n;
    while (ms--) { n = 2000U; while (n--); }
}

/* 等待 TX 彻底空闲 */
static void prv_WaitTxDone(void)
{
    while (UART_GetFlagStatus(LED_TM1652_UARTx, UART_FLAG_TXBUSY) == SET);
}

/* 发送一字节 */
static void prv_SendByte(uint8_t b)
{
    while (UART_GetFlagStatus(LED_TM1652_UARTx, UART_FLAG_TXE) == RESET);
    UART_SendData_8bit(LED_TM1652_UARTx, b);
}

/* 发送一帧，帧后等待 TX 完成 + 5ms 锁存 */
static void prv_SendFrame(const uint8_t *p, uint8_t len)
{
    uint8_t i;
    for (i = 0; i < len; i++) prv_SendByte(p[i]);
    prv_WaitTxDone();
    prv_DelayMs(5U);
}

/* 初始化 UART2 on PB04，19200 baud，8O1，TX-only */
static void prv_UART_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    UART_InitTypeDef uart = {0};

    SYSCTRL_AHBPeriphClk_Enable(SYSCTRL_AHB_PERIPH_GPIOB, ENABLE);
    SYSCTRL_APBPeriphClk_Enable1(SYSCTRL_APB1_PERIPH_UART2, ENABLE);

    gpio.Pins = LED_TM1652_PIN_DAT;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_Init(LED_TM1652_PORT_B, &gpio);
    PB04_AFx_UART2TXD();

    uart.UART_BaudRate            = 19200U;
    uart.UART_Over                = UART_Over_16;
    uart.UART_Source              = UART_Source_PCLK;
    uart.UART_UclkFreq            = TM1652_PCLK_FREQ;
    uart.UART_StartBit            = UART_StartBit_FE;
    uart.UART_StopBits            = UART_StopBits_1;
    uart.UART_Parity              = UART_Parity_Odd;   /* 奇校验，TM1652 必须 */
    uart.UART_HardwareFlowControl = UART_HardwareFlowControl_None;
    uart.UART_Mode                = UART_Mode_Tx;
    UART_Init(LED_TM1652_UARTx, &uart);
}

/* 将 s_buf 刷新到 TM1652（数据帧 + 控制帧） */
static void prv_Flush(void)
{
    uint8_t frame[TM1652_GRID_MAX + 1U];
    uint8_t ctrl[2];
    uint8_t i;

    frame[0] = TM1652_CMD_DATA;
    for (i = 0; i < TM1652_GRID_MAX; i++) frame[i + 1U] = s_buf[i];
    prv_SendFrame(frame, (uint8_t)(TM1652_GRID_MAX + 1U));

    ctrl[0] = TM1652_CMD_CTRL;
    ctrl[1] = s_ctrl;
    prv_SendFrame(ctrl, 2U);
}

void LED_Init(void)
{
    prv_UART_Init();
    LED_ClearBuf();
    prv_DelayMs(20U);       /* 等待 TM1652 上电就绪 */
    LED_SetBrightness(3);   /* 先关屏 / 设置亮度 */
    LED_Refresh();
}

void LED_SetBrightness(uint8_t level)
{
    uint8_t ctrl[2];
    if (level == 0U) {
        s_ctrl = TM1652_CTRL_OFF;
    } else {
        if (level > 5U) level = 5U;
        s_ctrl = s_brightTable[level - 1U];
    }
    ctrl[0] = TM1652_CMD_CTRL;
    ctrl[1] = s_ctrl;
    prv_SendFrame(ctrl, 2U);
}

void LED_Refresh(void)
{
    prv_Flush();
}

void LED_WriteBuf(uint8_t grid, uint8_t seg_data)
{
    if (grid < TM1652_GRID_MAX) s_buf[grid] = seg_data;
}

void LED_ClearBuf(void)
{
    uint8_t i;
    for (i = 0; i < TM1652_GRID_MAX; i++) s_buf[i] = 0x00U;
}

void LED_FillBuf(uint8_t data)
{
    uint8_t i;
    for (i = 0; i < TM1652_GRID_MAX; i++) s_buf[i] = data;
}


/* ================================================================ */
#else
    #error "LED_DRIVER_TYPE 未设置或无效，请在 led_driver.h 中配置"
#endif
