/**
 * @file    uart_frame.c
 * @author  HueyLee
 * @brief   Communication Protocol Stack
 * @date		2026??3??1??
 */
#include "common.h"
#include "uart_frame.h"
#include "treadmills.h"
#include "param.h"
#include "led_disp.h"
#include "indecate.h"
#include "beep.h"
#include "cfg.h"
#include "keypad.h"


T_QUEUE rx_queue;
u8 rx_buf[RX_BUFF_SIZE];

uart_frame_t uart_frame;
uart_frame_t uart_tx_frame;

extern u8 uart_time_error;

//????????????
static void App_PortInit(void)
{
    stc_gpio_cfg_t stcGpioCfg;

    DDL_ZERO_STRUCT(stcGpioCfg);

    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio,TRUE); //???GPIO??????

    ///<TX
    stcGpioCfg.enDir = GpioDirOut;
    Gpio_Init(GpioPort3, GpioPin5, &stcGpioCfg);
    Gpio_SetAfMode(GpioPort3, GpioPin5, GpioAf1);          //????P35 ????URART1_TX

    ///<RX
    stcGpioCfg.enDir = GpioDirIn;
    Gpio_Init(GpioPort3, GpioPin6, &stcGpioCfg);
    Gpio_SetAfMode(GpioPort3, GpioPin6, GpioAf1);          //????P36 ????URART1_RX
}

static void _UartBaudCfg(void)
{
    uint16_t timer=0;

    stc_uart_baud_cfg_t stcBaud;
    stc_bt_cfg_t stcBtCfg;

    DDL_ZERO_STRUCT(stcBaud);
    DDL_ZERO_STRUCT(stcBtCfg);

    //??????????
    Sysctrl_SetPeripheralGate(SysctrlPeripheralBt,TRUE);//??0/2????????
    Sysctrl_SetPeripheralGate(SysctrlPeripheralUart1,TRUE);

    stcBaud.bDbaud  = 0u;//????????????
    stcBaud.u32Baud = 4800;//????????????
    stcBaud.enMode  = UartMode1; //?????????????????
    stcBaud.u32Pclk = Sysctrl_GetPClkFreq(); //???PCLK
    timer = Uart_SetBaudRate(M0P_UART1, &stcBaud);

    stcBtCfg.enMD = BtMode2;
    stcBtCfg.enCT = BtTimer;
    Bt_Init(TIM1, &stcBtCfg);//????basetimer1?????????????????
    Bt_ARRSet(TIM1,timer);
    Bt_Cnt16Set(TIM1,timer);
    Bt_Run(TIM1);

}


static void App_UartInit(void)
{
    stc_uart_cfg_t  stcCfg;

    _UartBaudCfg();

    stcCfg.enRunMode = UartMode1;//?????????????????4????????
    Uart_Init(M0P_UART1, &stcCfg);

    ///< UART???????
    Uart_EnableIrq(M0P_UART1, UartRxIrq);
    Uart_ClrStatus(M0P_UART1, UartRC);
    EnableNvic(UART1_IRQn, IrqLevel3, TRUE);
}


void uart0_init(void)
{
		App_PortInit();
		App_UartInit();
	
		Uart_SendDataPoll(M0P_UART1, 0x00);
    Uart_SendDataPoll(M0P_UART1, 0x00);
    delay1ms(50); 
}

// UART1 ????????
void Uart1_IRQHandler(void)
{
		u8 dat;
    if(TRUE == Uart_GetStatus(M0P_UART1, UartRC))
    {
        Uart_ClrStatus(M0P_UART1, UartRC);
				dat = Uart_ReceiveData(M0P_UART1);
				uart_time_error=0;
				//LED1=!LED1;
				Enter_queue(&rx_queue,dat);				
    }
}

/* ?????????? */
void UART_Send_Buf(u8 UARTPort, u8 *ptr, u8 len) {
    while(len--) {
        Uart_SendDataPoll(M0P_UART1, *ptr++);
    }
}

void uart_frame_init(void) {
    Create_Queue(&rx_queue, &rx_buf[0], RX_BUFF_SIZE);
}

/**
 * @brief ?????????
 */
void cmd_proc(void) {
    static u16 sync_speed_max_seen = (u16)0xFFFFu;
    static u8  prev_lower_st       = (u8)0xFEu;

    u8 f = uart_frame.m.func;
    u8 s = uart_frame.m.sub_func;
		treadmills.ack = 1;
	
//		// ????????????????????16???????????????2
//    #define PARSE_VAL(h, l) ((u16)(((u16)(h) << 8) | (l)) >> 1)
	
    switch (f) {
        case 0x01: 
            if (s == 0x00) {
                treadmills.param.fbk_speed =
                    (u16)(((u16)uart_frame.m.d1_h << 8) | uart_frame.m.d1_l);
                treadmills.lower_status = uart_frame.m.d2_l;
                treadmills.param.speed_max =
                    (u16)(((u16)uart_frame.m.d3_h << 8) | uart_frame.m.d3_l);
                treadmills.param.speed_min =
                    (u16)(((u16)uart_frame.m.d4_h << 8) | uart_frame.m.d4_l);

                if (treadmills.param.speed_max != 0U) {
                    treadmills.param.speed = treadmills.param.speed_max;

                    if (sync_speed_max_seen != treadmills.param.speed_max) {
                        sync_speed_max_seen = treadmills.param.speed_max;
                        uart_frame_tx_d1(0x03, 0x00, treadmills.param.speed_max);
                    }
                }

                if (treadmills.lower_status == LOWER_ST_WAIT_START	
                    && prev_lower_st != LOWER_ST_WAIT_START
                    && treadmills.param.speed_max != 0U
                    && treadmills.status != STATUS_ERROR
                    && treadmills.status != STATUS_RUN
                    && treadmills.status != STATUS_RUNNING) {
                    treadmills.param.speed = treadmills.param.speed_max;
                    treadmills.status = STATUS_RUN;
                }

                prev_lower_st = treadmills.lower_status;
            }
            else if (s == 0x01) { 
                u16 err_high_low = (u16)((uart_frame.m.d1_h << 8) | uart_frame.m.d1_l);
                u8  err_secondary = uart_frame.m.d2_l; // ??????????

				if (err_high_low > 0 || err_secondary > 0) {
                    treadmills.error_code = err_high_low;     // ?????????
                    treadmills.error_sub_code = err_secondary; // ??????????
                    
                    if(treadmills.status != STATUS_ERROR) {
                        treadmills.status = STATUS_ERROR;    // ?????????
                    }
                }
            }
            break;

        case 0x02:
            if (s == 0x00) {
                // ????????????? (???????1????)
//                treadmills.param.v_bus = PARSE_VAL(uart_frame.m.d1_h, uart_frame.m.d1_l);
//                treadmills.param.v_12v = PARSE_VAL(uart_frame.m.d2_h, uart_frame.m.d2_l);
//								treadmills.param.temp = (u16)((uart_frame.m.d3_h << 8) | uart_frame.m.d3_l);
               treadmills.param.temp = (u16)((((u16)uart_frame.m.d3_h << 8) | uart_frame.m.d3_l) >> 1);
            }
//						else if (s == 0x01) {
//							
//						}
            break;
    }
}


/**
 * @brief ????????
 */
void uart_frame_loop(void) {
    static u8 step = 0;
    static u8 sum_calc = 0;
    u8 dat;

    while (Denter_queue(&rx_queue, &dat)) {
        if (step == 0) {
            if (dat == PROTOCOL_SOF1) step = 1;
            continue;
        }
        if (step == 1) {
            if (dat == PROTOCOL_SOF2) step = 2;
            else step = 0;
            continue;
        }

        uart_frame.raw[step] = dat;

        if (step >= 2 && step <= 12) {
            if (step == 2) {
                if (dat != CTRL_CODE_UP && dat != CTRL_CODE_DOWN) { step = 0; continue; }
                sum_calc = dat;
            } else {
                sum_calc += dat;
            }
        }

        if (++step >= 17) {
            if (uart_frame.m.cs_l == sum_calc && 
                uart_frame.m.eof1 == PROTOCOL_EOF1 && 
                uart_frame.m.eof2 == PROTOCOL_EOF2) {
                
                treadmills.ack = 1;
                cmd_proc(); 
            }
            step = 0; 
        }
    }
}

/**
 * @brief ???????
 */
void uart_frame_tx(u8 func, u8 sub, u8 data_low) {
    u8 i;
    u8 sum = 0;
    
    // ???????
    memset(uart_tx_frame.raw, 0, 17);
    
    // ???????
    uart_tx_frame.m.sof1     = PROTOCOL_SOF1;
    uart_tx_frame.m.sof2     = PROTOCOL_SOF2;
    uart_tx_frame.m.ctrl     = PROTOCOL_LEN;
    uart_tx_frame.m.func     = func;
    uart_tx_frame.m.sub_func = sub;
    uart_tx_frame.m.d1_l     = data_low; // ???????????????1???
    uart_tx_frame.m.eof1     = PROTOCOL_EOF1;
    uart_tx_frame.m.eof2     = PROTOCOL_EOF2;

    // ???????? (ctrl + func + sub + data)
    for (i = 2; i <= 12; i++) {
        sum += uart_tx_frame.raw[i];
    }
    uart_tx_frame.m.cs_l = sum; // ????? 15 ???

    // ???????? 17 ???
    UART_Send_Buf(UART1, uart_tx_frame.raw, 17);
}

void uart_frame_tx_d1(u8 func, u8 sub, u16 data1_hl) {
    u8 i;
    u8 sum = 0;

    memset(uart_tx_frame.raw, 0, 17);

    uart_tx_frame.m.sof1     = PROTOCOL_SOF1;
    uart_tx_frame.m.sof2     = PROTOCOL_SOF2;
    uart_tx_frame.m.ctrl     = PROTOCOL_LEN;
    uart_tx_frame.m.func     = func;
    uart_tx_frame.m.sub_func = sub;
    uart_tx_frame.m.d1_h     = (u8)(data1_hl >> 8);
    uart_tx_frame.m.d1_l     = (u8)(data1_hl & 0xFFu);
    uart_tx_frame.m.eof1     = PROTOCOL_EOF1;
    uart_tx_frame.m.eof2     = PROTOCOL_EOF2;

    for (i = 2; i <= 12; i++) {
        sum += uart_tx_frame.raw[i];
    }
    uart_tx_frame.m.cs_l = sum;

    UART_Send_Buf(UART1, uart_tx_frame.raw, 17);
}
