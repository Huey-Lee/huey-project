#include "BSP_MODBUS.h"
#include "my_usart.h"	
#include "ringfifo.h"
#include <stdlib.h>
#include "stdio.h"
#include "uart.h"
#include "motor.h"
#include "uart_frame.h"
modbus__ mod;
extern ctr_t ctr;
extern motor_t motor;
extern ringfifo_t uart1_rx_ringfifo;
uint8_t modbus_buf[modbus_buflen]={0};
uint16_t REG_BUF[REGISTERNUM]={0,0,0,0};


void u8_to_u16(uint8_t * data,uint16_t * buf,uint8_t u16len)
{
	for(uint8_t i=0;i<u16len;i++)
	{
		buf[i] = (data[i*2] << 8) + data[i*2+1];
	}
}
void u16_to_u8(uint8_t * data,uint16_t * buf,uint8_t u16len)
{
	for(uint8_t i=0;i<u16len;i++)
	{
		data[i*2] 		= (buf[i] >> 8) & 0xff;
		data[i*2 + 1] = (buf[i]     ) & 0xff;
	}
}
/*******************************************************************************
** 函数名称: modbus_back_message(modbus__ modbus)
** 功能描述: modbus反馈消息
** 参数说明: 
** 返回说明: 错误返回：1   正确返回：0
** 创建人员: 张涛
** 创建日期: 2019/6/24
**------------------------------------------------------------------------------
** 修改人员:
** 修改日期:
** 修改描述:
**------------------------------------------------------------------------------
********************************************************************************/
void modbus_back_message(modbus__ * modbus)
{
	uint8_t bufe[modbus_buflen]={0};
	uint8_t reg_buf[REGISTERNUM*2]={0};
	switch(modbus->funcode)
	{
		case(0x03):
		bufe[0] = modbus->master_id;
		bufe[1] = modbus->funcode;
		bufe[2] = modbus->regnum * 2;
		u16_to_u8(reg_buf,&REG_BUF[modbus->startreg],modbus->regnum);
		memcpy(&bufe[3],reg_buf,modbus->regnum*2);
		modbus->countcrc = Crc16(bufe,3+modbus->regnum*2);
		bufe[3+modbus->regnum*2] = (modbus->countcrc ) & 0xff;
		bufe[4+modbus->regnum*2] = (modbus->countcrc >> 8) & 0xff;
		__NOP();
		uart_send(bufe,5+modbus->regnum*2);
		
		
		break;
		case(0x06):
		bufe[0] = modbus->master_id;
		bufe[1] = modbus->funcode;
		bufe[2] = modbus->start_reg[0];
		bufe[3] = modbus->start_reg[1];
		bufe[4] = (modbus->registe[modbus->startreg] >> 8) & 0xff;
		bufe[5] = (modbus->registe[modbus->startreg] ) & 0xff;
		modbus->countcrc = Crc16(bufe,6);
		bufe[6] = (modbus->countcrc ) & 0xff;
		bufe[7] = (modbus->countcrc >> 8) & 0xff;
		__NOP();
		uart_send(bufe,8);
		break;
		case(0x10):
		bufe[0] = modbus->master_id;
		bufe[1] = modbus->funcode;
		bufe[2] = modbus->start_reg[0];
		bufe[3] = modbus->start_reg[1];
		bufe[4] = modbus->reg_num[0];
		bufe[5] = modbus->reg_num[1];
		modbus->countcrc = Crc16(bufe,6);
		bufe[6] = (modbus->countcrc ) & 0xff;
		bufe[7] = (modbus->countcrc >> 8) & 0xff;
		__NOP();
		uart_send(bufe,8);
		break;
	}
}
/*******************************************************************************
** 函数名称: uint8_t modbus_command_handle(uint8_t * data,uint8_t len)
** 功能描述: modbus命令处理
** 参数说明: command：传递命令存放的数组指针  len：命令长度
** 返回说明: 错误返回：1   正确返回：0
** 创建人员: 张涛
** 创建日期: 2019/6/24
**------------------------------------------------------------------------------
** 修改人员:
** 修改日期:
** 修改描述:
**------------------------------------------------------------------------------
********************************************************************************/
uint8_t modbus_command_handle(modbus__ * modbus)
{
	modbus__ modcom={0};
	uint8_t regbuf[REGISTERNUM*2]={0};
	uint16_t REGBUF[REGISTERNUM]={0};
	
	if(mod.end == 1)
	{
		memset(&modcom,0,sizeof(modcom));
		modcom.buf = modbus->buf;
		modcom.registe = modbus->registe;
		ringfifo_get(&uart1_rx_ringfifo,&modcom.master_id,1);
		ringfifo_get(&uart1_rx_ringfifo,&modcom.funcode,1);
		ringfifo_get(&uart1_rx_ringfifo,modcom.start_reg,2);
		modcom.startreg = ((modcom.start_reg[0] << 8)&0xff00) | modcom.start_reg[1];
		switch(modcom.funcode)
		{
			case(0x03):
				ringfifo_get(&uart1_rx_ringfifo,modcom.reg_num,2);
				modcom.regnum = (modcom.reg_num[0] << 8) + modcom.reg_num[1];
				__NOP();
				modbus_back_message(&modcom);
			
			
				break;
			case(0x06):					
				ringfifo_get(&uart1_rx_ringfifo,regbuf,2);
				modcom.registe[modcom.startreg] = regbuf[0]<<8 | regbuf[1];
				__NOP();
				modbus_back_message(&modcom);
			
				modbus_register_handle();
			
				break;
			case(0x10):
				ringfifo_get(&uart1_rx_ringfifo,modcom.reg_num,2);
				modcom.regnum = (modcom.reg_num[0] << 8) + modcom.reg_num[1];
				ringfifo_get(&uart1_rx_ringfifo,&modcom.Bitnum,1);
				ringfifo_get(&uart1_rx_ringfifo,regbuf,modcom.Bitnum);
				u8_to_u16(regbuf,REGBUF,modcom.Bitnum/2);

				for(uint8_t i=0;i<modcom.Bitnum/2;i++)
					modcom.registe[modcom.startreg+i] = REGBUF[i];
				__NOP();
				modbus_back_message(&modcom);
				
				modbus_register_handle();
				
				break;
		}
		
		__NOP();
		
		uart1_ringfifo_init();
		modbus_init(ID_master,ID_slave);
	}
	return 0;
}
void modbus_register_handle(void)
{
	switch(REG_BUF[0])
	{
			case(CMD_SPEED):
				motor.set_speed_scale = REG_BUF[1];
				if(motor.set_speed_scale < 100){
					motor.set_speed_scale = 100;
					REG_BUF[1] = 100;}
				if(motor.set_speed_scale> motor.adjust_speed_max){
					motor.set_speed_scale = motor.adjust_speed_max;
					REG_BUF[1] = motor.adjust_speed_max;}
			
			
			break;
			case(CMD_TREADMILLS_SPEED_MAX):
				motor.adjust_speed_max = REG_BUF[1];
				motor.voltage_param = (motor.adjust_max_voltage - motor.adjust_min_voltage)/(motor.adjust_speed_max - 100);
			break;
			case(CMD_VOLTAGE_MAX):
				motor.adjust_max_voltage = REG_BUF[1];
				motor.voltage_param = (motor.adjust_max_voltage - motor.adjust_min_voltage)/(motor.adjust_speed_max - 100);
			break;
			case(CMD_VOLTAGE_MIN):
				motor.adjust_min_voltage = REG_BUF[1];
				motor.voltage_param = (motor.adjust_max_voltage - motor.adjust_min_voltage)/(motor.adjust_speed_max - 100);
			break;
	}
}


unsigned int Crc16(unsigned char *pBuf, unsigned char num)
{
	unsigned char i,j; 
	unsigned int wCrc=0xFFFF;
	
	for(i=0;i<num;i++)
	{
		wCrc^=(unsigned int)(pBuf[i]);
		for(j=0;j<8;j++)
		{
			if(wCrc&1)
			{
				wCrc>>=1;
				wCrc^=0xA001; 
			}
			else
				wCrc >>= 1; 
		}
	}
	return wCrc;
}
void modbus_init(uint8_t masterid,uint8_t salverid)
{
		memset(&mod,0,sizeof(mod));
		memset(modbus_buf,0,sizeof(modbus_buf));
		mod.master_id = masterid;
		mod.salver_id = salverid;
		mod.buf = modbus_buf;
	  mod.registe = REG_BUF;
		mod.start = 0;
		mod.end = 0;
}
/***************************************************************************
*
*
*串口中断接收数据，并且把数据放入环形队列当中
*
*
***************************************************************************/
#if MODBUS_EN
void Uart1_IRQHandler(void)
{
		uint8_t reg;
    if(TRUE == Uart_GetStatus(M0P_UART1, UartRC))
    {
        Uart_ClrStatus(M0P_UART1, UartRC);

        reg = Uart_ReceiveData(M0P_UART1);
			
				if(mod.start == 0)
				{
					if(reg == mod.master_id)
						mod.start = 1;
					else goto end;
				}
				if(mod.start == 1)
				{
					mod.buf[mod.number] = reg;
					
					if(mod.number == 1){
						mod.funcode = reg;
						__NOP();
					}
					switch(mod.funcode)
					{
						case(0x00):
							if(mod.number >= 1){
							modbus_init(ID_master,ID_slave);
							goto end;
							}
							break;
						case(0x03)://读寄存器
												if(mod.number == 2){
													mod.startreg |= (reg << 8);
													mod.start_reg[0]=reg;
												}
												if(mod.number == 3){
													mod.startreg |= (reg);
													mod.start_reg[1]=reg;
												}
												if(mod.number == 4){
													mod.regnum |= (reg << 8);
													mod.reg_num[0] = reg;
												}
												if(mod.number == 5){
													mod.regnum |= (reg);
													mod.reg_num[1] = reg;
												}
												
												if(mod.number == 6)
													mod.master_crc[0] = reg;
												if(mod.number == 7){
													mod.master_crc[1] = reg;
													mod.countcrc = Crc16(mod.buf,mod.number-1);
													mod.count_crc[0] = (mod.countcrc )&0xff;
													mod.count_crc[1] = (mod.countcrc >> 8)&0xff;
													if(mod.count_crc[0] == mod.master_crc[0] && mod.count_crc[1] == mod.master_crc[1])
													{
														mod.end = 1;
														mod.start = 0;
														ringfifo_putin(&uart1_rx_ringfifo,mod.buf,mod.number+1);
													}
												}
												if(mod.number >= 8){
													modbus_init(ID_master,ID_slave);
												 goto end;
												}
							break;
						case(0x06)://写一个寄存器
												if(mod.number == 2){
													mod.startreg |= (reg << 8);
													mod.start_reg[0]=reg;
												}
												if(mod.number == 3){
													mod.startreg |= (reg);
													mod.start_reg[1]=reg;
												}
												if(mod.number == 6)
													mod.master_crc[0] = reg;
												if(mod.number == 7){
													mod.master_crc[1] = reg;
													mod.countcrc = Crc16(mod.buf,mod.number-1);
													mod.count_crc[0] = (mod.countcrc )&0xff;
													mod.count_crc[1] = (mod.countcrc >> 8)&0xff;
													if(mod.count_crc[0] == mod.master_crc[0] && mod.count_crc[1] == mod.master_crc[1])
													{
														mod.end = 1;
														mod.start = 0;
														ringfifo_putin(&uart1_rx_ringfifo,mod.buf,mod.number+1);
													}
												}
												if(mod.number >= 8){
													modbus_init(ID_master,ID_slave);
												 goto end;
												}
							break;
						case(0x10)://写多个寄存器
												if(mod.number == 2){
													mod.startreg |= (reg << 8);
													mod.start_reg[0]=reg;
												}
												if(mod.number == 3){
													mod.startreg |= (reg);
													mod.start_reg[1]=reg;
												}
												if(mod.number == 4){
													mod.regnum |= (reg << 8);
													mod.reg_num[0] = reg;
												}
												if(mod.number == 5){
													mod.regnum |= (reg);
													mod.reg_num[1] = reg;
												}
												if(mod.number == 6)
													mod.Bitnum = reg;//6
												
//												if(mod.number >= 7 && bit<mod.Bitnum)// 0 1 , 2 3 , 4 5
//												{
//													if(bit%2 == 0)
//													mod.registe[mod.startreg+(bit/2)] |= (reg << 8) & 0xff00;
//													else if(bit%2 == 1)
//													mod.registe[mod.startreg+(bit/2)] |= (reg) & 0xff;
//													bit++;
//												}
												
												if(mod.number == 7+mod.Bitnum)
													mod.master_crc[0] = reg;
												if(mod.number == 8+mod.Bitnum){
													mod.master_crc[1] = reg;
													mod.countcrc = Crc16(mod.buf,mod.number-1);
													mod.count_crc[0] = (mod.countcrc )&0xff;
													mod.count_crc[1] = (mod.countcrc >> 8)&0xff;
													if(mod.count_crc[0] == mod.master_crc[0] && mod.count_crc[1] == mod.master_crc[1])
													{
														mod.end = 1;
														mod.start = 0;
														ringfifo_putin(&uart1_rx_ringfifo,mod.buf,mod.number+1);
													}
												}
												if(mod.number >= 9+mod.Bitnum){
													modbus_init(ID_master,ID_slave);
												 goto end;
												}
							break;
						default:
							modbus_init(ID_master,ID_slave);
							goto end;
					}
					
					mod.number++;
					__NOP();
				}
				end :;
    }
}
#endif
