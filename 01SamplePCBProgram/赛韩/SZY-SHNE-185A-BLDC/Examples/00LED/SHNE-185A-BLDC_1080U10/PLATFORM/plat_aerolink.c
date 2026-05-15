#include "plat_aerolink.h"
#include "sys_aerobuf.h"
#include "bsp_uart.h"

// 接收状态枚举
typedef enum {
    S_WAIT_H1 = 0, 
	S_WAIT_H2, 
	S_CID, 
	S_FC, 
	S_SFC, 
    S_DATA, 
	S_CS_H, 
	S_CS_L, 
	S_TAIL1, 
	S_TAIL2
} RecvState_t;

/**
 * @brief 接收处理引擎 (10ms 或 20ms 调度一次)
 */
void AeroLink_Handler(void) {
    static RecvState_t state = S_WAIT_H1;
    static AeroRecvFrame_t rx_f;
    static uint16_t cs_calc = 0, cs_target = 0;
    static uint8_t  d_idx = 0;
    uint8_t byte;

    // 只要环形队列里有数据，就全速处理
    while (AeroBuf_Pop(&uart1_rx_buf, &byte)) {
        switch (state) {
            case S_WAIT_H1:
                if (byte == TREAD_HEAD1) state = S_WAIT_H2;
                break;

            case S_WAIT_H2:
                state = ((byte == TREAD_HEAD2) || (byte == TREAD_HEAD2_ALT)) ? S_CID : S_WAIT_H1;
                break;

            case S_CID:
                if (byte == TREAD_CID_RECV) {
                    cs_calc = byte; // 开始计算校验和：CID + FC + SFC + Data
                    state = S_FC;
                } else state = S_WAIT_H1;
                break;

            case S_FC:
                rx_f.fc = byte; cs_calc += byte;
                state = S_SFC;
                break;

            case S_SFC:
                rx_f.sfc = byte; cs_calc += byte;
                d_idx = 0;
                state = S_DATA;
                break;

            case S_DATA:
                rx_f.data[d_idx++] = byte;
                cs_calc += byte;
                if (d_idx >= 8) state = S_CS_H;
                break;

            case S_CS_H:
                cs_target = (uint16_t)byte << 8;
                state = S_CS_L;
                break;

            case S_CS_L:
                cs_target |= byte;
                // 16位校验和比对
                if (cs_target == cs_calc) state = S_TAIL1;
                else state = S_WAIT_H1; // 校验失败，重置
                break;

            case S_TAIL1:
                state = (byte == TREAD_TAIL1) ? S_TAIL2 : S_WAIT_H1;
                break;

            case S_TAIL2:
                if (byte == TREAD_TAIL2) {
                    // 协议解析完成，派发给业务层
                    AeroLink_OnFrameReceived(&rx_f);
                }
                state = S_WAIT_H1;
                break;
        }
    }
}

/**
 * @brief 内部函数：计算校验和
 * 逻辑：控制码 + 功能码 + 子功能码 + 8位数据位 = 16位校验和
 */
static uint16_t AeroLink_CalcChecksum(uint8_t fc, uint8_t sfc, uint8_t *data) {
    uint16_t sum = TREAD_CID_SEND; // 初始值为 0x0A
	sum += fc;
    sum += sfc;
    for (uint8_t i = 0; i < 8; i++) {
        sum += data[i];
    }
    return sum;
}

/**
 * @brief 执行发送逻辑 (17字节严格对齐)
 */
void AeroLink_Send(uint8_t fc, uint8_t sfc, uint8_t *pData) {
    uint8_t payload[8] = {0};
    
    // 1. 准备 8 字节数据负载
    if (pData != (void*)0) {
        for(uint8_t i=0; i<8; i++) payload[i] = pData[i];
    }

    // 2. 计算校验码 (基于控制码和数据位)
    uint16_t cs = AeroLink_CalcChecksum(fc, sfc, payload);

    // 3. 顺序通过 UART1 发送 (下控物理口)
    BSP_Uart_WriteByte(CW_UART1, TREAD_HEAD1);     // 1: 0xAA
    BSP_Uart_WriteByte(CW_UART1, TREAD_HEAD2);     // 2: 0xBB
    BSP_Uart_WriteByte(CW_UART1, TREAD_CID_SEND);  // 3: 0x0A
    BSP_Uart_WriteByte(CW_UART1, fc);              // 4: 功能码
    BSP_Uart_WriteByte(CW_UART1, sfc);             // 5: 子功能码
    
    // 6-13: 数据位 (Data1_H/L, Data2_H/L, Data3_H/L, Data4_H/L)
    for (uint8_t i = 0; i < 8; i++) {
        BSP_Uart_WriteByte(CW_UART1, payload[i]);
    }

    // 14-15: 校验码 (高位在前)
    BSP_Uart_WriteByte(CW_UART1, (uint8_t)(cs >> 8));
    BSP_Uart_WriteByte(CW_UART1, (uint8_t)(cs & 0xFF));
    
    // 16-17: 帧尾
    BSP_Uart_WriteByte(CW_UART1, TREAD_TAIL1);     // 0xEE
    BSP_Uart_WriteByte(CW_UART1, TREAD_TAIL2);     // 0xFF
}
