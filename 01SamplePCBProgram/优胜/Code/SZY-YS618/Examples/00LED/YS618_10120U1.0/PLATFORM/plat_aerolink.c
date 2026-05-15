#include "plat_aerolink.h"
#include "sys_aerobuf.h"
#include "bsp_uart.h"

typedef enum { 
		S_HEAD1, 
		S_HEAD2, 
		S_CID, 
		S_FC, 
		S_SFC, 
		S_DATA, 
		S_CS_H, 
		S_CS_L, 
		S_TAIL1, 
		S_TAIL2 
} LinkState_t;

/**
 * @brief �ڲ�У����㣺CID + 8λ�����ۼ�
 */
static uint16_t CalcChecksum(uint8_t cid, uint8_t fc, uint8_t sfc, uint8_t *data) {
    uint16_t sum = cid;
	sum += fc;
    sum += sfc;
    for (uint8_t i = 0; i < 8; i++) sum += data[i];
    return sum;
}

/**
 * @brief ����״̬��
 */
void AeroLink_Handler(void) {
    static LinkState_t state = S_HEAD1;
    static AeroFrame_t rx;
    static uint16_t cs_calc = 0, cs_match = 0;
    static uint8_t d_idx = 0;
    uint8_t byte;

    while (AeroBuf_Pop(&uart1_rx_buf, &byte)) {
        switch (state) {
            case S_HEAD1: if (byte == TREAD_HEAD1) state = S_HEAD2; break;
            case S_HEAD2: state = (byte == TREAD_HEAD2) ? S_CID : S_HEAD1; break;
            case S_CID: 
                if (byte == TREAD_CID_RECV) {
                    rx.cid = byte; cs_calc = byte; state = S_FC;
                } else state = S_HEAD1;
                break;
            case S_FC:  rx.fc = byte;  cs_calc += byte;  state = S_SFC;  break; 
            case S_SFC: rx.sfc = byte; cs_calc += byte;  d_idx = 0; state = S_DATA; break; 
            case S_DATA: 
                rx.data[d_idx++] = byte; cs_calc += byte; // ����λ����У��
                if (d_idx >= 8) state = S_CS_H;
                break;
            case S_CS_H: cs_match = (uint16_t)byte << 8; state = S_CS_L; break;
            case S_CS_L: 
                cs_match |= byte;
                state = (cs_match == cs_calc) ? S_TAIL1 : S_HEAD1; // У�����
                break;
            case S_TAIL1: state = (byte == TREAD_TAIL1) ? S_TAIL2 : S_HEAD1; break;
            case S_TAIL2: 
                if (byte == TREAD_TAIL2) AeroLink_OnFrameReceived(&rx);
                state = S_HEAD1;
                break;
        }
    }
}

/**
 * @brief �����߼�
 */
void AeroLink_Send(uint8_t fc, uint8_t sfc, uint8_t *pData) {
    uint8_t payload[8] = {0};
    if (pData) for(uint8_t i=0; i<8; i++) payload[i] = pData[i];
    uint16_t cs =  CalcChecksum(TREAD_CID_SEND, fc, sfc, payload);

    BSP_Uart_WriteByte(CW_UART1, TREAD_HEAD1);
    BSP_Uart_WriteByte(CW_UART1, TREAD_HEAD2);
    BSP_Uart_WriteByte(CW_UART1, TREAD_CID_SEND);
    BSP_Uart_WriteByte(CW_UART1, fc);
    BSP_Uart_WriteByte(CW_UART1, sfc);
    for (uint8_t i = 0; i < 8; i++) BSP_Uart_WriteByte(CW_UART1, payload[i]);
    BSP_Uart_WriteByte(CW_UART1, (uint8_t)(cs >> 8));
    BSP_Uart_WriteByte(CW_UART1, (uint8_t)(cs & 0xFF));
    BSP_Uart_WriteByte(CW_UART1, TREAD_TAIL1);
    BSP_Uart_WriteByte(CW_UART1, TREAD_TAIL2);
}

void AeroLink_SendEmergencyStopRaw(void)
{
    static const uint8_t k_emergency[] = { 0xAAu, 0xBBu, 0x0Au, 0x02u, 0x02u };
    for (uint8_t i = 0; i < (uint8_t)sizeof(k_emergency); i++)
        BSP_Uart_WriteByte(CW_UART1, k_emergency[i]);
}
