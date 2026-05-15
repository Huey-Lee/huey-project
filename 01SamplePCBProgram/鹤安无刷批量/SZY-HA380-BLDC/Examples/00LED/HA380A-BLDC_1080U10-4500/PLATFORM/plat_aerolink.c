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

static uint16_t CalcChecksum(uint8_t cid, uint8_t fc, uint8_t sfc, const uint8_t *data8)
{
    uint16_t sum = (uint16_t)cid + (uint16_t)fc + (uint16_t)sfc;
    for (uint8_t i = 0u; i < 8u; i++) {
        sum += (uint16_t)data8[i];
    }
    return sum;
}

void AeroLink_Handler(void)
{
    static LinkState_t state = S_HEAD1;
    static AeroFrame_t rx;
    static uint16_t    cs_calc = 0u, cs_match = 0u;
    static uint8_t     d_idx  = 0u;
    uint8_t            byte;

    while (AeroBuf_Pop(&uart1_rx_buf, &byte)) {
        switch (state) {
            case S_HEAD1:
                state = (byte == TREAD_HEAD1) ? S_HEAD2 : S_HEAD1;
                break;
            case S_HEAD2:
                state = (byte == TREAD_HEAD2) ? S_CID : S_HEAD1;
                break;
            case S_CID:
                if (byte == TREAD_CID_RECV) {
                    rx.cid = byte;
                    cs_calc = (uint16_t)byte;
                    state = S_FC;
                } else {
                    state = S_HEAD1;
                }
                break;
            case S_FC:
                rx.fc   = byte;
                cs_calc += (uint16_t)byte;
                state = S_SFC;
                break;
            case S_SFC:
                rx.sfc = byte;
                cs_calc += (uint16_t)byte;
                d_idx  = 0u;
                state  = S_DATA;
                break;
            case S_DATA:
                rx.data[d_idx++] = byte;
                cs_calc += (uint16_t)byte;
                if (d_idx >= 8u) {
                    state = S_CS_H;
                }
                break;
            case S_CS_H:
                cs_match = (uint16_t)byte << 8;
                state    = S_CS_L;
                break;
            case S_CS_L:
                cs_match |= (uint16_t)byte;
                state = (cs_match == cs_calc) ? S_TAIL1 : S_HEAD1;
                break;
            case S_TAIL1:
                state = (byte == TREAD_TAIL1) ? S_TAIL2 : S_HEAD1;
                break;
            case S_TAIL2:
                if (byte == TREAD_TAIL2) {
                    AeroLink_OnFrameReceived(&rx);
                }
                state = S_HEAD1;
                break;
        }
    }
}

void AeroLink_Send(uint8_t fc, uint8_t sfc, uint8_t *pData)
{
    uint8_t  payload[8] = {0};
    uint16_t cs;

    if (pData) {
        for (uint8_t i = 0u; i < 8u; i++) {
            payload[i] = pData[i];
        }
    }

    cs = CalcChecksum(TREAD_CID_SEND, fc, sfc, payload);

    BSP_Uart_WriteByte(CW_UART1, TREAD_HEAD1);
    BSP_Uart_WriteByte(CW_UART1, TREAD_HEAD2);
    BSP_Uart_WriteByte(CW_UART1, TREAD_CID_SEND);
    BSP_Uart_WriteByte(CW_UART1, fc);
    BSP_Uart_WriteByte(CW_UART1, sfc);
    for (uint8_t i = 0u; i < 8u; i++) {
        BSP_Uart_WriteByte(CW_UART1, payload[i]);
    }
    BSP_Uart_WriteByte(CW_UART1, (uint8_t)(cs >> 8));
    BSP_Uart_WriteByte(CW_UART1, (uint8_t)(cs & 0xFFu));
    BSP_Uart_WriteByte(CW_UART1, TREAD_TAIL1);
    BSP_Uart_WriteByte(CW_UART1, TREAD_TAIL2);
}
