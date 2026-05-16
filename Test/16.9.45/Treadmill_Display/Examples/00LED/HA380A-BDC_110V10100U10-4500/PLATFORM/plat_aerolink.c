#include "plat_aerolink.h"
#include "sys_aerobuf.h"
#include "bsp_uart.h"

typedef enum {
    RX_WAIT_STX = 0,
    RX_CMD,
    RX_LEN,
    RX_DATA,
    RX_CS_H,
    RX_CS_L,
    RX_ETX
} AeroRxState_t;

static uint8_t AeroLink_Checksum(const uint8_t *data, uint8_t len)
{
    uint8_t x = 0;
    for (uint8_t i = 0; i < len; i++) {
        x ^= data[i];
    }
    return x;
}

/* XOR of CMD, LEN, payload (same as BDC uart_frame_tx / uart_frame_loop). */
static uint8_t AeroLink_ChecksumXor(uint8_t cmd, uint8_t len, const uint8_t *pData)
{
    uint8_t buf[2 + AEROLINK_MAX_DATA];
    uint8_t i = 0;
    buf[i++] = cmd;
    buf[i++] = len;
    for (uint8_t j = 0; j < len; j++) {
        buf[i++] = (pData != (void *)0) ? pData[j] : 0u;
    }
    return AeroLink_Checksum(buf, i);
}

/* (CMD+LEN+sum(data))%7 — 与现场协议 7F 08 01 00 02 00 7E 一致: (8+1+0)%7=0x02 */
static uint8_t AeroLink_ChecksumMod7(uint8_t cmd, uint8_t len, const uint8_t *pData)
{
    uint16_t s = (uint16_t)cmd + (uint16_t)len;
    for (uint8_t j = 0; j < len; j++) {
        s += (pData != (void *)0) ? (uint16_t)pData[j] : 0u;
    }
    return (uint8_t)(s % 7u);
}

/* TX: READ_PARAM(8,1,byte) 使用 %7 得到 0x02；其它命令用异或。 */
static uint8_t AeroLink_ChecksumWireTx(uint8_t cmd, uint8_t len, const uint8_t *pData)
{
    if (cmd == CMD_READ_PARAM) {
        return AeroLink_ChecksumMod7(cmd, len, pData);
    }
    return AeroLink_ChecksumXor(cmd, len, pData);
}

/* RX: 上显/下显任一侧可能用异或或 %7 */
static int AeroLink_ChecksumWireMatchRx(uint8_t cmd, uint8_t len, const uint8_t *pData, uint8_t wire)
{
    return (wire == AeroLink_ChecksumXor(cmd, len, pData)) ||
           (wire == AeroLink_ChecksumMod7(cmd, len, pData));
}

static void AeroLink_BuildChecksumBytes(uint8_t cmd, uint8_t len, const uint8_t *pData,
                                        uint8_t *out_h, uint8_t *out_l)
{
    *out_h = AeroLink_ChecksumWireTx(cmd, len, pData);
    *out_l = 0;
}

void AeroLink_Send(uint8_t cmd, uint8_t len, const uint8_t *pData)
{
    uint8_t cs_h, cs_l;

    if (len > AEROLINK_MAX_DATA) {
        len = AEROLINK_MAX_DATA;
    }
    AeroLink_BuildChecksumBytes(cmd, len, pData, &cs_h, &cs_l);

    BSP_Uart_WriteByte(CW_UART1, AEROLINK_STX);
    BSP_Uart_WriteByte(CW_UART1, cmd);
    BSP_Uart_WriteByte(CW_UART1, len);
    for (uint8_t i = 0; i < len; i++) {
        uint8_t v = (pData != (void *)0) ? pData[i] : 0u;
        BSP_Uart_WriteByte(CW_UART1, v);
    }
    BSP_Uart_WriteByte(CW_UART1, cs_h);
    BSP_Uart_WriteByte(CW_UART1, cs_l);
    BSP_Uart_WriteByte(CW_UART1, AEROLINK_ETX);
}

void AeroLink_Handler(void)
{
    static AeroRxState_t state = RX_WAIT_STX;
    static AeroRecvFrame_t rx_f;
    static uint8_t d_idx = 0;
    uint8_t byte;

    while (AeroBuf_Pop(&uart1_rx_buf, &byte)) {
        switch (state) {
            case RX_WAIT_STX:
                if (byte == AEROLINK_STX) {
                    state = RX_CMD;
                }
                break;

            case RX_CMD:
                rx_f.cmd = byte;
                state = RX_LEN;
                break;

            case RX_LEN:
                rx_f.len = byte;
                if (rx_f.len > AEROLINK_MAX_DATA) {
                    state = RX_WAIT_STX;
                    if (byte == AEROLINK_STX) {
                        state = RX_CMD;
                    }
                } else {
                    d_idx = 0;
                    state = (rx_f.len > 0u) ? RX_DATA : RX_CS_H;
                }
                break;

            case RX_DATA:
                rx_f.data[d_idx++] = byte;
                if (d_idx >= rx_f.len) {
                    state = RX_CS_H;
                }
                break;

            case RX_CS_H:
                if (!AeroLink_ChecksumWireMatchRx(rx_f.cmd, rx_f.len, rx_f.data, byte)) {
                    state = RX_WAIT_STX;
                    if (byte == AEROLINK_STX) {
                        state = RX_CMD;
                    }
                } else {
                    state = RX_CS_L;
                }
                break;

            case RX_CS_L:
                /* 与协议示例一致：校验低位为 0 */
                if (byte != 0u) {
                    state = RX_WAIT_STX;
                    if (byte == AEROLINK_STX) {
                        state = RX_CMD;
                    }
                } else {
                    state = RX_ETX;
                }
                break;

            case RX_ETX:
                state = RX_WAIT_STX;
                if (byte == AEROLINK_ETX) {
                    AeroLink_OnFrameReceived(&rx_f);
                } else if (byte == AEROLINK_STX) {
                    state = RX_CMD;
                }
                break;
        }
    }
}
