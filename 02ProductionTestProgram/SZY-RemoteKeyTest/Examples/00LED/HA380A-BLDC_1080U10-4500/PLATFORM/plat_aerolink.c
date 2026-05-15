/* Demo：不解析下控协议，仅排空 UART1 接收缓冲，避免溢出 */
#include "plat_aerolink.h"
#include "sys_aerobuf.h"
#include "bsp_uart.h"

void AeroLink_Handler(void)
{
    uint8_t b;
    while (AeroBuf_Pop(&uart1_rx_buf, &b)) {
        /* discard */
    }
}

void AeroLink_OnFrameReceived(AeroFrame_t *f)
{
    (void)f;
}

void AeroLink_Send(uint8_t fc, uint8_t sfc, uint8_t *pData)
{
    (void)fc;
    (void)sfc;
    (void)pData;
}
