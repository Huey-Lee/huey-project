#include "plat_rf_store.h"
#include "treadmills.h"

#if TM_RF_PAIRING_ENABLE

#include "cw32l010_flash.h"

/* CW32L010：512 字节/页，64KB Flash 末页，勿与应用程序重叠（通常固件远小于 0xFE00） */
#define RFSTORE_FLASH_PAGE   127u
#define RFSTORE_FLASH_ADDR   ((uint32_t)(RFSTORE_FLASH_PAGE * 512u))

#define RFSTORE_MAGIC        0xA55Au

typedef struct __attribute__((packed)) {
    uint16_t magic;
    uint16_t addr;
} RFStore_Record_t;

static _Bool     s_rf_bound;
static uint16_t  s_rf_addr;

void RFStore_Init(void)
{
    const RFStore_Record_t *rec = (const RFStore_Record_t *)(uint32_t)RFSTORE_FLASH_ADDR;

    s_rf_bound = 0;
    s_rf_addr  = 0;
    if (rec->magic == (uint16_t)RFSTORE_MAGIC) {
        s_rf_bound = 1;
        s_rf_addr  = rec->addr;
    }
}

_Bool RFStore_IsBound(void)
{
    return s_rf_bound;
}

uint16_t RFStore_GetAddr(void)
{
    return s_rf_addr;
}

uint8_t RFStore_WritePairedAddr(uint16_t addr)
{
    RFStore_Record_t rec;
    uint8_t          st;

    rec.magic = (uint16_t)RFSTORE_MAGIC;
    rec.addr  = addr;

    __disable_irq();
    FLASH_UnlockAllPages();
    st = FLASH_ErasePage(RFSTORE_FLASH_PAGE);
    if (st == 0u) {
        st = FLASH_WriteBytes(RFSTORE_FLASH_ADDR, (uint8_t *)&rec, (uint16_t)sizeof(rec));
    }
    FLASH_LockAllPages();
    __enable_irq();

    if (st == 0u) {
        s_rf_bound = 1;
        s_rf_addr  = addr;
    }
    return st;
}

#else /* !TM_RF_PAIRING_ENABLE */

void RFStore_Init(void) {}

_Bool RFStore_IsBound(void)
{
    return 0;
}

uint16_t RFStore_GetAddr(void)
{
    return 0u;
}

uint8_t RFStore_WritePairedAddr(uint16_t addr)
{
    (void)addr;
    return 0u;
}

#endif /* TM_RF_PAIRING_ENABLE */
