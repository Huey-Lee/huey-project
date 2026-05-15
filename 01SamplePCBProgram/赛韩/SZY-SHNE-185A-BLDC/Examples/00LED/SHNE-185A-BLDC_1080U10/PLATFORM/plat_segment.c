#include "plat_segment.h"

static const uint8_t Seg_Num[] = {
    SEG_RAW(_SEG_A|_SEG_B|_SEG_C|_SEG_D|_SEG_E|_SEG_F),
    SEG_RAW(_SEG_B|_SEG_C),
    SEG_RAW(_SEG_A|_SEG_B|_SEG_G|_SEG_E|_SEG_D),
    SEG_RAW(_SEG_A|_SEG_B|_SEG_C|_SEG_D|_SEG_G),
    SEG_RAW(_SEG_F|_SEG_G|_SEG_B|_SEG_C),
    SEG_RAW(_SEG_A|_SEG_F|_SEG_G|_SEG_C|_SEG_D),
    SEG_RAW(_SEG_A|_SEG_F|_SEG_E|_SEG_D|_SEG_C|_SEG_G),
    SEG_RAW(_SEG_A|_SEG_B|_SEG_C),
    SEG_RAW(_SEG_A|_SEG_B|_SEG_C|_SEG_D|_SEG_E|_SEG_F|_SEG_G),
    SEG_RAW(_SEG_A|_SEG_B|_SEG_C|_SEG_D|_SEG_F|_SEG_G)
};

uint8_t Disp_GetDigitCode(uint8_t digit)
{
    if (digit > 9) return SEG_RAW(SEG_OFF);
    return Seg_Num[digit];
}

uint8_t Disp_AddDot(uint8_t raw_code)
{
#if DISP_COMMON_CATHODE
    return (uint8_t)(raw_code | _SEG_DP);
#else
    return (uint8_t)(raw_code & ~_SEG_DP);
#endif
}
