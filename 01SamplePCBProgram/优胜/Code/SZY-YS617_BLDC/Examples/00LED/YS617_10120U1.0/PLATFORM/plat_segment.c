/**
 * @file plat_segment.c
 */

#include "plat_segment.h"

// 段码表
// 数字段码表
static const uint8_t Seg_Num[] = {
    SEG_RAW(_SEG_A|_SEG_B|_SEG_C|_SEG_D|_SEG_E|_SEG_F),      	// 0
    SEG_RAW(_SEG_B|_SEG_C),                                  	// 1
    SEG_RAW(_SEG_A|_SEG_B|_SEG_G|_SEG_E|_SEG_D),             	// 2
    SEG_RAW(_SEG_A|_SEG_B|_SEG_C|_SEG_D|_SEG_G),             	// 3
    SEG_RAW(_SEG_F|_SEG_G|_SEG_B|_SEG_C),                    	// 4
    SEG_RAW(_SEG_A|_SEG_F|_SEG_G|_SEG_C|_SEG_D),               	// 5
    SEG_RAW(_SEG_A|_SEG_F|_SEG_E|_SEG_D|_SEG_C|_SEG_G),         // 6
    SEG_RAW(_SEG_A|_SEG_B|_SEG_C),                              // 7
    SEG_RAW(_SEG_A|_SEG_B|_SEG_C|_SEG_D|_SEG_E|_SEG_F|_SEG_G),  // 8
    SEG_RAW(_SEG_A|_SEG_B|_SEG_C|_SEG_D|_SEG_F|_SEG_G)          // 9
};

uint8_t Disp_GetDigitCode(uint8_t digit) {
    if (digit > 9) return SEG_RAW(SEG_OFF);
    return Seg_Num[digit];
}


uint8_t Disp_AddDot(uint8_t raw_code) {
    // 逻辑：如果是共阴，小数点位或1；如果是共阳，小数点位与0
    #if DISP_COMMON_CATHODE
        return (uint8_t)(raw_code | _SEG_DP);
    #else
        return (uint8_t)(raw_code & ~_SEG_DP);
    #endif
}
