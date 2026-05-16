#ifndef __PLAT_SEGMENT_H
#define __PLAT_SEGMENT_H

#include <stdint.h>

/* 7-seg: a..g = bit0..6, DP = bit7 (same as UI; TM1640 maps via UI_SEG_D_TO_SEG) */

#define _SEG_A    (1u << 0)
#define _SEG_B    (1u << 1)
#define _SEG_C    (1u << 2)
#define _SEG_D    (1u << 3)
#define _SEG_E    (1u << 4)
#define _SEG_F    (1u << 5)
#define _SEG_G    (1u << 6)
#define _SEG_DP   (1u << 7)

#define DISP_COMMON_CATHODE    1

#if DISP_COMMON_CATHODE
#define SEG_RAW(x)  ((uint8_t)(x))
#else
#define SEG_RAW(x)  ((uint8_t)~(x))
#endif

#define SEG_U      (_SEG_B | _SEG_C | _SEG_D | _SEG_E | _SEG_F)
#define SEG_C      (_SEG_A | _SEG_D | _SEG_E | _SEG_F)
#define SEG_P      (_SEG_A | _SEG_B | _SEG_E | _SEG_F | _SEG_G)
#define SEG_E      (_SEG_A | _SEG_D | _SEG_E | _SEG_F | _SEG_G)
#define SEG_OFF    (0x00)

uint8_t Disp_GetDigitCode(uint8_t digit);
uint8_t Disp_AddDot(uint8_t raw_code);

#endif
