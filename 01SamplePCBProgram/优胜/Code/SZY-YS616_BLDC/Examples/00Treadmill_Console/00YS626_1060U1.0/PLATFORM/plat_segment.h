/**
 * @file plat_segment.h
 * @brief 数码管段码抽象层 - 工业级标准
 */

#ifndef __PLAT_SEGMENT_H
#define __PLAT_SEGMENT_H

#include <stdint.h>
// 正常
//      --- a ---
//     |         |
//     f         b
//     |         |
//      --- g ---
//     |         |
//     e         c
//     |         |
//      --- d ---    dp	
//当前
//      --- b ---
//     |         |
//     a         c
//     |         |
//      --- d ---
//     |         |
//     f         e
//     |         |
//      --- g ---     dp


/* 物理引脚到字节位的映射 (根据 PCB 走线修改) */
#define _SEG_F    (1 << 1)//(1 << 0)//
#define _SEG_A    (1 << 2)//(1 << 1)//
#define _SEG_B    (1 << 3)//(1 << 2)//
#define _SEG_C    (1 << 5)//(1 << 5)//
#define _SEG_D    (1 << 6)//(1 << 4)//
#define _SEG_E    (1 << 0)//(1 << 5)//
#define _SEG_G    (1 << 4)//(1 << 6)//
#define _SEG_DP   (1 << 7)

/* 硬件极性配置 (1: 共阴/高电平亮, 0: 共阳/低电平亮) */
#define DISP_COMMON_CATHODE    1 

/* 核心转换引擎 (SEG_RAW) */
#if DISP_COMMON_CATHODE
    #define SEG_RAW(x)  ((uint8_t)(x))         // 原样输出
#else
    #define SEG_RAW(x)  ((uint8_t)~(x))        // 位取反
#endif

#define SEG_U      (_SEG_B | _SEG_C | _SEG_D | _SEG_E | _SEG_F)
#define SEG_C      (_SEG_A | _SEG_D | _SEG_E | _SEG_F)
#define SEG_P      (_SEG_A | _SEG_B | _SEG_E | _SEG_F| _SEG_G)
#define SEG_E      (_SEG_A | _SEG_D | _SEG_E | _SEG_F| _SEG_G)


#define SEG_OFF    (0x00)                                       // 全灭

/* 外部接口 */

/**
 * @brief 获取数字 0-9 的硬件段码
 */
uint8_t Disp_GetDigitCode(uint8_t digit);

/**
 * @brief 处理带小数点的段码
 * @param raw_code 原始段码 (来自上述接口)
 * @return 叠加了小数点后的硬件段码
 */
uint8_t Disp_AddDot(uint8_t raw_code);

#endif
