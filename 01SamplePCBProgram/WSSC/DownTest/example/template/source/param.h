/*
 * param.h
 *
 *  Created on: 2020年3月9日
 *      Author: Administrator
 */
#ifndef USER_PARAM_H_
#define USER_PARAM_H_
#include "common.h"

#define SPEED_ADC_MAX  (5)
#define SPEED_ADC_MIN  (-5)

#define TIME_MAX       (5940)
#define TIME_MIN       (300)

#define DISTANCE_MAX  (99000)//99.00
#define DISTANCE_MIN  (1000)

#define CALORIE_MAX   (990000)
#define CALORIE_MIN   (20000)

#define PARAM_SET_SPEED_ADC  (0)
#define PARAM_SET_TIME       (1)
#define PARAM_SET_DISTANCE   (2)
#define PARAM_SET_CALORIE    (3)
#define PARAM_SET_TEMP       (4)
#define PARAM_SET_BEEP       (5) //蜂1鸣器开关   //2023.3.27


//设置参数的初始值
#define DEFAULT_SPEED_STEP           (1)     // 5一档 0.1
#define DEFAULT_SPEED_MAX            (100)   // 最高速2500转 / 10 ：5.67
#define DEFAULT_SPEED_MIN            (10)    // 最低速500转 /10

#define DEFAULT_SPEED_SINGLE         (1)     //速度单次加/减值
#define DEFAULT_DISTANCE_SINGLE      (1000)   //路程设置单次加/减距离值


//#define DEFAULT_SET_KIV_1KM_VALUE    (3)     //调试先随机取的，后续此值由下控决定好了，写统一
//#define DEFAULT_SET_KIV_2KM_VALUE    (3)
//#define DEFAULT_SET_KIV_3KM_VALUE    (3)
//#define DEFAULT_SET_KIV_4KM_VALUE    (3)
//#define DEFAULT_SET_KIV_5KM_VALUE    (3)
//#define DEFAULT_SET_KIV_6KM_VALUE    (3)
//#define DEFAULT_SET_KIV_7KM_VALUE    (3)
//#define DEFAULT_SET_KIV_8KM_VALUE    (3)
//#define DEFAULT_SET_KIV_9KM_VALUE    (3)
//#define DEFAULT_SET_KIV_10KM_VALUE   (3)
//#define DEFAULT_SET_KIV_11KM_VALUE   (3)
//#define DEFAULT_SET_KIV_12KM_VALUE   (3)


typedef struct _param_t
{
	u8 en;
	u8 index;
	s8 speed_adc;//用户需要调准档位 速度参数
	u8 book_en;
}param_t;

extern param_t param;
extern void param_init(void);

#endif /* USER_PARAM_H_ */
