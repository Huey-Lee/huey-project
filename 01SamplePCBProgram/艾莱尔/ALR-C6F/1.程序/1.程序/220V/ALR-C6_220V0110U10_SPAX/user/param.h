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

#define TIME_MAX       (5940)     //模式设置最大时间60min//99min
#define TIME_MIN       (300)      //模式设置最小时间5min

#define DISTANCE_MAX   (99000)    //99KM，模式设置最大距离16km
#define DISTANCE_MIN   (1000)      //模式设置最小距离0.5km

#define CALORIE_MAX    (999000)   //990KCAL，模式设置最大卡路里160
#define CALORIE_MIN    (20000)    //模式设置最小卡路里20
#define CALORIE_LOW_MIN (20000)

#define CALORIE_MASK    (1000000) //最大值999000

#define PARAM_SET_SPEED_ADC  (0)  //速度模式
#define PARAM_SET_DISTANCE   (1)
#define PARAM_SET_CALORIE    (2)
#define PARAM_SET_TIME       (3)

//设置参数的初始值
#define DEFAULT_SPEED_STEP             (10)   //(50)
#define DEFAULT_SPEED_MAX              (1000)  //此处修改记得和treadmills.h里的最大速度一同修改
#define DEFAULT_SPEED_MIN              (100)  //最小速度
#define DEFAULT_VOLTAGE_MAX            (172)  //此处修改记得和treadmills.h里的电压一同修改
#define DEFAULT_VOLTAGE_MIN            (40)   //此处修改记得和treadmills.h里的电压一同修改
#define DEFAULT_OVER_CURRENT_MAX       (30)   //


#define DEFAULT_SET_KIV_1KM_VALUE    (6)     //调试先随机取的，后续此值由下控决定好了，写统一
#define DEFAULT_SET_KIV_2KM_VALUE    (5)
#define DEFAULT_SET_KIV_3KM_VALUE    (4)  
#define DEFAULT_SET_KIV_4KM_VALUE    (3)  
#define DEFAULT_SET_KIV_5KM_VALUE    (3) 
#define DEFAULT_SET_KIV_6KM_VALUE    (3)
#define DEFAULT_SET_KIV_7KM_VALUE    (3)
#define DEFAULT_SET_KIV_8KM_VALUE    (3)
#define DEFAULT_SET_KIV_9KM_VALUE    (3)
#define DEFAULT_SET_KIV_10KM_VALUE   (3)
#define DEFAULT_SET_KIV_11KM_VALUE   (3)
#define DEFAULT_SET_KIV_12KM_VALUE   (3)


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
