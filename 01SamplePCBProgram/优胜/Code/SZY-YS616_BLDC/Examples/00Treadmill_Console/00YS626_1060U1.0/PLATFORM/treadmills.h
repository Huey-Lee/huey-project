#ifndef __TREADMILLS_H
#define __TREADMILLS_H

#include "main.h"


/* 全局参数配置区 (在此统一修改) */
// 时间设定
#define TM_TIME_MIN      60      		// 60秒 (即 1分钟m)
#define TM_TIME_MAX      5940    		// 99分钟(秒)
#define TM_TIME_STEP     60      		// 步长1分钟
#define TM_TIME_SET      1800      		

// 路程设定
#define TM_DIST_MIN      1000.0f 		// 1000米 (即 1.0 km) 
#define TM_DIST_MAX      99000.0f
#define TM_DIST_STEP	 1000.0f
#define TM_DIST_SET      1000.0f      		

// 卡路里设定
#define TM_CAL_MIN       10000.0f  	 // 50000卡 (即 50 大卡)   
#define TM_CAL_MAX       990000.0f    
#define TM_CAL_STEP      10000.0f   
#define TM_CAL_SET     	 50000.0f      		

#define TM_SPEED_MIN     10     
#define TM_SPEED_MAX     60    
#define TM_SPEED_STEP    1  

/* 卡路里算法常数 */
#define CAL_PER          70.0f       // 70 kcal/km
 

/* 系统主状态 */
typedef enum {
    TM_STATE_BOOT = 0,			// 开机引导
    TM_STATE_IDLE,          // 待机空闲
    TM_STATE_SETTING,       // 模式设置 (M键进入)
    TM_STATE_COUNTDOWN,     // 3-2-1 倒计时
    TM_STATE_RUNNING,       // 运行中
    TM_STATE_STOPPING,      // 停止中
    TM_STATE_ERROR          // 报错
} TM_State_t;

/* 显示与设置索引 */
typedef enum {
    DISP_SPEED = 0,     // 速度
    DISP_TIME,          // 时间
    DISP_DIST,          // 距离
    DISP_CAL,           // 卡路里
    DISP_NONE           // 待机或无焦点
} TM_DispIndex_t;

/* 目标倒数模式 */
typedef enum {
    TARGET_NONE = 0,        // 自由跑 (正计时)
    TARGET_TIME,            // 时间倒数
    TARGET_DIST,            // 距离倒数
    TARGET_CAL              // 卡路里倒数
} TargetMode_t;

typedef enum { 
	BOOT_FULL, 
	BOOT_VER_U, 
	BOOT_VER_C, 
	BOOT_DONE 
} BootPhase_t;

/* 核心控制结构体 */
typedef struct {
    TM_State_t    	state;
    BootPhase_t   	boot_phase;
	TargetMode_t    target_mode;
    TM_DispIndex_t 	disp_index;  		// 统一的索引：设置时表示设置项，运行时表示轮显项
	
	/* 核心 Model 数据 */																	// 0:速度, 1:时间, 2:距离, 3:卡路里
    uint16_t     	speed;						// 速度值
    uint32_t  		time;					// 时间值
		float  				dist;							// 路程值
		float  				cal;							// 卡路里值    
    
	/* 系统控制位 */
		uint16_t  		timer_100ms;    	// 通用计时器 (引导/倒计时/OSD)
		uint8_t  			steady_timer; 	  	// 调节常亮计时器(100ms为单位)	
		uint16_t 			cycle_timer; 		// 轮显计数器 (100ms为单位)  
		_Bool     		safety_key;				// 安全锁
    uint16_t  		error_code;				// 错误码
} TM_Control_t;

extern TM_Control_t g_TM;

void Treadmill_Init(void);
void Treadmill_Manager_100ms(void);
void Treadmill_On_Event(uint8_t key_id, uint8_t evt); // 统一事件接口
#endif
