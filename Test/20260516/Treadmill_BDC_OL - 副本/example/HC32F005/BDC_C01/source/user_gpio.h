/*
 * Function: 用户按键等 GPIO 与初始化接口。
 * Method:   user_gpio_init 配置输入与中断；与 Port3 向量匹配。
 * Name:     Huey
 * Date:     May 16, 2026 18:00
 */

#ifndef USER_GPIO_H
#define USER_GPIO_H
#include "hc32f005.h"

void user_gpio_init(void);

#endif
