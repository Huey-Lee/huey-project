#ifndef __PLAT_SAFETYLOCK_H
#define __PLAT_SAFETYLOCK_H

#include "main.h"

/* 10ms 调度中调用：去抖 + 边沿触发业务回调，不阻塞 */
void SafetyLock_Poll_10ms(void);

#endif
