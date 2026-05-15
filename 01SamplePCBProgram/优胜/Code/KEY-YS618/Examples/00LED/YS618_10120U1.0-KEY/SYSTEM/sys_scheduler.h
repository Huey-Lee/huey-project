#ifndef __SYS_SCHEDULER_H
#define __SYS_SCHEDULER_H

#include "cw32l010.h"


typedef struct {
    void (*pTask)(void);    // ������ָ��
    uint32_t Delay;         // ��ʼ�ӳ٣�����ƽ̯CPU���أ�
    uint32_t Period;        // ִ������
    uint32_t RunMe;         // ���м�����>0ִ�У�
} sTask;

void Scheduler_Update(void);   // ��ʱ���е���
void Scheduler_Dispatch(void); // mainѭ���е���

void Task_1ms(void);
void Task_10ms(void);
void Task_100ms(void);
void Task_250ms(void);
void Task_300ms(void);
void Task_500ms(void);
void Task_1000ms(void);


#endif
