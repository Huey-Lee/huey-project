#ifndef __PLAT_AEROLINK_H
#define __PLAT_AEROLINK_H

#include "main.h"

/* Э�鳣�� - �ϸ�ƥ�� 17 �ֽ� */
#define TREAD_HEAD1      0xAA
#define TREAD_HEAD2      0xBB
#define TREAD_TAIL1      0xEE
#define TREAD_TAIL2      0xFF
#define TREAD_CID_SEND   0x0A  // �Ͽ� -> �¿�
#define TREAD_CID_RECV   0x09  // �¿� -> �Ͽ�

/* ������ FC ���� */
#define FC_HEARTBEAT     0x01  // ����/״̬
#define FC_CONTROL       0x02  // ��ͣ����
#define FC_GEAR          0x03  // ��λ����
#define FC_TEST          0xFF  // ��������


/* �ṹ������֡ */
typedef struct {
    uint8_t cid;
    uint8_t fc;
    uint8_t sfc;
    uint8_t data[8]; // ��Ӧͼ���е� ����1H~����4L
} AeroFrame_t;

/* API */
void AeroLink_Handler(void); 
void AeroLink_Send(uint8_t fc, uint8_t sfc, uint8_t *pData);
/** �¿ؼ�ͣ�̶� 5 �ֽڣ���Э�������ָͣ�һ�£�AA BB 0A 02 02�� */
void AeroLink_SendEmergencyStopRaw(void);
extern void AeroLink_OnFrameReceived(AeroFrame_t *f); // ҵ��ص�

#endif
