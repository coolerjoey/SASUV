#ifndef _TIMER_H
#define _TIMER_H
#include "sys.h"
	

#define arr		9999	//��������Ϊ1us*10 000=10ms
#define psc   83		//����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ,1MHz,ʱ��1us

void Motor_Timer3_Init(void);		//��ʱ����ʼ������
void Motor_PWM_Set_Min(void);		//����������Ϳ��
void Motor_PWM_Set_Max(void);		//����������߿��
void Motor_PWM_Set_Val(u16 val0,u16 val1,u16 val2,u16 val3);    //���������ƶ����
void Motor_Set_Throttle_range(void);
#endif
