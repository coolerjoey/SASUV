#ifndef _TIMER_H
#define _TIMER_H
#include "sys.h"
	

#define arr		9999	//脉冲周期为1us*10 000=10ms
#define psc   83		//设置用来作为TIMx时钟频率除数的预分频值,1MHz,时钟1us

void Motor_Timer3_Init(void);		//定时器初始化函数
void Motor_PWM_Set_Min(void);		//设置脉冲最低宽度
void Motor_PWM_Set_Max(void);		//设置脉冲最高宽度
void Motor_PWM_Set_Val(u16 val0,u16 val1,u16 val2,u16 val3);    //设置脉冲制定宽度
void Motor_Set_Throttle_range(void);
#endif
