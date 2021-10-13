#ifndef _MOTORS_H
#define _MOTORS_H

#ifdef __cplusplus
 extern "C" {
#endif
	 
#include "sys.h"

#define MOTORS_MAX_NUMER 6	//电机个数最大值

#define MOTOR_1 0
#define MOTOR_2 1
#define MOTOR_3 2
#define MOTOR_4 3
#define MOTOR_5 4
#define MOTOR_6 5
#define MOTOR_7 6
#define MOTOR_8 7


#define SYS_FRE 180

#define FRAME_HAIZHE1 0			//机架类型定义
	 
#define MOTOR_CH1_4 TIM4
#define MOTOR_CH5_8 TIM1
#define Motor_PWM_Init()	{TIM4_PWM_Init(20000-1,SYS_FRE/2-1);TIM1_PWM_Init(20000-1,SYS_FRE-1);}	//注意此时系统超频

typedef struct{
//	uint8_t armed_enable;   //电机解锁标志 
	float roll_fac;			//横滚角比例因子
	float pitch_fac;		//俯仰角比例因子
	float yaw_fac;			//偏航角比例因子
	float throttle_fac;		//深度比例因子
	float forward_fac;
	float lateral_fac;
}Motor;


extern u16 thrust_return[MOTORS_MAX_NUMER];	//遥控模式下电机油门回传
extern u16 motor_out[MOTORS_MAX_NUMER];	//最终输出pwm
extern	float roll_in;					//横滚输入通道
extern	float pitch_in;					//俯仰输入通道
extern	float yaw_in;						//偏航输入通道
extern	float throttle_in;			//油门输入通道
extern u16 throttle_min;		//最小油门
extern u16 throttle_max;		//最大油门

//extern Motor motor;

#define INPUT_MANUAL true
#define INPUT_AUTO	false

void Moter_Set_PWM_Pulse(u8 ch,u16 pwm);
void motors_output(void);
void set_roll(float roll_in ,bool is_manual);
void set_pitch(float pitch_in ,bool is_manual);
void set_yaw(float yaw_in ,bool is_manual);
void set_throttle(float throttle_in ,bool is_manual);
void set_forward(float _forward_in ,bool is_manual);
void set_lateral(float _lateral_in ,bool is_manual);
void set_test_pwm(u16 pwm_test[MOTORS_MAX_NUMER]);


void setup_motors(void);
void calc_thrust_to_pwm(void);
u16 motor_test_pwm(u8,u8);
void Motor_PWM_Set_Min(void);
void Motor_PWM_Set_Mid(void);
void Motor_PWM_Set_Max(void);
void Motor_Set_Throttle_range(void);
void Motor_ESC_Mount(void);

float get_roll(void);
float get_pitch(void);
float get_yaw(void);
float get_throttle(void);
float get_forward(void);
float get_lateral(void);
u16* get_motorout(void);

#ifdef __cplusplus
}
#endif

#endif

