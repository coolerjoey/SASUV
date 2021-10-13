#include "motors.h"
#include "LowPassFilter.h"
#include "global_para.h"
#include "delay.h"

extern SYS_FLAGS sys_flag;

Motor motor[MOTORS_MAX_NUMER];

u16 motor_out[MOTORS_MAX_NUMER];	//最终输出pwm
u16 motor_test[MOTORS_MAX_NUMER];	//测试pwm
float roll_in= 0.0;						//横滚输入通道
float pitch_in= 0.0;					//俯仰输入通道
float yaw_in= 0.0;						//偏航输入通道
float throttle_in= 0.0;				//油门输入通道
float forward_in = 0.0;
float lateral_in = 0.0;

//添加电机
void add_motor(uint8_t num, float roll_fac, float pitch_fac, float yaw_fac,float throttle_fac,float forward_fac, float lateral_fac){
	motor[num].roll_fac= roll_fac;
	motor[num].pitch_fac= pitch_fac;
	motor[num].yaw_fac= yaw_fac;
	motor[num].throttle_fac= throttle_fac;
	motor[num].forward_fac = forward_fac;
	motor[num].lateral_fac = lateral_fac;
}

/*电机模型初始化(只表示转动方向，不表示增益大小)
---------------------------
CW	<1500	反转		力向下
	>1500	正转		力向上
---------------------------
CCW	<1500	正转		力向下
	>1500	反转		力向上
---------------------------
即应该保持在相同的控制量下，正反奖推进器的推力方向是一致的
*/
void setup_motors(){
	switch(fp.frame_class){
//						Motor	Roll_Factor				Pitch_Factor			Yaw_Factor				Throttle_Factor				Forward_Factor				Lateral_Factor	
		case FRAME_HAIZHE1:
			add_motor(MOTOR_1, fp.motor1_fac.roll_fac, fp.motor1_fac.pitch_fac, fp.motor1_fac.yaw_fac, fp.motor1_fac.throttle_fac, fp.motor1_fac.forward_fac, fp.motor1_fac.lateral_fac);		
			add_motor(MOTOR_2, fp.motor2_fac.roll_fac, fp.motor2_fac.pitch_fac, fp.motor2_fac.yaw_fac, fp.motor2_fac.throttle_fac, fp.motor2_fac.forward_fac, fp.motor2_fac.lateral_fac);
			add_motor(MOTOR_3, fp.motor3_fac.roll_fac, fp.motor3_fac.pitch_fac, fp.motor3_fac.yaw_fac, fp.motor3_fac.throttle_fac, fp.motor3_fac.forward_fac, fp.motor3_fac.lateral_fac);
			add_motor(MOTOR_4, fp.motor4_fac.roll_fac, fp.motor4_fac.pitch_fac, fp.motor4_fac.yaw_fac, fp.motor4_fac.throttle_fac, fp.motor4_fac.forward_fac, fp.motor4_fac.lateral_fac);
			add_motor(MOTOR_5, fp.motor5_fac.roll_fac, fp.motor5_fac.pitch_fac, fp.motor5_fac.yaw_fac, fp.motor5_fac.throttle_fac, fp.motor5_fac.forward_fac, fp.motor5_fac.lateral_fac);
			add_motor(MOTOR_6, fp.motor6_fac.roll_fac, fp.motor6_fac.pitch_fac, fp.motor6_fac.yaw_fac, fp.motor6_fac.throttle_fac, fp.motor6_fac.forward_fac, fp.motor6_fac.lateral_fac);
			break;
		default:break;
	}
}

void calc_thrust_to_pwm(){
//	if(is_zero(forward_in) && vp.control_mode!=MANUAL){	//在原地定向时叠加定向死区	
//		if(fabs(yaw_in)<fp.throttle_deadzone_swerve){
//			yaw_in = yaw_in>0?yaw_in+fp.throttle_deadzone_swerve:yaw_in-fp.throttle_deadzone_swerve;
//		}
//	}
	if(vp.control_mode==STABILIZE && is_zero(forward_in)){	//静止定向时脱离推进器死区
		if(fabs(yaw_in)<fp.throttle_deadzone_swerve){
			yaw_in = yaw_in<0?-fp.throttle_deadzone_swerve:fp.throttle_deadzone_swerve;
		}
	}
	float motor_rpy_out[MOTORS_MAX_NUMER];
	for (u8 i=0; i<MOTORS_MAX_NUMER; i++) {
		if (sys_flag.motors_armed) 
		{
			motor_rpy_out[i] = roll_in * motor[i].roll_fac +
							 pitch_in * motor[i].pitch_fac +
							 yaw_in * motor[i].yaw_fac +
							 throttle_in * motor[i].throttle_fac +
							 forward_in * motor[i].forward_fac +
							 lateral_in * motor[i].lateral_fac;
			motor_rpy_out[i] = constrain_float(motor_rpy_out[i],-1.0f,1.0f);
			switch(fp.esc_type){
				case ONE_WAY:
					motor_out[i] = THRUST_MIN+ motor_rpy_out[i]*THRUST_RANGE;break;
				case TWO_WAY:
					motor_out[i] = 1500 + motor_rpy_out[i]*THRUST_RANGE/2;break;
			}		
		}
	}
}

//电机输出
void output_to_motors(){
	if(sys_flag.battery_voltage_low) return;
	if(sys_flag.motor_test){
		for(u8 i=0;i<MOTORS_MAX_NUMER;i++){
			Moter_Set_PWM_Pulse(i,motor_test[i]);
		}
		return;
	}
	if(!sys_flag.motors_armed || !sys_flag.relay_12V_enable){
		switch(fp.esc_type){
			case ONE_WAY:Motor_PWM_Set_Min();break;
			case TWO_WAY:Motor_PWM_Set_Mid();break;
		}
		return;
	}
	for(u8 i=0;i<MOTORS_MAX_NUMER;i++){
		Moter_Set_PWM_Pulse(i,motor_out[i]);
	}
}

//电机输出
void motors_output(){
	//每次都检查QGC是否修改电机模型参数
	setup_motors();
	//根据海蜇号模型计算各电机pwm波
	calc_thrust_to_pwm();
	//分配pwm给电机
	output_to_motors();
}

/*
	@func:设定各自由度输出
	@param
		is_manual: 表示调用此函数是手动(手柄,遥控器)输入还是控制算法(定向,定深)输入
					若是自动输入，需要检测手柄输入是否正常，防止和上位机或者遥控器失联后保持最后输入，无法停止
*/
#define manual_input_enable (sys_flag.health.gcs_heartbeat || sys_flag.ppm_connected)
void set_roll(float _roll_in, bool is_manual){
	if(is_manual) _roll_in *= manual_input_enable;
	roll_in = _roll_in;
}
void set_pitch(float _pitch_in, bool is_manual){
	if(is_manual) _pitch_in *= manual_input_enable;
	pitch_in = _pitch_in;
}
void set_yaw(float _yaw_in, bool is_manual){
	if(is_manual) _yaw_in *= manual_input_enable;
	yaw_in = _yaw_in;
}
void set_throttle(float _throttle_in, bool is_manual){
	if(is_manual) _throttle_in *= manual_input_enable;
	throttle_in = _throttle_in;
}

void set_forward(float _forward_in, bool is_manual){
	if(is_manual) _forward_in *= manual_input_enable;
	forward_in = _forward_in;
}

void set_lateral(float _lateral_in, bool is_manual){
	if(is_manual) _lateral_in *= manual_input_enable;
	lateral_in = _lateral_in;
}
void set_test_pwm(u16 pwm_test[6]){
	for(int i=0;i<6;i++)
		motor_test[i] = pwm_test[i];
}


float get_roll(){return roll_in;}
float get_pitch(){return pitch_in;}
float get_yaw(){return yaw_in;}
float get_throttle(){return throttle_in;}
float get_forward(){return forward_in;}
float get_lateral(){return lateral_in;}
u16* get_motorout(){return motor_out;}


//电机测试模式pwn输出,per为输入油门百分比,status为每个电机的测试状态
//u16 motor_test_pwm(u8 status,u8 per){
//	u16 pwm;
//	pwm = (manual_throttle_min+((float)per/100)*(throttle_max-manual_throttle_min)*status);
//	return (constrain_pwm(pwm,manual_throttle_min,throttle_max));
//}




void Moter_Set_PWM_Pulse(u8 ch,u16 pwm){
	switch(ch){
		case 0:
			MOTOR_CH1_4->CCR4=pwm;	//注意：第一个通道是定时器4通道4
			break;
		case 1:
			MOTOR_CH1_4->CCR3=pwm;
				break;
		case 2:
			MOTOR_CH1_4->CCR2=pwm;
				break;
		case 3:
			MOTOR_CH1_4->CCR1=pwm;
				break;
		case 4:
			MOTOR_CH5_8->CCR4=pwm;
				break;
		case 5:
			MOTOR_CH5_8->CCR3=pwm;
				break;
		case 6:
			MOTOR_CH5_8->CCR2=pwm;
				break;
		case 7:
			MOTOR_CH5_8->CCR1=pwm;
				break;
		default:
			//TODO:提示超出最大通道数
			break;
	}
}

void Motor_PWM_Set_Min(void)  //占空比10%,高电平时间1ms
{	
	for(u8 i=0;i<MOTORS_MAX_NUMER;i++){
		Moter_Set_PWM_Pulse(i,1000);
	}
}
void Motor_PWM_Set_Mid(void)  //占空比10%,高电平时间1ms
{	
	for(u8 i=0;i<MOTORS_MAX_NUMER;i++){
		Moter_Set_PWM_Pulse(i,1500);
	}
}

void Motor_PWM_Set_Max(void)
{		
	for(u8 i=0;i<MOTORS_MAX_NUMER;i++){
		Moter_Set_PWM_Pulse(i,2000);
	}		
}

//电调挂载
void Motor_ESC_Mount(){
	static u8 num=0;
	switch(num++){
		case 0:
			printf("[motor] mount ESC... \r\n");
			Motor_PWM_Init();	   //电机pwm频率为100hz
			switch(fp.esc_type){
				case ONE_WAY:	//单向电调
					Motor_PWM_Set_Min();		//设置最小油门门限
					printf("[motor] set MIN pwm... \r\n");
					break;
				case TWO_WAY:	//双向电调
					Motor_PWM_Set_Mid();		//设置中间油门门限
					printf("[motor] set MID pwm... \r\n");
					break;
			}
			break;
		case 2:
			printf("[motor] Mount ESC complete! \r\n");
			num = 0;
			sys_flag.esc_init_complete = true;
			break;
		default:
			break;
	}
}

//油门行程设定：单项模式 双向模式 -> 一次设置好之后无须再变，每次上电mount即可 -> 此函数还需调节，查看说明书
void Motor_Set_Throttle_range(){
	static u8 num=0;
	switch(num++){
		case 0:
			printf("[motor] set throttle range... \r\n");
			Motor_PWM_Init();	   //电机pwm频率为50hz
			printf("[motor] set MAX pwm... \r\n");
			Motor_PWM_Set_Max();		//设置最高油门门限
			break;
		case 3:
			switch(fp.esc_type){
				case ONE_WAY:	//单向电调
					Motor_PWM_Set_Min();		//设置最大油门门限
					printf("[motor] set MIN pwm... \r\n");
					break;
				case TWO_WAY:	//双向电调
					Motor_PWM_Set_Mid();		//设置中间油门门限
					printf("[motor] set mid pwm... \r\n");
					break;
			}
			printf("[motor] throttle range set complete! \r\n");
			num = 0;
			sys_flag.esc_init_complete = true;
			break;
		default:
			break;
	}
}


