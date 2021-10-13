#include "motors.h"
#include "LowPassFilter.h"
#include "global_para.h"
#include "delay.h"

extern SYS_FLAGS sys_flag;

Motor motor[MOTORS_MAX_NUMER];

u16 motor_out[MOTORS_MAX_NUMER];	//�������pwm
u16 motor_test[MOTORS_MAX_NUMER];	//����pwm
float roll_in= 0.0;						//�������ͨ��
float pitch_in= 0.0;					//��������ͨ��
float yaw_in= 0.0;						//ƫ������ͨ��
float throttle_in= 0.0;				//��������ͨ��
float forward_in = 0.0;
float lateral_in = 0.0;

//��ӵ��
void add_motor(uint8_t num, float roll_fac, float pitch_fac, float yaw_fac,float throttle_fac,float forward_fac, float lateral_fac){
	motor[num].roll_fac= roll_fac;
	motor[num].pitch_fac= pitch_fac;
	motor[num].yaw_fac= yaw_fac;
	motor[num].throttle_fac= throttle_fac;
	motor[num].forward_fac = forward_fac;
	motor[num].lateral_fac = lateral_fac;
}

/*���ģ�ͳ�ʼ��(ֻ��ʾת�����򣬲���ʾ�����С)
---------------------------
CW	<1500	��ת		������
	>1500	��ת		������
---------------------------
CCW	<1500	��ת		������
	>1500	��ת		������
---------------------------
��Ӧ�ñ�������ͬ�Ŀ������£��������ƽ���������������һ�µ�
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
//	if(is_zero(forward_in) && vp.control_mode!=MANUAL){	//��ԭ�ض���ʱ���Ӷ�������	
//		if(fabs(yaw_in)<fp.throttle_deadzone_swerve){
//			yaw_in = yaw_in>0?yaw_in+fp.throttle_deadzone_swerve:yaw_in-fp.throttle_deadzone_swerve;
//		}
//	}
	if(vp.control_mode==STABILIZE && is_zero(forward_in)){	//��ֹ����ʱ�����ƽ�������
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

//������
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

//������
void motors_output(){
	//ÿ�ζ����QGC�Ƿ��޸ĵ��ģ�Ͳ���
	setup_motors();
	//���ݺ��غ�ģ�ͼ�������pwm��
	calc_thrust_to_pwm();
	//����pwm�����
	output_to_motors();
}

/*
	@func:�趨�����ɶ����
	@param
		is_manual: ��ʾ���ô˺������ֶ�(�ֱ�,ң����)���뻹�ǿ����㷨(����,����)����
					�����Զ����룬��Ҫ����ֱ������Ƿ���������ֹ����λ������ң����ʧ���󱣳�������룬�޷�ֹͣ
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


//�������ģʽpwn���,perΪ�������Űٷֱ�,statusΪÿ������Ĳ���״̬
//u16 motor_test_pwm(u8 status,u8 per){
//	u16 pwm;
//	pwm = (manual_throttle_min+((float)per/100)*(throttle_max-manual_throttle_min)*status);
//	return (constrain_pwm(pwm,manual_throttle_min,throttle_max));
//}




void Moter_Set_PWM_Pulse(u8 ch,u16 pwm){
	switch(ch){
		case 0:
			MOTOR_CH1_4->CCR4=pwm;	//ע�⣺��һ��ͨ���Ƕ�ʱ��4ͨ��4
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
			//TODO:��ʾ�������ͨ����
			break;
	}
}

void Motor_PWM_Set_Min(void)  //ռ�ձ�10%,�ߵ�ƽʱ��1ms
{	
	for(u8 i=0;i<MOTORS_MAX_NUMER;i++){
		Moter_Set_PWM_Pulse(i,1000);
	}
}
void Motor_PWM_Set_Mid(void)  //ռ�ձ�10%,�ߵ�ƽʱ��1ms
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

//�������
void Motor_ESC_Mount(){
	static u8 num=0;
	switch(num++){
		case 0:
			printf("[motor] mount ESC... \r\n");
			Motor_PWM_Init();	   //���pwmƵ��Ϊ100hz
			switch(fp.esc_type){
				case ONE_WAY:	//������
					Motor_PWM_Set_Min();		//������С��������
					printf("[motor] set MIN pwm... \r\n");
					break;
				case TWO_WAY:	//˫����
					Motor_PWM_Set_Mid();		//�����м���������
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

//�����г��趨������ģʽ ˫��ģʽ -> һ�����ú�֮�������ٱ䣬ÿ���ϵ�mount���� -> �˺���������ڣ��鿴˵����
void Motor_Set_Throttle_range(){
	static u8 num=0;
	switch(num++){
		case 0:
			printf("[motor] set throttle range... \r\n");
			Motor_PWM_Init();	   //���pwmƵ��Ϊ50hz
			printf("[motor] set MAX pwm... \r\n");
			Motor_PWM_Set_Max();		//���������������
			break;
		case 3:
			switch(fp.esc_type){
				case ONE_WAY:	//������
					Motor_PWM_Set_Min();		//���������������
					printf("[motor] set MIN pwm... \r\n");
					break;
				case TWO_WAY:	//˫����
					Motor_PWM_Set_Mid();		//�����м���������
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


