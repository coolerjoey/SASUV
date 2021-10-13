#include "model_verify.h"
#include "mymath.h"
#include "model.h"
#include "parameter.h"
#include "motors.h"
#include "sensor.h"

u16 motor_pwm[6];
vec6f motor_force;
 
u16 input_force_to_pwm(int type, float force){
	float pwm=0;
		
	if(force>0){
		switch(type){
			case CW:
				pwm = (0.0004f*force*force-0.0321f*force+1.4875f)*1;	
				pwm = constrain_float(pwm, 1.0, 1.5);
				break;
			case CCW:
				pwm = (-3e-5f*force*force*force+0.0016f*force*force-0.0357f*force+1.4964f)*1;
				pwm = constrain_float(pwm, 1.0, 1.5);
				break;
		}
	}
	else if(force<0){
		force *= -1;
		switch(type){
			case CW:
				pwm = (-0.0007f*force*force+0.0377f*force+1.527f)*1; 
				pwm = constrain_float(pwm, 1.5, 2.0);
				break;
			case CCW:
				pwm = (-0.0004f*force*force+0.031f*force+1.5219f)*1;	
				pwm = constrain_float(pwm, 1.5, 2.0);
				break;
		}
	}
	else if(is_equal(force, 0)) pwm=1.5;

	return pwm*1000;


}

void set_motor_force(int motor_num, float force){
	motor_pwm[motor_num] = input_force_to_pwm(model.Moter_Dir[motor_num],force);	//��->PWM
	motor_force[motor_num] = force;	//ģ��������
}
static bool show_task_pass=false;
void model_verify(){
	
	static bool model_verify_enable = false;
	static u32 start_time = 0;
	static int num = 0;
	if(sys_flag.model_verify != model_verify_enable){
		model_verify_enable = sys_flag.model_verify;
		if(!sys_flag.model_verify) return;	//ֹͣ����
		model_init(Sensor_latest.ins.euler, O31, O31, 0.01);	//��ʼ��֤�����г�ʼ��
		start_time = sys_time.one_second;
		show_task_pass = sys_flag.show_task_pass;
		sys_flag.show_task_pass = false;
		printf("Start model verify\r\n");
	}
	else if(!sys_flag.model_verify) return;
	if(sys_time.one_second-start_time == vp.model_test_sec){	//��ʱ�˳�����
		printf("Finish model verify \r\n");		
		model.model_init_complete = false;
		sys_flag.model_verify = false;
		sys_flag.show_task_pass = show_task_pass;
		model_verify_enable = false;
		memset(&model,0,sizeof(model));//���model
		num = 0;
		return;
	}
	model_updata_core(Sensor_latest.ins.euler, motor_force);//����ģ��ϵͳ
	for(int i=0;i<6;++i) Moter_Set_PWM_Pulse(i, motor_pwm[i]);	//��ֵʵ��ϵͳ
	
	//��������Ա�
//	printf("%.2f ",(float)num++/100.0f);
//	printf("%.5f %.5f %.5f ",model.ab[0], model.ab[1], model.ab[2]);//ģ�ͼ��ٶ�
//	printf("%.5f %.5f %.5f ",model.X_data[9], model.X_data[10], model.X_data[11]);//ģ�ͽ��ٶ�
//	printf("%.5f %.5f %.5f ",model.X_data[0], model.X_data[1], model.X_data[2]);
//	printf("%.5f %.5f %.5f ",Sensor_latest.ins.acc[0], Sensor_latest.ins.acc[1], Sensor_latest.ins.acc[2]);//ʵ�ʼ��ٶ�
//	printf("%.5f %.5f %.5f ",Sensor_latest.ins.gyro[0], Sensor_latest.ins.gyro[1], Sensor_latest.ins.gyro[2]);//ʵ�ʽ��ٶ�
//	printf("\r\n");
}




