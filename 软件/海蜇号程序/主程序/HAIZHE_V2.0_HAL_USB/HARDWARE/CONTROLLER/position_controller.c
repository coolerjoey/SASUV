#include "position_controller.h"
#include "global_para.h"
#include "motors.h"
#include "LowPassFilter.h"

float accel_z_cms = 1;//���ֱ���ٶ�cm/s/s
float vel_z_target;
float accel_z_target;
 
PidObject pidPosZ;//λ�û�PID
PidObject pidVelZ;//�ٶȻ�PID
PidObject pidAccZ;//���ٶȻ�PID
 
//λ��PID��������ʼ�� 
void position_Controller_Init( float posPidDt)
{
	pidInit(&pidPosZ, 0, fp.pidPosZ, posPidDt);	/*vx PID��ʼ��*/
	pidInit(&pidVelZ, 0, fp.pidVelZ, posPidDt);			/*x PID��ʼ��*/	
	pidInit(&pidAccZ, 0, fp.pidAccZ, posPidDt);			/*x PID��ʼ��*/
}

void posc_kp_ki_kd_update(){
	pidPosZ.kp = fp.pidPosZ.kp;
	pidPosZ.ki = fp.pidPosZ.ki;
	pidPosZ.kd = fp.pidPosZ.kd;
	pidPosZ.iLimit = fp.pidPosZ.iLimit;
	pidPosZ.outputLimit = fp.pidPosZ.outputLimit;
	pidPosZ.feedforward = fp.pidPosZ.feedfoward;

	pidVelZ.kp = fp.pidVelZ.kp;
	pidVelZ.ki = fp.pidVelZ.ki;
	pidVelZ.kd = fp.pidVelZ.kd;
	pidVelZ.iLimit = fp.pidVelZ.iLimit;
	pidVelZ.outputLimit = fp.pidVelZ.outputLimit;
	pidVelZ.feedforward = fp.pidVelZ.feedfoward;

	pidAccZ.kp = fp.pidAccZ.kp;
	pidAccZ.ki = fp.pidAccZ.ki;
	pidAccZ.kd = fp.pidAccZ.kd;
	pidAccZ.iLimit = fp.pidAccZ.iLimit;
	pidAccZ.outputLimit = fp.pidAccZ.outputLimit;
	pidAccZ.feedforward = fp.pidAccZ.feedfoward;

}
//�������pid
void pos_to_throttle(float pos_z_target){	
	float curr_alt = Sensor_latest.barometer.depth; //��ȡʵ�����
	float pos_z_error = pos_z_target - curr_alt;	//������
	//����⻷P����,��������ת��Ϊz�������ٶ�(����������)
	pidPosZ.desired = pos_z_target;
	float thr_out = pidUpdate(&pidPosZ,pos_z_error) / (THRUST_RANGE/2);
	set_throttle(fp.throttle_hover-thr_out,INPUT_AUTO);	//����ͣ���Ż����ϵ���PID��� ע�⣺����С��0���³�TODO
}

//������λ��ת��Ϊ�����ٶ�
void  pos_to_rate_z(float pos_z_target){
	//TODO:�����ǰ��������Ŀ����ˮ���Ϸ�10cm
	float curr_alt = Sensor_latest.barometer.depth;	//��ȡʵ�����
	float pos_z_error = pos_z_target - curr_alt;	//������
	//����⻷P����,��������ת��Ϊz�������ٶ�(����������)
//	 vel_z_target = sqrt_controller(pos_z_error,p_pos_z,accel_z_cms);
	pidPosZ.desired = pos_z_target;
	vel_z_target = pidUpdate(&pidPosZ,pos_z_error);
}

//�������ٶ�ת��Ϊ�������ٶ�
void rate_to_accel_z(){	
	float vel_z_error;
	
	static float depth_last;	//��һʱ�̵�z��λ��
	float vel_z_curr = (Sensor_latest.barometer.depth - depth_last)/loop_rate_hz;//��ȡʵ���ٶ�
	depth_last = Sensor_latest.barometer.depth;
	//TODO ��ͨ�˲�������z���ٶ����
//	vel_z_error = LowPassFilter(vel_z_target-vel_z_curr,1,loop_rate_hz);
	vel_z_error = vel_z_target-vel_z_curr;
	pidVelZ.desired = vel_z_target;
	accel_z_target = pidUpdate(&pidVelZ,vel_z_error);
}

//���������ٶ�ת��Ϊ��������
void accel_to_throttle(){	//���ٶȻ�PID����
	//��ȡʵ��z����ٶ�
	float accel_z_error;
	accel_z_error = accel_z_target - Sensor_latest.ins.acc[2];//z����ٶ���� ע�⣺Ҫȥ���������ٶȵ�Ӱ��TODO
	float thr_out = pidUpdate(&pidAccZ,accel_z_error) / (THRUST_RANGE/2);
	set_throttle(fp.throttle_hover-thr_out,INPUT_AUTO);	//����ͣ���Ż����ϵ���PID��� ע�⣺����С��0���³�TODO
}


float sqrt_controller(float error, float p, float second_ord_lim)
{

//    if (second_ord_lim < 0.0f || is_zero(second_ord_lim) || is_zero(p)) {
//        return error*p;
//    }

//	//���Ծ��룺�������ٶ��˶��ڵ�λʱ���ڵ�λ��
//    float linear_dist = second_ord_lim/sq(p);
////	printf("%.1f %.1f %.1f dist=%.2f \r\n",second_ord_lim,p,sq(p),linear_dist);

//	//��������С����ƽ��������
//    if (error > linear_dist) {
//        return safe_sqrt(2.0f*second_ord_lim*(error-(linear_dist/2.0f)));
//    } else if (error < -linear_dist) {
//        return -safe_sqrt(2.0f*second_ord_lim*(-error-(linear_dist/2.0f)));
//    } else {
//        return error*p;
//    }
	return error*p;
}

void pos_z_controller_run(float pos_z_target){
	posc_kp_ki_kd_update();
//	pos_to_rate_z(pos_z_target);
//	rate_to_accel_z();
//	accel_to_throttle();
	pos_to_throttle(pos_z_target);
}

void reset_pos_z_controller(){
	pidReset(&pidPosZ);
	pidReset(&pidVelZ);
	pidReset(&pidAccZ);
}

