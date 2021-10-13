#include "attitude_controller.h"
#include "mymath.h"
#include "sensor.h"
#include "motors.h"
#include "global_para.h"

vec3f euler_gyro_target;

PidObject pidAngleRoll;//roll�ǶȻ�PID
PidObject pidAnglePitch;//pitch�ǶȻ�PID
PidObject pidAngleYaw;//yaw�ǶȻ�PID
PidObject pidRateRoll;//roll���ٶȻ�PID
PidObject pidRatePitch;//pitch���ٶȻ�PID
PidObject pidRateYaw;//yaw���ٶȻ�PID

//��̬PID��������ʼ�� 
void attitude_Controller_Init(float attPidDt)
{
	pidInit(&pidAngleRoll, 0, fp.pidAngleRoll, attPidDt);			/*roll  �Ƕ�PID��ʼ��*/
	pidInit(&pidAnglePitch, 0, fp.pidAnglePitch, attPidDt);			/*pitch �Ƕ�PID��ʼ��*/
	pidInit(&pidAngleYaw, 0, fp.pidAngleYaw, attPidDt);				/*yaw   �Ƕ�PID��ʼ��*/

	pidInit(&pidRateRoll, 0, fp.pidRateRoll, attPidDt);				/*roll  ���ٶ�PID��ʼ��*/
	pidInit(&pidRatePitch, 0, fp.pidRatePitch, attPidDt);			/*pitch ���ٶ�PID��ʼ��*/
	pidInit(&pidRateYaw, 0, fp.pidRateYaw, attPidDt);				/*yaw   ���ٶ�PID��ʼ��*/
}

//��̬PIDϵ������ -> ��λ����ʱ�ı䣬�޸ĵ���fp�ڵ�pid����
void attc_kp_ki_kd_update(){
	pidAngleRoll.kp = fp.pidAngleRoll.kp;
	pidAngleRoll.ki = fp.pidAngleRoll.ki;
	pidAngleRoll.kd = fp.pidAngleRoll.kd;
	pidAngleRoll.iLimit = fp.pidAngleRoll.iLimit;
	pidAngleRoll.outputLimit = fp.pidAngleRoll.outputLimit;
	pidAngleRoll.feedforward = fp.pidAngleRoll.feedfoward;

	pidAnglePitch.kp = fp.pidAnglePitch.kp;
	pidAnglePitch.ki = fp.pidAnglePitch.ki;
	pidAnglePitch.kd = fp.pidAnglePitch.kd;
	pidAnglePitch.iLimit = fp.pidAnglePitch.iLimit;
	pidAnglePitch.outputLimit = fp.pidAnglePitch.outputLimit;
	pidAnglePitch.feedforward = fp.pidAnglePitch.feedfoward;

	pidAngleYaw.kp = fp.pidAngleYaw.kp;
	pidAngleYaw.ki = fp.pidAngleYaw.ki;
	pidAngleYaw.kd = fp.pidAngleYaw.kd;
	pidAngleYaw.iLimit = fp.pidAngleYaw.iLimit;
	pidAngleYaw.outputLimit = fp.pidAngleYaw.outputLimit;
	pidAngleYaw.feedforward = fp.pidAngleYaw.feedfoward;

	pidRateRoll.kp = fp.pidRateRoll.kp;
	pidRateRoll.ki = fp.pidRateRoll.ki;
	pidRateRoll.kd = fp.pidRateRoll.kd;
	pidRateRoll.iLimit = fp.pidRateRoll.iLimit;
	pidRateRoll.outputLimit = fp.pidRateRoll.outputLimit;
	pidRateRoll.feedforward = fp.pidRateRoll.feedfoward;

	pidRatePitch.kp = fp.pidRatePitch.kp;
	pidRatePitch.ki = fp.pidRatePitch.ki;
	pidRatePitch.kd = fp.pidRatePitch.kd;
	pidRatePitch.iLimit = fp.pidRatePitch.iLimit;
	pidRatePitch.outputLimit = fp.pidRatePitch.outputLimit;
	pidRatePitch.feedforward = fp.pidRatePitch.feedfoward;

	pidRateYaw.kp = fp.pidRateYaw.kp;
	pidRateYaw.ki = fp.pidRateYaw.ki;
	pidRateYaw.kd = fp.pidRateYaw.kd;
	pidRateYaw.iLimit = fp.pidRateYaw.iLimit;
	pidRateYaw.outputLimit = fp.pidRateYaw.outputLimit;
	pidRateYaw.feedforward = fp.pidRateYaw.feedfoward;
}

//�ǶȻ�PID
void input_eular_angle_to_eular_rate(float euler_roll_target,float euler_pitch_target,float euler_yaw_target){

	vec3f eular_angle_latest;
	get_euler_angle_latest(eular_angle_latest);//��ȡ��ǰʵ����̬��
	//����������ϵ�£���������ϵx��ָ�򶫣�ǰ��������Ϊy�ᣬ����x����תʱΪpitch
	euler_gyro_target[0] = pidUpdate(&pidAnglePitch, euler_pitch_target - eular_angle_latest[0]);
	euler_gyro_target[1] =  pidUpdate(&pidAngleRoll, euler_roll_target - eular_angle_latest[1]);
	float yawError = euler_yaw_target - eular_angle_latest[2] ;
	if (yawError > 180.0f) //��ת��һ��
		yawError -= 360.0f;
	else if (yawError < -180.0) //��ת��һ��
		yawError += 360.0f;
	euler_gyro_target[2] = pidUpdate(&pidAngleYaw, yawError);

	//pid����ֵ����
	pidAngleRoll.desired = euler_roll_target;
	pidAnglePitch.desired = euler_pitch_target;
	pidAngleYaw.desired = euler_yaw_target;
}

//���ٶȻ�PID
void input_rate_target_to_motor(vec3f output){
	vec3f euler_gyro_latest ;
	get_euler_gyro_latest(euler_gyro_latest);	//��ȡ��ǰʵ�ʽ��ٶ�
	output[0] = (pidUpdate(&pidRatePitch, euler_gyro_target[0] - euler_gyro_latest[0])) / (THRUST_RANGE/2);
	output[1] = (pidUpdate(&pidRateRoll, euler_gyro_target[1] - euler_gyro_latest[1])) / (THRUST_RANGE/2);	
	output[2] = (pidUpdate(&pidRateYaw, euler_gyro_target[2] - euler_gyro_latest[2])) / (THRUST_RANGE/2);

	//pid����ֵ����
	pidRatePitch.desired = euler_gyro_target[0];
	pidRateRoll.desired = euler_gyro_target[1];
	pidRateYaw.desired = euler_gyro_target[2];
}

void att_controller_run(float euler_roll_target,float euler_pitch_target,float euler_yaw_target){
	attc_kp_ki_kd_update();	//pid�������� -> ��ֹ��λ���޸�
	input_eular_angle_to_eular_rate(euler_roll_target,euler_pitch_target,euler_yaw_target);	//�ǶȻ�
	vec3f att_to_motor_output;
	input_rate_target_to_motor(att_to_motor_output);	//���ٶȻ�
	set_pitch(att_to_motor_output[0],INPUT_AUTO);//
	set_roll(att_to_motor_output[1],INPUT_AUTO);//
	set_yaw(att_to_motor_output[2],INPUT_AUTO);//
}

void reset_att_controller(){
	pidReset(&pidAngleRoll);
	pidReset(&pidAnglePitch);
	pidReset(&pidAngleYaw);
	pidReset(&pidRateRoll);
	pidReset(&pidRatePitch);
	pidReset(&pidRateYaw);
}

