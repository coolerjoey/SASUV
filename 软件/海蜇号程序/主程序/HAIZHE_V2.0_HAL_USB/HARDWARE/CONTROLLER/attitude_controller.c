#include "attitude_controller.h"
#include "mymath.h"
#include "sensor.h"
#include "motors.h"
#include "global_para.h"

vec3f euler_gyro_target;

PidObject pidAngleRoll;//roll角度环PID
PidObject pidAnglePitch;//pitch角度环PID
PidObject pidAngleYaw;//yaw角度环PID
PidObject pidRateRoll;//roll角速度环PID
PidObject pidRatePitch;//pitch角速度环PID
PidObject pidRateYaw;//yaw角速度环PID

//姿态PID控制器初始化 
void attitude_Controller_Init(float attPidDt)
{
	pidInit(&pidAngleRoll, 0, fp.pidAngleRoll, attPidDt);			/*roll  角度PID初始化*/
	pidInit(&pidAnglePitch, 0, fp.pidAnglePitch, attPidDt);			/*pitch 角度PID初始化*/
	pidInit(&pidAngleYaw, 0, fp.pidAngleYaw, attPidDt);				/*yaw   角度PID初始化*/

	pidInit(&pidRateRoll, 0, fp.pidRateRoll, attPidDt);				/*roll  角速度PID初始化*/
	pidInit(&pidRatePitch, 0, fp.pidRatePitch, attPidDt);			/*pitch 角速度PID初始化*/
	pidInit(&pidRateYaw, 0, fp.pidRateYaw, attPidDt);				/*yaw   角速度PID初始化*/
}

//姿态PID系数更新 -> 上位机随时改变，修改的是fp内的pid参数
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

//角度环PID
void input_eular_angle_to_eular_rate(float euler_roll_target,float euler_pitch_target,float euler_yaw_target){

	vec3f eular_angle_latest;
	get_euler_angle_latest(eular_angle_latest);//获取当前实际姿态角
	//东北天坐标系下，导航坐标系x轴指向东，前进方向定义为y轴，故绕x轴旋转时为pitch
	euler_gyro_target[0] = pidUpdate(&pidAnglePitch, euler_pitch_target - eular_angle_latest[0]);
	euler_gyro_target[1] =  pidUpdate(&pidAngleRoll, euler_roll_target - eular_angle_latest[1]);
	float yawError = euler_yaw_target - eular_angle_latest[2] ;
	if (yawError > 180.0f) //正转过一周
		yawError -= 360.0f;
	else if (yawError < -180.0) //反转过一周
		yawError += 360.0f;
	euler_gyro_target[2] = pidUpdate(&pidAngleYaw, yawError);

	//pid期望值更新
	pidAngleRoll.desired = euler_roll_target;
	pidAnglePitch.desired = euler_pitch_target;
	pidAngleYaw.desired = euler_yaw_target;
}

//角速度环PID
void input_rate_target_to_motor(vec3f output){
	vec3f euler_gyro_latest ;
	get_euler_gyro_latest(euler_gyro_latest);	//获取当前实际角速度
	output[0] = (pidUpdate(&pidRatePitch, euler_gyro_target[0] - euler_gyro_latest[0])) / (THRUST_RANGE/2);
	output[1] = (pidUpdate(&pidRateRoll, euler_gyro_target[1] - euler_gyro_latest[1])) / (THRUST_RANGE/2);	
	output[2] = (pidUpdate(&pidRateYaw, euler_gyro_target[2] - euler_gyro_latest[2])) / (THRUST_RANGE/2);

	//pid期望值更新
	pidRatePitch.desired = euler_gyro_target[0];
	pidRateRoll.desired = euler_gyro_target[1];
	pidRateYaw.desired = euler_gyro_target[2];
}

void att_controller_run(float euler_roll_target,float euler_pitch_target,float euler_yaw_target){
	attc_kp_ki_kd_update();	//pid参数更新 -> 防止上位机修改
	input_eular_angle_to_eular_rate(euler_roll_target,euler_pitch_target,euler_yaw_target);	//角度环
	vec3f att_to_motor_output;
	input_rate_target_to_motor(att_to_motor_output);	//角速度环
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

