#include "position_controller.h"
#include "global_para.h"
#include "motors.h"
#include "LowPassFilter.h"

float accel_z_cms = 1;//最大垂直加速度cm/s/s
float vel_z_target;
float accel_z_target;
 
PidObject pidPosZ;//位置环PID
PidObject pidVelZ;//速度环PID
PidObject pidAccZ;//加速度环PID
 
//位置PID控制器初始化 
void position_Controller_Init( float posPidDt)
{
	pidInit(&pidPosZ, 0, fp.pidPosZ, posPidDt);	/*vx PID初始化*/
	pidInit(&pidVelZ, 0, fp.pidVelZ, posPidDt);			/*x PID初始化*/	
	pidInit(&pidAccZ, 0, fp.pidAccZ, posPidDt);			/*x PID初始化*/
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
//单环深度pid
void pos_to_throttle(float pos_z_target){	
	float curr_alt = Sensor_latest.barometer.depth; //获取实际深度
	float pos_z_error = pos_z_target - curr_alt;	//深度误差
	//深度外环P控制,把深度误差转化为z轴期望速度(开方控制器)
	pidPosZ.desired = pos_z_target;
	float thr_out = pidUpdate(&pidPosZ,pos_z_error) / (THRUST_RANGE/2);
	set_throttle(fp.throttle_hover-thr_out,INPUT_AUTO);	//在悬停油门基础上叠加PID输出 注意：油门小于0是下沉TODO
}

//把期望位置转化为期望速度
void  pos_to_rate_z(float pos_z_target){
	//TODO:如果当前沉底设置目标在水底上方10cm
	float curr_alt = Sensor_latest.barometer.depth;	//获取实际深度
	float pos_z_error = pos_z_target - curr_alt;	//深度误差
	//深度外环P控制,把深度误差转化为z轴期望速度(开方控制器)
//	 vel_z_target = sqrt_controller(pos_z_error,p_pos_z,accel_z_cms);
	pidPosZ.desired = pos_z_target;
	vel_z_target = pidUpdate(&pidPosZ,pos_z_error);
}

//把期望速度转化为期望加速度
void rate_to_accel_z(){	
	float vel_z_error;
	
	static float depth_last;	//上一时刻的z轴位置
	float vel_z_curr = (Sensor_latest.barometer.depth - depth_last)/loop_rate_hz;//获取实际速度
	depth_last = Sensor_latest.barometer.depth;
	//TODO 低通滤波器计算z轴速度误差
//	vel_z_error = LowPassFilter(vel_z_target-vel_z_curr,1,loop_rate_hz);
	vel_z_error = vel_z_target-vel_z_curr;
	pidVelZ.desired = vel_z_target;
	accel_z_target = pidUpdate(&pidVelZ,vel_z_error);
}

//把期望加速度转化为期望油门
void accel_to_throttle(){	//加速度环PID控制
	//获取实际z轴加速度
	float accel_z_error;
	accel_z_error = accel_z_target - Sensor_latest.ins.acc[2];//z轴加速度误差 注意：要去掉重力加速度的影响TODO
	float thr_out = pidUpdate(&pidAccZ,accel_z_error) / (THRUST_RANGE/2);
	set_throttle(fp.throttle_hover-thr_out,INPUT_AUTO);	//在悬停油门基础上叠加PID输出 注意：油门小于0是下沉TODO
}


float sqrt_controller(float error, float p, float second_ord_lim)
{

//    if (second_ord_lim < 0.0f || is_zero(second_ord_lim) || is_zero(p)) {
//        return error*p;
//    }

//	//线性距离：以线性速度运动在单位时间内的位移
//    float linear_dist = second_ord_lim/sq(p);
////	printf("%.1f %.1f %.1f dist=%.2f \r\n",second_ord_lim,p,sq(p),linear_dist);

//	//误差过大或过小进行平方跟控制
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

