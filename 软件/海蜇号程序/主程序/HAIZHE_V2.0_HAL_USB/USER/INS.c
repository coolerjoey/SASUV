#include "INS.h"
#include "parameter.h"

INS ins;

void ins_init(){
	imu_init();
	ec_init();
}

void ins_update(){

	ec_update();
	imu_update();

	//IMU -> 加速度，角速度，磁力，温度
	arm_copy_f32(imu.acc, ins.acc, 3);
	arm_copy_f32(imu.gyro, ins.gyro, 3);
	arm_copy_f32(imu.mag, ins.mag, 3);
	ins.temp = imu.temp;
	//东北天坐标系下角速度规定xy逆时针为正，z顺时针为正(导航习惯北偏东为正) -> 参照《秦永元》惯性导航P8
	ins.gyro[2] *= -1;

	//水平姿态手动切换
	if(sys_flag.health.ec && sys_flag.EC_att_enable){	
		ins.euler[0] = ec.euler[1];
		ins.euler[1] = ec.euler[0]; 
	}
	else if(sys_flag.health.imu){
		ins.euler[0] = imu.euler[0];	
		ins.euler[1] = imu.euler[1];
	}
	//水平姿态自动切换
	if(sys_flag.health.ec) ins.euler[2] = sys_flag.health.ec?ec.euler[2]:imu.euler[2];

	arm_scale_f32(ins.euler, DEG_TO_RAD, ins.euler_rad, 3) ;	//角度（弧度）
	arm_scale_f32(ins.gyro, DEG_TO_RAD, ins.gyro_rad, 3) ;	//角速度（弧度）
	//欧拉角 - 东北天坐标系下x轴为pitch，y轴为roll
	ins.roll = ins.euler[1];
	ins.pitch = ins.euler[0];
	ins.yaw = ins.euler[2];
	//欧拉角*100
	ins.roll_sensor = (int)(ins.roll*100); 
	ins.pitch_sensor = (int)(ins.pitch*100); 
	ins.yaw_sensor = (int)(ins.yaw*100);  	

	sys_flag.health.ins = sys_flag.health.ec&sys_flag.health.imu;

}

void ins_check(){
	ec_check();
	imu_check();
}

void ins_calibration(){
	ec_calibration();
	imu_calibration();
}

