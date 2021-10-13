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

	//IMU -> ���ٶȣ����ٶȣ��������¶�
	arm_copy_f32(imu.acc, ins.acc, 3);
	arm_copy_f32(imu.gyro, ins.gyro, 3);
	arm_copy_f32(imu.mag, ins.mag, 3);
	ins.temp = imu.temp;
	//����������ϵ�½��ٶȹ涨xy��ʱ��Ϊ����z˳ʱ��Ϊ��(����ϰ�߱�ƫ��Ϊ��) -> ���ա�����Ԫ�����Ե���P8
	ins.gyro[2] *= -1;

	//ˮƽ��̬�ֶ��л�
	if(sys_flag.health.ec && sys_flag.EC_att_enable){	
		ins.euler[0] = ec.euler[1];
		ins.euler[1] = ec.euler[0]; 
	}
	else if(sys_flag.health.imu){
		ins.euler[0] = imu.euler[0];	
		ins.euler[1] = imu.euler[1];
	}
	//ˮƽ��̬�Զ��л�
	if(sys_flag.health.ec) ins.euler[2] = sys_flag.health.ec?ec.euler[2]:imu.euler[2];

	arm_scale_f32(ins.euler, DEG_TO_RAD, ins.euler_rad, 3) ;	//�Ƕȣ����ȣ�
	arm_scale_f32(ins.gyro, DEG_TO_RAD, ins.gyro_rad, 3) ;	//���ٶȣ����ȣ�
	//ŷ���� - ����������ϵ��x��Ϊpitch��y��Ϊroll
	ins.roll = ins.euler[1];
	ins.pitch = ins.euler[0];
	ins.yaw = ins.euler[2];
	//ŷ����*100
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

