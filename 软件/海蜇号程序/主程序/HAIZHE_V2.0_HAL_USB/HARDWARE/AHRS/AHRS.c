#include "AHRS.h"
#include "parameter.h"
#include "IMU_JY901.h"
#include "EC_TCM.h"
#include "EC_DDM350B.h"

AHRS ahrs;

void ahrs_init(){
#if EC_TYPE==DDM350B
	ec_ddm_init();
#elif EC_TYPE==TCM
	ec_tcm_init();
#endif

#if IMU_TYPE==AHRS_JY901
	imu_JY901_init();
#endif
	printf("[OK] AHRS init complete \r\n");
}

void ahrs_read(){
	vec3f ec_euler;	//电子罗盘姿态和航向
	vec3f imu_euler;	//imu姿态和航向
#if EC_TYPE==DDM350B
	ec_ddm_update();
	ddm_get_euler(ec_euler);
#elif EC_TYPE==TCM
	ec_tcm_update();
	tcm_get_euler(ec_euler);	
#endif


#if IMU_TYPE==AHRS_JY901
	imu_JY901_read();
	JY901_get_euler(imu_euler);
	JY901_get_mag(ahrs.mag);
	JY901_get_temp(&ahrs.temp);
	JY901_get_gyro(ahrs.gyro);
	//东北天坐标系下角速度规定xy逆时针为正，z顺时针为正(导航习惯北偏东为正) -> 参照《秦永元》惯性导航P8
	ahrs.gyro[2] *= -1;
#endif
	//水平姿态手动切换
	if(sys_flag.health.ec && sys_flag.EC_att_enable){	
		ahrs.euler[0] = ec_euler[1];
		ahrs.euler[1] = ec_euler[0];	
	}
	else if(sys_flag.health.ahrs_mpu){
		ahrs.euler[0] = imu_euler[0];	
		ahrs.euler[1] = imu_euler[1];
	}
	//水平姿态自动切换
	if(sys_flag.health.ec) ahrs.euler[2] = sys_flag.health.ec?ec_euler[2]:imu_euler[2];;	

	sys_flag.health.ahrs = sys_flag.health.ec&sys_flag.health.ahrs_mpu;

}

void ahrs_check(){
	ec_tcm_check();
	imu_JY901_check();
}

