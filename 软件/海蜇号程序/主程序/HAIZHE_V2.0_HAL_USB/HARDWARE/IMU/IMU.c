#include "IMU.h"
#include "IMU_JY901.h"
#include "config.h"
#include "string.h"

IMU imu;

void imu_init(){
#if IMU_TYPE==JY901
	imu_JY901_init();
#endif
}

void imu_update(){
#if IMU_TYPE==JY901
	imu_JY901_read();
	arm_copy_f32(jy901.euler, imu.euler, 3);
	arm_copy_f32(jy901.acc, imu.acc, 3);
	arm_copy_f32(jy901.gyro,  imu.gyro, 3);
	arm_copy_f32(jy901.mag,  imu.mag, 3);
	imu.temp = jy901.temp;
#endif
}

void imu_get_euler(vec3f euler){
	arm_copy_f32(imu.euler, euler, 3);
}
void imu_get_acc(vec3f acc){
	arm_copy_f32(imu.acc, acc, 3);
}
void imu_get_gyro(vec3f gyro){
	arm_copy_f32(imu.gyro, gyro, 3);
}
void imu_get_mag(vec3f mag){
	arm_copy_f32(imu.mag, mag, 3);
}
void imu_get_temp(float *temp){
	*temp = imu.temp;
}

void imu_check(){
#if IMU_TYPE==JY901
	imu_JY901_check();
#endif
}

void imu_calibration(){
#if IMU_TYPE==JY901
	imu_JY901_ACC_calibration();
#endif

}

