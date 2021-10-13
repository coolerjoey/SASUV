#ifndef _IMU_H
#define _IMU_H

#include "sys.h"
#include "mymath.h"

#define GYRO_Z_LIMIT	360 //Z轴角速度最大(其实范围不知如此，为了在姿态控制和日志记录中去掉异常角速度值)

typedef struct{
	float temp;			//温度
	vec3f acc;//加速度
	vec3f gyro;//角速度
	vec3f euler;//角度
	vec3f mag;//磁场
}IMU;
extern IMU imu;

void imu_init(void);
void imu_update(void);
void imu_calibration(void);
void imu_get_acc(vec3f);
void imu_get_euler(vec3f);
void imu_get_gyro(vec3f);
void imu_get_mag(vec3f);
void imu_get_temp(float *temp);
void imu_check(void);


#endif
