#ifndef _INS_H
#define _INS_H

#include "sys.h"
#include "mymath.h"
#include "IMU.h"
#include "EC.h"


typedef struct{
	float temp;	//温度
	//以下方向都是按照x-y-z轴
	vec3f acc;//加速度
	vec3f gyro;//角速度
	vec3f gyro_rad;	//角速度（弧度）
	vec3f euler;//角度
	vec3f euler_rad; //角度（孤独）
	vec3f mag;//磁场
	//欧拉角
	float roll, pitch, yaw;
	int roll_sensor, pitch_sensor, yaw_sensor; 	//欧拉角*100
}INS;

extern INS ins;

void ins_init(void);
void ins_update(void);
void ins_check(void);
void ins_calibration(void);

#endif

