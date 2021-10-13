#ifndef _AHRS_H
#define _AHRS_H

#include "sys.h"
#include "mymath.h"


typedef struct{
	float temp;			//温度
	vec3f acc;//加速度
	vec3f gyro;//角速度
	vec3f euler;//角度
	vec3f mag;//磁场
}AHRS;

extern AHRS ahrs;

void ahrs_init(void);
void ahrs_read(void);
void ahrs_check(void);

#endif
