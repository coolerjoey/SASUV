#ifndef _EC_H
#define _EC_H

#include "mymath.h"

typedef struct{
	float temp;			//温度
	vec3f acc;//加速度
	vec3f euler;//角度
	vec3f mag;//磁场
}ElectronicCompass;
extern ElectronicCompass ec;

void ec_init(void);
void ec_update(void);
void ec_trigger(void);
void ec_check(void);
void get_euler(vec3f);
void ec_calibration(void);

#endif


