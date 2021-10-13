#ifndef _EC_H
#define _EC_H

#include "mymath.h"

typedef struct{
	float temp;			//�¶�
	vec3f acc;//���ٶ�
	vec3f euler;//�Ƕ�
	vec3f mag;//�ų�
}ElectronicCompass;
extern ElectronicCompass ec;

void ec_init(void);
void ec_update(void);
void ec_trigger(void);
void ec_check(void);
void get_euler(vec3f);
void ec_calibration(void);

#endif


