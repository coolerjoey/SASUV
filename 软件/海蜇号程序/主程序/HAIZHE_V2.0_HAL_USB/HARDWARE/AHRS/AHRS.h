#ifndef _AHRS_H
#define _AHRS_H

#include "sys.h"
#include "mymath.h"


typedef struct{
	float temp;			//�¶�
	vec3f acc;//���ٶ�
	vec3f gyro;//���ٶ�
	vec3f euler;//�Ƕ�
	vec3f mag;//�ų�
}AHRS;

extern AHRS ahrs;

void ahrs_init(void);
void ahrs_read(void);
void ahrs_check(void);

#endif
