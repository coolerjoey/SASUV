#ifndef _INS_H
#define _INS_H

#include "sys.h"
#include "mymath.h"
#include "IMU.h"
#include "EC.h"


typedef struct{
	float temp;	//�¶�
	//���·����ǰ���x-y-z��
	vec3f acc;//���ٶ�
	vec3f gyro;//���ٶ�
	vec3f gyro_rad;	//���ٶȣ����ȣ�
	vec3f euler;//�Ƕ�
	vec3f euler_rad; //�Ƕȣ��¶���
	vec3f mag;//�ų�
	//ŷ����
	float roll, pitch, yaw;
	int roll_sensor, pitch_sensor, yaw_sensor; 	//ŷ����*100
}INS;

extern INS ins;

void ins_init(void);
void ins_update(void);
void ins_check(void);
void ins_calibration(void);

#endif

