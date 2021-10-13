#ifndef _IMU_H
#define _IMU_H

#include "sys.h"
#include "mymath.h"

#define GYRO_Z_LIMIT	360 //Z����ٶ����(��ʵ��Χ��֪��ˣ�Ϊ������̬���ƺ���־��¼��ȥ���쳣���ٶ�ֵ)

typedef struct{
	float temp;			//�¶�
	vec3f acc;//���ٶ�
	vec3f gyro;//���ٶ�
	vec3f euler;//�Ƕ�
	vec3f mag;//�ų�
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
