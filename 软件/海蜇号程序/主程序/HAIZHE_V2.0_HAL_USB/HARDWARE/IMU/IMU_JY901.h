#ifndef _IMU_JY901_H
#define _IMU_JY901_H

#include "sys.h"
#include "mymath.h"

#define GYRO_Z_LIMIT	360 //Z����ٶ����(��ʵ��Χ��֪��ˣ�Ϊ������̬���ƺ���־��¼��ȥ���쳣���ٶ�ֵ)

struct STime
{
	u8 ucYear;
	u8 ucMonth;
	u8 ucDay;
	u8 ucHour;
	u8 ucMinute;
	u8 ucSecond;
	u16 usMiliSecond;
};
struct SAcc
{
	short a[3];
	short T;
};
struct SGyro
{
	s16 w[3];
	s16 T;
};
struct SAngle
{
	s16 Angle[3];
	s16 T;
};
struct SMag
{
	s16 h[3];
	s16 T;
};

struct SDStatus
{
	s16 sDStatus[4];
};

struct SPress
{
	long lPressure;
	long lAltitude;
};

struct SLonLat
{
	long lLon;
	long lLat;
};

struct SGPSV
{
	s16 sGPSHeight;
	s16 sGPSYaw;
	long lGPSVelocity;
};
struct SQ
{ s16 q[4];
};

typedef struct{
	int32_t roll_sensor;
	int32_t pitch_sensor;
	int32_t yaw_sensor;
	float temp;			//�¶�
	vec3f acc;//���ٶ�
	vec3f gyro;//���ٶ�
	vec3f euler;//�Ƕ�
	vec3f mag;//�ų�
}IMU_JY901;
extern IMU_JY901 jy901;

void imu_JY901_init(void);
void imu_JY901_read(void);
void imu_JY901_ACC_calibration(void);
void imu_JY901_set_freq(int rate);
void imu_JY901_set_baud(int rate);
void imu_JY901_print(void);
void JY901_get_acc(vec3f);
void JY901_get_euler(vec3f);
void JY901_get_gyro(vec3f);
void JY901_get_mag(vec3f);
void JY901_get_temp(float *temp);
void imu_JY901_check(void);


#endif
