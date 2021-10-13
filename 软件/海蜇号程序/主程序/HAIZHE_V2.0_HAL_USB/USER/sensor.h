#ifndef __SENSOR_H
#define __SENSOR_H	

#include "sys.h"
#include "mymath.h"
#include "INS.h"
#include "gps.h"
#include "barometer.h"
#include "mission.h"

//��ص�ѹ��ر���
#define POWER_MIN 11.20

typedef struct{
	nmea_utc_time utc;
	char gpssta;					//GPS״̬:0,δ��λ;1,�ǲ�ֶ�λ;2,��ֶ�λ;6,���ڹ���.		  
 	char posslnum;				//���ڶ�λ��������,0~12.
	float hdop;						//ˮƽ�������ӣ�0.5~99.9��						 			
	float altitude;			 	//���θ߶ȣ�-9999.9 �� 9999.9 �ף�,�����M��ʾ��λ�ס�				
	bool pos_status;			//��λ״̬��true=��Ч��λ��false=��Ч��λ
	double latitude;			//γ�� ���ȣ�
	char nshemi;					//γ�Ȱ��� N�������򣩻� S���ϰ���
	double longitude;			//���ȣ��ȣ�
	char ewhemi;					//���Ȱ��� E���������� W��������
	float speed;					//�������� m/s
	float orient;					//���溽��000.0~359.9 �ȣ����汱��Ϊ�ο���׼��
	float magdec;					//��ƫ�ǣ�000.0~180.0 �ȣ�ǰ��λ�������� 0��
	char magdir;					//��ƫ�Ƿ���E�������� W������
	char mode;						//A=������λ��D=��֣�E=���㣬N=������Ч
}gps_infor;

//��λ��������
typedef struct{
	Location pos;
	vec3f acc;
	vec3f v;
	vec3f att_rad;
	float dt;
}DR_infor;


typedef struct{
	INS ins;	//�ߵ�
	vec3f gpsvn;
	vec3f gpspos;		//��
	BAROMETER barometer;	//��ȼ�
	float temp;
	float power;
	gps_infor gps;
	float adc[4];
	
	Location position;	//ʵʱ��γ��
	Location gps_position;//GPS���յ��ľ�γ��
	Location dr_position;

	DR_infor SINS_DR,MA_DR;

	
}SENSOR;
extern SENSOR Sensor_latest,Sensor_target,Sensor_init;

void get_euler_angle_latest(vec3f);
void get_euler_gyro_latest(vec3f);
void get_eular_depth_target(vec3f euler, float *depth);
void Get_sensor_data(void);
void DR_pos_init(void);
void Sensor_vibration(void);
bool sys_check_ins(void);
bool sys_check_baro(void);
bool sys_check_BATT(void);
bool sys_check_GCS_heartbreat(void);
bool sys_check_position(void);
bool sys_check_surface(void);
bool sys_check_sd(void);
bool sys_check_flash(void);
bool sys_check_wp_path(void);
void sys_update_payload(void);

#endif

