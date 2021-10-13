#ifndef __SENSOR_H
#define __SENSOR_H	

#include "sys.h"
#include "mymath.h"
#include "INS.h"
#include "gps.h"
#include "barometer.h"
#include "mission.h"

//电池电压相关变量
#define POWER_MIN 11.20

typedef struct{
	nmea_utc_time utc;
	char gpssta;					//GPS状态:0,未定位;1,非差分定位;2,差分定位;6,正在估算.		  
 	char posslnum;				//用于定位的卫星数,0~12.
	float hdop;						//水平精度因子（0.5~99.9）						 			
	float altitude;			 	//海拔高度（-9999.9 到 9999.9 米）,后面的M表示单位米。				
	bool pos_status;			//定位状态，true=有效定位，false=无效定位
	double latitude;			//纬度 （度）
	char nshemi;					//纬度半球 N（北半球）或 S（南半球）
	double longitude;			//经度（度）
	char ewhemi;					//经度半球 E（东经）或 W（西经）
	float speed;					//地面速率 m/s
	float orient;					//地面航向（000.0~359.9 度，以真北方为参考基准）
	float magdec;					//磁偏角（000.0~180.0 度，前导位数不足则补 0）
	char magdir;					//磁偏角方向，E（东）或 W（西）
	char mode;						//A=自主定位，D=差分，E=估算，N=数据无效
}gps_infor;

//航位推算数据
typedef struct{
	Location pos;
	vec3f acc;
	vec3f v;
	vec3f att_rad;
	float dt;
}DR_infor;


typedef struct{
	INS ins;	//惯导
	vec3f gpsvn;
	vec3f gpspos;		//度
	BAROMETER barometer;	//深度计
	float temp;
	float power;
	gps_infor gps;
	float adc[4];
	
	Location position;	//实时经纬度
	Location gps_position;//GPS接收到的经纬度
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

