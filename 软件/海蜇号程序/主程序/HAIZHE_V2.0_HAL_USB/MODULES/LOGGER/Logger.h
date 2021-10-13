#ifndef __LOGGER_H
#define __LOGGER_H

#ifdef __cplusplus
 extern "C" {
#endif
   
#include <stdint.h>
#include "defines.h"
#include "mymath.h"

#define INT16_T_32 		"a"
#define INT8_T 			"b" 
#define UINT8_T 		"B" 
#define INT16_T 		"h" 
#define UINT16_T 		"H" 
#define INT32_T 		"i" 
#define UINT32_T 		"I" 
#define FLOAT 			"f" 
#define DOUBLE 			"d" 
#define CHAR_4 			"n" 
#define CHAR_16 		"N" 
#define CHAR_64 		"Z" 
#define INT16_T_P_100 	"c" 
#define UINT16_T_P_100 	"C" 
#define INT32_T_P_100 	"e" 
#define UINT32_T_P_100 	"E"
#define INT32_T_LAT_LON "L" 
#define UINT8_T_MODE 	"M"
#define INT64_T 		"q" 
#define UINT64_T 		"Q" 

/*
 unfortunately these need to be macros because of a limitation of
 named member structure initialisation in g++
*/
#define LOG_PACKET_HEADER	       uint8_t head1, head2, msgid;
#define LOG_PACKET_HEADER_INIT(id) HEAD_BYTE1, HEAD_BYTE2, id
#define LOG_PACKET_HEADER_LEN 3 // bytes required for LOG_PACKET_HEADER

// once the logging code is all converted we will remove these from
// this header
#define HEAD_BYTE1  0xA3    // Decimal 163
#define HEAD_BYTE2  0x95    // Decimal 149

#define NAME_LEN    4
#define FORMAT_LEN 16
#define LABELS_LEN 64


// structure used to define logging format
struct LogStructure {
  uint8_t msg_type;
  uint8_t msg_len;
  const char *name;	//不超过4字节
  const char *format;	//不超过16字节
  const char *labels;	//不超过64字节 "^[A-Za-z0-9,]{1,64}$"
};

/*
  log structures common to all vehicle types
 */
struct PACKED log_Format {
  LOG_PACKET_HEADER
  uint8_t type;
  uint8_t length;
  char name[4];
  char format[16];
  char labels[64];
};
#define FORMAT_LABELS "Type,Length,Name,Format,Columns"
#define FORMAT_FMT	(UINT8_T UINT8_T CHAR_4 CHAR_16 CHAR_64)

struct PACKED log_Parameter {
    LOG_PACKET_HEADER
    float time_s;
    char name[16];
    float value;
};
#define PARAMETER_LABELS "TimesS,Name,Value"
#define PARAMETER_FMT	(FLOAT CHAR_16 FLOAT)


struct PACKED log_TEST {
  LOG_PACKET_HEADER
  float time_s;
  uint16_t value;
};
#define TEST_LABELS "TimesS,value"
#define TEST_FMT	(FLOAT UINT16_T)


struct PACKED log_GPS {
    LOG_PACKET_HEADER
    float time_s;
    uint8_t  status;	//定位状态
    uint8_t  num_sats;	//卫星数
    float hdop;
    int32_t  latitude;
    int32_t  longitude;
    int32_t  altitude;
    float    ground_speed;	//地面速率
    float    ground_course;	//地面航向
    float vE;	//东向速度
	float vN;	//北向速度
};
#define GPS_LABELS "TimesS,Status,NSats,HDop,Lat,Lng,Alt,Spd,GCrs,VE,VN"
#define GPS_FMT	(FLOAT UINT8_T UINT8_T FLOAT INT32_T INT32_T INT32_T FLOAT FLOAT FLOAT FLOAT)

struct PACKED log_RCIN {
    LOG_PACKET_HEADER
    float time_s;
    float roll_in;
	float pitch_in;
	float yaw_in;
	float throttle_in;
	float forward_in;
	float lateral_in;
};
#define RCIN_LABELS "TimesS,Rin,Pin,Yin,Tin,Fin,Lin"
#define RCIN_FMT   (FLOAT FLOAT FLOAT FLOAT FLOAT FLOAT FLOAT)

struct PACKED log_RCOUT {
    LOG_PACKET_HEADER
    float time_s;
    uint16_t chan1;
    uint16_t chan2;
    uint16_t chan3;
    uint16_t chan4;
    uint16_t chan5;
    uint16_t chan6;
    uint16_t chan7;
    uint16_t chan8;
};
#define RCOUT_LABELS "TimesS,C1,C2,C3,C4,C5,C6,C7,C8"
#define RCOUT_FMT	(FLOAT UINT16_T UINT16_T UINT16_T UINT16_T UINT16_T UINT16_T UINT16_T UINT16_T)

struct PACKED log_BARO {
    LOG_PACKET_HEADER
    float time_s;
    float   depth;
    float   value;	//深度计读数，根据不同种类表示不同数据
    float 	temperature;
    float   value_surface;		//水面数值
};
#define BARO_LABELS "TimesS,Dep,Value,Temp,ValSur"
#define BARO_FMT	(FLOAT FLOAT FLOAT FLOAT FLOAT)

struct PACKED log_POS {
    LOG_PACKET_HEADER
    float time_s;
    int32_t lat;
    int32_t lng;
    float alt;
    float rel_home_alt;
    float rel_origin_alt;
};
#define POS_LABELS "TimesS,Lat,Lng,Alt,RelHomeAlt,RelOriginAlt"
#define POS_FMT	(FLOAT INT32_T INT32_T FLOAT FLOAT FLOAT)

struct PACKED log_POWR {
    LOG_PACKET_HEADER
    float time_s;
    float Vcc;
};
#define POWER_LABELS "TimesS,Vcc"
#define POWER_FMT	(FLOAT FLOAT)

struct PACKED log_Cmd {
    LOG_PACKET_HEADER
    float time_s;
    uint16_t command_total;
    uint16_t sequence;
    uint16_t command;
    float param1;
    float param2;
    float param3;
    float param4;
    float latitude;
    float longitude;
    float altitude;
};
#define CMD_LABELS "TimesS,CTot,CNum,CId,Prm1,Prm2,Prm3,Prm4,Lat,Lng,Alt"
#define CMD_FMT	(FLOAT UINT16_T UINT16_T UINT16_T UINT16_T FLOAT FLOAT FLOAT FLOAT FLOAT FLOAT FLOAT)

struct PACKED log_Attitude {
    LOG_PACKET_HEADER
    float time_s;
    float  control_roll;
    float  roll;
    float  control_pitch;
    float  pitch;
    float control_yaw;
    float yaw;
    float error_roll;
	float error_pitch;
    float error_yaw;
};
#define ATTITUDE_LABELS "TimesS,DesRoll,Roll,DesPitch,Pitch,DesYaw,Yaw,ErrRll,ErrPit,ErrYaw"
#define ATTITUDE_FMT	(FLOAT FLOAT FLOAT FLOAT FLOAT FLOAT FLOAT FLOAT FLOAT FLOAT)

struct PACKED log_IMU {
    LOG_PACKET_HEADER
    float time_s;
    vec3f gyro;
    vec3f accel;
//	vec3f acc_no_g;	//去除重力加速度的机体加速度
	vec3f euler;
	uint16_t pwm[6];
};
#define IMU_LABELS "TimesS,GX,GY,GZ,AX,AY,AZ,EX,EY,EZ,C1,C2,C3,C4,C5,C6"
//#define IMU_LABELS "Ts,GX,GY,GZ,AX,AY,AZ,R,P,Y"
//#define IMU_FMT	(FLOAT FLOAT FLOAT FLOAT FLOAT FLOAT FLOAT FLOAT FLOAT FLOAT)
//#define IMU_LABELS "TimesS,GyrX,GyrY,GyrZ,AccX,AccY,AccZ,Roll,Pitch,Yaw,C1,C2,C3,C4,C5,C6"
//#define IMU_LABELS "Ts,GX,GY,GZ,AX,AY,AZ,R1,P1,Y1,C1,C2,C3,C4,C5,C6"
#define IMU_FMT	(FLOAT FLOAT FLOAT FLOAT FLOAT FLOAT FLOAT FLOAT FLOAT FLOAT UINT16_T UINT16_T UINT16_T UINT16_T UINT16_T UINT16_T)

struct PACKED log_PID {
    LOG_PACKET_HEADER
    float time_s;
    float desired;
	float error;		//< error
	float prevError;	//< previous error
	float integ;		//< integral
	float deriv;		//< derivative
    float outP; 		//< proportional output (debugging)
	float outI; 		//< integral output (debugging)
	float outD; 		//< derivative output (debugging)
    float out;			//< out
};
#define PID_LABELS "TimesS,Des,Err,ErrPre,Int,Der,outP,outI,outD,Out"
#define PID_FMT	(FLOAT FLOAT FLOAT FLOAT FLOAT FLOAT FLOAT FLOAT FLOAT FLOAT)

struct PACKED log_Mode {
    LOG_PACKET_HEADER
    float time_s;
    uint8_t mode;
};
#define MODE_LABELS "TimesS,Mode"
#define MODE_FMT	(FLOAT UINT8_T)

struct PACKED log_WP {
    LOG_PACKET_HEADER
	float time_s;
	u8 nav_mode;	//导航模式
	u8 index;	//航点序号 0表示home点
	float alt;
	int lat;	
	int lng; 
};
#define WP_LABELS "TimesS,Mode,Num,Alt,Lat,Lng"
#define WP_FMT	(FLOAT UINT8_T UINT8_T FLOAT INT32_T INT32_T)

struct PACKED log_DR {
    LOG_PACKET_HEADER
    float time_s;
	u8 mode;
	float ax;
	float ay;
	float az;
	float vx;
	float vy;
	float vz;
	float dx;
	float dy;
	float dz;
	uint32_t lat;
	uint32_t lng;
	float alt;
};
#define DR_LABELS "TimesS,MODE,AnX,AnY,AnZ,VnX,VnY,VnZ,Dx,Dy,Dz,Lat,Lng,Alt"
#define DR_FMT	(FLOAT UINT8_T FLOAT FLOAT FLOAT FLOAT FLOAT FLOAT FLOAT FLOAT FLOAT UINT32_T UINT32_T FLOAT)

struct PACKED log_MODEL{
    LOG_PACKET_HEADER
    float time_s;
	vec6f force;
	float ax;
	float ay;
	float az;
	float wx;
	float wy;
	float wz;
};
#define MODEL_LABELS "TimesS,F1,F2,F3,F4,F5,F6,AccX,AccY,AccZ,GyroX,GyroY,GyroZ"
#define MODEL_FMT	(FLOAT FLOAT FLOAT FLOAT FLOAT FLOAT FLOAT FLOAT FLOAT FLOAT FLOAT FLOAT FLOAT)

//日志消息列表
enum LogMessages{
	LOG_TEST_MSG = 0,
	LOG_PARAMETER_MSG = 1,	//参数
	LOG_GPS_MSG = 2,		//GPS
	LOG_IMU_MSG = 3,		//IMU原始数据
	LOG_RCIN_MSG = 4,		//各自由度遥控输入
	LOG_RCOUT_MSG = 5,		//电机PWM输出
	LOG_BARO_MSG = 6,		//深度
	LOG_POWR_MSG = 7,		//电池
	LOG_ATTITUDE_MSG = 8,	//姿态
	LOG_CMD_MSG = 9,		//任务指令		
	LOG_MODE_MSG = 10,		//运动模式
	LOG_POS_MSG = 11,		//位置信息
	LOG_PID_ANG_RLL_MSG = 12,		//横滚角角度环PID
	LOG_PID_ANG_PIT_MSG = 13,		//纵倾角角度环PID
	LOG_PID_ANG_YAW_MSG = 14,		//航向角角度环PID
	LOG_PID_RAT_RLL_MSG = 15,		//横滚角角速度环PID
	LOG_PID_RAT_PIT_MSG = 16,		//纵倾角角速度环PID
	LOG_PID_RAT_YAW_MSG = 17,		//航向角角速度环PID
	LOG_PID_ACC_Z_MSG = 18,			//深度加速度PID
	LOG_PID_VEL_Z_MSG = 19,			//深度速度PID
	LOG_PID_POS_Z_MSG = 20,			//深度位置PID
	LOG_WP_MSG = 21,					//航点数据
	LOG_DR_MSG = 22,					//航位推算
	LOG_SINS_MSG = 23,
//	LOG_MA_DR_MSG,				//纯模型航位推算
//	LOG_SINS_MA_DR_MSG,			//惯导/模型组合航位推算
//	LOG_SINS_GPS_DR_MSG,		//惯导/GPS航位推算
	LOG_MODEL_MSG = 24, 				//模型验证输出

	LOG_ORGN_MSG,		//位置原点
	LOG_RPM_MSG,		//转速 TODO
	LOG_NKF1_MSG,		//EKF相关 TODO
	LOG_NKF2_MSG,
	LOG_NKF3_MSG,
	LOG_NKF4_MSG,
	LOG_NKF5_MSG,
	LOG_NKF6_MSG,
	LOG_NKF7_MSG,
	LOG_NKF8_MSG,
	LOG_NKF9_MSG,
	LOG_NKF10_MSG,
	LOG_NKQ1_MSG,
	LOG_NKQ2_MSG,
	LOG_XKF1_MSG,
	LOG_XKF2_MSG,
	LOG_XKF3_MSG,
	LOG_XKF4_MSG,
	LOG_XKF5_MSG,
	LOG_XKF6_MSG,
	LOG_XKF7_MSG,
	LOG_XKF8_MSG,
	LOG_XKF9_MSG,
	LOG_XKF10_MSG,
	LOG_XKQ1_MSG,
	LOG_XKQ2_MSG,
	LOG_RALLY_MSG,		//航点数据？
	LOG_FORMAT_MSG = 128, // this must remain #128

	_LOG_LAST_MSG_
};

uint8_t scan_files(void);
void logging_loop(void);

#ifdef __cplusplus
}
#endif
   
#endif /*__LOGGER_H*/
