#ifndef _DEFINES_H
#define _DEFINES_H

#include "sys.h"

#define PACKED __attribute__((packed))
#define ARRAY_SIZE(_arr) (sizeof(_arr) / sizeof(_arr[0]))
// get high or low bytes from 2 byte integer
#define LOWBYTE(i) ((uint8_t)(i))
#define HIGHBYTE(i) ((uint8_t)(((uint16_t)(i))>>8))

//枚举值不能用于#if预编译环境下
//https://blog.csdn.net/Shayne_Lee/article/details/89408942

//IMU类型
#define  JY901 	0

//电子罗盘类型
#define TCM			0
#define DDM350B		1

typedef struct{
	float roll;			//横滚输入通道
	float pitch;		//俯仰输入通道
	float yaw;
	float throttle;
	float forward;
	float lateral;
}Channel;

//电调型号
typedef enum{
	ONE_WAY = 0,	//单向电调
	TWO_WAY,		//双向电调
}ESC_TYPE;

//日志记录模式
typedef enum{
	LOG_NONE = 0,	//不记录
	LOG_Powered,	//上电记录
	LOG_Armed,		//解锁记录
}LOG_MODE;

//导航模式
typedef enum{
	DR_SINS = 0,	//纯惯导航位推算
	DR_MA,			//纯模型航位推算
	DR_GPS,			//纯GPS航位推算
	DR_SINS_MA,		//惯导/模型组合航位推算
	DR_SINS_GPS,	//惯导/GPS航位推算
}DR_MODE;

//航点路径
typedef enum{
	PATH_LINE = 0,		//直线
	PATH_PolyLine = 1,	//折线
	PATH_Square = 2,	//四边形
	PATH_Z = 3,			//Z字型
	PATH_Circule = 4,	//圆
}WP_PATH;

#endif
