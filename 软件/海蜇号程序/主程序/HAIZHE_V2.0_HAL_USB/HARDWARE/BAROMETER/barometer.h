#ifndef _BAROMETER_H
#define _BAROMETER_H

#include "sys.h"
#include "config.h"
#include "baro_av.h"

#define baro_av_adc_init() 0

//深度计类型定义
typedef enum{
	analog_vol = 0,	//模拟电压型
	analog_curr,	//模拟电流型
	pressure		//绝对压力型
}BARO_TYPE;

typedef struct{
	BARO_TYPE baro_type;//深度及类型
	u32 pressure;		//绝对压力
	float analog_vol;	//模拟电压
	float analog_curr;	//模拟电流
	float depth;		//深度(cm)
}BAROMETER;

extern BAROMETER barometer;

void baro_init(void);
void baro_read(void);

#endif
