#include "barometer.h"
#include "adc.h"

BAROMETER barometer={
	.baro_type = analog_vol,		//深度及类型
	.pressure = 0,					//绝对压力
	.analog_vol = 0,				//模拟电压
	.analog_curr = 0,				//模拟电流
	.depth = 0,						//深度(cm)
};

void baro_init(){	//设置初始深度值表达式
#if BARO_USE==BARO_AV
	barometer.baro_type = analog_vol;
#elif BARO_USE==BARO_MS5803
	barometer.baro_type = pressure;
#endif
	printf("[OK] Barometer Init complete \r\n");
}

void baro_read(){
	adc_update();
#if BARO_USE==BARO_AV
	barometer.pressure = ADC_VAL[0];
	barometer.analog_vol = ADC_VAL[0]*3.3/4096;
	barometer.depth = 100*(float)ADC_VAL[0]/BARO_AV_K;
#elif BARO_USE==BARO_MS5803
	
#endif
}



