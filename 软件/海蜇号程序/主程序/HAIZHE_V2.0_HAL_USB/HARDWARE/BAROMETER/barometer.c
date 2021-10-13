#include "barometer.h"
#include "adc.h"

BAROMETER barometer={
	.baro_type = analog_vol,		//��ȼ�����
	.pressure = 0,					//����ѹ��
	.analog_vol = 0,				//ģ���ѹ
	.analog_curr = 0,				//ģ�����
	.depth = 0,						//���(cm)
};

void baro_init(){	//���ó�ʼ���ֵ���ʽ
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



