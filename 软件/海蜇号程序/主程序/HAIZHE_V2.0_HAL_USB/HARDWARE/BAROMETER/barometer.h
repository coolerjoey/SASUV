#ifndef _BAROMETER_H
#define _BAROMETER_H

#include "sys.h"
#include "config.h"
#include "baro_av.h"

#define baro_av_adc_init() 0

//��ȼ����Ͷ���
typedef enum{
	analog_vol = 0,	//ģ���ѹ��
	analog_curr,	//ģ�������
	pressure		//����ѹ����
}BARO_TYPE;

typedef struct{
	BARO_TYPE baro_type;//��ȼ�����
	u32 pressure;		//����ѹ��
	float analog_vol;	//ģ���ѹ
	float analog_curr;	//ģ�����
	float depth;		//���(cm)
}BAROMETER;

extern BAROMETER barometer;

void baro_init(void);
void baro_read(void);

#endif
