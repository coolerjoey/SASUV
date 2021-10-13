#ifndef __ADC_H
#define __ADC_H
#include "sys.h"

extern ADC_HandleTypeDef hadc1;

void adc_init(void);
void ADC1_Init(void);
u16 Get_Adc(u32 ch);

#endif
