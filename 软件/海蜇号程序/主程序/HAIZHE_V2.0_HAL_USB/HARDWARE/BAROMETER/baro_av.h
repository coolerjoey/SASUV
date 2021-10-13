#ifndef _BARO_AV_H
#define _BARO_AV_H

#include "sys.h"
#include "adc.h"


#define baro_av_get_ADC() (float)Get_Adc(12)*3.3/4096

#define BARO_AV_K	106	//	106ad/m

#endif

