#ifndef _LOWPASSFILTER_H
#define _LOWPASSFILTER_H

#include "sys.h"

float constrain_float(float in,float max,float min);
float LowPassFilter(const float sample, float cutoff_freq, float dt);

#endif

