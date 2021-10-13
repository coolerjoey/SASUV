#ifndef _LOWPASSFILTER_H
#define _LOWPASSFILTER_H

#include "sys.h"

float LowPassFilter(const float sample, float cutoff_freq, float dt);

#endif

