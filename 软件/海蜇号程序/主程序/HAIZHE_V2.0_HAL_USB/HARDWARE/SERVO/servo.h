#ifndef _SERVO_H
#define _SERVO_H

#include "sys.h"
#include "timer.h"

#define DERVO_MUX_NUM 2
#define servo_init()	TIM2_PWM_Init(180-1,10000-1)

#endif
