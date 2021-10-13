#ifndef _MODEL_VERIFY_H
#define _MODEL_VERIFY_H

#include "sys.h"

void model_verify(void);
void set_motor_force(int motor_num, float force);
u16 input_force_to_pwm(int type, float force);

#endif

