#ifndef _POSITION_CONTROLLER_H
#define _POSITION_CONTROLLER_H

#include "sys.h"
#include "pid.h"

#define PID_ACC_Z_OUTPUT_LIMIT	200		/*加速度环积分限幅 -> 输出到电机的最大pwm*/
#define PID_VEL_Z_OUTPUT_LIMIT	250	/*速度环积分限幅 -> 最大加速度设置(cm^2/s)*/
#define PID_POS_Z_OUTPUT_LIMIT	120		/*位置环积分限幅 -> 最大速度设置(单位cm/s)*/

extern PidObject pidPosZ;//位置环PID
extern PidObject pidVelZ;//速度环PID
extern PidObject pidAccZ;//加速度环PID


void att_controller_run(float,float,float);
float sqrt_controller(float error, float p, float second_ord_lim);
void pos_z_controller_run(float pos_z_target);
void position_Controller_Init( float posPidDt);
void reset_pos_z_controller(void);
void pos_to_throttle(float pos_z_target);

#endif

