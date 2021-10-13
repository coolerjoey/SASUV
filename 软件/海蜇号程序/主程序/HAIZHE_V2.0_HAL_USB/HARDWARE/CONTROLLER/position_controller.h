#ifndef _POSITION_CONTROLLER_H
#define _POSITION_CONTROLLER_H

#include "sys.h"
#include "pid.h"

#define PID_ACC_Z_OUTPUT_LIMIT	200		/*���ٶȻ������޷� -> �������������pwm*/
#define PID_VEL_Z_OUTPUT_LIMIT	250	/*�ٶȻ������޷� -> �����ٶ�����(cm^2/s)*/
#define PID_POS_Z_OUTPUT_LIMIT	120		/*λ�û������޷� -> ����ٶ�����(��λcm/s)*/

extern PidObject pidPosZ;//λ�û�PID
extern PidObject pidVelZ;//�ٶȻ�PID
extern PidObject pidAccZ;//���ٶȻ�PID


void att_controller_run(float,float,float);
float sqrt_controller(float error, float p, float second_ord_lim);
void pos_z_controller_run(float pos_z_target);
void position_Controller_Init( float posPidDt);
void reset_pos_z_controller(void);
void pos_to_throttle(float pos_z_target);

#endif

