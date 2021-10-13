#ifndef _ATTITUDE_CONTROLLER_H
#define _ATTITUDE_CONTROLLER_H

#include "sys.h"
#include "pid.h"

/*�ǶȻ������޷� -> �����ٶ�����*/
#define PID_ANGLE_ROLL_OUTPUT_LIMIT    30.0
#define PID_ANGLE_PITCH_OUTPUT_LIMIT   30.0
#define PID_ANGLE_YAW_OUTPUT_LIMIT     180.0

//���ٶȻ������޷�(���������������pwm)
#define PID_RATE_ROLL_OUTPUT_LIMIT		100.0
#define PID_RATE_PITCH_OUTPUT_LIMIT		100.0
#define PID_RATE_YAW_OUTPUT_LIMIT		100.0

extern PidObject pidAngleRoll;//roll�ǶȻ�PID
extern PidObject pidAnglePitch;//pitch�ǶȻ�PID
extern PidObject pidAngleYaw;//yaw�ǶȻ�PID
extern PidObject pidRateRoll;//roll���ٶȻ�PID
extern PidObject pidRatePitch;//pitch���ٶȻ�PID
extern PidObject pidRateYaw;//yaw���ٶȻ�PID


void att_controller_run(float,float,float);
void attitude_Controller_Init(float attPidDt);
void reset_att_controller(void);
	
#endif
