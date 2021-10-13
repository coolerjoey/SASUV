#ifndef _ATTITUDE_CONTROLLER_H
#define _ATTITUDE_CONTROLLER_H

#include "sys.h"
#include "pid.h"

/*角度环积分限幅 -> 最大角速度设置*/
#define PID_ANGLE_ROLL_OUTPUT_LIMIT    30.0
#define PID_ANGLE_PITCH_OUTPUT_LIMIT   30.0
#define PID_ANGLE_YAW_OUTPUT_LIMIT     180.0

//角速度环积分限幅(即输出到电机的最大pwm)
#define PID_RATE_ROLL_OUTPUT_LIMIT		100.0
#define PID_RATE_PITCH_OUTPUT_LIMIT		100.0
#define PID_RATE_YAW_OUTPUT_LIMIT		100.0

extern PidObject pidAngleRoll;//roll角度环PID
extern PidObject pidAnglePitch;//pitch角度环PID
extern PidObject pidAngleYaw;//yaw角度环PID
extern PidObject pidRateRoll;//roll角速度环PID
extern PidObject pidRatePitch;//pitch角速度环PID
extern PidObject pidRateYaw;//yaw角速度环PID


void att_controller_run(float,float,float);
void attitude_Controller_Init(float attPidDt);
void reset_att_controller(void);
	
#endif
