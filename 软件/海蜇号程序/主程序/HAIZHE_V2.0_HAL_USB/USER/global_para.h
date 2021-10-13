#ifndef _GLOBAL_PARA_H
#define _GLOBAL_PARA_H
#include "timer.h"
#include "parameter.h"
#include "sensor.h"
#include "mode_task.h"
#include "scheduler.h"
#include "motors.h"
#include "attitude_controller.h"
#include "position_controller.h"
#include "ppm.h"

extern PARA_FIXED fp;
extern PARA_VARYING vp;
extern SYS_FLAGS flag;
extern Channel channel;
extern TASK_LIST Task_List[Task_MAX];
extern PARAM_ID_TYPE_VALUE param_id_type_value[];
extern SYS_TIME sys_time;

extern SENSOR Sensor_latest,Sensor_target,Sensor_init;

extern u32 second_task_start;					//任务开始运行时间

extern u16 loop_rate_hz;	//loop循环频率
extern u16 thrust_return[MOTORS_MAX_NUMER];	//遥控模式下电机油门回传

extern PidObject pidAngleRoll;//roll角度环PID
extern PidObject pidAnglePitch;//pitch角度环PID
extern PidObject pidAngleYaw;//yaw角度环PID
extern PidObject pidRateRoll;//roll角速度环PID
extern PidObject pidRatePitch;//pitch角速度环PID
extern PidObject pidRateYaw;//yaw角速度环PID
extern PidObject pidPosZ;//位置环PID
extern PidObject pidVelZ;//速度环PID
extern PidObject pidAccZ;//加速度环PID

extern	float roll_in;					//横滚输入通道
extern	float pitch_in;					//俯仰输入通道
extern	float yaw_in;						//偏航输入通道
extern	float throttle_in;			//油门输入通道
extern u16 motor_out[MOTORS_MAX_NUMER];	//最终输出pwm

extern s8 PPM_Left_UP_DOWN_PENCENT ;
extern s8 PPM_Left_LEFT_RIGHT_PENCENT ;
extern s8 PPM_RIGHT_UP_DOWN_PENCENT ;
extern s8 PPM_RIGHT_LEFT_RIGHT_PENSENT;

#define THRUST_MAX 1900
#define THRUST_MIN 1100
#define THRUST_RANGE (THRUST_MAX-THRUST_MIN)

#endif
