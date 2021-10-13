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

extern u32 second_task_start;					//����ʼ����ʱ��

extern u16 loop_rate_hz;	//loopѭ��Ƶ��
extern u16 thrust_return[MOTORS_MAX_NUMER];	//ң��ģʽ�µ�����Żش�

extern PidObject pidAngleRoll;//roll�ǶȻ�PID
extern PidObject pidAnglePitch;//pitch�ǶȻ�PID
extern PidObject pidAngleYaw;//yaw�ǶȻ�PID
extern PidObject pidRateRoll;//roll���ٶȻ�PID
extern PidObject pidRatePitch;//pitch���ٶȻ�PID
extern PidObject pidRateYaw;//yaw���ٶȻ�PID
extern PidObject pidPosZ;//λ�û�PID
extern PidObject pidVelZ;//�ٶȻ�PID
extern PidObject pidAccZ;//���ٶȻ�PID

extern	float roll_in;					//�������ͨ��
extern	float pitch_in;					//��������ͨ��
extern	float yaw_in;						//ƫ������ͨ��
extern	float throttle_in;			//��������ͨ��
extern u16 motor_out[MOTORS_MAX_NUMER];	//�������pwm

extern s8 PPM_Left_UP_DOWN_PENCENT ;
extern s8 PPM_Left_LEFT_RIGHT_PENCENT ;
extern s8 PPM_RIGHT_UP_DOWN_PENCENT ;
extern s8 PPM_RIGHT_LEFT_RIGHT_PENSENT;

#define THRUST_MAX 1900
#define THRUST_MIN 1100
#define THRUST_RANGE (THRUST_MAX-THRUST_MIN)

#endif
