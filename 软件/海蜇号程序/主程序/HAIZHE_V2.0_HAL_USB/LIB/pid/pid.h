#ifndef __PID_H
#define __PID_H
#include <stdbool.h>
//#include "config_param.h"

#define DEFAULT_PID_INTEGRATION_LIMIT 		500.0 //默认pid的积分限幅
#define DEFAULT_PID_OUTPUT_LIMIT      		0.0	  //默认pid输出限幅，0为不限幅
#define DEFAULT_PID_SLIDING_WINDOW			500		//默认pid积分滑动窗口大小

typedef struct
{
	float desired;		//< set point
	float error;        //< error
	float prevError;    //< previous error
	float integ;        //< integral 不限幅积分值
	float deriv;        //< derivative
	float kp;           //< proportional gain
	float ki;           //< integral gain
	float kd;           //< derivative gain
	float outP;         //< proportional output (debugging)
	float outI;         //< integral output (debugging) 限幅积分
	float outD;         //< derivative output (debugging)
	float iLimit;       //< integral limit
	float outputLimit;  //< total PID output limit, absolute value. '0' means no limit.
	float dt;           //< delta-time dt
	float feedforward;	//前馈输入
	float out;			//< out
	float *ptr;			//活动窗口指针
	bool sliding_window_enable;	//滑动窗口积分使能
	int sw_pos;		//
	bool reset_done;	//pid reset标志 -> 用于给ptr辅助申请内存
} PidObject;

typedef struct 
{
	float kp;
	float ki;
	float kd;
	float iLimit;       //< integral limit
	float outputLimit;  //< total PID output limit, absolute value. '0' means no limit.
	float feedfoward;
} pidInit_t;

/*pid结构体初始化*/
void pidInit(PidObject* pid, const float desired, const pidInit_t pidParam, const float dt);
void pidSetIntegralLimit(PidObject* pid, const float limit);/*pid积分限幅设置*/
void pidSetOutputLimit(PidObject* pid, const float limit);
void pidSetDesired(PidObject* pid, const float desired);	/*pid设置期望值*/
float pidUpdate(PidObject* pid, const float error);			/*pid更新*/
float pidGetDesired(PidObject* pid);	/*pid获取期望值*/
bool pidIsActive(PidObject* pid);		/*pid状态*/
void pidReset(PidObject* pid);			/*pid结构体复位*/
void pidSetError(PidObject* pid, const float error);/*pid偏差设置*/
void pidSetKp(PidObject* pid, const float kp);		/*pid Kp设置*/
void pidSetKi(PidObject* pid, const float ki);		/*pid Ki设置*/
void pidSetKd(PidObject* pid, const float kd);		/*pid Kd设置*/
void pidSetDt(PidObject* pid, const float dt);		/*pid dt设置*/

#endif /* __PID_H */
