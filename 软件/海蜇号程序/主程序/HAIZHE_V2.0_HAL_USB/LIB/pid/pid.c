#include "pid.h"
#include "mymath.h"
#include "malloc.h"

void pidInit(PidObject* pid, const float desired, const pidInit_t pidParam, const float dt)
{
	pid->error     = 0;
	pid->prevError = 0;
	pid->integ     = 0;
	pid->deriv     = 0;
	pid->desired = desired;
	pid->kp = pidParam.kp;
	pid->ki = pidParam.ki;
	pid->kd = pidParam.kd;
	pid->iLimit = pidParam.iLimit;
	pid->outputLimit = pidParam.outputLimit;
	pid->dt = dt;	
	pid->feedforward = pidParam.feedfoward;
	pid->ptr = NULL;
	pid->sliding_window_enable = false;
	pid->sw_pos = 0;
	pid->reset_done = true;
}

static float get_intergrator(PidObject* pid){
	if(is_zero(pid->ki)) return 0;

	float inter = pid->ki * pid->error * pid->dt;	//当前积分变量(注意：error需要提前赋值)
	pid->integ += inter;	//pid->integ是不限幅积分值，即滑动窗口积分值，然后对这个值进行限幅后赋值给pid->outI

	if(pid->reset_done){	//滑动窗口初始化
		pid->ptr = (float*)mymalloc(SRAMIN, DEFAULT_PID_SLIDING_WINDOW*sizeof(float));
		u16 per = my_mem_perused(SRAMIN);
		pid->sw_pos = 0;
		pid->reset_done = false;
	}
	if(pid->ptr!=NULL){
		*(pid->ptr + pid->sw_pos++)  = inter;	//滑动窗口赋值
	}
	if(pid->sw_pos == DEFAULT_PID_SLIDING_WINDOW){	//滑动窗口满，从头开始覆盖记录
		pid->sw_pos = 0;
		pid->sliding_window_enable = true;	//窗口已满说明可以开始滑动积分	
	}	
	if(pid->sliding_window_enable && pid->ptr!=NULL){	//开始滑动积分
		float inter_to_sub = *(pid->ptr+pid->sw_pos);
		pid->integ -= inter_to_sub;
	}
		
	//积分限幅
	if (pid->integ > pid->iLimit)	return pid->iLimit;
	if (pid->integ < -pid->iLimit)	return -pid->iLimit;
	return pid->integ;
}

float pidUpdate(PidObject* pid, const float error)
{
	float output;

	pid->error = error;   

	pid->deriv = (pid->error - pid->prevError) / pid->dt;

	pid->outP = pid->kp * pid->error;
	pid->outI = get_intergrator(pid);
	pid->outD = pid->kd * pid->deriv;
	/*
	前馈设置原则：
		对于强惯性且初始状态变化不大的系统，可以给定一个固定的前馈量。
		对于强惯性且初始状态不定(受不定干扰)的系统，可以根据干扰实时计算前馈量
		对于弱惯性系统(温度，航向),无需增加前馈量
	*/
//	pid->feedforward = func();	//计算前馈输入
	output = pid->outP + pid->outI + pid->outD + pid->feedforward;
	
	//输出限幅
	if (pid->outputLimit != 0)
	{
		if (output > pid->outputLimit)
			output = pid->outputLimit;
		else if (output < -pid->outputLimit)
			output = -pid->outputLimit;
	}
	
	pid->prevError = pid->error;

	pid->out = output;
	return output;
}

void pidSetIntegralLimit(PidObject* pid, const float limit) 
{
    pid->iLimit = limit;
}

void pidSetOutputLimit(PidObject* pid, const float limit) 
{
	pid->outputLimit = limit;
}

void pidSetError(PidObject* pid, const float error)
{
	pid->error = error;
}

void pidSetDesired(PidObject* pid, const float desired)
{
	pid->desired = desired;
}

float pidGetDesired(PidObject* pid)
{
	return pid->desired;
}

void pidSetKp(PidObject* pid, const float kp)
{
	pid->kp = kp;
}

void pidSetKi(PidObject* pid, const float ki)
{
	pid->ki = ki;
}

void pidSetKd(PidObject* pid, const float kd)
{
	pid->kd = kd;
}

void pidSetDt(PidObject* pid, const float dt) 
{
    pid->dt = dt;
}

void pidReset(PidObject* pid)
{
	pid->error     = 0;
	pid->prevError = 0;
	pid->integ     = 0;
	pid->deriv     = 0;
	myfree(SRAMIN, pid->ptr);
	pid->sliding_window_enable = false;
	pid->sw_pos = 0;
	pid->reset_done = true;
}
