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

	float inter = pid->ki * pid->error * pid->dt;	//��ǰ���ֱ���(ע�⣺error��Ҫ��ǰ��ֵ)
	pid->integ += inter;	//pid->integ�ǲ��޷�����ֵ�����������ڻ���ֵ��Ȼ������ֵ�����޷���ֵ��pid->outI

	if(pid->reset_done){	//�������ڳ�ʼ��
		pid->ptr = (float*)mymalloc(SRAMIN, DEFAULT_PID_SLIDING_WINDOW*sizeof(float));
		u16 per = my_mem_perused(SRAMIN);
		pid->sw_pos = 0;
		pid->reset_done = false;
	}
	if(pid->ptr!=NULL){
		*(pid->ptr + pid->sw_pos++)  = inter;	//�������ڸ�ֵ
	}
	if(pid->sw_pos == DEFAULT_PID_SLIDING_WINDOW){	//��������������ͷ��ʼ���Ǽ�¼
		pid->sw_pos = 0;
		pid->sliding_window_enable = true;	//��������˵�����Կ�ʼ��������	
	}	
	if(pid->sliding_window_enable && pid->ptr!=NULL){	//��ʼ��������
		float inter_to_sub = *(pid->ptr+pid->sw_pos);
		pid->integ -= inter_to_sub;
	}
		
	//�����޷�
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
	ǰ������ԭ��
		����ǿ�����ҳ�ʼ״̬�仯�����ϵͳ�����Ը���һ���̶���ǰ������
		����ǿ�����ҳ�ʼ״̬����(�ܲ�������)��ϵͳ�����Ը��ݸ���ʵʱ����ǰ����
		����������ϵͳ(�¶ȣ�����),��������ǰ����
	*/
//	pid->feedforward = func();	//����ǰ������
	output = pid->outP + pid->outI + pid->outD + pid->feedforward;
	
	//����޷�
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
