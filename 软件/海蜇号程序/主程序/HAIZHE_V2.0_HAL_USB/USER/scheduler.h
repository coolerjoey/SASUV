#ifndef __SCHEDULER_H
#define __SCHEDULER_H	

#include "timer.h"


typedef struct {
	void (*Function)(void) ;
	uint16_t rate_hz;
	uint16_t max_time_cost;
	const char *name;
}Task;

#define TASK_MAX_NUM 30	//�������������

extern u16 loop_rate_hz;	//loopѭ��Ƶ��
extern u16 loop_counter;
extern u32 fastloop_time_average;
extern u32 fastloop_run_num;
extern u32 fastloop_overrun_time_max;
extern u32 fastloop_overrun_num;
extern u32 task_time_average[TASK_MAX_NUM];		//ÿ�������ƽ������ʱ��
extern u32 task_run_num[TASK_MAX_NUM];		//ÿ�������ƽ������
extern u32 task_overrun_time_max[TASK_MAX_NUM];	//ÿ�����񳬹��涨ʱ�����������ʱ��
extern u32 task_overrun_num[TASK_MAX_NUM];		//ÿ������ʱ���д���

void scheduler_init(const Task *tasks,u8 num_tasks);
void scheduler_run(int time);
u16 time_available_us(void);


#endif

