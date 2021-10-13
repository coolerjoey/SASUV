#ifndef __SCHEDULER_H
#define __SCHEDULER_H	

#include "timer.h"


typedef struct {
	void (*Function)(void) ;
	uint16_t rate_hz;
	uint16_t max_time_cost;
	const char *name;
}Task;

#define TASK_MAX_NUM 30	//最大的子任务个数

extern u16 loop_rate_hz;	//loop循环频率
extern u16 loop_counter;
extern u32 fastloop_time_average;
extern u32 fastloop_run_num;
extern u32 fastloop_overrun_time_max;
extern u32 fastloop_overrun_num;
extern u32 task_time_average[TASK_MAX_NUM];		//每个任务的平均运行时常
extern u32 task_run_num[TASK_MAX_NUM];		//每个任务的平均次数
extern u32 task_overrun_time_max[TASK_MAX_NUM];	//每个任务超过规定时常的最大运行时常
extern u32 task_overrun_num[TASK_MAX_NUM];		//每个任务超时运行次数

void scheduler_init(const Task *tasks,u8 num_tasks);
void scheduler_run(int time);
u16 time_available_us(void);


#endif

