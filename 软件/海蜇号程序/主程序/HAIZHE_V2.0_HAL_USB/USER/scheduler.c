#include "scheduler.h"
#include "mymath.h"
#include "global_para.h"

u16 loop_counter=0;	//loop循环次数
u16 loop_rate_hz=100;	//loop循环频率
const Task *scheduler_list;
u8 num_of_tasks=0;
u16 task_time_started=0;//任务开始时间
u16 task_time_allowed=0;
u32 fastloop_time_average=0;
u32 fastloop_run_num=0;
u32 fastloop_overrun_time_max=0;
u32 fastloop_overrun_num=0;
u32 task_time_average[TASK_MAX_NUM]={0};		//每个任务的平均运行时常
u32 task_run_num[TASK_MAX_NUM]={0};		//每个任务的运行次数
u32 task_overrun_time_max[TASK_MAX_NUM]={0};	//每个任务超过规定时常的最大运行时常
u32 task_overrun_num[TASK_MAX_NUM]={0};		//每个任务超时运行次数

void scheduler_init(const Task *tasks,u8 num_tasks){
	scheduler_list = tasks;
	num_of_tasks = num_tasks;
	printf("[OK] tasks num:%d init \r\n",num_tasks);
	
}

/*运行任务
time:task_run可运行时间(单位:10us)
*/
void scheduler_run(int time){
	if(time<0) return;
	uint8_t i;	
	int time_available=time;
	static uint16_t last_run[TASK_MAX_NUM];	//上次执行该任务的循环次数
	uint16_t dt =0;	//从上次运行到现在间隔的循环圈数
	uint16_t interval_ticks;

	u16 time_taken;//任务花费时间
	if(loop_counter++ == USHRT_MAX){	//主循环运行次数
		loop_counter=0;	
		for(i=0;i<TASK_MAX_NUM;i++)
			last_run[i] = 0;
	}
	for(i=0;i<num_of_tasks;i++){
		dt = loop_counter-last_run[i];
		interval_ticks =  loop_rate_hz/scheduler_list[i].rate_hz;	//主循环运行几次才能运行此任务
		if(interval_ticks<1)
			interval_ticks=1;
		if(dt>=interval_ticks){	//循环圈数>=需要间隔圈数
			if(scheduler_list[i].max_time_cost<time_available){	//当前任务运行所需时长<循环剩余时长，执行该任务		
				last_run[i] = loop_counter;//记录运行此次任务的loop()循环圈数
				task_time_started = sys_time.loop_tick;//记录任务开始时间	
				task_time_allowed = scheduler_list[i].max_time_cost;//记录任务允许最长时常
				(*(scheduler_list[i].Function))();//跳转运行		
				time_taken = sys_time.loop_tick-task_time_started;//记录任务所花时间
				//计算任务平均时间，此处是动态平均时间，因为有些任务需要特定条件才开启
				static u32 dynamic_time = 0;
				if(sys_time.one_second > dynamic_time+10){
					memset(task_time_average,0,sizeof(task_time_average));
					memset(task_run_num,0,sizeof(task_run_num));
					memset(task_overrun_num,0,sizeof(task_overrun_num));
					memset(task_overrun_time_max,0,sizeof(task_overrun_time_max));
					dynamic_time = sys_time.one_second;
				}
				task_time_average[i] = (task_time_average[i]*task_run_num[i]+time_taken)/(task_run_num[i]+1);
				task_run_num[i]++;
				if(time_taken > scheduler_list[i].max_time_cost){
					task_overrun_num[i]++;
					//记录任务运行最长时间
					if(time_taken>task_overrun_time_max[i])	task_overrun_time_max[i] = time_taken;	
					//printf("[warnning] task \"%s\" overrun:%d<%d \r\n",scheduler_list[i].name,scheduler_list[i].max_time_cost,time_taken);
				}
				time_available -= time_taken;	//剩余可运行时间
				if(time_available<0) return;	//前一个任务挤占了全部剩余时间
			}
			else if(sys_flag.show_task_pass){	//时间不够无法执行的任务
				printf("[task] pass \"%s\" \r\n", scheduler_list[i].name);
			}
		}	
	}
}

u16 time_available_us(){
	u32 dt = sys_time.loop_tick - task_time_started;	//sys_time.loop_tick 在fastloop每次循环会清零
	if(dt>task_time_allowed) return 0;	//超过运行时间
	return task_time_allowed-dt;		//返回该程序剩余运行时间
}


