#include "scheduler.h"
#include "mymath.h"
#include "global_para.h"

u16 loop_counter=0;	//loopѭ������
u16 loop_rate_hz=100;	//loopѭ��Ƶ��
const Task *scheduler_list;
u8 num_of_tasks=0;
u16 task_time_started=0;//����ʼʱ��
u16 task_time_allowed=0;
u32 fastloop_time_average=0;
u32 fastloop_run_num=0;
u32 fastloop_overrun_time_max=0;
u32 fastloop_overrun_num=0;
u32 task_time_average[TASK_MAX_NUM]={0};		//ÿ�������ƽ������ʱ��
u32 task_run_num[TASK_MAX_NUM]={0};		//ÿ����������д���
u32 task_overrun_time_max[TASK_MAX_NUM]={0};	//ÿ�����񳬹��涨ʱ�����������ʱ��
u32 task_overrun_num[TASK_MAX_NUM]={0};		//ÿ������ʱ���д���

void scheduler_init(const Task *tasks,u8 num_tasks){
	scheduler_list = tasks;
	num_of_tasks = num_tasks;
	printf("[OK] tasks num:%d init \r\n",num_tasks);
	
}

/*��������
time:task_run������ʱ��(��λ:10us)
*/
void scheduler_run(int time){
	if(time<0) return;
	uint8_t i;	
	int time_available=time;
	static uint16_t last_run[TASK_MAX_NUM];	//�ϴ�ִ�и������ѭ������
	uint16_t dt =0;	//���ϴ����е����ڼ����ѭ��Ȧ��
	uint16_t interval_ticks;

	u16 time_taken;//���񻨷�ʱ��
	if(loop_counter++ == USHRT_MAX){	//��ѭ�����д���
		loop_counter=0;	
		for(i=0;i<TASK_MAX_NUM;i++)
			last_run[i] = 0;
	}
	for(i=0;i<num_of_tasks;i++){
		dt = loop_counter-last_run[i];
		interval_ticks =  loop_rate_hz/scheduler_list[i].rate_hz;	//��ѭ�����м��β������д�����
		if(interval_ticks<1)
			interval_ticks=1;
		if(dt>=interval_ticks){	//ѭ��Ȧ��>=��Ҫ���Ȧ��
			if(scheduler_list[i].max_time_cost<time_available){	//��ǰ������������ʱ��<ѭ��ʣ��ʱ����ִ�и�����		
				last_run[i] = loop_counter;//��¼���д˴������loop()ѭ��Ȧ��
				task_time_started = sys_time.loop_tick;//��¼����ʼʱ��	
				task_time_allowed = scheduler_list[i].max_time_cost;//��¼���������ʱ��
				(*(scheduler_list[i].Function))();//��ת����		
				time_taken = sys_time.loop_tick-task_time_started;//��¼��������ʱ��
				//��������ƽ��ʱ�䣬�˴��Ƕ�̬ƽ��ʱ�䣬��Ϊ��Щ������Ҫ�ض������ſ���
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
					//��¼���������ʱ��
					if(time_taken>task_overrun_time_max[i])	task_overrun_time_max[i] = time_taken;	
					//printf("[warnning] task \"%s\" overrun:%d<%d \r\n",scheduler_list[i].name,scheduler_list[i].max_time_cost,time_taken);
				}
				time_available -= time_taken;	//ʣ�������ʱ��
				if(time_available<0) return;	//ǰһ������ռ��ȫ��ʣ��ʱ��
			}
			else if(sys_flag.show_task_pass){	//ʱ�䲻���޷�ִ�е�����
				printf("[task] pass \"%s\" \r\n", scheduler_list[i].name);
			}
		}	
	}
}

u16 time_available_us(){
	u32 dt = sys_time.loop_tick - task_time_started;	//sys_time.loop_tick ��fastloopÿ��ѭ��������
	if(dt>task_time_allowed) return 0;	//��������ʱ��
	return task_time_allowed-dt;		//���ظó���ʣ������ʱ��
}


