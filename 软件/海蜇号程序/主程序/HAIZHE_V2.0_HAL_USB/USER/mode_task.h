#ifndef _MODE_TASK_H
#define _MODE_TASK_H
#include "sys.h"

extern u32 second_task_start;					//任务开始运行时间

#define Task_MAX 10 //最大任务数
/*动作参数设定*/
typedef struct{
	char task[10];
	uint8_t task_num;   //任务次数 
	uint8_t task_list;	//任务种类
	float task_beat;	//任务持续时间
	float task_val;			//任务值

}TASK_LIST;
extern TASK_LIST Task_List[Task_MAX];


u8 task_init(void);
void task_run(void);

#endif

