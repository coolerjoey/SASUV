#ifndef _MODE_TASK_H
#define _MODE_TASK_H
#include "sys.h"

extern u32 second_task_start;					//����ʼ����ʱ��

#define Task_MAX 10 //���������
/*���������趨*/
typedef struct{
	char task[10];
	uint8_t task_num;   //������� 
	uint8_t task_list;	//��������
	float task_beat;	//�������ʱ��
	float task_val;			//����ֵ

}TASK_LIST;
extern TASK_LIST Task_List[Task_MAX];


u8 task_init(void);
void task_run(void);

#endif

