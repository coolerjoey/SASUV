#ifndef _CONTROL_MODE_H
#define _CONTROL_MODE_H
#include "sys.h"

//����ģʽ�ṹ��
typedef enum  {
	STABILIZE =     0,  // manual angle with manual depth/throttle
    ALT_HOLD =      2,  // manual angle with automatic depth/throttle
    AUTO =          3,  // not implemented in sub // fully automatic waypoint control using mission commands
    GUIDED =        4,  // not implemented in sub // fully automatic fly to coordinate or fly at velocity/direction using GCS immediate commands
    LOITER =		5,
    RTL =			6,
    CIRCLE =        7,  // not implemented in sub // automatic circular flight with automatic throttle
    SURFACE =       9,  // automatically return to surface, pilot maintains horizontal control
    POSHOLD =      16,  // automatic position hold with manual override, with automatic throttle
    MANUAL =       19,  // Pass-through input with no stabilization
    TASK =		   20,
}CONTROL_MODE;
//����ģʽ�ַ����ṹ�� -> ���ڿ���̨�����ַ��л�ģʽ
typedef struct{
	char mode[10];
	CONTROL_MODE mode_type;
}CONTROL_MODE_CHAR;
extern CONTROL_MODE_CHAR control_mode_char[];

typedef enum  {
	Auto_Submerge,
	Auto_WP,
	Auto_Loiter,
	Auto_Surface,
	Auto_Circle,
}AutoMode;


extern uint8_t control_mode ;
extern int second_start;
extern u8 task_set;

extern u8 switch_yaw;		//Yaw PID���ƿ��أ�1��ʾ����������ر�

void update_control_mode(void);
bool set_mode(CONTROL_MODE mode);


#endif

