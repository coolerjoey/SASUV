#ifndef __PPM_H
#define __PPM_H
#include "timer.h"

#define PPM_CAP_Init(void)	TIM2_CAP_Init(27,3)
void ppm_init(void);
void ppm_update(void);

#define PPM_CHNNEL_MAX 8	//PPM最大通道数

//ppm对应遥控器的8个通道
enum rc_chnnel{
	chnnel_1 = 0,	//右摇杆左右 模拟通道
	chnnel_2,			//左摇杆上下	模拟
	chnnel_3,			//右摇杆上下	模拟
	chnnel_4,			//左摇杆左右	模拟
	chnnel_5,			//WFLY-SB T8FB-CH5	数字
	chnnel_6,			//WFLY-V1 T8FB-CH6	模拟
	chnnel_7,			//WFLY-V2 T8FB-CH7	模拟
	chnnel_8			//T8FB-CH8	模拟
};

typedef enum{
	WFLY = 0,
	T8FB = 1
}RC_TYPE;

typedef struct{
	u8 chnnel_min;	//PPM模拟通道最小值
	u8 chnnel_mid;	//PPM模拟通道中间值
	u8 chnnel_max;	//PPM模拟通道最大值
}PPM_CHNEL;

typedef struct{
	RC_TYPE type;
	u8 unconnect_val[8];	//未连接时的通道值
	PPM_CHNEL ppm_ch[8];	
}REMOTE_CONTROL;



#define DEAD_ZONE							5	//PPM模拟通道死区%

extern u8 PPM_RX_STA;	//PPM接收状态
extern u8 PPM_RX_NUM;	//PPM通道
extern u16 PPM_VAL[PPM_CHNNEL_MAX];	//PPM通道值

#endif


