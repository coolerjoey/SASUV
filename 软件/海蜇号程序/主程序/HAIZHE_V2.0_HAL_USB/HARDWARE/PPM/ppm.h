#ifndef __PPM_H
#define __PPM_H
#include "timer.h"

#define PPM_CAP_Init(void)	TIM2_CAP_Init(27,3)
void ppm_init(void);
void ppm_update(void);

#define PPM_CHNNEL_MAX 8	//PPM���ͨ����

//ppm��Ӧң������8��ͨ��
enum rc_chnnel{
	chnnel_1 = 0,	//��ҡ������ ģ��ͨ��
	chnnel_2,			//��ҡ������	ģ��
	chnnel_3,			//��ҡ������	ģ��
	chnnel_4,			//��ҡ������	ģ��
	chnnel_5,			//WFLY-SB T8FB-CH5	����
	chnnel_6,			//WFLY-V1 T8FB-CH6	ģ��
	chnnel_7,			//WFLY-V2 T8FB-CH7	ģ��
	chnnel_8			//T8FB-CH8	ģ��
};

typedef enum{
	WFLY = 0,
	T8FB = 1
}RC_TYPE;

typedef struct{
	u8 chnnel_min;	//PPMģ��ͨ����Сֵ
	u8 chnnel_mid;	//PPMģ��ͨ���м�ֵ
	u8 chnnel_max;	//PPMģ��ͨ�����ֵ
}PPM_CHNEL;

typedef struct{
	RC_TYPE type;
	u8 unconnect_val[8];	//δ����ʱ��ͨ��ֵ
	PPM_CHNEL ppm_ch[8];	
}REMOTE_CONTROL;



#define DEAD_ZONE							5	//PPMģ��ͨ������%

extern u8 PPM_RX_STA;	//PPM����״̬
extern u8 PPM_RX_NUM;	//PPMͨ��
extern u16 PPM_VAL[PPM_CHNNEL_MAX];	//PPMͨ��ֵ

#endif


