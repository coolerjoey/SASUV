#ifndef _DEFINES_H
#define _DEFINES_H

#include "sys.h"

#define PACKED __attribute__((packed))
#define ARRAY_SIZE(_arr) (sizeof(_arr) / sizeof(_arr[0]))
// get high or low bytes from 2 byte integer
#define LOWBYTE(i) ((uint8_t)(i))
#define HIGHBYTE(i) ((uint8_t)(((uint16_t)(i))>>8))

//ö��ֵ��������#ifԤ���뻷����
//https://blog.csdn.net/Shayne_Lee/article/details/89408942

//IMU����
#define  JY901 	0

//������������
#define TCM			0
#define DDM350B		1

typedef struct{
	float roll;			//�������ͨ��
	float pitch;		//��������ͨ��
	float yaw;
	float throttle;
	float forward;
	float lateral;
}Channel;

//����ͺ�
typedef enum{
	ONE_WAY = 0,	//������
	TWO_WAY,		//˫����
}ESC_TYPE;

//��־��¼ģʽ
typedef enum{
	LOG_NONE = 0,	//����¼
	LOG_Powered,	//�ϵ��¼
	LOG_Armed,		//������¼
}LOG_MODE;

//����ģʽ
typedef enum{
	DR_SINS = 0,	//���ߵ���λ����
	DR_MA,			//��ģ�ͺ�λ����
	DR_GPS,			//��GPS��λ����
	DR_SINS_MA,		//�ߵ�/ģ����Ϻ�λ����
	DR_SINS_GPS,	//�ߵ�/GPS��λ����
}DR_MODE;

//����·��
typedef enum{
	PATH_LINE = 0,		//ֱ��
	PATH_PolyLine = 1,	//����
	PATH_Square = 2,	//�ı���
	PATH_Z = 3,			//Z����
	PATH_Circule = 4,	//Բ
}WP_PATH;

#endif
