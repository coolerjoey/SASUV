#ifndef __ATKP_H
#define __ATKP_H
#include <stdint.h>
#include <stdbool.h>
#include "sys.h"

/*上行帧头*/
#define UP_BYTE1 0xAA
#define UP_BYTE2 0xAA

/*下行帧头*/
#define DOWN_BYTE1 0xAA
#define DOWN_BYTE2 0xAF

#define ATKP_MAX_DATA_SIZE 30

/*通讯数据结构*/
typedef struct
{
	uint8_t msgID;
	uint8_t dataLen;
	uint8_t data[ATKP_MAX_DATA_SIZE];
}atkp_t;

/*上行指令ID*/
typedef enum 
{
	UP_VERSION		= 0x00,
	UP_STATUS		= 0x01,
	UP_SENSER		= 0x02,
	UP_RCDATA		= 0x03,
	UP_GPSDATA		= 0x04,
	UP_POWER		= 0x05,
	UP_MOTOR		= 0x06,
	UP_SENSER2		= 0x07,
	UP_FLYMODE		= 0x0A,
	UP_SPEED 		= 0x0B,
	UP_PID1			= 0x10,
	UP_PID2			= 0x11,
	UP_PID3			= 0x12,
	UP_PID4			= 0x13,
	UP_PID5			= 0x14,
	UP_PID6			= 0x15,
	UP_RADIO		= 0x40,
	UP_MSG			= 0xEE,
	UP_CHECK		= 0xEF,
	
	UP_REMOTER		= 0x50,
	UP_PRINTF		= 0x51,
	
	UP_USER_DATA1	= 0xF1,
	UP_USER_DATA2	= 0xF2,
	UP_USER_DATA3	= 0xF3,
	UP_USER_DATA4	= 0xF4,
	UP_USER_DATA5	= 0xF5,
	UP_USER_DATA6	= 0xF6,
	UP_USER_DATA7	= 0xF7,
	UP_USER_DATA8	= 0xF8,
	UP_USER_DATA9	= 0xF9,
	UP_USER_DATA10	= 0xFA,
}upmsgID_e;


/*下行指令*/
#define  D_COMMAND_ACC_CALIB		0x01
#define  D_COMMAND_GYRO_CALIB		0x02
#define  D_COMMAND_MAG_CALIB		0x04
#define  D_COMMAND_BARO_CALIB		0x05
#define  D_COMMAND_ACC_CALIB_EXIT	0x20
#define  D_COMMAND_ACC_CALIB_STEP1	0x21
#define  D_COMMAND_ACC_CALIB_STEP2	0x22
#define  D_COMMAND_ACC_CALIB_STEP3	0x23
#define  D_COMMAND_ACC_CALIB_STEP4	0x24
#define  D_COMMAND_ACC_CALIB_STEP5	0x25
#define  D_COMMAND_ACC_CALIB_STEP6	0x26
#define  D_COMMAND_FLIGHT_LOCK		0xA0
#define  D_COMMAND_FLIGHT_ULOCK		0xA1

#define  D_ACK_READ_PID				0x01
#define  D_ACK_READ_VERSION			0xA0
#define  D_ACK_RESET_PARAM			0xA1
/*下行指令ID*/
typedef enum 
{
	DOWN_COMMAND	= 0x01,
	DOWN_ACK			= 0x02,
	DOWN_RCDATA		= 0x03,
	DOWN_POWER		= 0x05,
	DOWN_FLYMODE	= 0x0A,
	DOWN_PID1			= 0x10,
	DOWN_PID2			= 0x11,
	DOWN_PID3			= 0x12,
	DOWN_PID4			= 0x13,
	DOWN_PID5			= 0x14,
	DOWN_PID6			= 0x15,
	DOWN_RADIO		= 0x40,
	
	DOWN_REMOTER	= 0x50,
	DOWN_IAP 			=	0XF0,
}downmsgID_e;

/*自定义消息类型*/
#define MY_MID_GET_TASK 13		//任务运行确认包
#define MAV_MID_TASK_RUN	14//任务运行确认包
#define MY_DATA_NUM		220			//一个mav数据包最大长度
typedef struct{
	char head[4];//包头
	uint8_t seq;   //消息序列
	uint8_t mid;	//消息类别
	char data[MY_DATA_NUM]; //数据
	char tail[4];			//包尾
}MY_MSG;


void atkpSendPeriod(void);
void atkpRxANOTask(void);
bool task_parse(u8 *recv_pack);

#endif /*ATKP_H*/

