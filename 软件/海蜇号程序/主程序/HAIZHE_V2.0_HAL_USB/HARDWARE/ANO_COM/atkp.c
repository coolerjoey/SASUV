//协议版本：V4.01，使用匿名科创地面站V4.34下位�?

#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include "atkp.h"
#include "motors.h"
#include "pid.h"
#include "global_para.h"
#include "telem.h"
#include "flash.h"

//数据拆分宏定义，在发送大�?字节的数据类型时，比如int16、float等，需要把数据拆分成单独字节进行发�?
#define  BYTE0(dwTemp)       ( *( (u8 *)(&dwTemp)	)  )
#define  BYTE1(dwTemp)       ( *( (u8 *)(&dwTemp) + 1) )
#define  BYTE2(dwTemp)       ( *( (u8 *)(&dwTemp) + 2) )
#define  BYTE3(dwTemp)       ( *( (u8 *)(&dwTemp) + 3) )

//数据返回周期时间（单�?0ms�?
#define  PERIOD_STATUS		3
#define  PERIOD_SENSOR 		1
#define  PERIOD_RCDATA 		4
#define  PERIOD_POWER 		10
#define  PERIOD_MOTOR			4
#define  PERIOD_SENSOR2 	4
#define  PERIOD_SPEED   	5
#define  PERIOD_USERDATA   2

#define ATKP_RX_QUEUE_SIZE 	10 /*ATKP包接收队列消息个�?/

typedef struct  
{
	u16 roll;
	u16 pitch;
	u16 yaw;
	u16 thrust;
}joystickFlyui16_t;


bool isConnect = false;
bool isInit = false;
bool flyable = false;
static joystickFlyui16_t rcdata;
//static xQueueHandle rxQueue;

static bool task_recv_flag=false;	//任务接收标志位，用来防止回传任务接收成功时串口被占用，上位机看不到输�?

static void atkpSendPacket(atkp_t *p)
{
	u8 data_to_send[p->dataLen+5];
	u8 _cnt=0;
	
	data_to_send[_cnt++]=0xAA;	//帧头
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=p->msgID;//功能�?
	data_to_send[_cnt++]=p->dataLen;//数据长度
	for(u8 i=0;i<p->dataLen;i++){	//数据
		data_to_send[_cnt++]=p->data[i];
	}
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;	//校验�?
	telem_write((char*)data_to_send,p->dataLen+5);	//此处不能用telem_send
	//TODO:USB发�?
//	if(getusbConnectState())
//	{
//		usblinkSendPacket(p);
//	}	
}
/***************************发送至匿名上位机指�?*****************************/

//发送欧拉角，运行模式，电机解锁状�?
static void sendStatus(float roll, float pitch, float yaw, s32 alt, u8 fly_model, u8 armed)
{
	u8 _cnt=0;
	atkp_t p;
	vs16 _temp;
	vs32 _temp2 = alt;
	
	p.msgID = UP_STATUS;
	
	_temp = (int)(roll*100);
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = (int)(pitch*100);
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = (int)(yaw*100);
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	
	p.data[_cnt++]=BYTE3(_temp2);
	p.data[_cnt++]=BYTE2(_temp2);
	p.data[_cnt++]=BYTE1(_temp2);
	p.data[_cnt++]=BYTE0(_temp2);
	
	p.data[_cnt++] = fly_model;
	p.data[_cnt++] = armed;
	
	p.dataLen = _cnt;
	atkpSendPacket(&p);
}
//发送加速度计，陀螺仪，磁力计数据
static void sendSensor(s16 a_x,s16 a_y,s16 a_z,s16 g_x,s16 g_y,s16 g_z,s16 m_x,s16 m_y,s16 m_z)
{
	u8 _cnt=0;
	atkp_t p;
	vs16 _temp;
	
	p.msgID = UP_SENSER;

	_temp = a_x;
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = a_y;
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = a_z;	
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	
	_temp = g_x;	
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = g_y;	
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = g_z;	
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	
	_temp = m_x;	
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = m_y;	
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = m_z;	
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = 0;	
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);	
	
	p.dataLen = _cnt;
	atkpSendPacket(&p);
}
//发送遥控模式下的油门�?
static void sendRCData(u16 thrust,u16 yaw,u16 roll,u16 pitch,u16 aux1,u16 aux2,u16 aux3,u16 aux4,u16 aux5,u16 aux6)
{
	u8 _cnt=0;
	atkp_t p;
	
	p.msgID = UP_RCDATA;
	p.data[_cnt++]=BYTE1(thrust);
	p.data[_cnt++]=BYTE0(thrust);
	p.data[_cnt++]=BYTE1(yaw);
	p.data[_cnt++]=BYTE0(yaw);
	p.data[_cnt++]=BYTE1(roll);
	p.data[_cnt++]=BYTE0(roll);
	p.data[_cnt++]=BYTE1(pitch);
	p.data[_cnt++]=BYTE0(pitch);
	p.data[_cnt++]=BYTE1(aux1);
	p.data[_cnt++]=BYTE0(aux1);
	p.data[_cnt++]=BYTE1(aux2);
	p.data[_cnt++]=BYTE0(aux2);
	p.data[_cnt++]=BYTE1(aux3);
	p.data[_cnt++]=BYTE0(aux3);
	p.data[_cnt++]=BYTE1(aux4);
	p.data[_cnt++]=BYTE0(aux4);
	p.data[_cnt++]=BYTE1(aux5);
	p.data[_cnt++]=BYTE0(aux5);
	p.data[_cnt++]=BYTE1(aux6);
	p.data[_cnt++]=BYTE0(aux6);

	p.dataLen = _cnt;
	atkpSendPacket(&p);
}
//发送电压电�?
//static void sendPower(u16 votage, u16 current)
//{
//	u8 _cnt=0;
//	atkp_t p;
//	
//	p.msgID = UP_POWER;
//	
//	p.data[_cnt++]=BYTE1(votage);
//	p.data[_cnt++]=BYTE0(votage);
//	p.data[_cnt++]=BYTE1(current);
//	p.data[_cnt++]=BYTE0(current);
//	
//	p.dataLen = _cnt;
//	atkpSendPacket(&p);
//}
//发送电机pwm
//static void sendMotorPWM(u16 m_1,u16 m_2,u16 m_3,u16 m_4,u16 m_5,u16 m_6,u16 m_7,u16 m_8)
//{
//	u8 _cnt=0;
//	atkp_t p;
//	
//	p.msgID = UP_MOTOR;
//	
//	p.data[_cnt++]=BYTE1(m_1);
//	p.data[_cnt++]=BYTE0(m_1);
//	p.data[_cnt++]=BYTE1(m_2);
//	p.data[_cnt++]=BYTE0(m_2);
//	p.data[_cnt++]=BYTE1(m_3);
//	p.data[_cnt++]=BYTE0(m_3);
//	p.data[_cnt++]=BYTE1(m_4);
//	p.data[_cnt++]=BYTE0(m_4);
//	p.data[_cnt++]=BYTE1(m_5);
//	p.data[_cnt++]=BYTE0(m_5);
//	p.data[_cnt++]=BYTE1(m_6);
//	p.data[_cnt++]=BYTE0(m_6);
//	p.data[_cnt++]=BYTE1(m_7);
//	p.data[_cnt++]=BYTE0(m_7);
//	p.data[_cnt++]=BYTE1(m_8);
//	p.data[_cnt++]=BYTE0(m_8);
//	
//	p.dataLen = _cnt;
//	atkpSendPacket(&p);
//}
//发送深�?
//static void sendSensor2(s32 bar_alt,u16 csb_alt)
//{
//	u8 _cnt=0;
//	atkp_t p;
//	
//	p.msgID = UP_SENSER2;
//	
//	p.data[_cnt++]=BYTE3(bar_alt);
//	p.data[_cnt++]=BYTE2(bar_alt);
//	p.data[_cnt++]=BYTE1(bar_alt);
//	p.data[_cnt++]=BYTE0(bar_alt);
//	
//	p.data[_cnt++]=BYTE1(csb_alt);
//	p.data[_cnt++]=BYTE0(csb_alt);
//	
//	p.dataLen = _cnt;
//	atkpSendPacket(&p);
//}
//发送三组pid参数
static void sendPid(u8 group,float p1_p,float p1_i,float p1_d,float p2_p,float p2_i,float p2_d,float p3_p,float p3_i,float p3_d)
{
	u8 _cnt=0;
	atkp_t p;
	vs16 _temp;
	
	p.msgID = 0x10+group-1;

	_temp = p1_p * 10 ;
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = p1_i * 10 ;
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = p1_d * 10 ;
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = p2_p * 10 ;
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = p2_i * 10 ;
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = p2_d * 10 ;
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = p3_p * 10 ;
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = p3_i * 10 ;
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = p3_d * 10 ;
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	
	p.dataLen = _cnt;
	atkpSendPacket(&p);
}

static void sendCheck(u8 head, u8 check_sum)
{
	atkp_t p;
	
	p.msgID = UP_CHECK;
	p.dataLen = 2;
	p.data[0] = head;
	p.data[1] = check_sum;
	atkpSendPacket(&p);
}
/*
@param:
group:第几组用户数�?
*/
static void sendUserData(u8 group,s16 a_x,s16 a_y,s16 a_z,s16 v_x,s16 v_y,s16 v_z,s16 p_x,s16 p_y,s16 p_z)
{
	u8 _cnt=0;
	atkp_t p;
	vs16 _temp;
	
	p.msgID = UP_USER_DATA1+group-1;

	_temp = a_x;
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = a_y;
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = a_z;	
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	
	_temp = v_x;	
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = v_y;	
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = v_z;	
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	
	_temp = p_x;	
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = p_y;	
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = p_z;	
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);	
	
	p.dataLen = _cnt;
	atkpSendPacket(&p);
}
/****************************************************************************/
/*数据周期性发送给上位机，�?00ms调用一�?/
void atkpSendPeriod()
{
	if(!sys_flag.telem_enable) return;	//如果数传电台端口未使能，退�?
	static u16 count_ms = 1;
	if(task_recv_flag){
		telem_send(" \r\n SET OK \r\n");	//回传至匿名地面站
		task_recv_flag = false;
	}
	if(!(count_ms % PERIOD_STATUS))	//姿态角
	{
		sendStatus(Sensor_latest.ins.euler[0],Sensor_latest.ins.euler[1],Sensor_latest.ins.euler[2],Sensor_latest.barometer.depth,vp.control_mode,sys_flag.motors_armed);
	}
	if(!(count_ms % PERIOD_SENSOR))	//加速度、角速度、磁�?
	{
		sendSensor(Sensor_latest.ins.acc[0],Sensor_latest.ins.acc[1],Sensor_latest.ins.acc[2],\
		Sensor_latest.ins.gyro[0],Sensor_latest.ins.gyro[1],Sensor_latest.ins.gyro[2],\
		Sensor_latest.ins.mag.x,Sensor_latest.ins.mag.y,Sensor_latest.ins.mag.z);
	}
	if(!(count_ms % PERIOD_USERDATA))	/*用户数据*/
	{
		sendUserData(1,Sensor_target.ins.euler[0],Sensor_target.ins.euler[1],Sensor_target.ins.euler[2],Sensor_target.barometer.depth,0,0,0,0,0);	//目标�?
		sendUserData(2,get_roll()*100,get_pitch()*100,get_yaw()*100,get_throttle()*100,0,0,0,0,0);	//各自由度控制�?
	} 
	if(!(count_ms % PERIOD_RCDATA))	//遥控器控制量，需要把[-1,1]映射到[1000,2000]
	{
		sendRCData((get_throttle()*0.5f+1.5f)*1000,(get_yaw()*0.5f+1.5f)*1000,(get_roll()*0.5f+1.5f)*1000,(get_pitch()*0.5f+1.5f)*1000,0,0,0,0,0,0);
	}
//	if(!(count_ms % PERIOD_POWER))
//	{
//		float bat = Sensor_latest.voltage;
//		sendPower(bat*100,500);
//	}
//	if(!(count_ms % PERIOD_MOTOR))
//	{
//		sendMotorPWM(thrust_return[0],thrust_return[1],thrust_return[2],thrust_return[3],0,0,0,0);
//	}
//	if(!(count_ms % PERIOD_SENSOR2))
//	{
//		int baro = getBaroData() * 100.f;
//		sendSensor2(baro, 0);
//	}
	if(++count_ms>=65535) 
		count_ms = 1;	
}

static u8 atkpCheckSum(atkp_t *packet)
{
	u8 sum;
	sum = DOWN_BYTE1;
	sum += DOWN_BYTE2;
	sum += packet->msgID;
	sum += packet->dataLen;
	for(int i=0; i<packet->dataLen; i++)
	{
		sum += packet->data[i];
	}
	return sum;
}

static void atkpReceiveAnl(atkp_t *anlPacket)
{
//	printf("ANO msgID =%d \r\n",anlPacket->msgID);
	if(anlPacket->msgID	== DOWN_COMMAND){	//命令集合
		switch(anlPacket->data[0]){
			case D_COMMAND_ACC_CALIB:
				break;
			
			case D_COMMAND_GYRO_CALIB:
				break;
			
			case D_COMMAND_MAG_CALIB:				
				break;
			
			case D_COMMAND_BARO_CALIB:
				break;
			
			case D_COMMAND_ACC_CALIB_STEP1:
				break;
			case D_COMMAND_ACC_CALIB_STEP2:
				break;
			case D_COMMAND_ACC_CALIB_STEP3:
				break;
			case D_COMMAND_ACC_CALIB_STEP4:
				break;
			case D_COMMAND_ACC_CALIB_STEP5:
				break;
			case D_COMMAND_ACC_CALIB_STEP6:
				break;
			
			case D_COMMAND_FLIGHT_LOCK:
				printf("disarmed \r\n");
				flyable = false;
				break;
			
			case D_COMMAND_FLIGHT_ULOCK:
				printf("armed \r\n");
				flyable = true;
		}
	}			 
	else if(anlPacket->msgID == DOWN_ACK){
		if(anlPacket->data[0] == D_ACK_READ_PID){/*读取PID参数*/
			printf("send PID \r\n");
			sendPid(1, pidRateRoll.kp, pidRateRoll.ki, pidRateRoll.kd,
					   pidRatePitch.kp, pidRatePitch.ki, pidRatePitch.kd,
					   pidRateYaw.kp, pidRateYaw.ki, pidRateYaw.kd 
				   );	//角加速度环PID
			sendPid(2, pidAngleRoll.kp, pidAngleRoll.ki, pidAngleRoll.kd,
					   pidAnglePitch.kp, pidAnglePitch.ki, pidAnglePitch.kd,
					   pidAngleYaw.kp, pidAngleYaw.ki, pidAngleYaw.kd 
				   );	//角度环PID
			sendPid(3, pidPosZ.kp, pidPosZ.ki, pidPosZ.kd,
					   pidVelZ.kp, pidVelZ.ki, pidVelZ.kd,
					   pidAccZ.kp, pidAccZ.ki, pidAccZ.kd
				   );	//深度PID
		}
		if(anlPacket->data[0] == D_ACK_RESET_PARAM){/*恢复默认参数（上电时的初始参数）*/
			printf("reset PID \r\n");
			attitude_Controller_Init(1/loop_rate_hz);//姿态PID控制器初始化 
			position_Controller_Init(1/loop_rate_hz);//位置PID控制器初始化 
			
			sendPid(1, pidRateRoll.kp, pidRateRoll.ki, pidRateRoll.kd,
					   pidRatePitch.kp, pidRatePitch.ki, pidRatePitch.kd,
					   pidRateYaw.kp, pidRateYaw.ki, pidRateYaw.kd 
				   );
			sendPid(2, pidAngleRoll.kp, pidAngleRoll.ki, pidAngleRoll.kd,
					   pidAnglePitch.kp, pidAnglePitch.ki, pidAnglePitch.kd,
					   pidAngleYaw.kp, pidAngleYaw.ki, pidAngleYaw.kd 
				   );
			sendPid(3, pidPosZ.kp, pidPosZ.ki, pidPosZ.kd,
					   pidVelZ.kp, pidVelZ.ki, pidVelZ.kd,
					   pidAccZ.kp, pidAccZ.ki, pidAccZ.kd
				   );
			if(sys_flag.flash_check){
				flash_write_param((u8*)&pidRateRoll.kp,pidRateRoll_kp);
				flash_write_param((u8*)&pidRateRoll.ki,pidRateRoll_ki);
				flash_write_param((u8*)&pidRateRoll.kd,pidRateRoll_kd);
				flash_write_param((u8*)&pidRatePitch.kp,pidRatePitch_kp);
				flash_write_param((u8*)&pidRatePitch.ki,pidRatePitch_ki);
				flash_write_param((u8*)&pidRatePitch.kd,pidRatePitch_kd);
				flash_write_param((u8*)&pidRateYaw.kp ,pidRateYaw_kp);
				flash_write_param((u8*)&pidRateYaw.ki,pidRateYaw_ki);
				flash_write_param((u8*)&pidRateYaw.kd,pidRateYaw_kd);
				flash_write_param((u8*)&pidAngleRoll.kp,pidAngleRoll_kp);
				flash_write_param((u8*)&pidAngleRoll.ki,pidAngleRoll_ki);
				flash_write_param((u8*)&pidAngleRoll.kd,pidAngleRoll_kd);
				flash_write_param((u8*)&pidAnglePitch.kp,pidAnglePitch_kp);
				flash_write_param((u8*)&pidAnglePitch.ki,pidAnglePitch_ki);
				flash_write_param((u8*)&pidAnglePitch.kd,pidAnglePitch_kd);
				flash_write_param((u8*)&pidAngleYaw.kp,pidAngleYaw_kp);
				flash_write_param((u8*)&pidAngleYaw.ki,pidAngleYaw_ki);
				flash_write_param((u8*)&pidAngleYaw.kd ,pidAngleYaw_kd);
				flash_write_param((u8*)&pidPosZ.kp,pidPosZ_kp);
				flash_write_param((u8*)&pidPosZ.ki,pidPosZ_ki);
				flash_write_param((u8*)&pidPosZ.kd,pidPosZ_kd);
				flash_write_param((u8*)&pidVelZ.kp,pidVelZ_kp);
				flash_write_param((u8*)&pidVelZ.ki,pidVelZ_ki);
				flash_write_param((u8*)&pidVelZ.kd,pidVelZ_kd);
				flash_write_param((u8*)&pidAccZ.kp,pidAccZ_kp);
				flash_write_param((u8*)&pidAccZ.ki,pidAccZ_ki);
				flash_write_param((u8*)&pidAccZ.kd ,pidAccZ_kd);
			}
		}
	}
	else if(anlPacket->msgID == DOWN_RCDATA){	//电机roll等通道�?
		rcdata = *((joystickFlyui16_t*)anlPacket->data);
		rcdata = rcdata;
	}
	
	//点击“写入飞控�?上位机发�?组共18套pid参数
	else if(anlPacket->msgID == DOWN_PID1){
		pidRateRoll.kp  = 0.1*((s16)(*(anlPacket->data+0)<<8)|*(anlPacket->data+1));
		pidRateRoll.ki  = 0.1*((s16)(*(anlPacket->data+2)<<8)|*(anlPacket->data+3));
		pidRateRoll.kd  = 0.1*((s16)(*(anlPacket->data+4)<<8)|*(anlPacket->data+5));
		pidRatePitch.kp = 0.1*((s16)(*(anlPacket->data+6)<<8)|*(anlPacket->data+7));
		pidRatePitch.ki = 0.1*((s16)(*(anlPacket->data+8)<<8)|*(anlPacket->data+9));
		pidRatePitch.kd = 0.1*((s16)(*(anlPacket->data+10)<<8)|*(anlPacket->data+11));
		pidRateYaw.kp   = 0.1*((s16)(*(anlPacket->data+12)<<8)|*(anlPacket->data+13));
		pidRateYaw.ki   = 0.1*((s16)(*(anlPacket->data+14)<<8)|*(anlPacket->data+15));
		pidRateYaw.kd   = 0.1*((s16)(*(anlPacket->data+16)<<8)|*(anlPacket->data+17));
		u8 cksum = atkpCheckSum(anlPacket);
		sendCheck(anlPacket->msgID,cksum);
		if(sys_flag.flash_check){
			flash_write_param((u8*)&pidRateRoll.kp,pidRateRoll_kp);
			flash_write_param((u8*)&pidRateRoll.ki,pidRateRoll_ki);
			flash_write_param((u8*)&pidRateRoll.kd,pidRateRoll_kd);
			flash_write_param((u8*)&pidRatePitch.kp,pidRatePitch_kp);
			flash_write_param((u8*)&pidRatePitch.ki,pidRatePitch_ki);
			flash_write_param((u8*)&pidRatePitch.kd,pidRatePitch_kd);
			flash_write_param((u8*)&pidRateYaw.kp ,pidRateYaw_kp);
			flash_write_param((u8*)&pidRateYaw.ki,pidRateYaw_ki);
			flash_write_param((u8*)&pidRateYaw.kd,pidRateYaw_kd);
		}
	}
	else if(anlPacket->msgID == DOWN_PID2){
		pidAngleRoll.kp  = 0.1*((s16)(*(anlPacket->data+0)<<8)|*(anlPacket->data+1));
//		printf("change pidAngleRoll_kp to %.2f \r\n",pidAngleRoll.kp);
		pidAngleRoll.ki  = 0.1*((s16)(*(anlPacket->data+2)<<8)|*(anlPacket->data+3));
		pidAngleRoll.kd  = 0.1*((s16)(*(anlPacket->data+4)<<8)|*(anlPacket->data+5));
		pidAnglePitch.kp = 0.1*((s16)(*(anlPacket->data+6)<<8)|*(anlPacket->data+7));
		pidAnglePitch.ki = 0.1*((s16)(*(anlPacket->data+8)<<8)|*(anlPacket->data+9));
		pidAnglePitch.kd = 0.1*((s16)(*(anlPacket->data+10)<<8)|*(anlPacket->data+11));
		pidAngleYaw.kp   = 0.1*((s16)(*(anlPacket->data+12)<<8)|*(anlPacket->data+13));
		pidAngleYaw.ki   = 0.1*((s16)(*(anlPacket->data+14)<<8)|*(anlPacket->data+15));
		pidAngleYaw.kd   = 0.1*((s16)(*(anlPacket->data+16)<<8)|*(anlPacket->data+17));
		u8 cksum = atkpCheckSum(anlPacket);
		sendCheck(anlPacket->msgID,cksum);
		if(sys_flag.flash_check){
			flash_write_param((u8*)&pidAngleRoll.kp,pidAngleRoll_kp);
			flash_write_param((u8*)&pidAngleRoll.ki,pidAngleRoll_ki);
			flash_write_param((u8*)&pidAngleRoll.kd,pidAngleRoll_kd);
			flash_write_param((u8*)&pidAnglePitch.kp,pidAnglePitch_kp);
			flash_write_param((u8*)&pidAnglePitch.ki,pidAnglePitch_ki);
			flash_write_param((u8*)&pidAnglePitch.kd,pidAnglePitch_kd);
			flash_write_param((u8*)&pidAngleYaw.kp,pidAngleYaw_kp);
			flash_write_param((u8*)&pidAngleYaw.ki,pidAngleYaw_ki);
			flash_write_param((u8*)&pidAngleYaw.kd ,pidAngleYaw_kd);
		}
	}		
	else if(anlPacket->msgID == DOWN_PID3){
		pidPosZ.kp = 0.1*((s16)(*(anlPacket->data+0)<<8)|*(anlPacket->data+1));
//		printf("change pidPosZ_kp to %.2f \r\n",pidPosZ.kp);
		pidPosZ.ki = 0.1*((s16)(*(anlPacket->data+2)<<8)|*(anlPacket->data+3));
		pidPosZ.kd = 0.1*((s16)(*(anlPacket->data+4)<<8)|*(anlPacket->data+5));
		pidVelZ.kp = 0.1*((s16)(*(anlPacket->data+6)<<8)|*(anlPacket->data+7));
		pidVelZ.ki = 0.1*((s16)(*(anlPacket->data+8)<<8)|*(anlPacket->data+9));
		pidVelZ.kd = 0.1*((s16)(*(anlPacket->data+10)<<8)|*(anlPacket->data+11));
		pidAccZ.kp = 0.1*((s16)(*(anlPacket->data+12)<<8)|*(anlPacket->data+13));
		pidAccZ.ki = 0.1*((s16)(*(anlPacket->data+14)<<8)|*(anlPacket->data+15));
		pidAccZ.kd = 0.1*((s16)(*(anlPacket->data+16)<<8)|*(anlPacket->data+17));
		u8 cksum = atkpCheckSum(anlPacket);
		sendCheck(anlPacket->msgID,cksum);
		if(sys_flag.flash_check){
			flash_write_param((u8*)&pidPosZ.kp,pidPosZ_kp);
			flash_write_param((u8*)&pidPosZ.ki,pidPosZ_ki);
			flash_write_param((u8*)&pidPosZ.kd,pidPosZ_kd);
			flash_write_param((u8*)&pidVelZ.kp,pidVelZ_kp);
			flash_write_param((u8*)&pidVelZ.ki,pidVelZ_ki);
			flash_write_param((u8*)&pidVelZ.kd,pidVelZ_kd);
			flash_write_param((u8*)&pidAccZ.kp,pidAccZ_kp);
			flash_write_param((u8*)&pidAccZ.ki,pidAccZ_ki);
			flash_write_param((u8*)&pidAccZ.kd ,pidAccZ_kd);
		}
	}
	else if(anlPacket->msgID == DOWN_PID4){		
//		u8 cksum = atkpCheckSum(anlPacket);
//		sendCheck(anlPacket->msgID,cksum);
	}
	else if(anlPacket->msgID == DOWN_PID5){
//		u8 cksum = atkpCheckSum(anlPacket);
//		sendCheck(anlPacket->msgID,cksum);
	}
	else if(anlPacket->msgID == DOWN_PID6){
//		s16 temp1  = ((s16)(*(anlPacket->data+0)<<8)|*(anlPacket->data+1));
//		s16 temp2  = ((s16)(*(anlPacket->data+2)<<8)|*(anlPacket->data+3));
//		s16 temp3  = ((s16)(*(anlPacket->data+4)<<8)|*(anlPacket->data+5));
//		s16 enable = ((s16)(*(anlPacket->data+6)<<8)|*(anlPacket->data+7));
//		s16 m1_set = ((s16)(*(anlPacket->data+8)<<8)|*(anlPacket->data+9));
//		s16 m2_set = ((s16)(*(anlPacket->data+10)<<8)|*(anlPacket->data+11));
//		s16 m3_set = ((s16)(*(anlPacket->data+12)<<8)|*(anlPacket->data+13));
//		s16 m4_set = ((s16)(*(anlPacket->data+14)<<8)|*(anlPacket->data+15));
//		setMotorPWM(enable,m1_set,m2_set,m3_set,m4_set);
//		attitudePIDwriteToConfigParam();
//		positionPIDwriteToConfigParam();
//		u8 cksum = atkpCheckSum(anlPacket);
//		sendCheck(anlPacket->msgID,cksum);
	}
	else if(anlPacket->msgID == DOWN_IAP){	//进入IAP功能
		NVIC_SystemReset();
	}
} 

//任务列表解析
bool task_parse(u8 *recv_pack){
	MY_MSG my_msg;
	char task_num=0, *token,bracket,*task;
	u8 i=0;
	//%[^f]表示data取到字符f为止的字符串
	sscanf((const char*)recv_pack,"%s %d %d %[^f] %s",my_msg.head,(int*)&my_msg.seq, (int*)&my_msg.mid, my_msg.data, my_msg.tail);
	if(strcmp(my_msg.tail,"ff")) return false;	//判断包尾
//	printf("%s",recv_pack);
	switch(my_msg.mid){
		case MAV_MID_TASK_RUN:
//			printf("开始运�?\r\n");
			if(vp.control_mode!=AUTO){
				printf("Not in AUTO mode!\r\n");
				break;
			}
			if(sys_flag.motors_armed != true){
				printf("Motor Disarmed \r\n");
				break;
			}
			printf("Task run \r\n");
			sys_flag.task_run = true;
			sys_flag.log_creat = true;	//创建新的纪录文件
			sys_flag.task_auto_switch = true;		//设置任务目标�?
			break;
		case MY_MID_GET_TASK:
			task_num=0;
			i=0;
//			printf("task=%s \r\n",my_msg.data);
			token = strtok(my_msg.data, " ");// 获取第一个子字符�?//
			sscanf(token,"%d",(int*)&task_num);//提取任务个数
			vp.task_total_num = task_num;
			printf("auto run task num=%d \r\n",task_num);
			while( token != NULL ) {
				//函数返回第一个分隔符分隔的子串后，将第一参数设置为NULL，函数将返回剩下的子�?
				token = strtok(NULL, ","); //�?,'提取各个子任�?
//				printf("token[%d]=%s\r\n",i,token);
				strcpy(Task_List[i++].task,token);
			}
			for(i=0;i<vp.task_total_num;i++){
				//设置各个子任务的�?bracket用于提取"[]"
				sscanf((char *)Task_List[i].task,"%c %d %d %f %f %c",&bracket,(int*)&Task_List[i].task_num,(int*)&Task_List[i].task_list, &Task_List[i].task_val,&Task_List[i].task_beat,&bracket);
				Task_List[i].task_beat *= 10;//计时�?.1s为单�?
				vp.task_total_time += Task_List[i].task_beat;//10个子任务的时长累�?
				switch(Task_List[i].task_list){
					case 0:task = "NONE";Task_List[i].task_val=0.0;Task_List[i].task_beat=0.0;break;//强制将空白动作的设置清零	空白
					case 1:task = "DEPTH_HOLD";break;//定深
					case 2:task = "FORWARD";break;//前进
					case 3:task = "BACKWARD";break;//后退
					case 4:task = "MOVE_LEFT";break;//左移
					case 5:task = "MOVE_RIGHT";break;//右移
					case 6:task = "TURN_LEFT";break;//左转
					case 7:task = "TURN_RIGHT";break;//右转
				}
				task_recv_flag = true;
				//				telem_send(" \r\n SET OK \r\n");	//回传至匿名地面站	//不能在此回传，否则上位机接收不到，需要通过标志位task_recv_flag然后在发送函数回�?
				printf("TASK %d : %s %.1f ,Last %.1f \r\n",Task_List[i].task_num,task,Task_List[i].task_val,Task_List[i].task_beat);
			}
			break;
		default:
			break;
	
	}
	
	return true;
}

void atkpRxANOTask()
{
	static u8 RxBuffer[50];
	static u8 _data_len = 0,_data_cnt = 0;
	static u8 state = 0;
	u16 buf_bytes=telem_uart_available;
	u8 recv_buf[buf_bytes];
	memcpy(recv_buf,telem_uart_buf,buf_bytes);
	//为了不影响接收，即时清空接收缓存
	telem_uart_available = 0;
	memset(telem_uart_buf,0,sizeof(telem_uart_buf)/sizeof(char));
	u16 i = 0;
//	printf("乱码 \r\n");
//	if(buf_bytes>0){
//		for(i=0;i<buf_bytes;i++)
//			printf("%x ",recv_buf[i]);
//		printf("\r\n");
//	}
	if(buf_bytes>0)
		if(task_parse(recv_buf)) return;	//如果是任务列表则不进行下面解�?
	while(i<buf_bytes){
		u8 data=recv_buf[i++];
		if(state==0&&data==0xAA){
			state=1;
			RxBuffer[0]=data;
		}
		else if(state==1&&data==0xAF){
			state=2;
			RxBuffer[1]=data;
		}
		else if(state==2&&data<0XF1){	//功能�?
			state=3;
			RxBuffer[2]=data;
		}
		else if(state==3&&data<50){	//长度
			state = 4;
			RxBuffer[3]=data;
			_data_len = data;
			_data_cnt = 0;
		}
		else if(state==4&&_data_len>0){	//数据
			_data_len--;
			RxBuffer[4+_data_cnt++]=data;
			if(_data_len==0)
				state = 5;
		}
		else if(state==5){
			state = 0;
			RxBuffer[4+_data_cnt]=data;
//			for(int n=0;n<_data_cnt+5;n++)
//				printf("%x ",RxBuffer[n]);
//			printf("\r\n");
			u8 sum = 0;
			for(u8 i=0;i<(_data_cnt+4);i++)
				sum += RxBuffer[i];
			if(!(sum==data))		//判断sum
				break;		//校验位不对则退�?
			else{				//校验位正确进行解�?
				atkp_t p;
				p.msgID = RxBuffer[2];
				p.dataLen = RxBuffer[3];
				for(u8 i=0;i<_data_cnt;i++)
					p.data[i] = RxBuffer[i+4];
				printf("ANO_msgID = 0x%02x \r\n",p.msgID);
				atkpReceiveAnl(&p);
			}
		}
		else
			state = 0;
	}
}

