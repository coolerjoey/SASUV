#include "EC_DDM350B.h"
#include "delay.h"
#include "string.h"
#include "config.h"
#include "malloc.h"
/*
通信协议：
68 0D 00 84 00 00 32 10 00 44 03 31 26 71

ByteCount(UInt16) | Packet Frame(1-4092 UInt8) | CRC-16(UInt16)
		 		  /Frame ID    |    Payload    \
		 		  |(UInt8)	   | (1-4091 UInt8)|
CRC在线计算：http://www.ip33.com/crc.html -> CRC类型 CRC-16/XMODEM
设置回传角度、温度、加速度：00 0D 03 07 05 07 15 16 17 18 19 07 83
自动输出配置: 00 0F 18 00 00 00 00 00 00 00 00 00 00 E4 50
保存输出配置：00 05 09 6e dc
模块断电：00 05 0f 0e 1a
单次获取数据指令：00 05 04 bf 71
开启连续传输：00 05 15 bd 61	
	回传数据相应数据格式(Payload)：
	ID Count |  ID1    |  Value ID1    |   ID2   |   Value ID2   |   ID3   |  Value ID3 ……
	 (UInt8) | (UInt8) | (ID Specific) | (UInt8) | (ID Specific) | (UInt8) | (ID Specific)……
*/

DDM ddm;

static uint16_t mExpectedLen;

uint8_t DDM_CRC(void * data, uint32_t len)
{
	uint8_t * dataPtr = (uint8_t *)data;
	uint32_t index = 0;
	uint8_t crc = 0;
	while(len--) crc += dataPtr[index++];
	return crc;
}

void ec_ddm_trigger(){
	char get[5]={0x00,0x05,0x04,0xbf,0x71};
	EC_UART_write(get,5);	//获取指令	
}
void ec_ddm_continuous_mode(){
	char buf[5]={0x00,0x05,0x15,0xBD,0x61};
	EC_UART_write(buf,5); //启动连续发送
}
void ec_ddm_init(){
	EC_UART_Init(EC_UART,EC_UART_Baud);
	printf("[OK] DDM init baud %d \r\n",EC_UART_Baud);
}

static void HandleComm(uint8_t frameType, void * dataPtr, uint16_t dataLen)
{
	uint8_t * data = (uint8_t *)dataPtr;
	switch(frameType)
	{
		case ddm_kGetDataResp:	//数据返回
		{
			uint32_t pntr = 0;
			vec3f euler;
			for(int i=0;i<3;++i){
				int sig = (data[pntr]&0xf0)==0?1:-1;	//符号位
				int hun = data[pntr++]&0x0f;
				int unit = (data[pntr]>>4)*10 + (data[pntr]&0x0f);	//括号不能少
				pntr++;
				float dec = (data[pntr]>>4)*10 + (data[pntr]&0x0f);
				pntr++;
				euler[i] = sig*(hun*100+unit+dec/100.0f);
			}			
			memcpy(ddm.euler,euler,4*3);
			ddm.roll_sensor = (int)(euler[0]*100);
			ddm.pitch_sensor = (int)(euler[1]*100);
			ddm.yaw_sensor = (int)(euler[2]*100);
			vp.ec_parse_num_latest++;
//			printf("DDM:y=%.2f p=%.2f r=%.2f \r\n", heading, pitch, roll);
			break;
		}
		default:
		{
			//printf("Unknown frame %02X received\r\n",(uint16_t)frameType);
			break;
		}
	}
}

u8 DDM_buf[100] = "";
void ec_ddm_update(){
	static u8 recv_none = 0;
	if(EC_uart_available==0){
		if(recv_none++==10){	//连续10次没有接收到数据
			sys_flag.health.ec = false;
		}
		return ;
	}
	recv_none = 0;
	sys_flag.ec_recv=false;
	
	static int len_DDM_buf = 0;
	u8 RxBuffer[50]={0};
	u8 _data_cnt = 0;
	u8 state = 0;
	u16 buf_bytes=EC_uart_available;
	char recv_buf[buf_bytes];
	memcpy(recv_buf,EC_uart_buf,buf_bytes);
	EC_uart_available = 0;

	memcpy(DDM_buf+len_DDM_buf,recv_buf,buf_bytes);	//把新接收到的buf追加到还未处理完的buf结尾,注意不能用strncat,strcpy等字符串函数，因为数组中含0x00结束符 -> 断包重组
	len_DDM_buf += buf_bytes;	//更新长度
//	printf("length=%d \r\n",len_DDM_buf);
//	for(int i=0;i<len_DDM_buf;i++) printf("%02x ",DDM_buf[i]);printf("\r\n");
//	for(int i=0;i<buf_bytes;i++) printf("%02x ",recv_buf[i]);printf("\r\n");


//	return;
	u16 i = 0;
	while(i < len_DDM_buf){	
		u8 data=DDM_buf[i++];

		if(state==0&&data==0x68){
			RxBuffer[0]=data;
			state=1;
		}
		else if(state==1&&data==0x0d){
			if((len_DDM_buf-i+1)<0x0d){	//断包直接退出
				char temp_buf[100] = "";
				memcpy(temp_buf,DDM_buf+i-2,len_DDM_buf-i+2);
				memset(DDM_buf,0,sizeof(DDM_buf));
				memcpy(DDM_buf,temp_buf,100);
				len_DDM_buf = len_DDM_buf-i+2;	//减少长度
//				printf(" -> broke ");
				state=0;
				break;
			} 
			RxBuffer[1]=data;
			state=2;
		}
		else if(state==2&&data==0X00){	//地址码
			RxBuffer[2]=data;
			state=3;
		}
		else if(state==3&&data==0X84){	//命令字
			RxBuffer[3]=data;
			state=4;
			mExpectedLen = 0x0d;
			_data_cnt = 0;
		}
		else if(state==4){
			RxBuffer[4+_data_cnt++]=data;
			if(_data_cnt == mExpectedLen-3){	//1帧数据接收完
				state = 0;
				uint8_t crc = DDM_CRC(RxBuffer+1, mExpectedLen-1);	//计算CRC，
				uint8_t crcReceived = RxBuffer[mExpectedLen] ;	//接收到的CRC
				if(crc != crcReceived){ //对比CRC
//					printf("crc error \r\n");
					char temp_buf[100] = "";
					memcpy(temp_buf,DDM_buf+i,len_DDM_buf-i);
					memset(DDM_buf,0,sizeof(DDM_buf));
					memcpy(DDM_buf,temp_buf,100);
					len_DDM_buf -= i;	//减少长度
					i= 0;	//重头开始解析
//					break;		//校验位不对则退出
				}
				else{
					HandleComm(RxBuffer[3], RxBuffer+4, mExpectedLen - ddm_kPacketMinSize);
					char temp_buf[100] = "";
					memcpy(temp_buf,DDM_buf+i,len_DDM_buf-i);
					memset(DDM_buf,0,sizeof(DDM_buf));
					memcpy(DDM_buf,temp_buf,100);
					len_DDM_buf -= i;	//减少长度
					i= 0;	//重头开始解析
//					printf(" -> handle \r\n");
					recv_none = 0;
					sys_flag.health.ec = true;	
				}

			} 
		}
		else
			state = 0;
	}
}

void ec_ddm_check(){
	if(!sys_flag.health.ec){
		ec_ddm_continuous_mode();
	}
}

//设置磁偏角
void ec_ddm_setMagDec(float incline){
	
}

void ddm_get_euler(vec3f euler){
	arm_copy_f32(ddm.euler, euler, 3);
}


