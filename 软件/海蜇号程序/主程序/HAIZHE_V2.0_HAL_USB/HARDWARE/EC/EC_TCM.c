#include "EC_TCM.h"
#include "delay.h"
#include "string.h"
#include "config.h"
#include "malloc.h"
/*
ͨ��Э�飺
ByteCount(UInt16) | Packet Frame(1-4092 UInt8) | CRC-16(UInt16)
		 		  /Frame ID    |    Payload    \
		 		  |(UInt8)	   | (1-4091 UInt8)|
CRC���߼��㣺http://www.ip33.com/crc.html -> CRC���� CRC-16/XMODEM
���ûش��Ƕȡ��¶ȡ����ٶȣ�00 0D 03 07 05 07 15 16 17 18 19 07 83
�Զ��������: 00 0F 18 00 00 00 00 00 00 00 00 00 00 E4 50
����������ã�00 05 09 6e dc
ģ��ϵ磺00 05 0f 0e 1a
���λ�ȡ����ָ�00 05 04 bf 71
�����������䣺00 05 15 bd 61	
	�ش�������Ӧ���ݸ�ʽ(Payload)��
	ID Count |  ID1    |  Value ID1    |   ID2   |   Value ID2   |   ID3   |  Value ID3 ����
	 (UInt8) | (UInt8) | (ID Specific) | (UInt8) | (ID Specific) | (UInt8) | (ID Specific)����
*/

EC_TCM tcm;

//static uint32_t last_reading_ms = 0;
//static char linebuf[10];
//static uint8_t linebuf_len = 0;
//static uint8_t mOutData[tcm_kBufferSize], mInData[tcm_kBufferSize];
static const uint8_t kDataCount = 7;
static uint16_t mExpectedLen;

uint16_t TCM_CRC(void * data, uint32_t len)
{
	uint8_t * dataPtr = (uint8_t *)data;
	uint32_t index = 0;
	uint16_t crc = 0;
	while(len--)
	{
		crc = (unsigned char)(crc >> 8) | (crc << 8);
		crc ^= dataPtr[index++];
		crc ^= (unsigned char)(crc & 0xff) >> 4;
		crc ^= (crc << 8) << 4;
		crc ^= ((crc & 0xff) << 4) << 1;
	}
	return crc;
}

void ec_tcm_trigger(){
	char get[5]={0x00,0x05,0x04,0xbf,0x71};
	EC_UART_write(get,5);	//��ȡָ��	
}
void ec_tcm_continuous_mode(){
	char buf[5]={0x00,0x05,0x15,0xBD,0x61};
	EC_UART_write(buf,5); //������������
}
void ec_tcm_init(){
	EC_UART_Init(EC_UART,EC_UART_Baud);
//	ec_tcm_trigger();
	ec_tcm_continuous_mode();
	delay_ms(50);

	printf("[OK] TCM init baud %d \r\n",EC_UART_Baud);
}

static void HandleComm(uint8_t frameType, void * dataPtr, uint16_t dataLen)
{
	uint8_t * data = (uint8_t *)dataPtr;
	switch(frameType)
	{
		case tcm_kGetDataResp:	//���ݷ���
		{
			// Parse the data response
			uint8_t count = data[0];			// The number of data elements returned() ��ȡ��component����
			uint32_t pntr = 1;					// Used to retrieve the returned elements
			float heading, pitch, roll, temperature, accx, accy, accz;// The data elements we requested
			if(count != kDataCount)
			{
				printf("Received %u data elements instead of the %u requested\r\n", (uint16_t)count,(uint16_t)kDataCount);
				return;
			}
			// loop through and collect the elements
			while(count)
			{
				// The elements are received as {type (ie. kHeading), data}
				u8 type = data[pntr++];
				switch(type)
				// read the type and go to the first byte of the data
				{
					// Only handling the 4 elements we are looking for
					case tcm_kHeading:
					{
						int2float(&heading ,&(data[pntr]));
						// increase the pointer to point to the next data element type
						pntr += sizeof(heading);
						break;
					}
					case tcm_kPitch:
					{
						int2float(&pitch ,&(data[pntr]));
						// increase the pointer to point to the next data element type
						pntr += sizeof(pitch);
						break;
					}
					case tcm_kRoll:
					{
						int2float(&roll ,&(data[pntr]));
						// increase the pointer to point to the next data element type
						pntr += sizeof(roll);
						break;
					}
					case tcm_kTemperature:
					{
						int2float(&temperature ,&(data[pntr]));
						// increase the pointer to point to the next data element type
						pntr += sizeof(temperature);
						break;
					}
					case tcm_kAccelX:
					{
						int2float(&accx ,&(data[pntr]));
						pntr += sizeof(accx);
						break;
					}
					case tcm_kAccelY:
					{
						int2float(&accy ,&(data[pntr]));
						pntr += sizeof(accy);
						break;
					}
					case tcm_kAccelZ:
					{
						int2float(&accz ,&(data[pntr]));
						pntr += sizeof(accz);
						break;
					}
					default:
						// Message is a function that displays a formatted string
						// (similar to //printf)
						//printf("Unknown type: %02X\r\n",data[pntr - 1]);
						// unknown data type, so size is unknown, so skip everything
//						return;
						break;
				}
				count--;
				// One less element to read in
			}
			tcm.euler[0] = roll;
			tcm.euler[1] = pitch;
			tcm.euler[2] = heading;
			tcm.temp = temperature;
			tcm.roll_sensor = (int)roll*100;
			tcm.pitch_sensor = (int)pitch*100;
			tcm.yaw_sensor = (int)heading*100;
			tcm.acc[0] = accx*GRAVITY_MSS;	//TCM���ٶ������λ��G
			tcm.acc[1] = accy*GRAVITY_MSS;
			tcm.acc[2] = accz*GRAVITY_MSS;
			vp.ec_parse_num_latest++;
//			printf("TCM:y=%.2f p=%.2f r=%.2f \r\n", heading, pitch, roll);
			// send next data request
			break;
		}
		default:
		{
			//printf("Unknown frame %02X received\r\n",(uint16_t)frameType);
			break;
		}
	}
}

char TCM_buf[100] = "";
void ec_tcm_update(){
	static u8 recv_none = 0;
	if(EC_uart_available==0){
		if(recv_none++==10){	//����10��û�н��յ�����
			sys_flag.health.ec = false;
		}
		return ;
	}
	recv_none = 0;
	sys_flag.ec_recv=false;
	
	static int len_TCM_buf = 0;
	u8 RxBuffer[50];
	u8 _data_cnt = 0;
	u8 state = 0;
	u16 buf_bytes=EC_uart_available;
	char recv_buf[buf_bytes];
	memcpy(recv_buf,EC_uart_buf,buf_bytes);
	EC_uart_available = 0;

	memcpy(TCM_buf+len_TCM_buf,recv_buf,buf_bytes);	//���½��յ���buf׷�ӵ���δ�������buf��β,ע�ⲻ����strncat,strcpy���ַ�����������Ϊ�����к�0x00������ -> �ϰ�����
	len_TCM_buf += buf_bytes;	//���³���
//	printf("length=%d \r\n",len_TCM_buf);
//	for(int i=0;i<len_TCM_buf;i++) printf("%02x ",TCM_buf[i]);printf("\r\n");
//	for(int i=0;i<buf_bytes;i++) printf("%02x ",recv_buf[i]);printf("\r\n");


//	return;
	u16 i = 0;
	while(i < len_TCM_buf){	
		u8 data=TCM_buf[i++];

		if(state==0&&data==0x00){
			RxBuffer[0]=data;
			state=1;
		}
		else if(state==1&&data==0x29){
			if((len_TCM_buf-i+2)<0x29){	//�ϰ�ֱ���˳�
				char temp_buf[100] = "";
				memcpy(temp_buf,TCM_buf+i-2,len_TCM_buf-i+2);
				memset(TCM_buf,0,sizeof(TCM_buf));
				memcpy(TCM_buf,temp_buf,100);
				len_TCM_buf = len_TCM_buf-i+2;	//���ٳ���
//				printf(" -> broke ");
				state=0;
				break;
			} 
			RxBuffer[1]=data;
			state=2;
		}
		else if(state==2&&data==0X05){	//������
			RxBuffer[2]=data;
			state=3;
			mExpectedLen = 0x29;
			_data_cnt = 0;
		}
		else if(state==3){
			RxBuffer[3+_data_cnt++]=data;
			if(_data_cnt == mExpectedLen-3){
				state = 0;
				uint16_t crc = TCM_CRC(RxBuffer, mExpectedLen-2);	//����CRC
				uint16_t crcReceived = (RxBuffer[mExpectedLen-2] << 8) | RxBuffer[mExpectedLen-1] ;
				if(crc != crcReceived){ //�Ա�CRC
//					printf("crc error \r\n");
					char temp_buf[100] = "";
					memcpy(temp_buf,TCM_buf+i,len_TCM_buf-i);
					memset(TCM_buf,0,sizeof(TCM_buf));
					memcpy(TCM_buf,temp_buf,100);
					len_TCM_buf -= i;	//���ٳ���
					i= 0;	//��ͷ��ʼ����
//					break;		//У��λ�������˳�
				}
				else{
					HandleComm(RxBuffer[2], &RxBuffer[3], mExpectedLen - tcm_kPacketMinSize);
					char temp_buf[100] = "";
					memcpy(temp_buf,TCM_buf+i,len_TCM_buf-i);
					memset(TCM_buf,0,sizeof(TCM_buf));
					memcpy(TCM_buf,temp_buf,100);
					len_TCM_buf -= i;	//���ٳ���
					i= 0;	//��ͷ��ʼ����
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

//void ec_tcm_update(){
//	u8 RxBuffer[50];
//	u8 _data_cnt = 0;
//	u8 state = 0;
//	u16 buf_bytes=EC_uart_available;
//	static u8 recv_none = 0;
//	if(buf_bytes==0){
//		if(recv_none++==10){	//����10��û�н��յ�����
//			sys_flag.health.ec_tcm = false;
//		}
//		return ;
//	}
//	recv_none = 0;
//	u8 recv_buf[buf_bytes];
//	memcpy(recv_buf,TCM_uart_buf,buf_bytes);
//	EC_uart_available = 0;
//	memset(TCM_uart_buf,0,sizeof(TCM_uart_buf)/sizeof(char));	//Ϊ�˲�Ӱ����գ���ʱ��ս��ջ���
//	printf("\r\ntcm_buf = %d ",buf_bytes);
//	
//	//return;
//	u16 i = 0;
//	while(i<buf_bytes){
//		u8 data=recv_buf[i++];
//		printf("%02x ",data);
//		if(state==0&&data==0x00){
//			RxBuffer[0]=data;
//			state=1;
//		}
//		else if(state==1&&data==0x29){
//			if((buf_bytes-i+2)<0x29) printf(" -> broke ");
//			RxBuffer[1]=data;
//			state=2;
//		}
//		else if(state==2&&data==0X05){	//������
//			RxBuffer[2]=data;
//			state=3;
//			mExpectedLen = 0x29;
//			_data_cnt = 0;
//		}
//		else if(state==3){
//			RxBuffer[3+_data_cnt++]=data;
//			if(_data_cnt == mExpectedLen-3){
//				state = 0;
//				uint16_t crc = TCM_CRC(RxBuffer, mExpectedLen-2);	//����CRC
//				uint16_t crcReceived = (RxBuffer[mExpectedLen-2] << 8) | RxBuffer[mExpectedLen-1] ;
//				if(crc != crcReceived){ //�Ա�CRC
//	//				printf("crc error \r\n");
//					break;		//У��λ�������˳�
//				}
//				else{
//					HandleComm(RxBuffer[2], &RxBuffer[3], mExpectedLen - kPacketMinSize);
//					printf(" -> handle \r\n");
//					recv_none = 0;
//					sys_flag.health.ec_tcm = true;	
//				}

//			} 
//		}
//		else
//			state = 0;
//	}
//}

void ec_tcm_check(){
	if(!sys_flag.health.ec){
		ec_tcm_continuous_mode();
	}
}

void tcm_get_euler(vec3f euler){
	arm_copy_f32(tcm.euler, euler, 3);
}
void tcm_get_acc(vec3f acc){
	arm_copy_f32(tcm.acc, acc, 3);
}
void tcm_get_temp(float *temp){
	*temp = tcm.temp;
}

