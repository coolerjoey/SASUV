#include "IMU_JY901.h"
#include "config.h"
#include "string.h"
#include "malloc.h"
#include "delay.h"
#include "GCS_Mavlink.h"

struct STime		stcTime;
struct SAcc 		stcAcc;
struct SGyro		stcGyro;
struct SAngle	stcAngle;
struct SMag 		stcMag;
struct SDStatus stcDStatus;
struct SPress	stcPress;
struct SLonLat	stcLonLat;
struct SGPSV		stcGPSV;
struct SQ		stcQ;

float D0_AIN=0;
IMU_JY901 jy901;

void imu_JY901_init(){
	IMU_UART_Init(IMU_UART,IMU_UART_Baud);
//	char sleep_mode[]={0xff,0xaa,0x22,0x01,0x00};
//	JY901_UART_write(sleep_mode,sizeof(sleep_mode)/sizeof(char));
	printf("[OK] JY901 init baud %d \r\n",IMU_UART_Baud);
}

//解锁指令-对模块进行校准时需要先发送解锁指令
void imu_JY901_unlock(){
	char unlock[]={0xff,0xaa,0x69,0x88,0xb5};
	IMU_UART_write(unlock,sizeof(unlock));
	delay_ms(100);
}
//TCM保存当前配置
void imu_JY901_save_config(){
	char save_config[]={0xff,0xaa,0x00,0x00,0x00};
	IMU_UART_write(save_config,sizeof(save_config));
}
//TCM退出校准 -> 需要在校准后再发送，然后保存
void imu_JY901_quit_calibration(){
	char quit_calibration[]={0xff,0xaa,0x01,0x00,0x00};
	IMU_UART_write(quit_calibration,sizeof(quit_calibration));
	delay_ms(100);
}
//加计校准
void imu_JY901_ACC_calibration(){
	printf("[sensor] IMU acc calibration, wait... \r\n");
	imu_JY901_unlock();
	char ACC_calibration[]={0xff,0xaa,0x01,0x01,0x00};
	IMU_UART_write(ACC_calibration,sizeof(ACC_calibration));
	delay_ms(1000);
	delay_ms(1000);
	delay_ms(1000);//注意需要等待2S以上
	imu_JY901_quit_calibration();
	imu_JY901_save_config();
	gcs_send_text(MAV_SEVERITY_INFO, "[sensor] IMU acc calibration complete");
}

//设置回传速率
void imu_JY901_set_freq(int rate){
	imu_JY901_unlock();
	u8 _temp=0;
	switch(rate){
		case 1: _temp=0x03;break;
		case 2: _temp=0x04;break;
		case 5: _temp=0x05;break;
		case 10: _temp=0x06;break;
		case 20: _temp=0x07;break;
		case 50: _temp=0x08;break;
		case 100: _temp=0x09;break;
		case 200: _temp=0x0b;break;
	}
	printf("[imu] set freq to %dHZ \r\n",_temp);
	char freq[]={0xff,0xaa,0x03,_temp,0x00};
	IMU_UART_write(freq,sizeof(freq));
	delay_ms(100);
	imu_JY901_save_config();
}
//设置波特率
void imu_JY901_set_baud(int rate){
	imu_JY901_unlock();
	u8 _temp=0;
	switch(rate){
		case 2400: _temp=0x00;break;
		case 4800: _temp=0x01;break;
		case 9600: _temp=0x02;break;
		case 19200: _temp=0x03;break;
		case 38400: _temp=0x04;break;
		case 57600: _temp=0x05;break;
		case 115200: _temp=0x06;break;
		case 230400: _temp=0x07;break;
		case 460800: _temp=0x08;break;
		case 921600: _temp=0x09;break;
	}
	printf("[imu] set baud to %d \r\n",_temp);
	char baud[]={0xff,0xaa,0x04,_temp,0x00};
	IMU_UART_write(baud,sizeof(baud));
	delay_ms(100);
	imu_JY901_save_config();
}


void imu_JY901_read(){
	static u8 recv_none = 0;
	if(IMU_uart_available==0){
		if(recv_none++==10){	//连续10次没有接收到数据
			sys_flag.health.imu = false;
		}
		return ;
	}
	recv_none = 0;
	sys_flag.imu_recv = false;
	
	static u8 ucRxBuffer[80];
	static u8 ucRxCnt = 0;
	u16 buf_bytes=IMU_uart_available;
	u8 *recv_buf = mymalloc(SRAMIN,buf_bytes);
	memcpy(recv_buf,IMU_uart_buf,buf_bytes);
	IMU_uart_available = 0;
	u16 i = 0;
	while(i < buf_bytes){
		u8 data = *(recv_buf+i);
		i++;
		ucRxBuffer[ucRxCnt++]=data;	//将收到的数据存入缓冲区中
		if (ucRxBuffer[0]!=0x55) //数据头不对，则重新开始寻找0x55数据头
		{
			ucRxCnt=0;
		}
		else if (ucRxCnt == 11) {
//			for(int j=0;j<11;j++)
//				printf("%02x ",ucRxBuffer[j]);
//			printf("\r\n");
			sys_flag.health.imu = true;
			vp.imu_parse_num_latest++;
			switch(ucRxBuffer[1])//判断数据是哪种数据，然后将其拷贝到对应的结构体中，有些数据包需要通过上位机打开对应的输出后，才能接收到这个数据包的数据
			{
				case 0x50:	memcpy(&stcTime,&ucRxBuffer[2],8);break;//memcpy为编译器自带的内存拷贝函数，需引用"string.h"，将接收缓冲区的字符拷贝到数据结构体里面，从而实现数据的解析。
				case 0x51:		//加速度
					memcpy(&stcAcc,&ucRxBuffer[2],8);
					for(int i=0;i<3;++i) jy901.acc[i] = (float)stcAcc.a[i]/32768*16*GRAVITY_MSS;
//					printf("%.3f %.3f %.3f \r\n",jy901.acc[0],jy901.acc[1],jy901.acc[2]);
					break;
				case 0x52:		//角速度
					memcpy(&stcGyro,&ucRxBuffer[2],8);
					for(int i=0;i<3;++i) jy901.gyro[i] = (float)stcGyro.w[i]/32768*2000;
					if(jy901.gyro[2]>GYRO_Z_LIMIT) jy901.gyro[2] = 0;
					else if(jy901.gyro[2]<-GYRO_Z_LIMIT) jy901.gyro[2] = 0;
					break;
				case 0x53:		//角度
					memcpy(&stcAngle,&ucRxBuffer[2],8);
					for(int i=0;i<3;++i) jy901.euler[i] = (float)stcAngle.Angle[i]/32768*180;
					break;
				case 0x54:		//磁场
					memcpy(&stcMag,&ucRxBuffer[2],8);
					for(int i=0;i<3;++i) jy901.mag[i] = stcMag.h[i];
					break;
				case 0x55:		//端口状态
					memcpy(&stcDStatus,&ucRxBuffer[2],8);
					D0_AIN = (float)stcDStatus.sDStatus[0]*3.3f/4096;
//					printf("D0_AIN=%.2f \r\n",D0_AIN);
					break;
				case 0x56:		//气压、高度
					memcpy(&stcPress,&ucRxBuffer[2],8);break;
				case 0x57:		//经纬度
					memcpy(&stcLonLat,&ucRxBuffer[2],8);break;
				case 0x58:		//地速
					memcpy(&stcGPSV,&ucRxBuffer[2],8);break;
				case 0x59:		//四元素
					memcpy(&stcQ,&ucRxBuffer[2],8);
//					JY_Q.q1 = (float)stcQ.q[0]/32768;
//					JY_Q.q2 = (float)stcQ.q[1]/32768;
//					JY_Q.q3 = (float)stcQ.q[2]/32768;
//					JY_Q.q4 = (float)stcQ.q[3]/32768;
					break;
			}
			ucRxCnt=0;//清空缓存区
		}
	}
	myfree(SRAMIN,recv_buf);
}

void JY901_get_euler(vec3f euler){
	arm_copy_f32(jy901.euler, euler, 3);
}
void JY901_get_acc(vec3f acc){
	arm_copy_f32(jy901.acc, acc, 3);
}
void JY901_get_gyro(vec3f gyro){
	arm_copy_f32(jy901.gyro, gyro, 3);
}
void JY901_get_mag(vec3f mag){
	arm_copy_f32(jy901.mag, mag, 3);
}
void JY901_get_temp(float *temp){
	*temp = jy901.temp;
}

void imu_JY901_check(){
	if(!sys_flag.health.imu){
//		imu_tcm_continuous_mode();
	}
}

void JY901_JY901_print(void){

	char str[100];
	//输出时间
//	sprintf(str,"Time:20%d-%d-%d %d:%d:%.3f\r\n",stcTime.ucYear,stcTime.ucMonth,stcTime.ucDay,stcTime.ucHour,stcTime.ucMinute,(float)stcTime.ucSecond+(float)stcTime.usMiliSecond/1000);
//	printf(str);		
//	
	//输出加速度
	sprintf(str,"Acc:%.3f %.3f %.3f\r\n",jy901.acc[0],jy901.acc[1],jy901.acc[2]);
	printf("%s",str);
	
	//输出角速度
	sprintf(str,"Gyro:%.3f %.3f %.3f\r\n",jy901.gyro[0],jy901.gyro[1],jy901.gyro[2]);
	printf("%s",str);
	
	//输出角度
	sprintf(str,"Angle:%.3f %.3f %.3f\r\n",jy901.euler[0],jy901.euler[1],jy901.euler[2]);
	printf("%s",str);
//	
//	//输出磁场
//	sprintf(str,"Mag:%d %d %d\r\n",_mag.x,_mag.y,_mag.z);
//	printf(str);		
//	
//	//输出气压、高度
//	sprintf(str,"Pressure:%ld Height%.2f\r\n",stcPress.lPressure,(float)stcPress.lAltitude/100);
//	printf(str); 
//	
//	//输出端口状态
//	sprintf(str,"DStatus:%d %d %d %d\r\n",stcDStatus.sDStatus[0],stcDStatus.sDStatus[1],stcDStatus.sDStatus[2],stcDStatus.sDStatus[3]);
//	printf(str);
//	
//	//输出经纬度
//	sprintf(str,"Longitude:%ldDeg%.5fm Lattitude:%ldDeg%.5fm\r\n",stcLonLat.lLon/10000000,(double)(stcLonLat.lLon % 10000000)/1e5,stcLonLat.lLat/10000000,(double)(stcLonLat.lLat % 10000000)/1e5);
//	printf(str);
//	
//	//输出地速
//	sprintf(str,"GPSHeight:%.1fm GPSYaw:%.1fDeg GPSV:%.3fkm/h\r\n",(float)stcGPSV.sGPSHeight/10,(float)stcGPSV.sGPSYaw/10,(float)stcGPSV.lGPSVelocity/1000);
//	printf(str);
//	
//	//输出四元素
//	sprintf(str,"Four elements:%.5f %.5f %.5f %.5f\r\n\r\n",JY_Q.q1,JY_Q.q2,JY_Q.q3,JY_Q.q4);
//	printf(str);
}
