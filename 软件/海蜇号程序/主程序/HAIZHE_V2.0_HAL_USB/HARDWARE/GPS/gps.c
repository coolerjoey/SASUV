#include "gps.h"
#include "string.h"
#include "telem.h"
#include "parameter.h"

/*
GPS协议：NMEA-0183
前缀：单GPS-GP,单北斗-BD,单GLONASS-GL,多星-GN
序号	命令			说明					最大帧长		全称
1		$GNGGA	GPS/北斗定位信息			72			Global Positioning System Fix Data
2		$GNGSA	当前卫星信息				65			GPS DOP and Active Satellites
3		$GPGSV	可见 GPS 卫星信息			210			GPS Satellites in View
4		$BDGSV	可见北斗卫星信息			210			GPS Satellites in View
5		$GNRMC	推荐定位信息				70			Recommended Minimum Specific GPS/Transit Data
6		$GNVTG	地面速度信息				34			Track Made Good and Ground Speed
7		$GNGLL	大地坐标信息				--			Geographic Position
8		$GNZDA	当前时间(UTC1)信息		--
注意：
静态提取时RMC等信息不会传输航向角度，因为速度为零时无法计算
*/

#define gps_print printf
bool gps_infoshow=false;

nmea_msg gps;

void NMEA_GGA_Parse(const char *buf){
	char unit;
	double time;
	sscanf(buf,"$GNGGA,%lf,%lf,%c,%lf,%c,%hhd,%hhd,%f,%f,%c,%f,%c,%f,%[^*]",\
			&time,\
			&gps.gga_latitude,\
			&gps.gga_nshemi,\
			&gps.gga_longitude,\
			&gps.gga_ewhemi,\
			&gps.gga_gpssta,\
			&gps.gga_posslnum,\
			&gps.gga_hdop,\
			&gps.gga_altitude,\
			&unit,\
			&gps.gga_h,\
			&unit,\
			&gps.gga_dt,\
			&gps.gga_drbsl
		);
	gps.gga_utc.hour = (int)time/10000+8;
	gps.gga_utc.min = (int)time%10000/100;
	gps.gga_utc.sec = (int)time%100;
	gps.gga_utc.ms = (int)(time*100)%100;
	
	if(sys_flag.show_gps_infor2){
		gps_print("\r\n/********************/\r\n");
		gps_print("GNGGA MSG %s \r\n",buf);
		gps_print("utc=%2d:%2d:%2d \r\n",gps.gga_utc.hour,gps.gga_utc.min,gps.gga_utc.sec);
		gps_print("gga_latitude=%f \r\n",gps.gga_latitude);
		gps_print("gga_nshemi=%c \r\n",gps.gga_nshemi);
		gps_print("gga_longitude=%f \r\n",gps.gga_longitude);
		gps_print("gga_ewhemi=%c \r\n",gps.gga_ewhemi);
		gps_print("gga_gpssta=%d \r\n",gps.gga_gpssta);
		gps_print("gga_posslnum=%d \r\n",gps.gga_posslnum);
		gps_print("gga_hdop=%f \r\n",gps.gga_hdop);
		gps_print("gga_altitude=%f \r\n",gps.gga_altitude);
		gps_print("unit=%c \r\n",unit);
		gps_print("gga_h=%f \r\n",gps.gga_h);
		gps_print("unit=%c \r\n",unit);
	}
	
}

void NMEA_GNZDA_Parse(const char *buf){
}
void NMEA_GNGLL_Parse(const char *buf){
}
void NMEA_GNVTG_Parse(const char *buf){
}
void NMEA_GNRMC_Parse(const char *buf){
	double time,date;
	sscanf(buf,"$GNRMC,%lf,%c,%lf,%c,%lf,%c,%f,%f,%lf,%f,%c,%[^*]",\
		&time,\
		&gps.rmc_pos_status,\
		&gps.rmc_latitude,\
		&gps.rmc_nshemi,\
		&gps.rmc_longitude,\
		&gps.rmc_ewhemi,\
		&gps.rmc_speed,\
		&gps.rmc_orient,\
		&date,\
		&gps.rmc_magdec,\
		&gps.rmc_magdir,\
		&gps.rmc_mode
	);
	gps.rmc_utc.hour = (int)time/10000+8;
	gps.rmc_utc.min = (int)time%10000/100;
	gps.rmc_utc.sec = (int)time%100;
	gps.rmc_utc.ms = (int)(time*100)%100;
	gps.rmc_utc.day=(int)date/10000;
	gps.rmc_utc.month=(int)date/100%100;
	gps.rmc_utc.year=(int)date%100+2000;

	if(sys_flag.show_gps_infor2){
		gps_print("\r\n/********************/\r\n");
		gps_print("GNRMC MSG %s \r\n",buf);
		switch(gps.rmc_pos_status){
			case 'A':
				gps_print("有效定位 \r\n");
				break;
			case 'V':
				gps_print("无效定位 \r\n");
				break;
		}
		gps_print("rmc_utc=%d-%d-%d-%d:%d:%d \r\n",gps.rmc_utc.year,gps.rmc_utc.month,gps.rmc_utc.day,gps.rmc_utc.hour,gps.rmc_utc.min,gps.rmc_utc.sec);
		gps_print("rmc_status=%c \r\n",gps.rmc_pos_status);
		gps_print("rmc_latitude=%lf \r\n",gps.rmc_latitude);
		gps_print("rmc_nshemi=%c \r\n",gps.rmc_nshemi);
		gps_print("rmc_longitude=%lf \r\n",gps.rmc_longitude);
		gps_print("rmc_ewhemi=%c \r\n",gps.rmc_ewhemi);
		gps_print("rmc_speed=%f \r\n",gps.rmc_speed);
		gps_print("rmc_orient=%f \r\n",gps.rmc_orient);
		gps_print("rmc_magdec=%f \r\n",gps.rmc_magdec);
		gps_print("rmc_magdir=%c \r\n",gps.rmc_magdir);
		gps_print("rmc_mode=%c \r\n",gps.rmc_mode);
	}

}
void NMEA_BDGSV_Parse(const char *buf){
}
void NMEA_GPGSV_Parse(const char *buf){
}
void NMEA_GLGSV_Parse(const char *buf){
}
void NMEA_GNGSA_Parse(const char *buf){
}

void gps_read(){
	static u8 recv_none = 0;
	if(GPS_UART_available==0){
		if(recv_none++==30){	//连续30次（3s）没有接收到数据
			sys_flag.health.gps = false;
			memset(&gps,0,sizeof(gps)); //gps数据清空
		}
		return ;
	}
	recv_none = 0;
	sys_flag.gps_recv=false;
	
	char header[5];
	static char ucRxBuffer[80];
	static u8 ucRxCnt = 0;
	u16 buf_bytes=GPS_UART_available;
	u8 recv_buf[buf_bytes];
	memcpy(recv_buf,GPS_UART_BUF,buf_bytes);
	GPS_UART_available = 0;
	u16 i = 0;
	while(i < buf_bytes){		
		u8 data=recv_buf[i++];
//		gps_print("%c",data);
		ucRxBuffer[ucRxCnt++]=data;	//将收到的数据存入缓冲区中
		if(ucRxBuffer[0]!='$'){
			ucRxCnt=0;
		}
		else if(data=='*' && ucRxCnt!=0){
			ucRxBuffer[ucRxCnt] = recv_buf[i++];
			if(sys_flag.show_gps_infor1) gps_print("%s \r\n",ucRxBuffer);
			sscanf((const char*)ucRxBuffer+1,"%5s",header);
			if(strcmp(header,"GNZDA")==0){
//				NMEA_GNZDA_Parse(ucRxBuffer);
			}
			else if(strcmp(header,"GNGLL")==0){
//				NMEA_GNGLL_Parse(ucRxBuffer);
			}
			else if(strcmp(header,"GNVTG")==0){
//				NMEA_GNVTG_Parse(ucRxBuffer);
			}
			else if(strcmp(header,"GNRMC")==0){
				NMEA_GNRMC_Parse(ucRxBuffer);
			}
			else if(strcmp(header,"BDGSV")==0){
//				NMEA_BDGSV_Parse(ucRxBuffer);
			}
			else if(strcmp(header,"GPGSV")==0){
//				NMEA_GPGSV_Parse(ucRxBuffer);
			}
			else if(strcmp(header,"GLGSV")==0){
//				NMEA_GLGSV_Parse(ucRxBuffer);
			}
			else if(strcmp(header,"GNGSA")==0){
//				NMEA_GNGSA_Parse(ucRxBuffer);
			}
			else if(strcmp(header,"GNGGA")==0){
				NMEA_GGA_Parse(ucRxBuffer);
			}
			vp.gps_parse_num_latest++;
			sys_flag.health.gps = true;
			sys_flag.gps_recv=true;//标记接收到新的GPS数据
			memset(ucRxBuffer,0,sizeof(ucRxBuffer)/sizeof(ucRxBuffer[0]));
			ucRxCnt = 0;
		}
	}

}

void gps_init(){
	GPS_UART_Init(GPS_UART,GPS_UART_Baud);
	gps_print("[OK] GPS init baud %d \r\n",GPS_UART_Baud);
}
	

