#include "gps.h"
#include "string.h"
#include "telem.h"
#include "parameter.h"

/*
GPSЭ�飺NMEA-0183
ǰ׺����GPS-GP,������-BD,��GLONASS-GL,����-GN
���	����			˵��					���֡��		ȫ��
1		$GNGGA	GPS/������λ��Ϣ			72			Global Positioning System Fix Data
2		$GNGSA	��ǰ������Ϣ				65			GPS DOP and Active Satellites
3		$GPGSV	�ɼ� GPS ������Ϣ			210			GPS Satellites in View
4		$BDGSV	�ɼ�����������Ϣ			210			GPS Satellites in View
5		$GNRMC	�Ƽ���λ��Ϣ				70			Recommended Minimum Specific GPS/Transit Data
6		$GNVTG	�����ٶ���Ϣ				34			Track Made Good and Ground Speed
7		$GNGLL	���������Ϣ				--			Geographic Position
8		$GNZDA	��ǰʱ��(UTC1)��Ϣ		--
ע�⣺
��̬��ȡʱRMC����Ϣ���ᴫ�亽��Ƕȣ���Ϊ�ٶ�Ϊ��ʱ�޷�����
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
				gps_print("��Ч��λ \r\n");
				break;
			case 'V':
				gps_print("��Ч��λ \r\n");
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
		if(recv_none++==30){	//����30�Σ�3s��û�н��յ�����
			sys_flag.health.gps = false;
			memset(&gps,0,sizeof(gps)); //gps�������
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
		ucRxBuffer[ucRxCnt++]=data;	//���յ������ݴ��뻺������
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
			sys_flag.gps_recv=true;//��ǽ��յ��µ�GPS����
			memset(ucRxBuffer,0,sizeof(ucRxBuffer)/sizeof(ucRxBuffer[0]));
			ucRxCnt = 0;
		}
	}

}

void gps_init(){
	GPS_UART_Init(GPS_UART,GPS_UART_Baud);
	gps_print("[OK] GPS init baud %d \r\n",GPS_UART_Baud);
}
	

