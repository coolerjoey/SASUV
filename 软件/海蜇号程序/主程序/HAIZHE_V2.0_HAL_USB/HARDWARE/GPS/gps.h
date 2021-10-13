#ifndef _GPS_H
#define _GPS_H

#include "sys.h"
#include "uart.h"
#include "config.h"	

//GPS NMEA-0183Э����Ҫ�����ṹ�嶨�� 
#define MAX_SAT_NUM 12  //������Ǹ���
//������Ϣ
typedef struct  
{										    
 	u8 num;		//���Ǳ��
	u8 eledeg;	//��������
	u16 azideg;	//���Ƿ�λ��
	u8 sn;		//�����		   
}nmea_slmsg;  
//UTCʱ����Ϣ
typedef struct  
{										    
 	u16 year;	//���
	u8 month;	//�·�
	u8 day;	//����
	u8 hour; 	//Сʱ
	u8 min; 	//����
	u8 sec; 	//����
	u8 ms;		//����
}nmea_utc_time;   	   
//NMEA 0183 Э����������ݴ�Žṹ��
typedef struct  
{		
	//GGA��Ϣ
 	nmea_utc_time gga_utc;			//UTCʱ��,��ʽΪ hhmmss.ss
	double gga_latitude;	//γ�ȣ���ʽΪ ddmm.mmmmm���ȷָ�ʽ�� 		
	char gga_nshemi;					//��γ/��γ,N:��γ;S:��γ				  
	double gga_longitude;	   //���ȣ���ʽΪ ddmm.mmmmm���ȷָ�ʽ�� 			
	char gga_ewhemi;					//����/����,E:����;W:����
	u8 gga_gpssta;					//GPS״̬:0,δ��λ;1,�ǲ�ֶ�λ;2,��ֶ�λ;6,���ڹ���.				  
 	u8 gga_posslnum;				//���ڶ�λ��������,0~12.
	float gga_hdop;					//ˮƽ�������ӣ�0.5~99.9��						 			
	float gga_altitude;			 	//���θ߶ȣ�-9999.9 �� 9999.9 �ף�,�����M��ʾ��λ�ס�																		
	float gga_h;					//�����������Ժ�ƽ��ĸ߶�,�����M��ʾ��λ�ס�	
	float gga_dt;				//���ʱ�䣨�����һ�ν��յ�����źſ�ʼ���������ǲ�ֶ�λ������Ϊ�գ�Difference time
	u16 gga_drbsl;			//    ��ֲο���վ��ţ�0000 �� 1023����λ 0 Ҳ�����ͣ��ǲ�ֶ�λ������Ϊ��) Differential reference base station label
	
	//GSA��Ϣ��������ÿһ����ʾһ�����ǵĶ�λ״̬��
	char gsa_mode;	
	char gsa_fixmode;					//��λ����:1,û�ж�λ;2,2D��λ;3,3D��λ	
	char gsa_possl[MAX_SAT_NUM];				//���ڶ�λ�����Ǳ��
	float gsa_pdop[MAX_SAT_NUM];					//�ۺ�λ�þ������ӣ�0.5-99.9��
	float gsa_hdop[MAX_SAT_NUM];					//ˮƽ�������ӣ�0.5~99.9��		
	float gsa_vdop[MAX_SAT_NUM];					//��ֱ�������ӣ�0.5~99.9��	
	
	//GSV��Ϣ(������ÿ�� GSV ����������Ŀ����ǵ���Ϣ���������ǵ���Ϣ������һ��$GPGSV ����������)
	char gsv_msg_num;//GSV ���������
	char gsv_msg_serial;//���� GSV �ı�š�
	char gsv_sta_num;					//�ɼ����ǵ�������00~12��ǰ��� 0 Ҳ�������䣩��
	char gsv_sta_serial;//���Ǳ�ţ�01~32��ǰ��� 0 Ҳ�������䣩��
	char gsv_elevation; //�������ǣ�00~90 �ȣ�ǰ��� 0 Ҳ�������䣩��  
	u16 gsv_azimuth;//���Ƿ�λ�ǣ�000~359 �ȣ�ǰ��� 0 Ҳ�������䣩
	char gsv_SNR;//����ȣ�00~99dB��û�и��ٵ�����ʱΪ�գ���

	//RMC��Ϣ
	nmea_utc_time rmc_utc;//UTC ʱ�䣬hhmmss��ʱ���룩
	char rmc_pos_status;//��λ״̬��A=��Ч��λ��V=��Ч��λ
	double rmc_latitude;//γ�� ddmm.mmmmm���ȷ֣�
	char rmc_nshemi;//γ�Ȱ��� N�������򣩻� S���ϰ���
	double rmc_longitude;//���� dddmm.mmmmm���ȷ֣�
	char rmc_ewhemi;//���Ȱ��� E���������� W��������
	float rmc_speed;//�������ʣ�000.0~999.9 �ڣ�1��=1����/Сʱ=1.852����/Сʱ=0.514444m/s
	float rmc_orient;//���溽��000.0~359.9 �ȣ����汱��Ϊ�ο���׼��
	float rmc_magdec;//��ƫ�ǣ�000.0~180.0 �ȣ�ǰ��λ�������� 0��
	char rmc_magdir;	//��ƫ�Ƿ���E�������� W������
	char rmc_mode;//ģʽָʾ��A=������λ��D=��֣�E=���㣬N=������Ч��

	//VTG��Ϣ
	
	u16 vtg_geo_orient;//���汱Ϊ�ο���׼�ĵ��溽��000~359 �ȣ�ǰ��� 0 Ҳ�������䣩
	u16 vtg_mag_orient;//�Դű�Ϊ�ο���׼�ĵ��溽��(000~359 �ȣ�ǰ��� 0 Ҳ��������)
	float vtg_vel_knot;//��������(000.0~999.9 �ڣ�ǰ��� 0 Ҳ��������   
	float vtg_vel_kilo;//��������(0000.0~1851.8 ����/Сʱ��ǰ��� 0 Ҳ��������)
	char vtg_mode;//ģʽָʾ��A=������λ��D=��֣�E=���㣬N=������Ч��

	//GLL��Ϣ
	char gll_latitude[10];//γ�� ddmm.mmmmm���ȷ֣�
	char gll_nshemi;//γ�Ȱ��� N�������򣩻� S���ϰ���
	char gll_longitude[10];//���� dddmm.mmmmm���ȷ֣�
	char gll_ewhemi;//���Ȱ��� E���������� W��������
	char gll_utc[6];//UTC ʱ�䣺hhmmss��ʱ���룩
	char gll_pos_status;//��λ״̬��A=��Ч��λ��V=��Ч��λ
	char gll_mode;//ģʽָʾ��A=������λ��D=��֣�E=���㣬N=������Ч��       
	
	//ZDA��Ϣ
	char zda_utc[6];//UTC ʱ�䣺hhmmss��ʱ���룩
	char zda_day;//��
	char zda_mon;//��
	u16 zda_year;//��
	char zda_local_hour;//��������Сʱ��NEO-6M δ�õ���Ϊ 00��
	char zda_local_min;//����������ӣ�NEO-6M δ�õ���Ϊ 00��

	
	nmea_slmsg slmsg[MAX_SAT_NUM];		//���12������

}nmea_msg; 


extern nmea_msg gps;

	
void gps_init(void);
void gps_read(void);

#endif
