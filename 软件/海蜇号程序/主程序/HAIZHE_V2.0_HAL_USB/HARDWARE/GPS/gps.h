#ifndef _GPS_H
#define _GPS_H

#include "sys.h"
#include "uart.h"
#include "config.h"	

//GPS NMEA-0183协议重要参数结构体定义 
#define MAX_SAT_NUM 12  //最大卫星个数
//卫星信息
typedef struct  
{										    
 	u8 num;		//卫星编号
	u8 eledeg;	//卫星仰角
	u16 azideg;	//卫星方位角
	u8 sn;		//信噪比		   
}nmea_slmsg;  
//UTC时间信息
typedef struct  
{										    
 	u16 year;	//年份
	u8 month;	//月份
	u8 day;	//日期
	u8 hour; 	//小时
	u8 min; 	//分钟
	u8 sec; 	//秒钟
	u8 ms;		//毫秒
}nmea_utc_time;   	   
//NMEA 0183 协议解析后数据存放结构体
typedef struct  
{		
	//GGA信息
 	nmea_utc_time gga_utc;			//UTC时间,格式为 hhmmss.ss
	double gga_latitude;	//纬度，格式为 ddmm.mmmmm（度分格式） 		
	char gga_nshemi;					//北纬/南纬,N:北纬;S:南纬				  
	double gga_longitude;	   //经度，格式为 ddmm.mmmmm（度分格式） 			
	char gga_ewhemi;					//东经/西经,E:东经;W:西经
	u8 gga_gpssta;					//GPS状态:0,未定位;1,非差分定位;2,差分定位;6,正在估算.				  
 	u8 gga_posslnum;				//用于定位的卫星数,0~12.
	float gga_hdop;					//水平精度因子（0.5~99.9）						 			
	float gga_altitude;			 	//海拔高度（-9999.9 到 9999.9 米）,后面的M表示单位米。																		
	float gga_h;					//大地椭球面相对海平面的高度,后面的M表示单位米。	
	float gga_dt;				//差分时间（从最近一次接收到差分信号开始的秒数，非差分定位，此项为空）Difference time
	u16 gga_drbsl;			//    差分参考基站标号（0000 到 1023，首位 0 也将传送，非差分定位，此项为空) Differential reference base station label
	
	//GSA信息（多条，每一条显示一个卫星的定位状态）
	char gsa_mode;	
	char gsa_fixmode;					//定位类型:1,没有定位;2,2D定位;3,3D定位	
	char gsa_possl[MAX_SAT_NUM];				//用于定位的卫星编号
	float gsa_pdop[MAX_SAT_NUM];					//综合位置精度因子（0.5-99.9）
	float gsa_hdop[MAX_SAT_NUM];					//水平精度因子（0.5~99.9）		
	float gsa_vdop[MAX_SAT_NUM];					//垂直精度因子（0.5~99.9）	
	
	//GSV信息(多条，每条 GSV 语句最多包括四颗卫星的信息，其他卫星的信息将在下一条$GPGSV 语句中输出。)
	char gsv_msg_num;//GSV 语句总数。
	char gsv_msg_serial;//本句 GSV 的编号。
	char gsv_sta_num;					//可见卫星的总数（00~12，前面的 0 也将被传输）。
	char gsv_sta_serial;//卫星编号（01~32，前面的 0 也将被传输）。
	char gsv_elevation; //卫星仰角（00~90 度，前面的 0 也将被传输）。  
	u16 gsv_azimuth;//卫星方位角（000~359 度，前面的 0 也将被传输）
	char gsv_SNR;//信噪比（00~99dB，没有跟踪到卫星时为空）。

	//RMC信息
	nmea_utc_time rmc_utc;//UTC 时间，hhmmss（时分秒）
	char rmc_pos_status;//定位状态，A=有效定位，V=无效定位
	double rmc_latitude;//纬度 ddmm.mmmmm（度分）
	char rmc_nshemi;//纬度半球 N（北半球）或 S（南半球）
	double rmc_longitude;//经度 dddmm.mmmmm（度分）
	char rmc_ewhemi;//经度半球 E（东经）或 W（西经）
	float rmc_speed;//地面速率（000.0~999.9 节）1节=1海里/小时=1.852公里/小时=0.514444m/s
	float rmc_orient;//地面航向（000.0~359.9 度，以真北方为参考基准）
	float rmc_magdec;//磁偏角（000.0~180.0 度，前导位数不足则补 0）
	char rmc_magdir;	//磁偏角方向，E（东）或 W（西）
	char rmc_mode;//模式指示（A=自主定位，D=差分，E=估算，N=数据无效）

	//VTG信息
	
	u16 vtg_geo_orient;//以真北为参考基准的地面航向（000~359 度，前面的 0 也将被传输）
	u16 vtg_mag_orient;//以磁北为参考基准的地面航向(000~359 度，前面的 0 也将被传输)
	float vtg_vel_knot;//地面速率(000.0~999.9 节，前面的 0 也将被传输   
	float vtg_vel_kilo;//地面速率(0000.0~1851.8 公里/小时，前面的 0 也将被传输)
	char vtg_mode;//模式指示（A=自主定位，D=差分，E=估算，N=数据无效）

	//GLL信息
	char gll_latitude[10];//纬度 ddmm.mmmmm（度分）
	char gll_nshemi;//纬度半球 N（北半球）或 S（南半球）
	char gll_longitude[10];//经度 dddmm.mmmmm（度分）
	char gll_ewhemi;//经度半球 E（东经）或 W（西经）
	char gll_utc[6];//UTC 时间：hhmmss（时分秒）
	char gll_pos_status;//定位状态，A=有效定位，V=无效定位
	char gll_mode;//模式指示（A=自主定位，D=差分，E=估算，N=数据无效）       
	
	//ZDA信息
	char zda_utc[6];//UTC 时间：hhmmss（时分秒）
	char zda_day;//日
	char zda_mon;//月
	u16 zda_year;//年
	char zda_local_hour;//本地区域小时（NEO-6M 未用到，为 00）
	char zda_local_min;//本地区域分钟（NEO-6M 未用到，为 00）

	
	nmea_slmsg slmsg[MAX_SAT_NUM];		//最多12颗卫星

}nmea_msg; 


extern nmea_msg gps;

	
void gps_init(void);
void gps_read(void);

#endif
