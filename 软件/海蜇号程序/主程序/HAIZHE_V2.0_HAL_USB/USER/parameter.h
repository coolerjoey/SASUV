#ifndef _PARAMETER_H
#define _PARAMETER_H

#include "sys.h"	 
#include "mymath.h"
#include "pid.h"
#include "control_mode.h"
#include "ppm.h"
#include "motors.h"
#include "flash.h"
#include "defines.h"
#include "joystick.h"
#include "wpnav.h"

#define Task_MAX 10 //最大任务数

/*时间结构体*/
typedef struct{
	u16 year;
	u8 month;
	u8 day;
	u8 hour;
	u8 minute;
	u8 second;
	u8 week;
}TIME_STRUCT;

//系统从上电后经过的时间
typedef struct{
	uint64_t ten_micro;//系统10us
	uint64_t one_mill;//系统ms
	u32 one_second;//系统s
	u32 one_minute;//系统min
	u32 loop_tick;	//系统循环时钟
}SYS_TIME;

//安全检测种类
typedef struct{
	bool imu;
	bool baro;
	bool batt;
	bool joystick;
	bool gps;
	bool sd;
	bool flash;
}HEALTH_CHEABLE;

/*系统固定参数——写入flash参数*/
 typedef struct{	
	
	u16 param_num;	//固定参数个数
	u16 sysid_my_gcs;	//海蜇号系统id，用于mavlink通信
	u8 frame_class;	//海蜇号机架模型
	float power_min;//最小电池电压
	u8 log_mode;	//在电机上锁时是否记录日志 true-记录 false-不记录
	bool ROV_mode_enable;	//是否启用ROV功能 - true:开启网络，关闭无线电台;false:关闭网络，开启无线电台

	HEALTH_CHEABLE health_chenable;	//安装检测使能
	
	u32 uartA_baud;	//MPU9250
	u32 uartB_baud;	//WiFi
	u32 uartC_baud;	//TELEM
	u32 uartD_baud;	//GPS
	u32 uartE_baud;	//SERIAL2
	u32 uartF_baud;	//DEBUG
	
	vec3f attitude_init;				//初始姿态 roll,pitch,yaw
	vec3f sins_db;		//加速度计零漂范围

	DR_MODE dr_mode;	//导航模式
	WP_PATH wp_path;	//航点路径
	float wp_radius_cm;		//到航点的距离，在此距离范围内认为到达改航点
	bool exit_mission_rtl;	//任务结束是否返回home点
	u8	gps_fb_interval;		//模型辅助导航下GPS矫正间隔时间
	float forward_auto;		//AUTO模式下前进油门

	float throttle_hover;	//水下悬停油门(0.0-1.0)
	float throttle_deadzone_swerve;	//转向油门死区
	float throttle_deadzone_stabilize;	//水平姿态稳定油门死区
	float motor_gain;	//电机增益
	float motor_maxGain;	//遥控模式下电机最大增益
	float motor_minGain;	//遥控模式下电机最小增益
	float motor_numGainSetting;	//遥控模式下电机增益档位
	float speed_gain;	//自稳、导航模式下电机增益
	float speed_maxGain;	//自稳、导航模式下电机最大增益
	float speed_minGain;	//自稳、导航模式下电机最小增益
	float speed_numGainSetting; //自稳、导航模式下电机增益档位
	Motor motor1_fac;	//四个电机各自由度比例
	Motor motor2_fac;
	Motor motor3_fac;
	Motor motor4_fac;
	Motor motor5_fac;
	Motor motor6_fac;

	REMOTE_CONTROL remoteCtrl;	//无线遥控器
	u8 btn_function[16];	////没按下shift键的按钮功能映射
	u8 btn_sfunction[16];	//按下swift键的按钮功能映射

	ESC_TYPE esc_type;	//电调种类

	//向上位机发送数据频率
	u8 stream_RAW_SENSORS_freq;				
	u8 stream_EXTENDED_STATUS_freq;
	u8 stream_RC_CHANNELS_freq;
	u8 stream_RAW_CONTROLLER_freq;
	u8 stream_POSITION_freq;
	u8 stream_EXTRA1_freq;
	u8 stream_EXTRA2_freq;
	u8 stream_EXTRA3_freq;
	u8 stream_ALL_freq;
	bool stream_freq_enable;	//使能以上自定义数据流回传频率
	/*日志记录*/
	//各类型数据记录频率
	u8 log_TEST_rate; 		
	u8 log_PARAMETER_rate;	
	u8 log_GPS_rate;			
	u8 log_IMU_rate;			
	u8 log_RCIN_rate;			
	u8 log_RCOUT_rate;	
	u8 log_BARO_rate;		
	u8 log_POWR_rate;		
	u8 log_ATTITUDE_rate;		
	u8 log_MODE_rate;		
	u8 log_POS_rate;		
	u8 log_PID_ANG_RLL_rate;	
	u8 log_PID_ANG_PIT_rate;
	u8 log_PID_ANG_YAW_rate;	
	u8 log_PID_RAT_RLL_rate;	
	u8 log_PID_RAT_PIT_rate;	
	u8 log_PID_RAT_YAW_rate;	
	u8 log_PID_ACC_Z_rate;	
	u8 log_PID_VEL_Z_rate;	
	u8 log_PID_POS_Z_rate;	
	u8 log_DR_rate;		
	u8 log_SINS_rate;
	u8 log_WP_rate;			
	u8 log_MODEL_rate;		

	pidInit_t pidAngleRoll;
	pidInit_t pidAnglePitch;
	pidInit_t pidAngleYaw;
	pidInit_t pidRateRoll;
	pidInit_t pidRatePitch;
	pidInit_t pidRateYaw;
	pidInit_t pidPosZ;
	pidInit_t pidVelZ;
	pidInit_t pidAccZ;
	
}PARA_FIXED ;//__attribute__ ((aligned (4)));

/*系统变化参数*/
typedef struct{
	/*系统*/
	TIME_STRUCT sys_time;//系统时间(yyyy-mm-dd-hh-mm-ss)	
	u32 last_gcs_heartbeat_time;	//最新一次接收到GCS心跳的时间
	float sramin_available_latest;	//SRAMIN实时剩余空间
	float sramin_available_min;	//SRAMIN最小剩余空间
	int time_available_max;	//最长剩余运行时间
	int time_available_min;	//最短剩余运行时间
	/*task*/
	u8 task_total_num;		//auto模式下任务总个数
	int task_total_time;	//任务总时长
	float last_pilot_heading;	//定向朝向
	int last_pilot_yaw_input_ms; //开启定向的时间
	float last_pilot_depth;
	s16 screen_heading;	//屏幕定向角度
	s16 screen_depth;	//屏幕定深深度
	/*运行测试*/
	bool run_test_enable;		//运行测试使能
	u8 run_test_mode;	//运行测试模式
	char run_test_mode_char[10];	
	int run_test_minute;	//运行测试时间
	int run_test_num;		//运行测试次数
	/*mission*/
	CONTROL_MODE control_mode;	//控制模式
	AutoMode auto_mode;			//AUTO模式下子模式
	u16 waypoint_dest_sysid;
	u16 waypoint_dest_compid;
	u16 waypoint_count;
	u32 waypoint_timelast_request;	//
	u16 waypoint_request_last;
	u16 waypoint_request_i;
	u16 mission_item_reached_index;	//运行到航点的索引值

	s32 wp_bearing;		//到下个航点的角度
	u32 home_bearing;	//到返航点的角度(cd)
	u32 wp_distance;	//到下个航点的距离(cm)
	u32 home_distance;	//到返航点的距离(cm)
	/*回传消息*/
	u8 severity;
	char text[50];
	/*日志*/
	u32 log_return;	//要回传的log文件
	u32 log_delete;	//要删除的log文件
	uint64_t log_start_time;	//开始记录的时间
	/*数据量*/
	u8 ec_parse_num_sencond;	//1s内电子罗盘解析次数
	u8 ec_parse_num_latest;	
	u8 imu_parse_num_sencond;
	u8 imu_parse_num_latest;
	u8 gps_parse_num_sencond;
	u8 gps_parse_num_latest;
	/*上位机通信*/
	u16 data_size_one_second;	//1s钟的数据量
	u16 data_size_one_second_max;	//1s钟最大数据量
	u16 data_size_one_second_latest;	//1s钟实时数据量
	u32 mavlink_param_request_second; //上位机要求发送参数系统秒数
	u8 mavlink_recv_percentage;	//mavlink接收成功率
	/*ROV通信状态*/
	u8 sock_status;	//socket通信状态
	/*kf*/
	u8 kf_verify_sec;	//kf测试时常(s)
	/*模型辅助*/
	u8 model_test_sec;	//动力学模型测试时常(s)
	/*其他*/
	uint8_t motor_test;   //电机测试标志 
	u8 advanced_para;			//高级设置
	u8 openmv_cam_record;			//openmv记录设置
	float sins_db;		//加速度计扰动
}PARA_VARYING;

typedef struct{
	bool gcs_heartbeat;
	bool ins;	//惯导系统
	bool imu;	//IMU
	bool ec;	//电子罗盘
	bool baro;
	bool gps;
}HEALTH;

typedef enum{
	MISSION_STOPPED = 0,
	MISSION_RUNNING,
	MISSION_COMPLETE,
}MISSION_STATE;
typedef struct{
	MISSION_STATE state;
	bool nav_cmd_loaded		: 1;
	bool do_cmd_loaded		: 1;
	bool do_cmd_all_done	: 1;
}MISSION;

/*系统标志位*/ //后期改成位操作?
typedef struct{
	/*传感器*/
	HEALTH health;
	bool baro_vibration;	//深度计校准值
	bool acc_vibration;		//加速度计校准
	bool EC_att_enable;		//使用电子罗盘姿态
	/*电机*/
	bool motors_armed;	//电机解锁标志
	bool motor_test;	//电机测试
	/*电池*/
	bool battery_voltage_low;	//电池电压
	/*task*/
	bool task_auto_switch;//各任务设定值切换
	bool task_set_fail_flag; //任务列表接受失败标志
	bool task_run; //任务开始
	bool task_tasklists_recved; //接收到任务列表
	bool screen_heading;	//屏幕定向方向
	bool screen_depth;		//屏幕定深深度
	bool show_task_pass;	//显示任务超时
	/*mission*/
	MISSION mission;
	bool waypoint_receiving;	//航点接收标志
	bool GCS_show_WP_infor;		//GCS显示当前位置到目标航点的数据
	/*遥控器*/
	bool ppm_connected; //无线遥控器连接标志
	bool ppm_recv;	//遥控接收机ppm端口数据接收标志
	bool ppm_enable;	//遥控接收机和遥控器建立连接标志
	bool rc_roll_pitch_enable; //左摇杆翻滚模式
	bool joy_connected;	//有线手柄连接标志
	/*参数*/
	bool param_recv_flag ;//参数设定标志
	bool param_all_send;	//参数回传给上位机完毕标志
	bool param_send;
	/*相机*/
	bool openmv_get_video_list_flag;//openmv获取视频列表标志
	/*sd卡*/
	bool sdcard_detected;//sd卡在位参数
	bool log_check;	//查看sd卡记录文件，在点击“获取记录列表”时置位
	/*spi flash*/
	bool flash_check;//flash在位参数
	/*日志*/
	bool log_creat;//创建log文件标志，在任务开始时置位
	/*GPS*/
	bool gps_lock;	//gps锁定
	bool gps_log_update;	//gps更新记录
	/*kf*/
	bool kf_verify;
	/*其他*/
	bool data_recv;//获取时间标志
	bool relay_12V_enable;	//12V继电器上电标志
	bool esc_init_complete;	//电调初始化完成标志
	bool mavlink_enable;	//MAVLINK端口使能标志
	bool telem_enable;		//数传电台端口使能标志
	bool gcs_out_of_time;	//发送数据流超时
	bool jump_sdread;	//跳转到sd卡读取
	bool send_empty;	//发送fifo为空
	bool follow_enable;	//非MANUAL模式下随动使能（航向和深度）
	//console
	bool show_mavlink_stream1;	//console简单显示mavlink
	bool show_mavlink_stream2;	//console显示mavlink数据流
	bool show_gps_infor1;	//console显示原始GPS语句
	bool show_gps_infor2;	//console显示解析后的GPS语句
	/*串口通信相关*/
	bool mavlink_recv;	//mavlink连接
	bool mavlink_recv_dma;
	bool imu_recv;	//mpu读取
	bool imu_recv_dma;
	bool ec_recv;	//电子罗盘读取
	bool ec_recv_dma;
	bool gps_recv;	//gps读取
	bool gps_recv_dma;
	bool request_data_stream;	//获取数据流
	bool uart1_showrecv;	//显示串口接收数据流
	bool uart2_showrecv;
	bool uart3_showrecv;
	bool uart6_showrecv;
	bool uart7_showrecv;
	bool uart8_showrecv;
	//串口透明传输
	bool uart1_dictect_trans;
	bool uart2_dictect_trans;
	bool uart3_dictect_trans;
	bool uart7_dictect_trans;
	bool uart8_dictect_trans;
	/*模型*/
	bool model_verify;	//模型验证
	/*临时*/
	bool temp_logcreat;
}SYS_FLAGS;

//参数类型，参照 https://mavlink.io/en/messages/common.html -> MAV_PARAM_TYPE
typedef enum{
	UINT8 = 1,
	INT8,
	UINT16,
	INT16,
	UINT32,
	INT32,
	UINT64,
	INT64,
	REAL32,
	REAL64
}PARAM_TYPE;
//参数回传结构体
typedef struct{
	char param_id[16];			//参数名
	PARAM_TYPE param_type;		//参数类型
	void *param_value;			//参数值
	flash_para_list param_list;	//参数序列
}PARAM_ID_TYPE_VALUE;

void param_default_init(void);
bool param_num_check(void);

extern PARA_FIXED fp;
extern PARA_VARYING vp;
extern SYS_FLAGS sys_flag;
extern PARAM_ID_TYPE_VALUE param_id_type_value[];
extern SYS_TIME sys_time;
extern Channel channel;

#endif

