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

#define Task_MAX 10 //���������

/*ʱ��ṹ��*/
typedef struct{
	u16 year;
	u8 month;
	u8 day;
	u8 hour;
	u8 minute;
	u8 second;
	u8 week;
}TIME_STRUCT;

//ϵͳ���ϵ�󾭹���ʱ��
typedef struct{
	uint64_t ten_micro;//ϵͳ10us
	uint64_t one_mill;//ϵͳms
	u32 one_second;//ϵͳs
	u32 one_minute;//ϵͳmin
	u32 loop_tick;	//ϵͳѭ��ʱ��
}SYS_TIME;

//��ȫ�������
typedef struct{
	bool imu;
	bool baro;
	bool batt;
	bool joystick;
	bool gps;
	bool sd;
	bool flash;
}HEALTH_CHEABLE;

/*ϵͳ�̶���������д��flash����*/
 typedef struct{	
	
	u16 param_num;	//�̶���������
	u16 sysid_my_gcs;	//���غ�ϵͳid������mavlinkͨ��
	u8 frame_class;	//���غŻ���ģ��
	float power_min;//��С��ص�ѹ
	u8 log_mode;	//�ڵ������ʱ�Ƿ��¼��־ true-��¼ false-����¼
	bool ROV_mode_enable;	//�Ƿ�����ROV���� - true:�������磬�ر����ߵ�̨;false:�ر����磬�������ߵ�̨

	HEALTH_CHEABLE health_chenable;	//��װ���ʹ��
	
	u32 uartA_baud;	//MPU9250
	u32 uartB_baud;	//WiFi
	u32 uartC_baud;	//TELEM
	u32 uartD_baud;	//GPS
	u32 uartE_baud;	//SERIAL2
	u32 uartF_baud;	//DEBUG
	
	vec3f attitude_init;				//��ʼ��̬ roll,pitch,yaw
	vec3f sins_db;		//���ٶȼ���Ư��Χ

	DR_MODE dr_mode;	//����ģʽ
	WP_PATH wp_path;	//����·��
	float wp_radius_cm;		//������ľ��룬�ڴ˾��뷶Χ����Ϊ����ĺ���
	bool exit_mission_rtl;	//��������Ƿ񷵻�home��
	u8	gps_fb_interval;		//ģ�͸���������GPS�������ʱ��
	float forward_auto;		//AUTOģʽ��ǰ������

	float throttle_hover;	//ˮ����ͣ����(0.0-1.0)
	float throttle_deadzone_swerve;	//ת����������
	float throttle_deadzone_stabilize;	//ˮƽ��̬�ȶ���������
	float motor_gain;	//�������
	float motor_maxGain;	//ң��ģʽ�µ���������
	float motor_minGain;	//ң��ģʽ�µ����С����
	float motor_numGainSetting;	//ң��ģʽ�µ�����浵λ
	float speed_gain;	//���ȡ�����ģʽ�µ������
	float speed_maxGain;	//���ȡ�����ģʽ�µ���������
	float speed_minGain;	//���ȡ�����ģʽ�µ����С����
	float speed_numGainSetting; //���ȡ�����ģʽ�µ�����浵λ
	Motor motor1_fac;	//�ĸ���������ɶȱ���
	Motor motor2_fac;
	Motor motor3_fac;
	Motor motor4_fac;
	Motor motor5_fac;
	Motor motor6_fac;

	REMOTE_CONTROL remoteCtrl;	//����ң����
	u8 btn_function[16];	////û����shift���İ�ť����ӳ��
	u8 btn_sfunction[16];	//����swift���İ�ť����ӳ��

	ESC_TYPE esc_type;	//�������

	//����λ����������Ƶ��
	u8 stream_RAW_SENSORS_freq;				
	u8 stream_EXTENDED_STATUS_freq;
	u8 stream_RC_CHANNELS_freq;
	u8 stream_RAW_CONTROLLER_freq;
	u8 stream_POSITION_freq;
	u8 stream_EXTRA1_freq;
	u8 stream_EXTRA2_freq;
	u8 stream_EXTRA3_freq;
	u8 stream_ALL_freq;
	bool stream_freq_enable;	//ʹ�������Զ����������ش�Ƶ��
	/*��־��¼*/
	//���������ݼ�¼Ƶ��
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

/*ϵͳ�仯����*/
typedef struct{
	/*ϵͳ*/
	TIME_STRUCT sys_time;//ϵͳʱ��(yyyy-mm-dd-hh-mm-ss)	
	u32 last_gcs_heartbeat_time;	//����һ�ν��յ�GCS������ʱ��
	float sramin_available_latest;	//SRAMINʵʱʣ��ռ�
	float sramin_available_min;	//SRAMIN��Сʣ��ռ�
	int time_available_max;	//�ʣ������ʱ��
	int time_available_min;	//���ʣ������ʱ��
	/*task*/
	u8 task_total_num;		//autoģʽ�������ܸ���
	int task_total_time;	//������ʱ��
	float last_pilot_heading;	//������
	int last_pilot_yaw_input_ms; //���������ʱ��
	float last_pilot_depth;
	s16 screen_heading;	//��Ļ����Ƕ�
	s16 screen_depth;	//��Ļ�������
	/*���в���*/
	bool run_test_enable;		//���в���ʹ��
	u8 run_test_mode;	//���в���ģʽ
	char run_test_mode_char[10];	
	int run_test_minute;	//���в���ʱ��
	int run_test_num;		//���в��Դ���
	/*mission*/
	CONTROL_MODE control_mode;	//����ģʽ
	AutoMode auto_mode;			//AUTOģʽ����ģʽ
	u16 waypoint_dest_sysid;
	u16 waypoint_dest_compid;
	u16 waypoint_count;
	u32 waypoint_timelast_request;	//
	u16 waypoint_request_last;
	u16 waypoint_request_i;
	u16 mission_item_reached_index;	//���е����������ֵ

	s32 wp_bearing;		//���¸�����ĽǶ�
	u32 home_bearing;	//��������ĽǶ�(cd)
	u32 wp_distance;	//���¸�����ľ���(cm)
	u32 home_distance;	//��������ľ���(cm)
	/*�ش���Ϣ*/
	u8 severity;
	char text[50];
	/*��־*/
	u32 log_return;	//Ҫ�ش���log�ļ�
	u32 log_delete;	//Ҫɾ����log�ļ�
	uint64_t log_start_time;	//��ʼ��¼��ʱ��
	/*������*/
	u8 ec_parse_num_sencond;	//1s�ڵ������̽�������
	u8 ec_parse_num_latest;	
	u8 imu_parse_num_sencond;
	u8 imu_parse_num_latest;
	u8 gps_parse_num_sencond;
	u8 gps_parse_num_latest;
	/*��λ��ͨ��*/
	u16 data_size_one_second;	//1s�ӵ�������
	u16 data_size_one_second_max;	//1s�����������
	u16 data_size_one_second_latest;	//1s��ʵʱ������
	u32 mavlink_param_request_second; //��λ��Ҫ���Ͳ���ϵͳ����
	u8 mavlink_recv_percentage;	//mavlink���ճɹ���
	/*ROVͨ��״̬*/
	u8 sock_status;	//socketͨ��״̬
	/*kf*/
	u8 kf_verify_sec;	//kf����ʱ��(s)
	/*ģ�͸���*/
	u8 model_test_sec;	//����ѧģ�Ͳ���ʱ��(s)
	/*����*/
	uint8_t motor_test;   //������Ա�־ 
	u8 advanced_para;			//�߼�����
	u8 openmv_cam_record;			//openmv��¼����
	float sins_db;		//���ٶȼ��Ŷ�
}PARA_VARYING;

typedef struct{
	bool gcs_heartbeat;
	bool ins;	//�ߵ�ϵͳ
	bool imu;	//IMU
	bool ec;	//��������
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

/*ϵͳ��־λ*/ //���ڸĳ�λ����?
typedef struct{
	/*������*/
	HEALTH health;
	bool baro_vibration;	//��ȼ�У׼ֵ
	bool acc_vibration;		//���ٶȼ�У׼
	bool EC_att_enable;		//ʹ�õ���������̬
	/*���*/
	bool motors_armed;	//���������־
	bool motor_test;	//�������
	/*���*/
	bool battery_voltage_low;	//��ص�ѹ
	/*task*/
	bool task_auto_switch;//�������趨ֵ�л�
	bool task_set_fail_flag; //�����б����ʧ�ܱ�־
	bool task_run; //����ʼ
	bool task_tasklists_recved; //���յ������б�
	bool screen_heading;	//��Ļ������
	bool screen_depth;		//��Ļ�������
	bool show_task_pass;	//��ʾ����ʱ
	/*mission*/
	MISSION mission;
	bool waypoint_receiving;	//������ձ�־
	bool GCS_show_WP_infor;		//GCS��ʾ��ǰλ�õ�Ŀ�꺽�������
	/*ң����*/
	bool ppm_connected; //����ң�������ӱ�־
	bool ppm_recv;	//ң�ؽ��ջ�ppm�˿����ݽ��ձ�־
	bool ppm_enable;	//ң�ؽ��ջ���ң�����������ӱ�־
	bool rc_roll_pitch_enable; //��ҡ�˷���ģʽ
	bool joy_connected;	//�����ֱ����ӱ�־
	/*����*/
	bool param_recv_flag ;//�����趨��־
	bool param_all_send;	//�����ش�����λ����ϱ�־
	bool param_send;
	/*���*/
	bool openmv_get_video_list_flag;//openmv��ȡ��Ƶ�б��־
	/*sd��*/
	bool sdcard_detected;//sd����λ����
	bool log_check;	//�鿴sd����¼�ļ����ڵ������ȡ��¼�б�ʱ��λ
	/*spi flash*/
	bool flash_check;//flash��λ����
	/*��־*/
	bool log_creat;//����log�ļ���־��������ʼʱ��λ
	/*GPS*/
	bool gps_lock;	//gps����
	bool gps_log_update;	//gps���¼�¼
	/*kf*/
	bool kf_verify;
	/*����*/
	bool data_recv;//��ȡʱ���־
	bool relay_12V_enable;	//12V�̵����ϵ��־
	bool esc_init_complete;	//�����ʼ����ɱ�־
	bool mavlink_enable;	//MAVLINK�˿�ʹ�ܱ�־
	bool telem_enable;		//������̨�˿�ʹ�ܱ�־
	bool gcs_out_of_time;	//������������ʱ
	bool jump_sdread;	//��ת��sd����ȡ
	bool send_empty;	//����fifoΪ��
	bool follow_enable;	//��MANUALģʽ���涯ʹ�ܣ��������ȣ�
	//console
	bool show_mavlink_stream1;	//console����ʾmavlink
	bool show_mavlink_stream2;	//console��ʾmavlink������
	bool show_gps_infor1;	//console��ʾԭʼGPS���
	bool show_gps_infor2;	//console��ʾ�������GPS���
	/*����ͨ�����*/
	bool mavlink_recv;	//mavlink����
	bool mavlink_recv_dma;
	bool imu_recv;	//mpu��ȡ
	bool imu_recv_dma;
	bool ec_recv;	//�������̶�ȡ
	bool ec_recv_dma;
	bool gps_recv;	//gps��ȡ
	bool gps_recv_dma;
	bool request_data_stream;	//��ȡ������
	bool uart1_showrecv;	//��ʾ���ڽ���������
	bool uart2_showrecv;
	bool uart3_showrecv;
	bool uart6_showrecv;
	bool uart7_showrecv;
	bool uart8_showrecv;
	//����͸������
	bool uart1_dictect_trans;
	bool uart2_dictect_trans;
	bool uart3_dictect_trans;
	bool uart7_dictect_trans;
	bool uart8_dictect_trans;
	/*ģ��*/
	bool model_verify;	//ģ����֤
	/*��ʱ*/
	bool temp_logcreat;
}SYS_FLAGS;

//�������ͣ����� https://mavlink.io/en/messages/common.html -> MAV_PARAM_TYPE
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
//�����ش��ṹ��
typedef struct{
	char param_id[16];			//������
	PARAM_TYPE param_type;		//��������
	void *param_value;			//����ֵ
	flash_para_list param_list;	//��������
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

