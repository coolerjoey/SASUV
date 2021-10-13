#include "haizhe.h"

//#include "tm7812b.h"

extern u16 loop_rate_hz;	//loopѭ��Ƶ��
extern u16 loop_counter;

const Task task_list[]={
	//����						����Ƶ��hz��		���ʱ��(��λ10us)		������	
	{fifty_hz_loop,				50,			10,					"fifty_hz_loop"},
	{KF_AHRS_verify,			100,		50,					"KF_AHRS_verify"},	//kf����
	{model_verify,				100,		50,					"model_verify"},	//ģ����֤
#ifdef QGC_ENABLE
	{gcs_check_input,			100,		200,				"gcs_check_input"},	//����վ������(�ֱ�����...)
	{gcs_send_heartbeat,		1,			100,				"gcs_send_heartbeat"},	//�����վ�����������Լ�һЩϵͳ״̬(������)
	{gcs_data_stream_send,		50,			300,				"gcs_data_stream_send"},	//�����վ����������,�����ӵ�����վ�������
	{gcs_send_deferred,			50,			300,				"gcs_send_deferred"},	//�����վ���͵ȴ��������mavlink��Ϣ
#else
	{ppm_update,				10,			10,					"ppm_update"},	//����ң�ؽ��ջ�PPM�ź�
#endif
	{sys_status_check,			1, 			25,					"sys_status_check"},	
	{rtc_update,				2,			30,					"rtc_update"},
	{gps_read,					10,			100,				"gps_read"},	//��ȡgps����
	{baro_read,					10,			60,					"baro_read"},	//��ȡ�������
	{logging_loop,				100, 		200, 				"logging"}, 	//д��־
	{console_debug,				1,			100,				"console_debug"},	//Debug���յ���
	{user_twenty_hz_task,		20,			10,					"user_twenty_hz_task"},	//10hz����
	{user_three_hz_task,		3,			10,					"user_three_hz_task"},	//3hz����
	{user_one_hz_task,			1,			10,					"user_one_hz_task"},	//20hz����
	//{openmv_mavmsg_stream,		10,					100},	//��ȡopenmv��Ƶ�б�
	
};
	
void setup(){
	system_init();
	load_default_param();	//��spi flash��ȡϵͳĬ�ϲ���
	init_haizhe();//�豸��ʼ��
	scheduler_init(task_list,sizeof(task_list)/sizeof(task_list[0]));	//�����б��ʼ��
	console_init(task_list,sizeof(task_list)/sizeof(task_list[0]));		//���������б�console
}

void fast_loop(uint16_t hz){			
	Get_sensor_data();	//������̬�����		
//	if(!sys_flag.model_verify)
//		model_update(Sensor_latest.ins.euler);
	if(!sys_flag.kf_verify && vp.control_mode==AUTO && sys_flag.motors_armed) //���п������˲� && fp.dr_mode!=DR_GPS
		KF_update(Sensor_latest.ins.euler,Sensor_latest.ins.gyro,Sensor_latest.ins.acc,1,0.01,15);	
	update_control_mode();//ģʽ���£����п����㷨	
	motors_output();	//������
	mavsend("",0);
	//��̬��¼�������������ʱ�� -> TODO��Ϊ�������ڼ�¼
	fastloop_time_average=(fastloop_time_average*fastloop_run_num+sys_time.loop_tick)/(fastloop_run_num+1);
	fastloop_run_num++;
	if(sys_time.loop_tick > 500){
		fastloop_overrun_num++;
		if(sys_time.loop_tick > fastloop_overrun_time_max) fastloop_overrun_time_max=sys_time.loop_tick;
	} 
	static u32 dynamic_time = 0;
	if(sys_time.one_second > dynamic_time+10){
		fastloop_time_average=0;
		fastloop_run_num=0;
		fastloop_overrun_time_max=0;
		fastloop_overrun_num=0;
		dynamic_time = sys_time.one_second;
	}
}

u16 period=1000;	//��Ƶ��Ϊ100hz(����10ms)����100Khz���ж�Ƶ����Ϊ1000����ʱ���ж�
void loop(uint8_t hz){
	sys_time.loop_tick = 0;	//ѭ����ʼʱ������
//	u16 period=100000/hz;	//��Ƶ��Ϊ100hz(����10ms)����100Khz���ж�Ƶ����Ϊ1000����ʱ���ж�
	loop_rate_hz = hz;
	fast_loop(hz);
	int time_available= period-sys_time.loop_tick;
	if(time_available<0) return;
	scheduler_run(time_available);
	//����ʣ������е�ʱ�� -> ���ڵ��������� TODO��Ϊ�������ڼ���
	time_available = period-sys_time.loop_tick;
	if(time_available<vp.time_available_min) vp.time_available_min=time_available;
	else if(time_available>vp.time_available_max) vp.time_available_max = time_available;
	while(sys_time.loop_tick<period){}	//�ȴ�һ�����ڹ�ȥ
}

void fifty_hz_loop(){
}

//ϵͳ��״̬���
void sys_status_check(){
	relay_12V_check();//����̵������ϼ��
	sys_check_BATT();
	sys_check_ins();
	sys_check_baro();
	sys_check_GCS_heartbreat();
	sys_check_sd();
//	USBD_USR_ConnectDetect();	//USB���Ӽ��
	sys_check_flash();
	sys_check_wp_path();
	sys_update_payload();
}

void user_one_hz_task(void){
	if(!sys_flag.gps_lock) led_act = !led_act;
//	RGBLED_BLUE_twinkle();	//rgbled�����˸
	
}	

void user_three_hz_task(void){
	
}	

void user_twenty_hz_task(void){	
	static u32 run_test_start_second = 0;
	if(sys_flag.gps_lock) led_act = !led_act;
	if(vp.run_test_enable){	//ʹ�����в��� -> ���Ը�ģʽ�ĳ��������Ƿ�����
		static int num=0;
		static int num_previous = -1;
		if(num==vp.run_test_num){	//ȫ�����в��Խ���
			printf("ȫ�����в��Խ������л����ֶ����� \r\n");
			set_mode(MANUAL);
			sys_flag.motors_armed = false;
			vp.run_test_enable = false;
			num = 0;
			num_previous = -1;
		}
		else if((sys_time.one_second-run_test_start_second) > vp.run_test_minute*60){	//�������н���
			num++;
			printf("��%d�����в��Խ��� \r\n",num);
			set_mode(MANUAL);
			run_test_start_second = sys_time.one_second;
			sys_flag.motors_armed = false;
		}
		else if(num_previous != num){	//��������֮������ʼ��һ�����в���,Ϊ�˱�����־
			num_previous = num;
			if(!set_mode((CONTROL_MODE)vp.run_test_mode)){
				printf("��%d�����в����л�%sģʽʧ�ܣ�\r\n",num,vp.run_test_mode_char);
				set_mode(MANUAL);
				sys_flag.motors_armed = false;
				vp.run_test_enable = false;
				num = 0;
				num_previous = -1;
			}
			sys_flag.motors_armed = true;
			run_test_start_second = sys_time.one_second;
			printf("------------------------------------------------\r\n");
			printf("��%d�����в��Կ�ʼ���л���%sģʽ \r\n",num+1,vp.run_test_mode_char);
		}
	}
	else run_test_start_second = sys_time.one_second;
}	

int main(void)
{		
	setup();
	while(1){
		loop(100);
	}

}

								    
