#include "haizhe.h"

//#include "tm7812b.h"

extern u16 loop_rate_hz;	//loop循环频率
extern u16 loop_counter;

const Task task_list[]={
	//函数						调用频率hz，		最大时长(单位10us)		函数名	
	{fifty_hz_loop,				50,			10,					"fifty_hz_loop"},
	{KF_AHRS_verify,			100,		50,					"KF_AHRS_verify"},	//kf测试
	{model_verify,				100,		50,					"model_verify"},	//模型验证
#ifdef QGC_ENABLE
	{gcs_check_input,			100,		200,				"gcs_check_input"},	//地面站输入检查(手柄输入...)
	{gcs_send_heartbeat,		1,			100,				"gcs_send_heartbeat"},	//向地面站发送心跳包以及一些系统状态(。。。)
	{gcs_data_stream_send,		50,			300,				"gcs_data_stream_send"},	//向地面站发送数据流,需连接到地面站才有输出
	{gcs_send_deferred,			50,			300,				"gcs_send_deferred"},	//向地面站发送等待队列里的mavlink消息
#else
	{ppm_update,				10,			10,					"ppm_update"},	//解析遥控接收机PPM信号
#endif
	{sys_status_check,			1, 			25,					"sys_status_check"},	
	{rtc_update,				2,			30,					"rtc_update"},
	{gps_read,					10,			100,				"gps_read"},	//读取gps数据
	{baro_read,					10,			60,					"baro_read"},	//读取深度数据
	{logging_loop,				100, 		200, 				"logging"}, 	//写日志
	{console_debug,				1,			100,				"console_debug"},	//Debug接收调试
	{user_twenty_hz_task,		20,			10,					"user_twenty_hz_task"},	//10hz任务
	{user_three_hz_task,		3,			10,					"user_three_hz_task"},	//3hz任务
	{user_one_hz_task,			1,			10,					"user_one_hz_task"},	//20hz任务
	//{openmv_mavmsg_stream,		10,					100},	//获取openmv视频列表
	
};
	
void setup(){
	system_init();
	load_default_param();	//从spi flash读取系统默认参数
	init_haizhe();//设备初始化
	scheduler_init(task_list,sizeof(task_list)/sizeof(task_list[0]));	//任务列表初始化
	console_init(task_list,sizeof(task_list)/sizeof(task_list[0]));		//传递任务列表到console
}

void fast_loop(uint16_t hz){			
	Get_sensor_data();	//更新姿态和深度		
//	if(!sys_flag.model_verify)
//		model_update(Sensor_latest.ins.euler);
	if(!sys_flag.kf_verify && vp.control_mode==AUTO && sys_flag.motors_armed) //运行卡尔曼滤波 && fp.dr_mode!=DR_GPS
		KF_update(Sensor_latest.ins.euler,Sensor_latest.ins.gyro,Sensor_latest.ins.acc,1,0.01,15);	
	update_control_mode();//模式更新，运行控制算法	
	motors_output();	//电机输出
	mavsend("",0);
	//动态记录主任务运行相关时间 -> TODO改为滑动窗口记录
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

u16 period=1000;	//若频率为100hz(周期10ms)，在100Khz的中断频率下为1000个定时器中断
void loop(uint8_t hz){
	sys_time.loop_tick = 0;	//循环开始时间重置
//	u16 period=100000/hz;	//若频率为100hz(周期10ms)，在100Khz的中断频率下为1000个定时器中断
	loop_rate_hz = hz;
	fast_loop(hz);
	int time_available= period-sys_time.loop_tick;
	if(time_available<0) return;
	scheduler_run(time_available);
	//计算剩余可运行的时间 -> 用于调整子任务 TODO改为滑动窗口计算
	time_available = period-sys_time.loop_tick;
	if(time_available<vp.time_available_min) vp.time_available_min=time_available;
	else if(time_available>vp.time_available_max) vp.time_available_max = time_available;
	while(sys_time.loop_tick<period){}	//等待一个周期过去
}

void fifty_hz_loop(){
}

//系统各状态检测
void sys_status_check(){
	relay_12V_check();//电调继电器开合检测
	sys_check_BATT();
	sys_check_ins();
	sys_check_baro();
	sys_check_GCS_heartbreat();
	sys_check_sd();
//	USBD_USR_ConnectDetect();	//USB连接检测
	sys_check_flash();
	sys_check_wp_path();
	sys_update_payload();
}

void user_one_hz_task(void){
	if(!sys_flag.gps_lock) led_act = !led_act;
//	RGBLED_BLUE_twinkle();	//rgbled红灯闪烁
	
}	

void user_three_hz_task(void){
	
}	

void user_twenty_hz_task(void){	
	static u32 run_test_start_second = 0;
	if(sys_flag.gps_lock) led_act = !led_act;
	if(vp.run_test_enable){	//使能运行测试 -> 测试各模式的程序运行是否正常
		static int num=0;
		static int num_previous = -1;
		if(num==vp.run_test_num){	//全部运行测试结束
			printf("全部运行测试结束，切换回手动控制 \r\n");
			set_mode(MANUAL);
			sys_flag.motors_armed = false;
			vp.run_test_enable = false;
			num = 0;
			num_previous = -1;
		}
		else if((sys_time.one_second-run_test_start_second) > vp.run_test_minute*60){	//单次运行结束
			num++;
			printf("第%d次运行测试结束 \r\n",num);
			set_mode(MANUAL);
			run_test_start_second = sys_time.one_second;
			sys_flag.motors_armed = false;
		}
		else if(num_previous != num){	//单次运行之后不立马开始新一次运行测试,为了保存日志
			num_previous = num;
			if(!set_mode((CONTROL_MODE)vp.run_test_mode)){
				printf("第%d次运行测试切换%s模式失败！\r\n",num,vp.run_test_mode_char);
				set_mode(MANUAL);
				sys_flag.motors_armed = false;
				vp.run_test_enable = false;
				num = 0;
				num_previous = -1;
			}
			sys_flag.motors_armed = true;
			run_test_start_second = sys_time.one_second;
			printf("------------------------------------------------\r\n");
			printf("第%d次运行测试开始，切换到%s模式 \r\n",num+1,vp.run_test_mode_char);
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

								    
