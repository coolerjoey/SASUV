#include "console.h"
#include "config.h"
#include "atkp.h"
#include "sensor.h"
#include "motors.h"
#include "Logger.h"
#include "joystick.h"
#include "delay.h"
#include "GCS_Mavlink.h"
#include "flash.h"
#include "malloc.h"
#include "relay.h"
#include "mission.h"
#include "position_vector.h"
#include "wpnav.h"
#include "sdio_sdcard.h"
#include "parameter.h"
#include "exfuns.h"
#include "model.h"
#include "model_verify.h"
#include "model_test.h"
#include "kf_test.h"
#include "kf.h"

const Task *task_list_copy=NULL;
u8 task_num=0;

static void empty_fun(char *argv){}

const COMMAND_LIST command_list[]={
	{"console","enter debug",empty_fun},
	{"quit","quit debug",empty_fun},
	{"lscmd","list command",handle_lscmd},

	//无参数
	{"lstt","list task time",handle_lstt},
	{"lsll","list log list",handle_lsll},
	{"lsstatus","list some status of AUV",handle_lsstatus},
	{"lsadc","list adc value",handle_lsadc},
	{"lsbaro","list barometer info",handle_lsbaro},
	{"lspwr","list power voltage",handle_lspwr},
	{"lsang","list euler angle",handle_lsang},
	{"lsgyro","list euler rate",handle_lsgyro},
	{"lsacc","list accelerate info",handle_lsacc},
	{"calacc","calibrate IMU accelerator",handle_calacc},
	{"lsgps","list gps info",handle_lsgps},
	{"lstemp","list temperature",handle_lstemp},
	{"lsrc","list radio input",handle_lsrc},
	{"lsppm","list ppm receiver channel value",handle_lsppm},
	{"lspwm","list motor PWM out",handle_lspwm},
	{"lstime","list UTC+8.0 time",handle_lstime},
	{"lsjoy","list joystick value",handle_lsjoy},
	{"powerup","relay on",handle_powerup},
	{"powerdown","relay off",handle_powerdown},
	{"arm","arm throttle",handle_arm},
	{"disarm","disarm throttle",handle_disarm},
	{"MANUAL","swhitch to MANUAL mode",handle_MANUAL},
	{"STABILIZE","swhitch to STABILIZE mode",handle_STABILIZE},
	{"AUTO","swhitch to AUTO mode",handle_AUTO},
	{"ALT_HOLD","swhitch to ALT_HOLD mode",handle_ALT_HOLD},
	{"stpwm","stop pwm test",handle_stpwm},
	{"reset","jump to bootloader",handle_reset},
	{"sdread","jump to SDcard Reader",handle_sdread},
	{"paraset","set parameters to default value",handle_paraset},
	{"lswpcnt","list waypoint count",handle_lswpcnt},
	{"lsdatesize","list datesize to GCS in one second",handle_lsdatesize},
	{"lsparams","list all flash parameters",handle_lsparams},
	{"lscomm","show mavlink receive pencentage",handle_lscomm},
	{"sendparam","send all flash parameters on mavlink",handle_send_allparams},
	{"lssramin","list SRAMIN information",handle_lssramin},
	{"parasave","save flash parameters to parameters.txt",handle_parasave},
	{"lssocsta","show socket status",handle_lssocsta},
	//有参数
	{"setECatt","set att source",handle_setECatt},
	{"setrate","set IMU rate",handle_setfreq},
	{"setbaud","set IMU baudrate",handle_setbaud},
	{"lswp","list two waypoints distance and bearing \r\n \t \"lswp [i] [j]\": list distance and bearing from waypoint i to j,where 0<=i,j<wp_count",handle_lswp},
	{"settime","set system time \r\n \t settime [year month day hour minute second] ",handle_settime},
	{"ttpwm","test pwm test \r\n \t ttpwm 0 [pwm1 pwm2 pwm3 pwm4 pwm5 pwm6]: Test all motor, Where pwm1-6 should be 1000-2000 \r\n \t ttpwm i [pwm_i]: Test motor i, Where pwm_i should be 1000-2000",handle_ttpwm},
	{"setparam","set flash parameter value \r\n\t Usege: \"setparam [i] [v]\": set [i]th flash parameter value to [v],where 0<=i<param_num",handle_setparam},
	{"showmav","show mavlink datestreamt \r\n\t showmavlink [i]: i=0-hide mavlink datestream; i=1-show mavlink datestream",handle_showmavlink},
	{"lsparse","show sensor parse frequence \r\n\t \"lsparse [sensor] \": sensor=tcm/mpu/gps",handle_ls_sensorparse},
	{"setheading","set hold_heading \r\n\t \"setheading [v] [i]\": v=heading, shoule be 0<=v<=360; i=0-cancel heading, i=1-set hold_heading [v]",handle_setheading},
	{"showgps","show GPS message \r\n\t \"showgps [i] \": i=0-hide GPS datestream; i=1-show GPS raw message; i=2-show GPS parse message",handle_showgps},
	{"kfverify","verify kf \r\n\t Usege: \"kfverify [i] [j]\": i=0-stop kfverify; i=1-start kfverify, j-verify duration(s)",handle_kfverify},
	{"kftest","kf test  \r\n\t  \"kftest [i] \": i=0-stop kftest; i=1-start kftest, j-test duration(s);",handle_kftest},
	{"DCMupdate","",handle_DCM_update},
	{"showpass","show task pass information \r\n\t Usege: \"showpass [i]\": i=0-stop show task pass information, i=1-start show task pass information",handle_showpass},
	{"showwp","send waypoint information to GCS \r\n\t Usege: \"showwp [i]\": i=0-stop send waypoint information to GCS, i=1-start send waypoint information to GCS",handle_showwp},
	{"modelver","model verify \r\n\t Usege: \"modelver [i] [j] [f1]~[f6]\": [i]=0-stop verify model, [i]=1-start verify model; [j]-test duration(s); [f1~f6]-motors force",handle_modelver},
	{"modeltest","model update tet \r\n\t Usege: \"modeltest [i] [j]\": [i]-test duration(s); [j]-test type",handle_modeltest},
	{"pwm2force","test pwm to force \r\n\t Usege: \"pwm2force [i] [j]\": [i]-type; [j]-pwm",handle_pwm2force},
	{"force2pwm","test force to pwm \r\n\t Usege: \"force2pwm [i] [j]\": [i]-type; [j]-force",handle_force2pwm},
	{"runtest","Usege: \"runtest [mode] [minute] [num]\"",handle_runtest},
	{"showUART","Usege: \"showUART [uart] [show] \"",handle_showUART},
	{"UARTTC","Usege: \"UARTTC [uart] \"",handle_UARTTC},
	
};

static void handle_lscmd(char *argv){
	printf("\r\n===============command lists===============\r\n");
	for(int i=0;i<sizeof(command_list)/sizeof(command_list[0]);i++){
		printf("%d [%s]: %s\r\n",i+1,command_list[i].command,command_list[i].desciption);
	}
	printf("\r\n===========================================\r\n");
}

static void handle_lstt(char *argv){
	printf("=================================最近%.1fs内的时间参数=================================\r\n",fastloop_run_num/100.0f);
	printf("Index Run_num \t Time_average(10us) \t Time_max(10us) \t Overrun_num  Task_name\r\n");	//任务名 平均花费时间 最大花费时间 超过规定时常的运行次数
	printf("---------------------------------------------------------------------------------\r\n");
	printf(" %d \t %d \t\t %d \t\t\t %d \t\t %d \t\t %s \r\n",0,fastloop_run_num,fastloop_time_average,fastloop_overrun_time_max,fastloop_overrun_num,"fast_loop");	//主任务相关运行时间
	for(int i=0;i<task_num;i++){
		printf(" %d \t %d \t\t %d \t\t\t %d \t\t %d \t\t %s \r\n",i+1,task_run_num[i],task_time_average[i],task_overrun_time_max[i],task_overrun_num[i],task_list_copy[i].name);
	}
	printf("---------------------------------------------------------------------------------\r\n");
	//运行完loop还剩余的时间
	printf("time_available- min: %.1f%% max; %.1f%% \r\n",(float)vp.time_available_min/10.0f,(float)vp.time_available_max/10.0f);
	printf("==================================================================================\r\n");
}
static void handle_lsll(char *argv){
	u8 res = scan_files();
	if(res != 0) printf("list log failed! Error code: %d \r\n",res);
}

static void handle_lsadc(char *argv){
	printf("adc_baro=%.2f adc_power=%.2f adc_1=%.2f adc_2=%.2f \r\n",
		Sensor_latest.adc[0],Sensor_latest.adc[1],Sensor_latest.adc[2],Sensor_latest.adc[3]);
}

static void handle_lsbaro(char *argv){
	float baro_val;
	switch(Sensor_latest.barometer.baro_type){
		case analog_vol:baro_val = Sensor_latest.barometer.analog_vol ;break;
		case analog_curr:baro_val = Sensor_latest.barometer.analog_curr; break;
		case pressure:baro_val = (float)Sensor_latest.barometer.pressure;break;
	}
	printf("barometer: type:%d val:%.2f depth:%.2f \r\n",Sensor_latest.barometer.baro_type,baro_val,Sensor_latest.barometer.depth);
}
static void handle_lspwr(char *argv){
	printf("power voltage: %.2fV ",Sensor_latest.power);
	if(Sensor_latest.power > fp.power_min) printf("\r\n");
	else printf("-> THE VOLTAGE IS LOWER THAN %.2fV !\r\n",fp.power_min);
}
static void handle_lsang(char *argv){
	if(sys_flag.health.ec) printf("EC: AngXYZ= %.2f %.2f %.2f \r\n",ec.euler[0],ec.euler[1],ec.euler[2]);
	else printf("EC Error !\r\n");
	if(sys_flag.health.imu) printf("IMU: AngXYZ= %.2f %.2f %.2f \r\n",imu.euler[0],imu.euler[1],imu.euler[2]);
	else printf("IMU Error !\r\n");
}
//设置水平姿态来源
static void handle_setECatt(char *argv){
	int flag;
	u8 res = sscanf(argv,"%d",&flag);
	if(res != 1){
		printf("input error! Usege: \"setECatt [i] \": [i]=0-, [i]=1-\r\n");
		return;
	}
	switch(flag){
		case 0:
			sys_flag.EC_att_enable = false;
			printf("[setECatt] 姿态来源切换为IMU \r\n");
			break;
		case 1:
			if(sys_flag.health.ec){
				sys_flag.EC_att_enable = true;
				printf("[setECatt] 姿态来源切换为电子罗盘 \r\n");
			}
			else printf("[setECatt] 此系统未装配电子罗盘! \r\n");
			break;
	}

}
static void handle_lsacc(char *argv){
	if(sys_flag.health.imu) {
		printf("IMU: accx=%.2f accy=%.2f accz=%.2f \r\n",imu.acc[0],imu.acc[1],imu.acc[2]);
		return;
	}
	printf("IMU Error !\r\n");

}
//校准九轴加计
static void handle_calacc(char *argv){
	if(!sys_flag.health.imu){ 
		printf("IMU Error !\r\n");
		return;
	}
	ins_calibration();
}
//设置九轴回传速率
static void handle_setfreq(char *argv){
	if(!sys_flag.health.imu){ 
		printf("IMU Error !\r\n");
		return;
	}
	int rate;
	u8 res = sscanf(argv,"%d",&rate);
	if(res != 1){
		printf("input error! Usege: \"setrate [i]-rate \r\n");
		return;
	} 
//	imu_set_freq(rate);
}
//设置九轴回传波特率
static void handle_setbaud(char *argv){
	if(!sys_flag.health.imu){ 
		printf("IMU Error !\r\n");
		return;
	}
	int rate;
	u8 res = sscanf(argv,"%d",&rate);
	if(res != 1){
		printf("input error! Usege: \"setbaud [i]-rate \r\n");
		return;
	} 
//	imu_set_baud(rate);
}

static void handle_lsgyro(char *argv){
	if(sys_flag.health.imu){	
		printf("roll_rate=%.2f pit_rate=%.2f yaw_rate=%.2f \r\n",Sensor_latest.ins.gyro[0],Sensor_latest.ins.gyro[1],Sensor_latest.ins.gyro[2]);
		return;
	}
	printf("MPU Error !\r\n");
}
static void handle_lsgps(char *argv){
	if(!sys_flag.gps_lock){
		printf("GPS didn't lock! \r\n");
		return;
	}
	char gpssta[20];
	switch(Sensor_latest.gps.gpssta){
		case 0:
			sprintf(gpssta,"%s \r\n","未定位");
			break;
		case 1:
			sprintf(gpssta,"%s \r\n","非差分定位");
			break;
		case 2:
			sprintf(gpssta,"%s \r\n","差分定位");
			break;
		case 6:
			sprintf(gpssta,"%s \r\n","正在估算");
			break;
		default:
			break;
	}
	printf(gpssta);
	printf("UTC=%d_%d_%d-%d_%d_%d \r\n",Sensor_latest.gps.utc.year,Sensor_latest.gps.utc.month,Sensor_latest.gps.utc.day,
		Sensor_latest.gps.utc.hour,Sensor_latest.gps.utc.min,Sensor_latest.gps.utc.sec);
	printf("用于定位的卫星数为%d \r\n",Sensor_latest.gps.posslnum);
	printf("水平精度因子%.2f \r\n",Sensor_latest.gps.hdop);
	printf("海拔高度%.2f \r\n",Sensor_latest.gps.altitude);
	printf("处在 %c%c 半球 \r\n",Sensor_latest.gps.ewhemi,Sensor_latest.gps.nshemi);
	printf("纬度%lf \r\n",Sensor_latest.gps.latitude);
	printf("经度%lf \r\n",Sensor_latest.gps.longitude);
	printf("地面速率%.2f \r\n",Sensor_latest.gps.speed);
	printf("地面航向%.2f \r\n",Sensor_latest.gps.orient);
	printf("磁偏角%.2f \r\n",Sensor_latest.gps.magdec);
	printf("磁偏角方向%c \r\n",Sensor_latest.gps.magdir);
}
static void handle_lstemp(char *argv){
}
static void handle_lsrc(char *argv){
	printf("r_in:%.2f p_in:%.2f y_in:%.2f t_in:%.2f f_in:%.2f l_in:%.2f \r\n",
		get_roll(),get_pitch(),get_yaw(),get_throttle(),get_forward(),get_lateral());
}
static void handle_lsppm(char *argv){
	if(!sys_flag.ppm_connected)
		printf("remote controller not connect! \r\n");
	else{
		printf("PPM_VAL[1-8]:");
		for(int i=0;i<8;i++){
			printf(" %d",PPM_VAL[i]);
		}
		printf("\r\n");
	}
}
static void handle_lspwm(char *argv){
	u16 *pwm = get_motorout();
	printf("motorout 1-%d: ",MOTORS_MAX_NUMER);
	for(int i=0;i<MOTORS_MAX_NUMER;i++){
		printf("%d ",*(pwm++));
	}
	printf("\r\n");
}
static void handle_lstime(char *argv){
	if(!sys_flag.data_recv){
		printf("didn't receive data-time!\r\n");
		return;
	}
	else{
		printf("UTC Time: %d-%d-%d_%d:%d:%d \r\n",
			vp.sys_time.year,vp.sys_time.month,vp.sys_time.day,vp.sys_time.hour,vp.sys_time.minute,vp.sys_time.second);
	}
}
static void handle_settime(char *argv){
	bool input_error=false;
	TIME_STRUCT time;
	u8 num = sscanf(argv,"%hd %hhd %hhd %hhd %hhd %hhd",&time.year,&time.month,&time.day,&time.hour,&time.minute,&time.second);
	if(num != 6) input_error=true;
	else{
		if(time.year<2020) input_error = true;
		else if(time.month>12 || time.month<1) input_error = true;
		else if(time.day>31 || time.day<1) input_error = true;
		else if(time.hour>24 || time.hour<1) input_error = true;
		else if(time.minute>60) input_error = true;
		else if(time.second>60) input_error = true;
	}
	if(input_error){
		printf("Usege: settime [year month day hour minute second]\r\n");
		return;
	}
	vp.sys_time.year = time.year;
	vp.sys_time.month = time.month;
	vp.sys_time.day = time.day;
	vp.sys_time.hour = time.hour;
	vp.sys_time.minute = time.minute;
	vp.sys_time.second = time.second;
	sys_flag.data_recv = true;
	printf("set time: %d_%d_%d-%d_%d_%d \r\n",time.year,time.month,time.day,time.hour,time.minute,time.second);
}
static void handle_lsjoy(char *argv){
	if(!sys_flag.joy_connected){
		printf("joystick not connect! \r\n");
		return;
	}
	printf("left_up:%d left_lr_%d right_ud:%d right_lr:%d button:%d \r\n",
		joystic.left_ud,joystic.left_lr,joystic.right_ud,joystic.right_lr,joystic.button);
}
static void handle_lsstatus(char *argv){
	char arm_status[10];
	char mode_status[10];
	char relay_status[10];
	char batt_status[10];
	switch((u8)sys_flag.motors_armed){
		case true: memcpy(arm_status,"Armed",10); break;
		case false: memcpy(arm_status,"Disarmed",10); break;
	}
	switch((u8)vp.control_mode){
		case STABILIZE : memcpy(mode_status,"STABILIZE",10); break;
		case ALT_HOLD : memcpy(mode_status,"ALT_HOLD",10); break;      
		case AUTO : memcpy(mode_status,"AUTO",10); break;         
		case GUIDED : memcpy(mode_status,"GUIDED",10); break;       
		case CIRCLE : memcpy(mode_status,"CIRCLE",10); break;        
		case SURFACE : memcpy(mode_status,"SURFACE",10); break;      
		case POSHOLD : memcpy(mode_status,"POSHOLD",10); break;      
		case MANUAL : memcpy(mode_status,"MANUAL",10); break;
	}
	switch((u8)get_relay_12V_status()){
		case on: memcpy(relay_status,"RELAY_ON",10); break;
		case off: memcpy(relay_status,"RELAY_OFF",10); break;
	}
	switch((u8)sys_flag.battery_voltage_low){
		case true: memcpy(batt_status,"BATT_LOW",10); break;
		case false: memcpy(batt_status,"BATT_OK",10); break;
	}
	
	printf("[status]: \r\n%s \r\n%s \r\n%s \r\n%s \r\n",arm_status,mode_status,relay_status,batt_status);
	printf("mymalloc FreeRAM: %.1f%% \r\n",100.0f-(float)my_mem_perused(SRAMIN)/10.0f);//系统剩余ram信息
}
static void handle_ttpwm(char *argv){
	if(!sys_flag.relay_12V_enable){
		printf("Motors aren't POWERED! Use 'powerup' to Power motor \r\n");
		return;
	}
	int test_num = 0;
	bool input_error = false;
	u16 pwm_test[6]={1500,1500,1500,1500,1500,1500};
	sscanf(argv,"%d",&test_num);
	if(test_num<0 || test_num>6){
		printf("Usege: ttpwm 0 [pwm1 pwm2 pwm3 pwm4 pwm5 pwm6]: Test all motor, Where pwm1-6 should be 1000~2000 \
			\r\n       ttpwm i [pwm_i]: Test motor i, Where i should be 1~6, pwm_i should be 1000~2000 \r\n");
		return;
	}
	if(test_num == 0){
		u8 num = sscanf(argv,"%d %hd %hd %hd %hd %hd %hd",&test_num,&pwm_test[0],&pwm_test[1],&pwm_test[2],&pwm_test[3],&pwm_test[4],&pwm_test[5]);
		if(num != 7) input_error=true;
		else{
			for(int i=0;i<6;i++){
				if(pwm_test[i]>2000 || pwm_test[i]<1000) 
					input_error=true;
			}
		}
		if(input_error){
			printf("Usege: ttpwm 0 [pwm1 pwm2 pwm3 pwm4 pwm5 pwm6]: Test all motor, Where pwm1-6 should be 1000~2000\r\n");
			return;
		}
		printf("motor test start! test pwm1-6: ");
		for(int i=0;i<6;i++){
			printf("%d ",pwm_test[i]);
		}
		printf("\r\n");
	}
	else{
		sscanf(argv,"%d %hd",&test_num,&pwm_test[test_num-1]);
		if(pwm_test[test_num-1]>2000 || pwm_test[test_num-1]<1000) 
			input_error=true;
		if(input_error){
			printf("Usege: ttpwm i [pwm_i]: Test motor i, Where i should be 1~6, pwm_i should be 1000~2000 \r\n");
			return;
		}
		printf("motor test start! test pwm%d: %d \r\n",test_num,pwm_test[test_num-1]);
	}
	
	sys_flag.motor_test = true;
	set_test_pwm(pwm_test);
}

static void handle_stpwm(char *argv){
	printf("stop pwm test \r\n");
	sys_flag.motor_test = false;
}
static void handle_reset(char *argv){
	printf("\r\n**************\r\n");
	printf("NVIC_SystemReset\r\n");
	printf("**************\r\n");
	gcs_send_text(MAV_SEVERITY_INFO,"NVIC_SystemReset! Jump to BootLoader");
	delay_ms(1000);
	NVIC_SystemReset();
}
static void handle_powerup(char *argv){
	printf("power up! \r\n");
	sys_flag.relay_12V_enable = true;
}
static void handle_powerdown(char *argv){
	printf("power down! \r\n");
	sys_flag.relay_12V_enable = false;
}
static void handle_arm(char *argv){
	printf("arm \r\n");
	sys_flag.motors_armed = true;
}
static void handle_disarm(char *argv){
	printf("disarm! \r\n");
	sys_flag.motors_armed = false;
}
static void handle_MANUAL(char *argv){
	if(set_mode(MANUAL)){
		printf("switch to NANUAL mode \r\n");
		return;
	}
	printf("switch to NANUAL mode Failed !\r\n");
}
static void handle_STABILIZE(char *argv){
	if(set_mode(STABILIZE)){
		printf("switch to STABILIZE mode \r\n");
		return;
	}
	printf("switch to STABILIZE mode Failed !\r\n");
}
static void handle_AUTO(char *argv){
	if(set_mode(AUTO)){
		printf("switch ro AUTO mode \r\n");
		return;
	}
	printf("switch ro AUTO mode Failed !\r\n");
}
static void handle_ALT_HOLD(char *argv){
	if(set_mode(ALT_HOLD)){
		printf("switch ro ALT_HOLD mode \r\n");
		return;
	}
	printf("switch ro ALT_HOLD mode Failed !\r\n");
}

static void jump_sdread(){
	pFunction JumpToApplication;
	uint32_t JumpAddress;
	uint32_t SDCard_Reader_ADDRESS = 0x08100000;
	INTX_DISABLE();
//	u8 res = SD_DeInit();
//	printf("sd_deinit res = %d \r\n",res);
//	SysTick->CTRL = 0;
	JumpAddress = *(__IO uint32_t*) (SDCard_Reader_ADDRESS + 4);
	/* Jump to user application */
	JumpToApplication = (pFunction) JumpAddress;
	/* Initialize user application's Stack Pointer */
	__set_MSP(*(__IO uint32_t*) SDCard_Reader_ADDRESS);
	JumpToApplication();
	while(1);
}
//跳转有问题，需要通过bootloader跳转？
static void handle_sdread(char *argv){
	jump_sdread();
//	sys_flag.jump_sdread = true;


}

//记录当前flash到sd卡
static void handle_parasave(char *argv){
	char param_file_name[]="parameters.txt";
	FRESULT res = f_open(ftemp, param_file_name, FA_CREATE_NEW | FA_WRITE); 
	if(res == FR_OK){
		printf("create %s \r\n",param_file_name);
	}
	else if(res == FR_EXIST){	//存在同名文件覆盖写
		f_unlink(param_file_name);
		f_open(ftemp, param_file_name, FA_CREATE_NEW | FA_WRITE); 
		printf("rewrite %s \r\n",param_file_name);
	}
	printf("recording to %s, please wait... \r\n",param_file_name);
	char *param = (char*)mymalloc(SRAMIN, 100);
	for(int i=0;i<fp.param_num;++i){
		sprintf(param,"%d\t%s\t",i,param_id_type_value[i].param_id);
		switch(param_id_type_value[i].param_type){
			case UINT8: sprintf(param,"%sUINT8\t%d\r\n",param,*(u8*)param_id_type_value[i].param_value);break;
			case INT8: sprintf(param,"%sINT8\t%d\r\n",param,*(s8*)param_id_type_value[i].param_value);break;
			case UINT16: sprintf(param,"%sUINT16\t%d\r\n",param,*(u16*)param_id_type_value[i].param_value);break;
			case INT16: sprintf(param,"%sINT16\t%d\r\n",param,*(s16*)param_id_type_value[i].param_value);break;
			case UINT32: sprintf(param,"%sUINT32\t%d\r\n",param,*(u32*)param_id_type_value[i].param_value);break;
			case INT32: sprintf(param,"%sINT32\t%d\r\n",param,*(s32*)param_id_type_value[i].param_value);break;
			case UINT64: sprintf(param,"%sUINT64\t%lld\r\n",param,*(uint64_t*)param_id_type_value[i].param_value);break;
			case INT64: sprintf(param,"%sINT64\t%lld\r\n",param,*(int64_t*)param_id_type_value[i].param_value);break;
			case REAL32: sprintf(param,"%sREAL32\t%.4f\r\n",param,*(float*)param_id_type_value[i].param_value);break;
			case REAL64: sprintf(param,"%sREAL64\t%.4f\r\n",param,*(double*)param_id_type_value[i].param_value);break;
		}
		f_lseek(ftemp,f_size(ftemp));
		f_write(ftemp, param, strlen(param), &bw);
	}

	f_close(ftemp);
	printf("recording finish! \r\n");
}
static void handle_paraset(char *argv){
	printf("reset parameters to default value ! \r\n");
	//[TODO] 记录当前参数值到sd卡
	flash_write_core();
}
//显示任务个数
static void handle_lswpcnt(char *argv){
	printf("mission num: %d \r\n",vp.waypoint_count);
}
//显示单个航点的信息或两个经纬度坐标之间的距离和方向
static void handle_lswp(char *argv){
	s16 wp1_index,wp2_index;
	u8 res = sscanf(argv,"%hd %hd",&wp1_index,&wp2_index);
	if(res == 1){	//显示单个航点信息
		if(wp1_index == -1){		//显示当前位置到下个目标航点的信息
			Mission_Command cmd;
			cmd.content.location.lat = Sensor_latest.position.lat;
			cmd.content.location.lng = Sensor_latest.position.lng;
			vec3f wp1, wp2;
			pv_location_to_vector(wp_nav.destination, wp1);	
			pv_location_to_vector(cmd.content.location, wp2);
			float wp_distance = pv_get_horizontal_distance_cm(wp1, wp2);
			float wp_bearing = pv_get_bearing_cd(wp1, wp2);
			
			printf("realtime information to next waypoint: distance=%.2fm bearing=%.2f° \r\n",wp_distance/100,wp_bearing/100);
			return;
		
		}
		if( wp1_index<0 || wp1_index>=vp.waypoint_count){
			printf("input error! Usege: \"lswp [i] \": list the information of waypoint i, where 0<=i<wp_count \r\n");
			return;
		}
		
		Mission_Command cmd;
		if(!mission_cmd_from_bin(wp1_index,&cmd)){
			printf("read %d th mission error! \r\n",wp1_index);
			return;
		}	
		printf("waypoint %d: lat=%f lon=%f  alt=%.2f \r\n",wp1_index,cmd.content.location.lat/1e7f,cmd.content.location.lng/1e7f,cmd.content.location.alt/100.0f);
	}
	else if(res == 2){	//显示两个航点的距离和角度
		if( wp1_index<0 || wp1_index>=vp.waypoint_count || wp2_index<0 || wp2_index>=vp.waypoint_count){
			printf("input error! Usege: \"lswp [i] [j]\": list distance and bearing from waypoint i to j,where 0<=i,j<wp_count \r\n");
			return;
		}
		
		Mission_Command cmd1,cmd2;
		if(!mission_cmd_from_bin(wp1_index,&cmd1)){
			printf("read %d th mission error! \r\n",wp1_index);
			return;
		}
		if(!mission_cmd_from_bin(wp2_index,&cmd2)){
			printf("read %d th mission error! \r\n",wp2_index);
			return;
		}
		vec3f wp1, wp2;
		pv_location_to_vector(cmd1.content.location, wp1);	
		pv_location_to_vector(cmd2.content.location, wp2);
		float wp_distance = pv_get_horizontal_distance_cm(wp1, wp2);
		float wp_bearing = pv_get_bearing_cd(wp1, wp2);
		
		printf("waypoint from %d to %d: distance=%.2fm bearing=%.2f° \r\n",wp1_index,wp2_index,wp_distance/100,wp_bearing/100);

	}
	else{
		printf("input error! Usege: \r\n\t\"lswp [i] \": list the information of waypoint i, where 0<=i<wp_count \r\n");
		printf("\t\"lswp [i] [j]\": list distance and bearing from waypoint i to j,where 0<=i,j<wp_count \r\n");
	}
	
}

static void handle_lsdatesize(char *argv){
	printf("datesize: latest-%d bytes, max-%d bytes \r\n",vp.data_size_one_second_latest,vp.data_size_one_second_max);
}
//查看flash参数
static void handle_lsparams(char *argv){
	printf("=================================\r\n");
	printf("index\tname\ttype\tvalue\r\n");
	for(int i=0;i<fp.param_num;++i){
		printf("%d\t%s\t",i,param_id_type_value[i].param_id);
		switch(param_id_type_value[i].param_type){
			case UINT8: printf("UINT8\t%d\r\n",*(u8*)param_id_type_value[i].param_value);break;
			case INT8: printf("INT8\t%d\r\n",*(s8*)param_id_type_value[i].param_value);break;
			case UINT16: printf("UINT16\t%d\r\n",*(u16*)param_id_type_value[i].param_value);break;
			case INT16: printf("INT16\t%d\r\n",*(s16*)param_id_type_value[i].param_value);break;
			case UINT32: printf("UINT32\t%d\r\n",*(u32*)param_id_type_value[i].param_value);break;
			case INT32: printf("INT32\t%d\r\n",*(s32*)param_id_type_value[i].param_value);break;
			case UINT64: printf("UINT64\t%lld\r\n",*(uint64_t*)param_id_type_value[i].param_value);break;
			case INT64: printf("INT64\t%lld\r\n",*(int64_t*)param_id_type_value[i].param_value);break;
			case REAL32: printf("REAL32\t%.4f\r\n",*(float*)param_id_type_value[i].param_value);break;
			case REAL64: printf("REAL64\t%.4f\r\n",*(double*)param_id_type_value[i].param_value);break;
		}
	}
	printf("=================================\r\n");
}
//修改flash参数 -> 防止有时无法通过QGC修改
static void handle_setparam(char *argv){
	int param_index = 0;
	char param_value[20];
	u8 res = sscanf(argv,"%d %s",&param_index,param_value);
	if(res!=2 || param_index > fp.param_num){
		printf("input error! Usege: \"setparam [i] [v]\": set [i]th flash parameter value to [v],where 0<=i<param_num \r\n");
		return;
	}
	switch(param_id_type_value[param_index].param_type){
		case UINT8:
			sscanf(param_value,"%hhd",(u8*)param_id_type_value[param_index].param_value);
			flash_write_param((u8*)param_id_type_value[param_index].param_value,param_id_type_value[param_index].param_list);break;
		case INT8:
			sscanf(param_value,"%hhd",(s8*)param_id_type_value[param_index].param_value);
			flash_write_param((u8*)param_id_type_value[param_index].param_value,param_id_type_value[param_index].param_list);break;
		case UINT16:
			sscanf(param_value,"%hd",(u16*)param_id_type_value[param_index].param_value);
			flash_write_param((u8*)param_id_type_value[param_index].param_value,param_id_type_value[param_index].param_list);break;
		case INT16:
			sscanf(param_value,"%hd",(s16*)param_id_type_value[param_index].param_value);
			flash_write_param((u8*)param_id_type_value[param_index].param_value,param_id_type_value[param_index].param_list);break;
		case UINT32:
			sscanf(param_value,"%d",(u32*)param_id_type_value[param_index].param_value);
			flash_write_param((u8*)param_id_type_value[param_index].param_value,param_id_type_value[param_index].param_list);break;
		case INT32:
			sscanf(param_value,"%d",(s32*)param_id_type_value[param_index].param_value);
			flash_write_param((u8*)param_id_type_value[param_index].param_value,param_id_type_value[param_index].param_list);break;
		case UINT64:
			sscanf(param_value,"%lld",(uint64_t*)param_id_type_value[param_index].param_value);
			flash_write_param((u8*)param_id_type_value[param_index].param_value,param_id_type_value[param_index].param_list);break;
		case INT64:
			sscanf(param_value,"%lld",(int64_t*)param_id_type_value[param_index].param_value);
			flash_write_param((u8*)param_id_type_value[param_index].param_value,param_id_type_value[param_index].param_list);break;
		case REAL32:
			sscanf(param_value,"%f",(float*)param_id_type_value[param_index].param_value);
			flash_write_param((u8*)param_id_type_value[param_index].param_value,param_id_type_value[param_index].param_list);break;
		case REAL64:
			sscanf(param_value,"%lf",(double*)param_id_type_value[param_index].param_value);
			flash_write_param((u8*)param_id_type_value[param_index].param_value,param_id_type_value[param_index].param_list);break;
	}
	printf("change \"%s\" value to %s \r\n",param_id_type_value[param_index].param_id,param_value);
}
//显示mavlink数据流
static void handle_showmavlink(char *argv){
	int flag;
	u8 res = sscanf(argv,"%d",&flag);
	switch(flag){
		case 0:
			sys_flag.show_mavlink_stream1 = false;
			sys_flag.show_mavlink_stream2 = false;
			printf("[showmav] stop show mavlink datestream\r\n");
			break;
		case 1:
			sys_flag.show_mavlink_stream1 = true;
			printf("[showmav] show mavlink abstract\r\n");
			break;
		case 2:
			printf("[showmav] show mavlink datestream\r\n");
			sys_flag.show_mavlink_stream2 = true;
			break;
		default:printf("[showmav] input error! Usege: \"showmavlink [i] \": i=0-hide mavlink datestream; i=1-show mavlink abstract; i=2-show mavlink datestream \r\n");break;
	}
}
//显示mavlink接收成功率
static void handle_lscomm(char *argv){
	printf("mavlink receive successful pencentage: %d%%\r\n",vp.mavlink_recv_percentage);
}
//以mavlink发送flash全部参数
static void handle_send_allparams(char *argv){
	gcs_send_allparams();
}
//显示片内SRAM利用率
static void handle_lssramin(char *argv){
	argv = argv;
	printf("SRAMIN total_size=%dKB, avaiable_latest=%.1f%%, avaiable_min=%.1f%% \r\n",
		MEM1_MAX_SIZE/1024,vp.sramin_available_latest,vp.sramin_available_min);
	
}
//显示各传感器解析频率
static void handle_ls_sensorparse(char *argv){
	char name[5]="";
	u8 res = sscanf(argv,"%s",name);
 	if(strcmp(name,"ec") == 0) printf("EC parse fre= %d Hz\r\n",vp.ec_parse_num_sencond);
	else if(strcmp(name,"imu") == 0) printf("IMU parse fre= %d Hz\r\n",vp.imu_parse_num_sencond);
	else if(strcmp(name,"gps") == 0) printf("gps parse fre= %d Hz\r\n",vp.gps_parse_num_sencond);
	else{
		printf("input error! Usege: \"lsparse [sensor] \": sensor=tcm/mpu/gps \r\n");
	}
	
}
//设置定向角度
static void handle_setheading(char *argv){
	float heading;
	u8 set;
	u8 res = sscanf(argv,"%f %hhd",&heading,&set);
	if(res != 2){
		printf("input error! Usege: \"setheading [v] [i]\": v=heading, shoule be 0<=v<=360; i=0-cancel heading, i=1-set hold_heading [v]\r\n");
		return;
	}
	switch(set){
		case 0:
			printf("取消屏幕定向 \r\n");
			sys_flag.screen_heading = false;
			break;
		case 1:
			if(heading>360 || heading<0){
				printf("input error! Usege: \"setheading [v] [i]\": v=heading, shoule be 0<=v<=360; i=0-cancel heading, i=1-set hold_heading [v]\r\n");
				return;
			}
			vp.screen_heading = heading;
			sys_flag.screen_heading = true;
			printf("开启屏幕定向，定向角度 %d \r\n",vp.screen_heading);
			break;
		default:
			printf("input error! Usege: \"setheading [v] [i]\": v=heading; i=0-cancel heading, i=1-set hold_heading [v]\r\n");
			return;
	}
}
//static void handle_attpid(char *argv){
//	
//}
//显示socket状态
static void handle_lssocsta(char *argv){
	char status[20]="";
	switch(vp.sock_status){
		case 0x00:sprintf(status,"%s","SOCK_CLOSED");break;
		case 0x13:sprintf(status,"%s","SOCK_INIT");break;
		case 0x14:sprintf(status,"%s","SOCK_LISTEN");break;
		case 0x15:sprintf(status,"%s","SOCK_SYNSENT");break;
		case 0x16:sprintf(status,"%s","SOCK_SYNRECV");break;
		case 0x17:sprintf(status,"%s","SOCK_ESTABLISHED");break;
		case 0x18:sprintf(status,"%s","SOCK_FIN_WAIT");break;
		case 0x1a:sprintf(status,"%s","SOCK_CLOSING");break;
		case 0x1b:sprintf(status,"%s","SOCK_TIME_WAIT");break;
		case 0x1c:sprintf(status,"%s","SOCK_CLOSE_WAIT");break;
		case 0x1d:sprintf(status,"%s","SOCK_LAST_ACK");break;
		case 0x22:sprintf(status,"%s","SOCK_UDP");break;
		case 0x32:sprintf(status,"%s","SOCK_IPRAW");break;
		case 0x42:sprintf(status,"%s","SOCK_MACRAW");break;
		case 0x5f:sprintf(status,"%s","SOCK_PPPOE");break;
	}
	printf("[lssocsta] socket status : %s \r\n",status);
}
//显示gps数据流
static void handle_showgps(char *argv){
	int flag;
	u8 res = sscanf(argv,"%d",&flag);
	switch(flag){
		case 0:
			sys_flag.show_gps_infor1 = false;
			sys_flag.show_gps_infor2 = false;
			printf("[showgps] stop show GPS datestream\r\n");
			break;
		case 1:
			sys_flag.show_gps_infor1 = true;
			sys_flag.show_gps_infor2 = false;
			printf("[showgps] show GPS abstract\r\n");
			break;
		case 2:
			printf("[showgps] show GPS datestream\r\n");
			sys_flag.show_gps_infor1 = false;
			sys_flag.show_gps_infor2 = true;
			break;
		default:printf("[showgps] input error! Usege: \"showgps [i] \": i=0-hide GPS datestream; i=1-show GPS raw message; i=2-show GPS parse message \r\n");break;
	}
}
//九轴kf验证
static void handle_kfverify(char *argv){
	u8 flag, sec;
	u8 res = sscanf(argv,"%hhd %hhd",&flag,&sec);
	switch(flag){
		case 0:
			sys_flag.kf_verify = false;
			printf("[kfverify] stop kfverify\r\n");
			break;
		case 1:
			sys_flag.kf_verify = true;
			vp.kf_verify_sec = sec;
			printf("[kfverify] start kfverify, duration: %ds!\r\n",sec);
			break;
		default:printf("[kfverify] input error! Usege: \"kfverify [i] [j]\": i=0-stop kfverify; i=1-start kfverify, j-verify duration(s); \r\n");break;
	}
}
//kf DCM测试
static void handle_DCM_update(char *argv){
	vec3f att, v1_data, v2_data;
	mat3f R_data;
	arm_matrix_instance_f32 R, v1, v2;
	set2vecf(O31,3,v1_data,&v1);
	set2vecf(O31,3,v2_data,&v2);
	u8 res = sscanf(argv,"%f %f %f %f %f %f",&att[0],&att[1],&att[2],&v1_data[0],&v1_data[1],&v1_data[2]);
	if(res != 6){
		printf("input error! Usege: \"DCMupdate \r\n");
		return;
	}
	DCM_update(att, R_data,&R);
	arm_mat_mult_f32(&R,&v1,&v2);
	print_mat_name(v1, "v1");
	print_mat_name(R, "R");
	print_mat_name(v2, "v2");
}
//kf测试
static void handle_kftest(char *argv){
	u8 sec, type;
	u8 res = sscanf(argv,"%hhd %hhd",&sec,&type);
	if(res != 2){
		printf("input error! Usege: \"kftest [i] [j]\": [i]-test duration(s); [j]-test type\r\n");
		return;
	}
	kF_test(sec, (KF_TEST_TYPE)type);
}
//显示被跳过的子任务
static void handle_showpass(char *argv){
	int flag;
	u8 res = sscanf(argv,"%d",&flag);
	if(res != 1){
		printf("input error! Usege: \"showpass [i]\": i=0-stop show task pass information, i=1-start show task pass information\r\n");
		return;
	}
	switch(flag){
		case 0:
			sys_flag.show_task_pass = false;
			printf("[showpass] stop show task pass information \r\n");
			break;
		case 1:
			sys_flag.show_task_pass = true;
			printf("[showpass] start show task pass information \r\n");
			break;
		default:
			printf("input error! Usege: \"showpass [i]\": i=0-stop show task pass information, i=1-start show task pass information\r\n");
			break;
	}
}
static void handle_showwp(char *argv){
	int flag;
	u8 res = sscanf(argv,"%d",&flag);
	if(res != 1){
		printf("input error! Usege: \"showwp [i]\": i=0-stop send waypoint information to GCS, i=1-start send waypoint information to GCS\r\n");
		return;
	}
	switch(flag){
		case 0:
			sys_flag.GCS_show_WP_infor = false;
			printf("[showwp] stop send waypoint information to GCS \r\n");
			break;
		case 1:
			sys_flag.GCS_show_WP_infor = true;
			printf("[showwp] start send waypoint information to GCS \r\n");
			break;
		default:
			printf("input error! Usege: \"showwp [i]\": i=0-stop send waypoint information to GCS, i=1-start send waypoint information to GCS\r\n");
			break;
	}
}
//动力学模型验证model verify
bool check_force(float force){
	bool res = true;
	return res;
}
static void handle_modelver(char *argv){
	int flag,sec;
	float force[6];
	u8 res = sscanf(argv,"%d %d %f %f %f %f %f %f",&flag, &sec, &force[0], &force[1], &force[2], &force[3], &force[4], &force[5]);
	if(res != 8){
		printf("input error! Usege: \"modelver [i] [j] [f1]~[f6]\": [i]=0-stop verify model, [i]=1-start verify model; [j]-test duration(s); [f1~f6]-motors force\r\n");
		return;
	}
	switch(flag){
		case 0:
			sys_flag.model_verify = false;
			printf("[modelver] stop verify model \r\n");
			break;
		case 1:
			//判断输入的力是否正确
			for(int i=0;i<6;++i){
				if(!check_force(force[i])){
					sys_flag.model_verify = false;
					printf("force %d input is ERROR! \r\n",i+1);
					return;
				}
			}
			for(int i=0;i<6;++i){
				set_motor_force(i,force[i]);
			}
			sys_flag.model_verify = true;
			vp.model_test_sec = sec;
			printf("[modelver] start verify model \r\n");
			break;
		default:
			printf("input error! Usege: \"modelver [i] [j] [f1]~[f6]\": [i]=0-stop verify model, [i]=1-start verify model; [j]-test duration(s); [f1~f6]-motors force\r\n");
			break;
	}	
}
//模型更新测试
static void handle_modeltest(char *argv){
	u8 sec, type;
	u8 res = sscanf(argv,"%hhd %hhd",&sec,&type);
	if(res != 2){
		printf("input error! Usege: \"modeltest [i] [j]\": [i]-test duration(s); [j]-test type\r\n");
		return;
	}
	model_test(sec, (MODEL_TEST_TYPE)type);
}

//测试pwm转化成力
static void handle_pwm2force(char *argv){
	int type;
	u16 pwm;
	int res = sscanf(argv,"%d %hd",&type,&pwm);
	if(res != 2){
		printf("input error! Usege: \"pwm2force [i] [j]\": [i]-type; [j]-pwm\r\n");
		return;
	}
	printf("[pwm2force] force=%.5fN \r\n",input_pwm_to_force(type, pwm));
}
//测试力转化成pwm
static void handle_force2pwm(char *argv){
	int type;
	float force;
	int res = sscanf(argv,"%d %f",&type,&force);
	if(res != 2){
		printf("input error! Usege: \"force2pwm [i] [j]\": [i]-type; [j]-force\r\n");
		return;
	}
	printf("[force2pwm] pwm=%d \r\n",input_force_to_pwm(type, force));
}
// 运行测试函数
static void handle_runtest(char *argv){
	if(vp.run_test_enable){
		printf("正在运行测试中...\r\n");
		return;
	}
	char mode_char[10];	//模式
	int minute,num;	//运行分钟，运行次数
	int mode=-1;
	int res = sscanf(argv,"%s %d %d",mode_char,&minute,&num);
	if(res != 3){
		printf("input error! Usege: \"runtest [mode] [i] [j]\": [i]-minute; [j]-num\r\n");
		return;
	}
	for(int i=0;i<11;i++){
		if(strcmp(mode_char,control_mode_char[i].mode) == 0){
			mode = control_mode_char[i].mode_type;
			memcpy(vp.run_test_mode_char,control_mode_char[i].mode,10);
		}
	}
	if(mode==-1){
		printf("输入模式不存在! \r\n");
		return;
	}
	//TODO判断运行分钟和次数的有效性
	vp.run_test_minute = minute;
	vp.run_test_mode = mode;
	vp.run_test_num = num;
	vp.run_test_enable = true;
	printf("================================================\r\n");
	printf("开始模式%s运行测试，单次测试时常%dmin，测试%d次 \r\n",vp.run_test_mode_char,vp.run_test_minute,vp.run_test_num);
	printf("================================================\r\n");
}

//输出串口数据流
static void handle_showUART(char *argv){
	int uart,show;
	int res = sscanf(argv,"%d %d",&uart,&show);
	if(res != 2){
		printf("input error! Usege: \"showUART [i] [j]\": [i]-; [j]-\r\n");
		return;
	}
	switch(uart){
		case 1: sys_flag.uart1_showrecv = (show==0)?false:true; break;
		case 2: sys_flag.uart2_showrecv = (show==0)?false:true; break;
		case 3: sys_flag.uart3_showrecv = (show==0)?false:true; break;
		case 6: sys_flag.uart6_showrecv = (show==0)?false:true; break;
		case 7: sys_flag.uart7_showrecv = (show==0)?false:true; break;
		case 8: sys_flag.uart8_showrecv = (show==0)?false:true; break;
	}
	if(show) printf("[showUART] 开启显示串口%d的接收数据流 \r\n",uart);
	else printf("[showUART] 关闭显示串口%d的接收数据流 \r\n",uart);
}

//串口透传模式 -> 用于传感器校准
//注意：进入透传模式之后，程序进入死循环，之后复位才能退出
static void handle_UARTTC(char *argv){
	int uart;
	int res = sscanf(argv,"%d",&uart);
	if(res != 1){
		printf("input error! Usege: \"UARTTC [i] \": [i]-;\r\n");
		return;
	}
	switch(uart){
		case 1: sys_flag.uart1_dictect_trans = true; break;
		case 2: sys_flag.uart2_dictect_trans = true; break;
		case 3: sys_flag.uart3_dictect_trans = true; break;
		case 7: sys_flag.uart7_dictect_trans = true; break;
		case 8: sys_flag.uart8_dictect_trans = true; break;
	}
	printf("[UARTTC] 开启串口%d透明传输 \r\n",uart);
	while(1);
}

/*
调试端口初始化
*/
void console_init(const Task *tasks,u8 num_tasks){
	task_list_copy = tasks;
	task_num = num_tasks;
}
/*调试解析指令*/
static void console_parse(char *recv_buf){
	char com[10] = {0};
	if(sscanf(recv_buf,"%s ",com)) {
		for(int i=0;i<sizeof(command_list)/sizeof(command_list[0]);i++){
			if(strcmp(com,command_list[i].command) == 0){
				printf("Console>> %s \r\n",command_list[i].command);
				char *argv = strchr(recv_buf,' ');	//查找' '第一次出现的地址，此后的数据当作参数看待 
				(*(command_list[i].Function))(argv);
				return;
			}
		}
	}
	printf("without this Command! use \"lscmd\" to check all commands \r\n");
}

void console_debug(){
	static bool console_enable = false;
	if(!(CONSOLE_uart_available&0x8000)){ 					 
		if(!console_enable){
			static u8 loop_num = 0;
			loop_num++;
			printf("one hz loop %d \r\n",loop_num);
			if(loop_num == 100) loop_num = 0;	
		}
		return;
	}
//	u8 len=CONSOLE_uart_available&0x3fff;//得到此次接收到的数据长度
	if(strcmp((char*)CONSOLE_uart_buf,"console")==0){
		console_enable = true;
	}
	else if(strcmp((char*)CONSOLE_uart_buf,"quit")==0){
		console_enable = false;
		printf("Console>> quit debug!\r\n");
	}
	if(console_enable){
		console_parse((char *)CONSOLE_uart_buf);
	}
	CONSOLE_uart_available=0;
	memset(CONSOLE_uart_buf,0,sizeof(CONSOLE_uart_buf)/sizeof(char));
}

