#include <string.h>
#include "Logger.h"
#include "exfuns.h"
#include "config.h"
#include "sensor.h"
#if defined(USE_RTTHREAD)
#include <entry.h>
#endif
#include "attitude_controller.h"
#include "position_controller.h"
#include "GCS_Mavlink.h"
#include "kf.h"
#include "mission.h"
#include "model.h"

//#define TEST

bool wp_write_flag=false; //航点数据记录标志
char LOG_FILE_NAME[100];

//#define TIME (sys_time.ten_micro*10-vp.log_start_time)
#define TIME (sys_time.ten_micro/100000.0f-vp.log_start_time)

/*
log_structure结构体用于存储解码格式，即mid对应的数据大类名称、数据类型、每个数据类型对应的数据名称
例如：
mid为LOG_PIDW1_MSG的是PID1数据大类，数据组成是Qffffffffff，其中Q-TimeUS，第一个f-Tar，第二个f-Act。。。
上位机只有首先接收到以下的解码信息才能在后续正确解析bin文件。
注意：每类文件的LABLES里面不能用"index"作为数据成员的描述！
*/
	
/*
Format characters in the format string for binary log messages
	a   : int16_t[32]
	b   : int8_t
	B   : uint8_t
	h   : int16_t
	H   : uint16_t
	i   : int32_t
	I   : uint32_t
	f   : float
	d   : double
	n   : char[4]
	N   : char[16]
	Z   : char[64]
	c   : int16_t * 100
	C   : uint16_t * 100
	e   : int32_t * 100
	E   : uint32_t * 100
	L   : int32_t latitude/longitude
	M   : uint8_t flight mode
	q   : int64_t
	Q   : uint64_t
*/

const struct LogStructure log_structure[] = {
	{ LOG_FORMAT_MSG, sizeof(struct log_Format), "FMT", FORMAT_FMT, FORMAT_LABELS}, 
    { LOG_TEST_MSG, sizeof(struct log_TEST),"TEST", TEST_FMT, TEST_LABELS},
    { LOG_PARAMETER_MSG, sizeof(struct log_Parameter), "PARM", PARAMETER_FMT, PARAMETER_LABELS},    
    { LOG_GPS_MSG, sizeof(struct log_GPS), "GPS",  GPS_FMT, GPS_LABELS }, 
    { LOG_IMU_MSG, sizeof(struct log_IMU), "IMU",  IMU_FMT,     IMU_LABELS }, 
    { LOG_RCIN_MSG, sizeof(struct log_RCIN), "RCIN",  RCIN_FMT, RCIN_LABELS}, 
    { LOG_RCOUT_MSG, sizeof(struct log_RCOUT), "RCOU", RCOUT_FMT , RCOUT_LABELS}, 
    { LOG_BARO_MSG, sizeof(struct log_BARO), "BARO",  BARO_FMT, BARO_LABELS }, 
    { LOG_POWR_MSG, sizeof(struct log_POWR), "POWR", POWER_FMT, POWER_LABELS },
    { LOG_CMD_MSG, sizeof(struct log_Cmd), "CMD", CMD_FMT, CMD_LABELS}, 
	{ LOG_ATTITUDE_MSG, sizeof(struct log_Attitude),"ATT", ATTITUDE_FMT, ATTITUDE_LABELS}, 
    { LOG_MODE_MSG, sizeof(struct log_Mode), "MODE", MODE_FMT, MODE_LABELS}, 
   	{ LOG_PID_ANG_RLL_MSG, sizeof(struct log_PID), "ACAR", PID_FMT, PID_LABELS}, 
	{ LOG_PID_ANG_PIT_MSG, sizeof(struct log_PID), "ACAP", PID_FMT, PID_LABELS}, 
	{ LOG_PID_ANG_YAW_MSG, sizeof(struct log_PID), "ACAY", PID_FMT, PID_LABELS}, 
	{ LOG_PID_RAT_RLL_MSG, sizeof(struct log_PID), "ACRR", PID_FMT, PID_LABELS}, 
	{ LOG_PID_RAT_PIT_MSG, sizeof(struct log_PID), "ACRP", PID_FMT, PID_LABELS}, 
	{ LOG_PID_RAT_YAW_MSG, sizeof(struct log_PID), "ACRY", PID_FMT, PID_LABELS}, 
	{ LOG_PID_ACC_Z_MSG, sizeof(struct log_PID), "PCAZ", PID_FMT, PID_LABELS}, 
	{ LOG_PID_VEL_Z_MSG, sizeof(struct log_PID), "PCVZ", PID_FMT, PID_LABELS}, 
	{ LOG_PID_POS_Z_MSG, sizeof(struct log_PID), "PCPZ", PID_FMT, PID_LABELS}, 	
	{ LOG_WP_MSG, sizeof(struct log_WP), "WP", WP_FMT, WP_LABELS}, 
	{ LOG_DR_MSG, sizeof(struct log_DR), "DR", DR_FMT, DR_LABELS}, 
	{ LOG_SINS_MSG, sizeof(struct log_DR), "SINS", DR_FMT, DR_LABELS}, 
	{ LOG_MODEL_MSG, sizeof(struct log_MODEL), "MD", MODEL_FMT, MODEL_LABELS}, 
};

uint8_t WriteBlock(const void *pBuffer, uint16_t size)
{
//	u32 totalSpace,freeSpace;
  /* Mount SD Card */
//  if(f_mount(fs[0], "0:", 0) != FR_OK)
//    return 1;

  /* Open file to write */
//  if(f_open(file, LOG_FILE_NAME, FA_OPEN_APPEND | FA_WRITE) != FR_OK)
//	if(f_open(file, LOG_FILE_NAME, FA_OPEN_EXISTING | FA_WRITE) != FR_OK)
//    return 2;

  /* Check freeSpace space */
//  if(exf_getfree((u8 *)"0:",&totalSpace,&freeSpace) != FR_OK)
//    return 3;

  /* free space is less than 1kb */
//  if(freeSpace < 1)
//    return 4;

	//查找文件的结尾
	f_lseek(file,f_size(file));
	
  /* Writing*/
  f_write (file, pBuffer, size, &bw);


  /* Close file */
//  if(f_close(file) != FR_OK)
//    return 5;

  /* Unmount SDCARD */
//  if(f_mount(NULL, "0:", 1) != FR_OK)
//    return 6;

  return 0;
}

void Fill_Format(const struct LogStructure *s, struct log_Format *pkt)
{
  memset(pkt, 0, sizeof(*pkt));
  pkt->head1 = HEAD_BYTE1;
  pkt->head2 = HEAD_BYTE2;
  pkt->msgid = LOG_FORMAT_MSG;
  pkt->type = s->msg_type;
  pkt->length = s->msg_len;
  strncpy(pkt->name, s->name, sizeof(pkt->name));
  strncpy(pkt->format, s->format, sizeof(pkt->format));
  strncpy(pkt->labels, s->labels, sizeof(pkt->labels));
}

/*
  write a structure format to the log
 */
uint8_t Write_Format(const struct LogStructure *s)
{
  struct log_Format pkt;

  Fill_Format(s, &pkt);

  return WriteBlock(&pkt, sizeof(pkt));
}

//检查头部格式是否符合要求
bool Check_Format(const struct LogStructure *s){
//	u8  name_len, format_len, labels_len;
//	char *temp;
//	temp = s->format;
//	while()
	return true;	
}


void Log_Init(void)
{
#ifdef TEST
	sprintf(LOG_FILE_NAME,"log_%04d-%02d-%02d_%02d-%02d-%02d.txt",
			vp.sys_time.year,vp.sys_time.month,vp.sys_time.day,vp.sys_time.hour,vp.sys_time.minute,vp.sys_time.second);
#else
	sprintf(LOG_FILE_NAME,"log_%04d-%02d-%02d_%02d-%02d-%02d.bin",
		vp.sys_time.year,vp.sys_time.month,vp.sys_time.day,vp.sys_time.hour,vp.sys_time.minute,vp.sys_time.second);
#endif
	uint8_t i;
	//  if(f_mount(&fs, "0:", 0) != FR_OK)
	//    return;
	f_open(file, LOG_FILE_NAME, FA_CREATE_NEW | FA_WRITE); 
//	if(res == FR_OK){
//		f_close(file);
//	} else if(res == FR_EXIST){	//存在同名文件覆盖写
//		f_unlink(LOG_FILE_NAME);
//		f_open(file, LOG_FILE_NAME, FA_CREATE_NEW | FA_WRITE); 
//	}
	//卸载SD卡
//	if(f_mount(NULL, "0:", 1) != FR_OK)
//	return;

	//初始化时，将解码格式写入文件首部
#ifndef TEST
	for(i = 0; i < ARRAY_SIZE(log_structure); i++) {
		if(Check_Format(&log_structure[i])) Write_Format(&log_structure[i]);
	}
#else
	char header[]="time,roll,pitch,yaw,accx,accy,accz,gyrox,gyroy,gyroz,depth,power,pwm1,pwm2,pwm3,pwm4\r\n";
	f_lseek(file,f_size(file));
	f_write (file, header, strlen(header), &bw);
#endif
	sys_flag.log_creat = true;
	wp_write_flag = false;	//重置航点记录标志
	printf("[log] %s created \r\n",LOG_FILE_NAME);
	char text[50]="";
	sprintf(text,"[Log] %s created!",LOG_FILE_NAME);
	gcs_send_text(MAV_SEVERITY_INFO, text);
}

void Log_create(){
	if(sys_flag.log_creat){	//已经创建日志文件，检测是否需要结束当前记录再创建新的
		if(fp.log_mode == LOG_Powered){}	//上电记录方式
		else{
			if(sys_flag.motors_armed){}	//解锁记录方式-电机解锁
			else{	////解锁记录方式-电机上锁
				f_close(file);
				sys_flag.log_creat = false;
				printf("[Log] %s saved! \r\n",LOG_FILE_NAME);
				char text[50]="";
				sprintf(text,"[Log] %s saved!",LOG_FILE_NAME);
				gcs_send_text(MAV_SEVERITY_INFO, text);
				
			} 	
		}
	}
	else if(fp.log_mode==LOG_Powered || sys_flag.motors_armed){	//上锁记录模式 或者 电机解锁记录
		Log_Init();
		vp.log_start_time = sys_time.ten_micro/100000.0f; //sys_time.one_second+sys_time.one_mill/1000.0f+

	}
}

//测试函数
static void Log_Write_Test()
{
	static int num = 0;
	num++;
	if(num==100) num=0;
	struct log_TEST pkt = {
	LOG_PACKET_HEADER_INIT(LOG_TEST_MSG),
	.time_s = TIME,
	.value = num,
	};
	WriteBlock(&pkt, sizeof(pkt));
}

/*
//  write a parameter to the log
// */
//bool Log_Write_Parameter(const char *name, float value)
//{
//    struct log_Parameter pkt = {
//        LOG_PACKET_HEADER_INIT(LOG_PARAMETER_MSG),
//        time_us : AP_HAL::micros64(),
//        name  : {},
//        value : value
//    };
//    strncpy(pkt.name, name, sizeof(pkt.name));
//    return WriteBlock(&pkt, sizeof(pkt));
//}

// Write an GPS packet
static void Log_Write_GPS()
{
	if(!sys_flag.gps_log_update) return;
	sys_flag.gps_log_update = false;
    struct log_GPS pkt = {
        LOG_PACKET_HEADER_INIT(LOG_GPS_MSG),
        .time_s = TIME,
		.status	= Sensor_latest.gps.gpssta,	//定位状态
	    .num_sats = Sensor_latest.gps.posslnum,
	    .hdop = Sensor_latest.gps.hdop,
	    .latitude = (int32_t)(Sensor_latest.gps.latitude*1e7f),//
	    .longitude = (int32_t)(Sensor_latest.gps.longitude*1e7f),
	    .altitude = (int32_t)Sensor_latest.gps.altitude ,
	    .ground_speed = Sensor_latest.gps.speed,
	    .ground_course = Sensor_latest.gps.orient,
	    .vE = Sensor_latest.gps.speed*sin(Sensor_latest.ins.euler_rad[2]),
	    .vN = Sensor_latest.gps.speed*cos(Sensor_latest.ins.euler_rad[2]),
    };
    WriteBlock(&pkt, sizeof(pkt));
}

// Write an raw accel/gyro data packet
static void Log_Write_IMU()
{
    u16 *chan = get_motorout();
    struct log_IMU pkt = {
		LOG_PACKET_HEADER_INIT(LOG_IMU_MSG),
		.time_s = TIME,
		.gyro[0] = Sensor_latest.ins.gyro[0], 
		.gyro[1] = Sensor_latest.ins.gyro[1],
		.gyro[2] = Sensor_latest.ins.gyro[2],
		.accel[0] = Sensor_latest.ins.acc[0], 
		.accel[1] = Sensor_latest.ins.acc[1], 
		.accel[2] = Sensor_latest.ins.acc[2],
		.euler[0] = Sensor_latest.ins.euler[0],
		.euler[1] = Sensor_latest.ins.euler[1],
		.euler[2] = Sensor_latest.ins.euler[2],
//		.acc_no_g[0] = Sensor_latest.ins.acc_no_g[0],
//		.acc_no_g[1] = Sensor_latest.ins.acc_no_g[1],
//		.acc_no_g[2] = Sensor_latest.ins.acc_no_g[2],
		.pwm[0] = *(chan),
		.pwm[1] = *(++chan),
		.pwm[2] = *(++chan),
		.pwm[3] = *(++chan),
		.pwm[4] = *(++chan),
		.pwm[5] = *(++chan),
//		.gyro_error = 0, 
//		.accel_error = 0,
    };
    WriteBlock(&pkt, sizeof(pkt));

}

// Write an RCIN packet
static void Log_Write_RCIN(void)
{
	
    struct log_RCIN pkt = {
        LOG_PACKET_HEADER_INIT(LOG_RCIN_MSG),
		.time_s = TIME,
		.roll_in = get_roll(),
		.pitch_in = get_pitch(),
		.yaw_in = get_yaw(),
		.throttle_in = get_throttle(),
		.forward_in = get_forward(),
		.lateral_in = get_lateral(),
    };
    WriteBlock(&pkt, sizeof(pkt));
}

// Write an RCOUT packet
static void Log_Write_RCOUT(void)
{
	
	u16 *chan = get_motorout();
    struct log_RCOUT pkt = {
        LOG_PACKET_HEADER_INIT(LOG_RCOUT_MSG),
		.time_s = TIME,
		.chan1 = *(chan),
		.chan2 = *(++chan),
		.chan3 = *(++chan),
		.chan4 = *(++chan),
		.chan5 = *(++chan),
		.chan6 = *(++chan),
		.chan7 = *(++chan),
		.chan8 = *(++chan),
    };
    WriteBlock(&pkt, sizeof(pkt));
}

// Write a BARO packet
static void Log_Write_Baro()
{

	float baro_val, baro_val_surface;
	switch(Sensor_latest.barometer.baro_type){
		case analog_vol:
			baro_val = Sensor_latest.barometer.analog_vol ;
			baro_val_surface = Sensor_init.barometer.analog_vol;
			break;
		case analog_curr:
			baro_val = Sensor_latest.barometer.analog_curr; 
			baro_val_surface = Sensor_init.barometer.analog_curr;
			break;
		case pressure:
			baro_val = (float)Sensor_latest.barometer.pressure;
			baro_val_surface = Sensor_init.barometer.pressure;
			break;
	}
    struct log_BARO pkt = {
        LOG_PACKET_HEADER_INIT(LOG_BARO_MSG),
		.time_s = TIME,
		.depth = Sensor_latest.barometer.depth,
		.value = baro_val,
		.temperature = Sensor_latest.temp,
		.value_surface = baro_val_surface,	
    };
    WriteBlock(&pkt, sizeof(pkt));
}

static void Log_Write_Power(void)
{
	
    struct log_POWR pkt = {
        LOG_PACKET_HEADER_INIT(LOG_POWR_MSG),
        .time_s = TIME,
        .Vcc = Sensor_latest.power,
    };
    WriteBlock(&pkt, sizeof(pkt));
//	printf("%lld \r\n",pkt.time_s);
}


// Write an attitude packet
static void Log_Write_Attitude(void)
{

    struct log_Attitude pkt = {
        LOG_PACKET_HEADER_INIT(LOG_ATTITUDE_MSG),
		.time_s = TIME,
		.control_roll = Sensor_target.ins.euler[0],
		.roll = Sensor_latest.ins.euler[0],
		.control_pitch = Sensor_target.ins.euler[1],
		.pitch = Sensor_latest.ins.euler[1],
		.control_yaw = Sensor_target.ins.euler[2],
		.yaw = Sensor_latest.ins.euler[2],
		.error_roll = Sensor_target.ins.euler[0]-Sensor_latest.ins.euler[0],
		.error_pitch = Sensor_target.ins.euler[1]-Sensor_latest.ins.euler[1],
		.error_yaw = Sensor_target.ins.euler[2]-Sensor_latest.ins.euler[2],
    };
//    WriteBlock(&pkt, sizeof(pkt));
}


// Write a command processing packet
//bool Log_Write_MavCmd(uint16_t cmd_total, const mavlink_mission_item_t& mav_cmd)
//{
//    struct log_Cmd pkt = {
//        LOG_PACKET_HEADER_INIT(LOG_CMD_MSG),
//        time_us         : AP_HAL::micros64(),
//        command_total   : (uint16_t)cmd_total,
//        sequence        : (uint16_t)mav_cmd.seq,
//        command         : (uint16_t)mav_cmd.command,
//        param1          : (float)mav_cmd.param1,
//        param2          : (float)mav_cmd.param2,
//        param3          : (float)mav_cmd.param3,
//        param4          : (float)mav_cmd.param4,
//        latitude        : (float)mav_cmd.x,
//        longitude       : (float)mav_cmd.y,
//        altitude        : (float)mav_cmd.z
//    };
//    return WriteBlock(&pkt, sizeof(pkt));
//}

// Write a mode packet.
static void Log_Write_Mode()
{
    struct log_Mode pkt = {
        LOG_PACKET_HEADER_INIT(LOG_MODE_MSG),
        .time_s = TIME,
        .mode = vp.control_mode,
    };
    WriteBlock(&pkt, sizeof(pkt));
}
//记录航点
static void Log_Write_WP(){
	static int waypoint_index=-1;
	if(waypoint_index == vp.mission_item_reached_index) return;
	waypoint_index  = vp.mission_item_reached_index;
	Mission_Command cmd;
	if(!mission_cmd_from_bin(vp.mission_item_reached_index,&cmd)){
		printf("[log] record waypoint ERROR!\r\n");
		return;
	}
	struct log_WP pkt = {
		LOG_PACKET_HEADER_INIT(LOG_WP_MSG),
		.time_s = TIME,
		.nav_mode = fp.dr_mode,
        .index = cmd.index,
		.alt = cmd.content.location.alt/100.0f,
		.lat = (int)cmd.content.location.lat,///1.0e7f,
		.lng = (int)cmd.content.location.lng,///1.0e7f,
	};
	WriteBlock(&pkt, sizeof(pkt));
//	printf("%.2f %f %f \r\n",pkt.alt,pkt.lat,pkt.lng);

}
//航位推算结果
static void Log_Write_DR(){
	vec3f an,vn,pos;
	switch(fp.dr_mode){
		case DR_SINS: 	//纯惯导或融合导航输出信息都是sins
		case DR_SINS_GPS:
		case DR_SINS_MA:
			arm_copy_f32(sins.an, an, 3);
			arm_copy_f32(sins.vn, vn, 3);
//			arm_copy_f32(sins.pos, pos, 3);
			break;
		case DR_GPS:
			arm_copy_f32(O31, an, 3);
			arm_copy_f32(Sensor_latest.gpsvn, vn, 3);
			arm_copy_f32(Sensor_latest.gpspos, pos, 3);
			break;
		case DR_MA:
			arm_copy_f32(O31, an, 3);
			arm_copy_f32(model.vn, vn, 3);
			arm_copy_f32(model.pos, pos, 3);
//			arm_scale_f32(pos, 1e-7f, pos, 3);
			break;
		
	}
	if(!kf.KFinit_complete) return;
	struct log_DR pkt = {
        LOG_PACKET_HEADER_INIT(LOG_DR_MSG),
        .time_s = TIME,
        .mode = fp.dr_mode,
		.ax = an[0],
		.ay = an[1],
		.az = an[2],
        .vx = vn[0],
        .vy = vn[1],
        .vz = vn[2],
        .lat = pos[0]*1e7f,
        .lng = pos[1]*1e7f,
        .alt = pos[2],
    };
	WriteBlock(&pkt, sizeof(pkt));
}

//模型验证输出
static void Log_Write_MODEL(){
	struct log_MODEL pkt = {
        LOG_PACKET_HEADER_INIT(LOG_MODEL_MSG),
        .time_s = TIME,
		.ax = model.ab[0],
		.ay = model.ab[1],
		.az = model.ab[2],
        .wx = model.X_data[9],
        .wy = model.X_data[10],
        .wz = model.X_data[11],
    };
	for(int i=0;i<6;++i) pkt.force[i] = model.U_data[i];
	WriteBlock(&pkt, sizeof(pkt));
//	printf("%.5f %.5f %.5f\r\n",model.X_data[11],model.U_data[4],model.U_data[5]);
}

static void Log_Write_SINS(){
	if(!kf.KFinit_complete) return;
	struct log_DR pkt = {
        LOG_PACKET_HEADER_INIT(LOG_SINS_MSG),
        .time_s = TIME,
        .mode = fp.dr_mode,
		.ax = sins.an[0],
		.ay = sins.an[1],
		.az = sins.an[2],
        .vx = sins.vn[0],
        .vy = sins.vn[1],
        .vz = sins.vn[2],
        .dx = sins.dn[0],
		.dy = sins.dn[1],
		.dz = sins.dn[2],
        .lat = (uint32_t)(sins.pos[0]*1e7f),
        .lng = (uint32_t)(sins.pos[1]*1e7f),
        .alt = sins.pos[2],
    };
	WriteBlock(&pkt, sizeof(pkt));
}

static void WriteBlock_PID(PidObject pid, u8 logtype){
	struct log_PID pkt = {
		LOG_PACKET_HEADER_INIT(logtype),
		.time_s = TIME,
		.desired = pid.desired,
		.error = pid.error,
		.prevError = pid.prevError,
		.integ = pid.integ,		
		.deriv = pid.deriv,		
	    .outP = pid.outP, 	
		.outI = pid.outI, 		
		.outD = pid.outD, 		
	    .out = pid.out,	
	};
	WriteBlock(&pkt, sizeof(pkt));
}
static void Log_Write_PID_ANG_RLL(){
	WriteBlock_PID(pidAngleRoll,LOG_PID_ANG_RLL_MSG);
}	
static void Log_Write_PID_ANG_PIT(){
	WriteBlock_PID(pidAnglePitch,LOG_PID_ANG_PIT_MSG);
}
static void Log_Write_PID_ANG_YAW(){
	WriteBlock_PID(pidAngleYaw,LOG_PID_ANG_YAW_MSG);
}
static void Log_Write_PID_RAT_RLL(){
	WriteBlock_PID(pidRateRoll,LOG_PID_RAT_RLL_MSG);
}	
static void Log_Write_PID_RAT_PIT(){
	WriteBlock_PID(pidRatePitch,LOG_PID_RAT_PIT_MSG);
}	
static void Log_Write_PID_RAT_YAW(){
	WriteBlock_PID(pidRateYaw,LOG_PID_RAT_YAW_MSG);
}	
static void Log_Write_PID_ACC_Z(){
	WriteBlock_PID(pidAccZ,LOG_PID_ACC_Z_MSG);
}	
static void Log_Write_PID_VEL_Z(){
	WriteBlock_PID(pidVelZ,LOG_PID_VEL_Z_MSG);
}	
static void Log_Write_PID_POS_Z(){
	WriteBlock_PID(pidPosZ,LOG_PID_POS_Z_MSG);
}	

//// Write a POS packet
//void Log_Write_POS(AP_AHRS &ahrs)
//{
//    Location loc;
//    if (!ahrs.get_position(loc)) {
//        return;
//    }
//    float home, origin;
//    ahrs.get_relative_position_D_home(home);
//    ahrs.get_relative_position_D_origin(origin);
//    struct log_POS pkt = {
//        LOG_PACKET_HEADER_INIT(LOG_POS_MSG),
//        time_us        : AP_HAL::micros64(),
//        lat            : loc.lat,
//        lng            : loc.lng,
//        alt            : loc.alt*1.0e-2f,
//        rel_home_alt   : -home,
//        rel_origin_alt : -origin
//    };
//    WriteBlock(&pkt, sizeof(pkt));
//}

//void Log_Write_Origin(uint8_t origin_type, const Location &loc)
//{
//    uint64_t time_us = AP_HAL::micros64();
//    struct log_ORGN pkt = {
//        LOG_PACKET_HEADER_INIT(LOG_ORGN_MSG),
//        time_us     : time_us,
//        origin_type : origin_type,
//        latitude    : loc.lat,
//        longitude   : loc.lng,
//        altitude    : loc.alt
//    };
//    WriteBlock(&pkt, sizeof(pkt));
//}


 //遍历文件
 //path:路径
 //返回值:执行结果
uint8_t scan_files()
{
	FRESULT res;	 
    res = f_opendir(&dir,(const TCHAR*)"0:"); //打开一个目录
    if (res == FR_OK) 
	{	
		while(1)
		{
	        res = f_readdir(&dir, &fileinfo);                   //读取目录下的一个文件
	        if (res != FR_OK || fileinfo.fname[0] == 0) break;  //错误了/到末尾了,退出
	        //if (fileinfo.fname[0] == '.') continue;             //忽略上级目录
			printf("%s, size:%.1fKb \r\n",fileinfo.fname,(float)(fileinfo.fsize/1024.0));			//打印文件名和文件大小	  
		} 
    }	   
    return res;	  
}

bool log_out_of_time=false;
u8 logRates[_LOG_LAST_MSG_-1]={0};
u8 logTicks[_LOG_LAST_MSG_-1]={0};

//日志记录频率更新
void logRates_update(){
	logRates[LOG_TEST_MSG] = fp.log_TEST_rate;
	logRates[LOG_PARAMETER_MSG] = fp.log_PARAMETER_rate;
	logRates[LOG_GPS_MSG] = fp.log_GPS_rate;
	logRates[LOG_IMU_MSG] = fp.log_IMU_rate;
	logRates[LOG_RCIN_MSG] = fp.log_RCIN_rate;
	logRates[LOG_RCOUT_MSG] = fp.log_RCOUT_rate;
	logRates[LOG_BARO_MSG] = fp.log_BARO_rate;
	logRates[LOG_POWR_MSG] = fp.log_POWR_rate;
	logRates[LOG_ATTITUDE_MSG] = fp.log_ATTITUDE_rate;
	logRates[LOG_MODE_MSG] = fp.log_MODE_rate;
	logRates[LOG_POS_MSG] = fp.log_POS_rate;
	logRates[LOG_PID_ANG_RLL_MSG] = fp.log_PID_ANG_RLL_rate;
	logRates[LOG_PID_ANG_PIT_MSG] = fp.log_PID_ANG_PIT_rate;
	logRates[LOG_PID_ANG_YAW_MSG] = fp.log_PID_ANG_YAW_rate;
	logRates[LOG_PID_RAT_RLL_MSG] = fp.log_PID_RAT_RLL_rate;
	logRates[LOG_PID_RAT_PIT_MSG] = fp.log_PID_RAT_PIT_rate;
	logRates[LOG_PID_RAT_YAW_MSG] = fp.log_PID_RAT_YAW_rate;
	logRates[LOG_PID_ACC_Z_MSG] = fp.log_PID_ACC_Z_rate;
	logRates[LOG_PID_VEL_Z_MSG] = fp.log_PID_VEL_Z_rate;
	logRates[LOG_PID_POS_Z_MSG] = fp.log_PID_POS_Z_rate;
	logRates[LOG_DR_MSG] = fp.log_DR_rate;
	logRates[LOG_SINS_MSG] = fp.log_SINS_rate;
	logRates[LOG_WP_MSG] = fp.log_WP_rate;
	logRates[LOG_MODEL_MSG] = fp.log_MODEL_rate;
}
//判断是否可以记录当前日志
bool should_log(enum LogMessages log_type){
	if(time_available_us()<1){
		log_out_of_time = true;
		return false;
	} 
	if (log_type >= _LOG_LAST_MSG_) {
		return false;
	}
	u8 rate = logRates[log_type];	//此类数据流输出频率
	if(rate ==0) return false;
	if (logTicks[log_type] == 0) {
		if (rate > 100) rate = 100;	//最多以100hz记录
		logTicks[log_type] = (100 / rate) - 1 ;	//计算需要循环多少圈才发送此类数据流
		return true;
	}
	logTicks[log_type]--;
	return false;
}
//同步文件
bool sync_log(){
	static u32 sec = 0;
	if(sys_time.one_second > sec+10){
		f_sync(file);  //每隔一段时间保存文件记录，防止系统崩溃数据丢失（注意：改函数不能频繁调用，十分消耗资源）
		sec = sys_time.one_second;
		return true;
	}
	return false;
}
#ifdef TEST
void logging_loop(){
	if(!sys_flag.sdcard_detected || !sys_flag.data_recv) return;	//没有检测到sd卡或者没有接收到UTC时间，退出
	Log_create();	//检测是否需要创建日志
	if(!sys_flag.log_creat) return;//若日志还未创建，退出
	char data[200]={0};//TimeS,Roll,Pitch,Yaw,Accx,Accy,Accz,Gyrox,Gyroy,Gyroz,Depth,Power
	u16 *chan = get_motorout();
	sprintf(data,"%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%d,%d,%d\r\n",
					TIME,
					Sensor_latest.ins.euler[0],Sensor_latest.ins.euler[1],Sensor_latest.ins.euler[2],
					Sensor_latest.ins.acc[0],Sensor_latest.ins.acc[1],Sensor_latest.ins.acc[2],
					Sensor_latest.ins.gyro[0],Sensor_latest.ins.gyro[1],Sensor_latest.ins.gyro[2],
					Sensor_latest.barometer.depth,Sensor_latest.power,
					*(chan),*(++chan),*(++chan),*(++chan)
	);
	f_lseek(file,f_size(file));
	f_write (file, data, strlen(data), &bw);
}

#else


//日志循环 -> 100hz调用
void logging_loop(){
	if(fp.log_mode == LOG_NONE) return;
	logRates_update();	//更新记录频率，防止上位机随时更改
	log_out_of_time = false;	//重置超时标记
	if(!sys_flag.sdcard_detected || !sys_flag.data_recv) return;	//没有检测到sd卡或者没有接收到UTC时间，退出
	Log_create();	//检测是否需要创建日志
	if(!sys_flag.log_creat) return;//若日志还未创建，退出
	if(sync_log()) return;	//定期同步文件
	if(should_log(LOG_TEST_MSG)) Log_Write_Test();
	if(log_out_of_time) return;
	if(should_log(LOG_IMU_MSG)) Log_Write_IMU();
//	if(log_out_of_time) return;
//	if(should_log(LOG_RCOUT_MSG)) Log_Write_RCOUT();
	if(log_out_of_time) return;
	if(should_log(LOG_GPS_MSG)) Log_Write_GPS();
	if(log_out_of_time) return;
	if(should_log(LOG_PARAMETER_MSG)){}
	if(log_out_of_time) return;
	if(should_log(LOG_RCIN_MSG)) Log_Write_RCIN();
	if(log_out_of_time) return;
	if(should_log(LOG_BARO_MSG)) Log_Write_Baro();
	if(log_out_of_time) return;
	if(should_log(LOG_POWR_MSG)) Log_Write_Power();
	if(log_out_of_time) return;
	if(should_log(LOG_ATTITUDE_MSG)) Log_Write_Attitude();
	if(log_out_of_time) return;
	if(should_log(LOG_MODE_MSG)) Log_Write_Mode();
	if(log_out_of_time) return;
	if(should_log(LOG_PID_ANG_RLL_MSG))Log_Write_PID_ANG_RLL();
	if(log_out_of_time) return;
	if(should_log(LOG_PID_ANG_PIT_MSG)) Log_Write_PID_ANG_PIT();
	if(log_out_of_time) return;
	if(should_log(LOG_PID_ANG_YAW_MSG)) Log_Write_PID_ANG_YAW();
	if(log_out_of_time) return;
	if(should_log(LOG_PID_RAT_RLL_MSG)) Log_Write_PID_RAT_RLL();
	if(log_out_of_time) return;
	if(should_log(LOG_PID_RAT_PIT_MSG)) Log_Write_PID_RAT_PIT();
	if(log_out_of_time) return;
	if(should_log(LOG_PID_RAT_YAW_MSG)) Log_Write_PID_RAT_YAW();
	if(log_out_of_time) return;
	if(should_log(LOG_PID_ACC_Z_MSG)) Log_Write_PID_ACC_Z();
	if(log_out_of_time) return;
	if(should_log(LOG_PID_VEL_Z_MSG)) Log_Write_PID_VEL_Z();
	if(log_out_of_time) return;
	if(should_log(LOG_PID_POS_Z_MSG)) Log_Write_PID_POS_Z();
	if(log_out_of_time) return;
	if(should_log(LOG_DR_MSG)) Log_Write_DR();
	if(log_out_of_time) return;
	if(should_log(LOG_SINS_MSG)) Log_Write_SINS();
	if(log_out_of_time) return;
	if(should_log(LOG_WP_MSG)) Log_Write_WP();
	if(log_out_of_time) return;
	if(should_log(LOG_MODEL_MSG)) Log_Write_MODEL();
}	

#endif


