#include "sensor.h"
#include "global_para.h"
#include "adc.h"
#include "GCS_Mavlink.h"
#include "led.h"
#include "model.h"
#include "sdio_sdcard.h"
#include "malloc.h"
#include "delay.h"
#include "wpnav.h"
#include "kf.h"
#include "exfuns.h"

SENSOR Sensor_latest,Sensor_target,Sensor_init,PID_latest;
extern nmea_msg gps;

static void gps_update(){
	if(!sys_flag.health.gps) {	//GPSû�յ�����
		sys_flag.gps_lock = false;
//		memset(&Sensor_latest.gps,0,sizeof(Sensor_latest.gps));	//�˴����������ǰGPS���ݣ���ֹQGC��Ϊʧ�������л�
		return;
	}
	static u32 second = 0;
	if(!sys_flag.gps_recv) return;
	if(gps.rmc_pos_status!='A') {	//�����ݵ�δ��λ
		if(sys_time.one_second-second >= 3)	
			sys_flag.gps_lock = false;//3s��δ��Ч��λ����GPSδ����
//			memset(&Sensor_latest.gps,0,sizeof(Sensor_latest.gps));
		return; //��δ��λ
	}
	//���յ�GPS�ҳɹ���λ
	sys_flag.gps_lock = true;
	sys_flag.gps_log_update = true;
	second = sys_time.one_second;
	Sensor_latest.gps.gpssta = gps.gga_gpssta;					 
	Sensor_latest.gps.posslnum = gps.gga_posslnum;
	Sensor_latest.gps.hdop = gps.gga_hdop;
	Sensor_latest.gps.altitude = gps.gga_altitude;			 	
	//�Ѷȷָ�ʽddmm.mmmmת��Ϊ�ȸ�ʽ -> ���͸�QGC
	/*
	1��=60�֣�1��=60��
	���磺113.125����ǣ�113�ȣ�0.125x60=7.5�֣�ȡ�����־���7��0.5x60=30��
	���Ծ���113��7��30�� 
	*/
	u16 _temp = gps.rmc_latitude/100;
	Sensor_latest.gps.latitude = _temp + (gps.rmc_latitude - _temp*100)/60.0;//doubleȡ���޷�ʹ��%
	Sensor_latest.gps.nshemi = gps.rmc_nshemi;		
	_temp = gps.rmc_longitude/100;
	Sensor_latest.gps.longitude = _temp + (gps.rmc_longitude - _temp*100)/60.0;
	Sensor_latest.gps.ewhemi = gps.rmc_ewhemi;	
	Sensor_latest.gps.speed = gps.rmc_speed*0.514444f;	//��ת��Ϊm/s		
	Sensor_latest.gps.orient = gps.rmc_orient;					
	Sensor_latest.gps.magdec = gps.rmc_magdec;					
	Sensor_latest.gps.magdir = gps.rmc_magdir;
	Sensor_latest.gps.mode = gps.rmc_mode;	
	Sensor_latest.gps.utc = gps.rmc_utc;

	//kf�������ݸ���
	Sensor_latest.gpspos[0] = Sensor_latest.gps.latitude;
	Sensor_latest.gpspos[1] = Sensor_latest.gps.longitude;
	Sensor_latest.gpspos[2] = Sensor_latest.gps.altitude;

	
	Sensor_latest.gpsvn[0] = Sensor_latest.gps.speed * sinf(Sensor_latest.ins.euler_rad[2]); //�����ٶ�Sensor_latest.gps.orient
	Sensor_latest.gpsvn[1] = Sensor_latest.gps.speed * cosf(Sensor_latest.ins.euler_rad[2]); //�����ٶ�
	Sensor_latest.gpsvn[2] = 0;
	memset(&gps,0,sizeof(gps));	
}

static void pwr_update(){
	Sensor_latest.power = 4*ADC_VAL[1]*3.3/4096; 	
}
static void _ins_update(){
	ins_update();
	memcpy(&Sensor_latest.ins, &ins, sizeof(ins));
//	//ȥ���������ٶ�Ӱ��Ĵ�������ٶ� -> ������֤����ѧģ��
//	arm_matrix_instance_f32 Rbn, Rnb, g, gb;
//	vec3f gb_data, g_data = {0,0,9.8};
//	mat3f Rbn_data, Rnb_data;
//	DCM_update(Sensor_latest.ins.euler, Rbn_data, &Rbn);
//	arm_mat_init_f32(&Rnb ,3, 3, *Rnb_data);
//	arm_mat_init_f32(&g ,3, 1, g_data);
//	arm_mat_init_f32(&gb ,3, 1, gb_data);
//	arm_mat_trans_f32(&Rbn, &Rnb);
//	arm_mat_mult_f32(&Rnb, &g, &gb);
//	for(int i=0;i<3;++i) Sensor_latest.ins.acc_no_g[i] = Sensor_latest.ins.acc[i]-gb_data[i];
}


static void temp_update(){	//����¶�Դ��ȡƽ��
	Sensor_latest.temp = ins.temp;
}

static void baro_update(){
	//ע�⣺��ȼ���ͻ�䣬��Ҫ�Ӹ���ͨ�˲�
	float alpha = 0.1;

	Sensor_latest.barometer.analog_vol = barometer.analog_vol;
	Sensor_latest.barometer.analog_curr = barometer.analog_curr;
	Sensor_latest.barometer.pressure = barometer.pressure;
	Sensor_latest.barometer.depth += alpha*((barometer.depth- Sensor_init.barometer.depth)-Sensor_latest.barometer.depth);
	if(Sensor_latest.barometer.depth<0) Sensor_latest.barometer.depth=0;	//��ȴ���0
}

static void position_update(){
//ʵʱλ�ø���
	switch(fp.dr_mode){
		case DR_SINS: 
		case DR_SINS_MA:
		case DR_SINS_GPS:
			Sensor_latest.position.lat = sins.pos[0]*1e7f;
			Sensor_latest.position.lng = sins.pos[1]*1e7f;
			Sensor_latest.position.alt = sins.pos[2];
			break; 
		case DR_MA: 	//��ģ��
			Sensor_latest.position.lat = model.pos[0];
			Sensor_latest.position.lng = model.pos[1];
			Sensor_latest.position.alt = model.pos[2];
			break;
		case DR_GPS:	//��GPS
			Sensor_latest.position.lat = Sensor_latest.gps.latitude * 1e7;
			Sensor_latest.position.lng = Sensor_latest.gps.longitude * 1e7;
			Sensor_latest.position.alt = Sensor_latest.barometer.depth;
			break;
	}
}


//��λ����λ�ó�ʼ��
void DR_pos_init(){
	Sensor_latest.SINS_DR.pos = Sensor_latest.position;
	Sensor_latest.MA_DR.pos = Sensor_latest.position;
}

//��ȡ��ǰ��̬   ֻ������PIDʱ����
void get_euler_angle_latest(vec3f euler){
	arm_copy_f32(Sensor_latest.ins.euler, euler, 3);
}

//��ȡ��ǰ���ٶ�
void get_euler_gyro_latest(vec3f gyro){
	arm_copy_f32(Sensor_latest.ins.gyro, gyro, 3);
}
//��ȡ������̬���
void get_eular_depth_target(vec3f euler, float *depth){
	arm_copy_f32(Sensor_target.ins.euler, euler, 3);
	*depth = Sensor_target.barometer.depth;
}

//��¼��������ǰ�Ĵ�����ֵ
void Sensor_vibration(){
	if(!sys_flag.baro_vibration){	//���У׼
		float baro_analog_vol=0,baro_analog_curr=0,baro_depth=0;
		u32 baro_pressure=0;
		u8 num = 0;
		while(num<20){
			baro_read();
			baro_analog_vol += barometer.analog_vol;
			baro_analog_curr += barometer.analog_curr;
			baro_depth += barometer.depth;
			baro_pressure += barometer.pressure;
			num++;
			delay_ms(50);
		}
		sys_flag.baro_vibration = true;
		Sensor_init.barometer.analog_vol = baro_analog_vol/20.0f;
		Sensor_init.barometer.analog_curr = baro_analog_curr/20.0f;
		Sensor_init.barometer.pressure = baro_pressure/20;
		Sensor_init.barometer.depth = baro_depth/20.0f;
		Sensor_latest.barometer.depth = 0;	//��ȹ�0
		printf("[sensor] baro sensor vibration complete \r\n");
		gcs_send_text(MAV_SEVERITY_INFO, "[sensor] baro sensor vibration complete");
	}
//	if(!sys_flag.acc_vibration){	//����Ӽ�У׼��ע��Ҫȥ���������ٶȵ�Ӱ��
//		vec3f acc={0};
//		u8 num = 0;
//		vec3f gn = {0,0,9.8};
//		while(num<20){
//			ahrs_read();
//			arm_matrix_instance_f32 Gn;
//			arm_mat_init_f32(&Gn ,3, 1, gn);
//			DCM_update(ahrs.euler, sins.Cbn_data, &sins.Cbn);
//			arm_mat_trans_f32(&sins.Cbn, &sins.Cnb);
//			arm_mat_mult_f32(&sins.Cnb,&Gn,&sins.fb);	//fn=Cbn*fb
//			arm_sub_f32(ahrs.acc, sins.fb.pData, sins.db, 3);
//			arm_add_f32(acc, sins.db, acc, 3);
//			num++;
//			delay_ms(50);
//		}
//		sys_flag.acc_vibration = true;
//		arm_scale_f32(acc,1/20, sins.db,3);
//		printf("[sensor] acc sensor vibration complete \r\n");
//		printf("acc disturbance= %.2f %.2f %.2f \r\n",sins.db[0],sins.db[1],sins.db[2]);
//		gcs_send_text(MAV_SEVERITY_INFO, "[sensor] acc sensor vibration complete");
//	}
}

//ins(imu+��������)�������
bool sys_check_ins(){
	if(!fp.health_chenable.imu) return true;
	//û�н��յ�ins���� �� ���յ����ݵ�û�н����ɹ�
	if(!sys_flag.health.imu || (is_zero(imu.euler[0]) && is_zero(imu.euler[1]) && is_zero(imu.euler[2]))){
		gcs_send_text(MAV_SEVERITY_WARNING, "IMU error");
		return false;
	}
//	if(!sys_flag.health.ec || (is_zero(ec.euler[0]) && is_zero(ec.euler[1]) && is_zero(ec.euler[2]))){
//		gcs_send_text(MAV_SEVERITY_WARNING, "Electronic Compass error");
//		return false;
//	}
	return true;
}

//baro�������
bool sys_check_baro(){
	if(!fp.health_chenable.baro) return true;
	float baro_val;
	switch(Sensor_latest.barometer.baro_type){
		case analog_vol:baro_val = Sensor_latest.barometer.analog_vol ;break;
		case analog_curr:baro_val = Sensor_latest.barometer.analog_curr; break;
		case pressure:baro_val = (float)Sensor_latest.barometer.pressure;break;
	}
	if(is_zero(baro_val)) {
		gcs_send_text(MAV_SEVERITY_WARNING, "BARO error");
		sys_flag.health.baro = false;
		return false;
	}
	sys_flag.health.baro = true;
	return true;
}
//��ؽ������
bool sys_check_BATT(){
	if(!fp.health_chenable.batt) return true;
	if(Sensor_latest.power < fp.power_min){
		sys_flag.battery_voltage_low = true;
		gcs_send_text(MAV_SEVERITY_CRITICAL,"BATTERY VOLTAGE IS TOO LOW!");
		//TODO �͵����Զ�����
		return false;
	}
	sys_flag.battery_voltage_low = false;
	return true;
}
//��GCSͨ�ż��
bool sys_check_GCS_heartbreat(){
	if(sys_time.one_second > vp.last_gcs_heartbeat_time+3){
		sys_flag.health.gcs_heartbeat = false;
		sys_flag.joy_connected = false;
		if(fp.ROV_mode_enable){
			vp.control_mode = MANUAL;	//��ROVģʽ����3���Ӻ��QGC��ʧͨ�ţ�������Ϊ�ֶ�ģʽ�����������
			sys_flag.motors_armed = false;
		}
		return false;
	}
	sys_flag.health.gcs_heartbeat = true;
	return true;
		
}

//��λ��Ч���
bool sys_check_position(){
	if(!fp.health_chenable.gps) return true;
	return sys_flag.gps_lock;
}

//����Ƿ���ˮ��
bool sys_check_surface(){
	if(fabs(get_throttle())<0.2 && fabs(Sensor_latest.ins.acc[2])<1 && fabs(Sensor_latest.barometer.depth)<0.1){
		return true;
	}
	return false;
}

//sd�����
 bool sys_check_sd(){
	if(!fp.health_chenable.sd) return true;
	if(!sys_flag.sdcard_detected) SD_FATFS_Init();
	if(!sys_flag.sdcard_detected) gcs_send_text(MAV_SEVERITY_WARNING, "SD_Card error");
	return sys_flag.sdcard_detected;
 }

 //flash���
 bool sys_check_flash(){
	if(!fp.health_chenable.flash) return true;
	if(!sys_flag.flash_check) flash_ext_init();
	if(!sys_flag.flash_check) gcs_send_text(MAV_SEVERITY_WARNING, "EX_Flash error");
	return sys_flag.flash_check;
 }
//����Ŀ��켣
static u8 _wp_path = 0;
bool sys_check_wp_path(){
	if(_wp_path == fp.wp_path) return true;
	_wp_path = fp.wp_path;
	char text[50]={0}, _file_name[20]={0};
	switch(fp.wp_path){
		case PATH_LINE:	
			sprintf(text,"[mission] waypoints path change to: LINE");
			sprintf(_file_name,"mission_line.bin");
			break;	//ֱ��
		case PATH_PolyLine:
			sprintf(text,"[mission] waypoints path change to: PolyLine");
			sprintf(_file_name,"mission_polyline.bin");
			break;//����
		case PATH_Square: 
			sprintf(text,"[mission] waypoints path change to: Square");
			sprintf(_file_name,"mission_squre.bin");
			break;	//�ı���
		case PATH_Z:
			sprintf(text,"[mission] waypoints path change to: Z_Line");
			sprintf(_file_name,"mission_Z.bin");
			break;	//Z����
		case PATH_Circule: 
			sprintf(text,"[mission] waypoints path change to: Circule");
			sprintf(_file_name,"mission_circule.bin");
			break;	//Բ��
	}
	printf("%s \r\n",text);
	gcs_send_text(MAV_SEVERITY_INFO, text);
	if(f_open(ftemp, _file_name, FA_OPEN_EXISTING | FA_READ) != FR_OK){
		printf("[mission] ��û�д��ຽ���������! \r\n");
		gcs_send_text(MAV_SEVERITY_WARNING, "[mission] mission need to be uploaded!");
	}
	else {
		printf("[mission] mission need to be Re-downloaded! \r\n");
		gcs_send_text(MAV_SEVERITY_INFO, "[mission] mission need to be Re-downloaded!");
		vp.waypoint_count = f_size(ftemp)/MISSION_BIN_COMMAND_SIZE;;//ע���޸ĵ�ǰ�������
	}
	memcpy(mission_file_name,_file_name,20);
	return true;
}

void sys_update_payload(){
	//����λ��ͨ������������
//	printf("%d \r\n",vp.data_size_one_second);
	vp.data_size_one_second_latest = vp.data_size_one_second;
	vp.data_size_one_second_max = vp.data_size_one_second_max<vp.data_size_one_second_latest?vp.data_size_one_second_latest:vp.data_size_one_second_max;
	vp.data_size_one_second = 0;

	//�������̽���Ƶ�ʸ���
	vp.ec_parse_num_sencond = vp.ec_parse_num_latest;
	vp.ec_parse_num_latest = 0;
	//mpu����Ƶ�ʸ���
	vp.imu_parse_num_sencond = vp.imu_parse_num_latest;
	vp.imu_parse_num_latest = 0;
	//gps����Ƶ�ʸ���
	vp.gps_parse_num_sencond = vp.gps_parse_num_latest;
	vp.gps_parse_num_latest = 0;
	//sraminʣ��ռ����
	u16 perused = my_mem_perused(SRAMIN);
	vp.sramin_available_latest = 1.0f-perused/10.0f;
	if(vp.sramin_available_latest > (1-vp.sramin_available_min))
		vp.sramin_available_min = 1-vp.sramin_available_latest;
}

void setKFmeas(){
	if(!kf.KFinit_complete || sys_flag.kf_verify) return;
	static int num=0, cnt=1;
	switch(fp.dr_mode){
		case DR_SINS:  //���ߵ��ʹ�ģ�͵����޹۲���
		case DR_MA:		
			break;
		case DR_SINS_MA:
			if(num++ == 100){//1s��һ�ι۲��� -> �˲���������ʱ
				KF_SetMeas(model.vn, model.pos, 1);
				num=1;
				cnt++;
				//���һ��ʱ��ͨ��GPSУ׼ -> ��׼����Ӧ���Ǹ���ˮ�棬�ȴ����ǣ�Ȼ��У׼
				if(cnt%fp.gps_fb_interval == 0){
					KF_SetMeas(Sensor_latest.gpsvn, Sensor_latest.gpspos, 1);
					printf("[mission] GPS feedback \r\n");
					gcs_send_text(MAV_SEVERITY_INFO, "[mission] GPS feedback!");
				}
			}
			break;
		case DR_SINS_GPS:
			if(sys_flag.gps_lock){
				KF_SetMeas(Sensor_latest.gpsvn, Sensor_latest.gpspos, 1);	//���ù۲���
				arm_copy_f32(O31, Sensor_latest.gpsvn, 3);
				arm_copy_f32(O31, Sensor_latest.gpspos, 3);
			}
			break;
	}
}
/*****************************************************************
���������ݸ���
******************************************************************/
void Get_sensor_data()
{
	_ins_update();	//�ߵ�ϵͳ����
	gps_update();
	pwr_update();//��ȡ��Դ��ѹ����
	baro_update();//��ȡ��ȼ�����
	temp_update();//�����¶�����
	position_update();	//λ�ø���
	Sensor_vibration();	//��ȼ�У׼

	setKFmeas();	//�����˲��۲���
}

