#include "GCS_Common.h"
#include "mavlink_avoid_errors.h"
#include "mission.h"
#include "control_mode.h"
#include "joystick.h"
#include "motors.h"
#include "flash.h"
#include "malloc.h"
#include "delay.h"
#include "sensor.h"
#include "mode_task.h"
#include "scheduler.h"
#include "udp_demo.h"

u8 streamRates[NUM_STREAMS];
u8 stream_ticks[NUM_STREAMS];

//�����Ƿ������վ���͵Ĵ�����������Ӧ��50hz������
u8 stream_trigger(enum streams stream_num){
	if (stream_num >= NUM_STREAMS) {
		return false;
	}
	u8 rate = (u8)streamRates[stream_num];	//�������������Ƶ��
	if(rate ==0) return false;
	if (stream_ticks[stream_num] == 0) {
		// we're triggering now, setup the next trigger point
		if (rate > 50) {
				rate = 50;
		}
		stream_ticks[stream_num] = (50 / rate) - 1 ;	//������Ҫѭ������Ȧ�ŷ��ʹ���������
		return true;
	}
	// count down at 50Hz
	stream_ticks[stream_num]--;
	return false;
}

void send_hearbeat(){
	u8 _system_status = sys_flag.relay_12V_enable?0:MAV_STATE_POWEROFF;
	u32 _custom_mode = vp.control_mode;	
	u8 _mav_type = 12;
	u8 _autopilot = 3;
	u8 _base_mode = 16 | 8 | 64 | 1;
	_base_mode |= (sys_flag.motors_armed<<7);

	mavlink_msg_heartbeat_send(MAVLINK_COMM_1 , _mav_type, _autopilot, _base_mode, _custom_mode, _system_status);
}

void send_info(){

}
/*
����param_id_type_value�е�index������
*/
void send_param_index(u16 index){
	sys_flag.param_send = true;
	float _param_vaule = 0;
	switch(param_id_type_value[index].param_type){
		case UINT8:
			_param_vaule = *(u8*)param_id_type_value[index].param_value;break;
		case INT8:
			_param_vaule = *(s8*)param_id_type_value[index].param_value;break;
		case UINT16:
			_param_vaule = *(u16*)param_id_type_value[index].param_value;break;
		case INT16:
			_param_vaule = *(s16*)param_id_type_value[index].param_value;break;
		case UINT32:
			_param_vaule = *(u32*)param_id_type_value[index].param_value;break;
		case INT32:
			_param_vaule = *(s32*)param_id_type_value[index].param_value;break;
		case UINT64:
			_param_vaule = *(unsigned long long*)param_id_type_value[index].param_value;break;
		case INT64:
			_param_vaule = *(signed long long*)param_id_type_value[index].param_value;break;
		case REAL32:
			_param_vaule = *(float*)param_id_type_value[index].param_value;break;
		case REAL64:
			_param_vaule = *(double*)param_id_type_value[index].param_value;break;
	}
	mavlink_msg_param_value_send(
		 MAVLINK_COMM_1,						//ͨ��
		 param_id_type_value[index].param_id,	//������
		 _param_vaule,							//����ֵ
		 param_id_type_value[index].param_type, //��������
		 fp.param_num,							//ȫ����������
		 index);								//�˲����ڲ������еĴ���
	 if(!fp.ROV_mode_enable) {
	 	while(!sys_flag.send_empty) mavsend("",0);	//������ͨ������£����뱣֤�˲��������ͳ�ȥ
	 }
//	 sys_flag.param_send = false;

}
void send_all_param(){
//	for(int num=0;num<10;num++)
	{
		for(int i=0;i<fp.param_num;i++){
			send_param_index(i);	
			printf("[->gcs] send %d parameter \r\n",i);
			delay_ms(20);
		}
		printf("complete \r\n");
	}
	sys_flag.param_all_send = true;
}

void send_raw_imu(){
	vec3f accel, gyro, mag;
	memcpy(accel, Sensor_latest.ins.acc, sizeof(vec3f));
	memcpy(gyro, Sensor_latest.ins.gyro, sizeof(vec3f));
	memcpy(mag, Sensor_latest.ins.mag, sizeof(vec3f));
	mavlink_msg_raw_imu_send(
			MAVLINK_COMM_1,
			0,
			accel[0] * 1000.0f / GRAVITY_MSS,
			accel[1] * 1000.0f / GRAVITY_MSS,
			accel[2] * 1000.0f / GRAVITY_MSS,
			gyro[0] * 1000.0f,
			gyro[1] * 1000.0f,
			gyro[2] * 1000.0f,
			mag[0],
			mag[1],
			mag[2]
	);
}

void send_attitude(){
	vec3f euler, gyro; //��Ҫת���ɻ���
	memcpy(euler, Sensor_latest.ins.euler_rad, sizeof(vec3f));
	memcpy(gyro, Sensor_latest.ins.gyro_rad, sizeof(vec3f));
	mavlink_msg_attitude_send(
		MAVLINK_COMM_1, 
		0, 
		euler[1], 	//ע�⣬����������ϵ����x��ת����Ϊpitch
		euler[0], 
		euler[2], 
//		gyro[1], 
		Sensor_latest.barometer.depth*M_PI/180.0f,
		gyro[0], 
		gyro[2]);
}

void send_scaled_pressure(){
//	mavlink_msg_raw_pressure_send(
//				MAVLINK_COMM_1, 
//				0, 
//				1000,
//				100,
//				100,
//				Sensor_latest.temp*1000);

	mavlink_msg_scaled_pressure_send(
			MAVLINK_COMM_1, 
			0, 
			2000,
			200,
			Sensor_latest.temp*100);

//	mavlink_msg_scaled_pressure2_send(
//		MAVLINK_COMM_1, 
//		0, 
//		3000,
//		300,
//		Sensor_latest.temp*10);
}

void send_gps_raw(){
	u8 fix_type=0;
	if(!sys_flag.health.gps) fix_type = 0;
	else if(!sys_flag.gps_lock) fix_type = 1;
	else fix_type = 3;
	mavlink_msg_gps_raw_int_send(
		MAVLINK_COMM_1,
		0, 
		fix_type,//Sensor_latest.gps.gpssta, 				//GPS����״̬
		Sensor_latest.gps.latitude * 1E7,		//γ�� 28.46886569*1E7, //
		Sensor_latest.gps.longitude * 1E7, 	//���� 118.54543111*1E7, //
		Sensor_latest.gps.altitude * 1000,		//�߶�mm
		Sensor_latest.gps.hdop * 100, 				//ˮƽ��������
		USHRT_MAX,								//��ֱ�������� 
		(u16)Sensor_latest.gps.speed*100.0f, 		//�Ե��ٶ�cm/s(��->cm/s)
		Sensor_latest.gps.orient * 100, 		//���溽��(����*100)
		Sensor_latest.gps.posslnum);			//�ɼ�������
}
void send_statustext(u8 severity,const char text[50]){
	vp.severity = severity;
	memcpy(vp.text,text,50);
	send_message(MSG_STATUSTEXT);
}

void send_text(){
	mavlink_msg_statustext_send(MAVLINK_COMM_1,vp.severity,vp.text);
}

//���������б�
void send_tasklists(){
	if(Task_List[0].task_list == 0){ //û�н��յ������б�
		send_statustext(MAV_SEVERITY_INFO,"QAUV hasn't received TaskLists!");
		return;
	}
	mavlink_msg_tasklists_send(
		MAVLINK_COMM_1,
		Task_List[0].task_list,
		Task_List[0].task_beat,
		Task_List[0].task_val,
		Task_List[1].task_list,
		Task_List[1].task_beat,
		Task_List[1].task_val,
		Task_List[2].task_list,
		Task_List[2].task_beat,
		Task_List[2].task_val,
		Task_List[3].task_list,
		Task_List[3].task_beat,
		Task_List[3].task_val,
		Task_List[4].task_list,
		Task_List[4].task_beat,
		Task_List[4].task_val,
		Task_List[5].task_list,
		Task_List[5].task_beat,
		Task_List[5].task_val,
		Task_List[6].task_list,
		Task_List[6].task_beat,
		Task_List[6].task_val,
		Task_List[7].task_list,
		Task_List[7].task_beat,
		Task_List[7].task_val);
}

void send_power_status(){
	mavlink_msg_power_status_send(
		MAVLINK_COMM_1, 
		10*1000,	//Sensor_latest.power
		5*1000,
		1);
}
void send_extended_status1(){
	u32 control_sensors_present;
	control_sensors_present |= MAV_SYS_STATUS_SENSOR_3D_MAG;
	control_sensors_present |= MAV_SYS_STATUS_SENSOR_3D_GYRO; /* 0x01 3D gyro | */
	control_sensors_present |= MAV_SYS_STATUS_SENSOR_3D_ACCEL; /* 0x02 3D accelerometer | */
	control_sensors_present |= MAV_SYS_STATUS_SENSOR_3D_MAG; /* 0x04 3D magnetometer | */
	control_sensors_present |= MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE; /* 0x08 absolute pressure | */
	control_sensors_present |= MAV_SYS_STATUS_SENSOR_GPS; /* 0x20 GPS | */
	control_sensors_present |= MAV_SYS_STATUS_SENSOR_YAW_POSITION; /* 0x1000 yaw position | */
	control_sensors_present |= MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL; /* 0x2000 z/altitude control | */
	control_sensors_present |= MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS; /* 0x8000 motor outputs / control | */
	control_sensors_present |= MAV_SYS_STATUS_SENSOR_RC_RECEIVER; /* 0x10000 rc receiver | */
	control_sensors_present |= MAV_SYS_STATUS_LOGGING; /* 0x1000000 Logging | */
	control_sensors_present |= MAV_SYS_STATUS_SENSOR_BATTERY; /* 0x2000000 Battery | */

	mavlink_msg_sys_status_send(
		MAVLINK_COMM_1,
		control_sensors_present,
		control_sensors_present,
		control_sensors_present,
		1000,
		Sensor_latest.power * 1000, // mV
		-1,		// in 10mA units
		-1,		// in %
		0, // comm drops %,
		0, // comm drops in pkts,
		0, 0, 0, 0);
}

//���ͽ��յ���ǰ����,GCS���յ�����Ϣ�ᷢ����һ��������
static void send_next_waypoint(){
	mavlink_msg_mission_request_send(
		MAVLINK_COMM_1, 
		vp.waypoint_dest_sysid, 
		vp.waypoint_dest_compid, 
		vp.waypoint_request_i);	//waypoint_request_i����handle_mission_item�и���
}

//��GCS�����Ѿ���ɴ˺���
static void send_mission_item_reached(){
	mavlink_msg_mission_item_reached_send(MAVLINK_COMM_1, vp.mission_item_reached_index);
}

// deferred message handling
enum mav_message deferred_messages[MSG_RETRY_DEFERRED];
u8 next_deferred_message = 0;
u8 num_deferred_messages = 0;

bool gcs_try_send_message(enum mav_message id){
	//���ʣ�µĳ�������ʱ��С��250us���ߴ���MODE_AUTO����ʱ������
	//��С��250usʱ�ҵ������ʱ��ѭ�������ȼ�����ͨѶ
	if((time_available_us()<25 && sys_flag.motors_armed==true) || sys_flag.task_run==true){
		sys_flag.gcs_out_of_time = true;	
		return false;
	}
	switch(id){
		case MSG_HEARTBEAT:
			send_hearbeat();
			send_info();
			break;
		case MSG_RAW_IMU1:
			send_raw_imu();
			break;
		case MSG_RAW_IMU2:
			send_scaled_pressure();
			break;
		case MSG_ATTITUDE:
			send_attitude();
			break;
		case MSG_GPS_RAW:
			send_gps_raw();
			break;
		case MSG_EXTENDED_STATUS1:
			send_extended_status1();
			break;
		case MSG_NEXT_WAYPOINT:
			send_next_waypoint();
			break;
		case MSG_MISSION_ITEM_REACHED:
			send_mission_item_reached();
			break;
		case MSG_STATUSTEXT:
			send_text();
			break;
	}
	return true;
}

//mavlink�ڷ��͵�ʱ�����Ƚ���Ϣ�ŵ��ȴ����Ͷ�������������ʼ�������ݡ�
void send_message(enum mav_message id){
//	if(!sys_flag.mavlink_enable) return;	//���û�н��յ�gcs�������ݣ��˳�
	uint8_t i, nextid;
	while(num_deferred_messages != 0){	//��ǰ�ȴ����Ͷ����ﻹ����Ϣ
		if (!gcs_try_send_message(deferred_messages[next_deferred_message])) {
            break;
    }
		next_deferred_message++;
		if (next_deferred_message == MSG_RETRY_DEFERRED) {
				next_deferred_message = 0;
		}
		num_deferred_messages--;
	}
	if (id == MSG_RETRY_DEFERRED) {
		return;
	}
 // this message id might already be deferred
	for (i=0, nextid = next_deferred_message; i < num_deferred_messages; i++) {
			if (deferred_messages[nextid] == id) {
					// it's already deferred, discard
					return;
			}
			nextid++;
			if (nextid == MSG_RETRY_DEFERRED) {
					nextid = 0;
			}
	}

	if (num_deferred_messages != 0 ||!gcs_try_send_message(id)) {
			// can't send it now, so defer it
			if (num_deferred_messages == MSG_RETRY_DEFERRED) {
					// the defer buffer is full, discard
					return;
			}
			nextid = next_deferred_message + num_deferred_messages;
			if (nextid >= MSG_RETRY_DEFERRED) {
					nextid -= MSG_RETRY_DEFERRED;
			}
			deferred_messages[nextid] = id;
			num_deferred_messages++;
	}
}



/******************************�ָ���********************************/
/***************************����Ϊ�������**************************/

void handle_heartbeat(mavlink_message_t* msg){
	mavlink_heartbeat_t packet;
	mavlink_msg_heartbeat_decode(msg,&packet);
//	printf("recv heartbeat \r\n");
	
}

void handle_set_mode(mavlink_message_t* msg){	//ģʽ���ð�����
	bool result = false;
	mavlink_set_mode_t packet;
	mavlink_msg_set_mode_decode(msg, &packet);
	printf("[gcs->] set_mode:%d %d %d \r\n",packet.custom_mode,packet.target_system,packet.base_mode);
	result = set_mode((CONTROL_MODE)packet.custom_mode);	//�л���Ӧģʽ
	mavlink_msg_command_ack_send_buf(msg, MAVLINK_COMM_1, MAVLINK_MSG_ID_SET_MODE, result);
}

//�����б���
void handle_tasklists(mavlink_message_t* msg){
	mavlink_tasklists_t packet;
	mavlink_msg_tasklists_decode(msg, &packet);
	if(packet.task_1 == 0 && sys_flag.screen_heading){
		printf("ȡ����Ļ���� \r\n");
		send_statustext(MAV_SEVERITY_INFO,"Cancel Screen heading");
		sys_flag.screen_heading = false;
	}
	else if(packet.task_1 == 1){
		vp.screen_heading = packet.task_1_value;
		printf("������Ļ���򣬶���Ƕ� %d \r\n",vp.screen_heading);
		char text[50] = "";
		sprintf(text,"Set Screen heading %d",vp.screen_heading);
		send_statustext(MAV_SEVERITY_INFO,text);
		sys_flag.screen_heading = true;
	}
	if(packet.task_2 == 0 && sys_flag.screen_depth){
		printf("ȡ����Ļ���� \r\n");
		send_statustext(MAV_SEVERITY_INFO,"Cancel Screen DepthHolding");
		sys_flag.screen_depth = false;
	}
	else if(packet.task_2 == 1) {
		vp.screen_depth = packet.task_2_value;
		printf("������Ļ���������� %d \r\n",vp.screen_depth);
		char text[50] = "";
		sprintf(text,"Set Screen Depth %d",vp.screen_depth);
		send_statustext(MAV_SEVERITY_INFO,text);
		sys_flag.screen_depth = true;
	}
//	//�����б�ֵ
//	u8 TASK[][10]={"NONE","forward","backward","leftwrad","rightward","turnLeft","turnRight","depthHold"};
//	for(int i=0;i<8;i++){
//		Task_List[i].task_num = i;	
//		memcpy(Task_List[i].task,TASK[i],10);
//		//Task_List[i].task = TASK[i];
//	}
//	Task_List[0].task_list = packet.task_1;
//	Task_List[0].task_beat = packet.task_1_duration;
//	Task_List[0].task_val =  packet.task_1_value;
//	Task_List[1].task_list = packet.task_2;
//	Task_List[1].task_beat = packet.task_2_duration;
//	Task_List[1].task_val =  packet.task_2_value;
//	Task_List[2].task_list = packet.task_3;
//	Task_List[2].task_beat = packet.task_3_duration;
//	Task_List[2].task_val =  packet.task_3_value;
//	Task_List[3].task_list = packet.task_4;
//	Task_List[3].task_beat = packet.task_4_duration;
//	Task_List[3].task_val =  packet.task_4_value;
//	Task_List[4].task_list = packet.task_5;
//	Task_List[4].task_beat = packet.task_5_duration;
//	Task_List[4].task_val =  packet.task_5_value;
//	Task_List[5].task_list = packet.task_6;
//	Task_List[5].task_beat = packet.task_6_duration;
//	Task_List[5].task_val =  packet.task_6_value;
//	Task_List[6].task_list = packet.task_7;
//	Task_List[6].task_beat = packet.task_7_duration;
//	Task_List[6].task_val =  packet.task_7_value;
//	Task_List[7].task_list = packet.task_8;
//	Task_List[7].task_beat = packet.task_8_duration;
//	Task_List[7].task_val =  packet.task_8_value;

//	for(int i=0;i<8;i++){
//		if(Task_List[i].task_list == 0) break;	//���㵽�հ�Ϊֹ���������
//		vp.task_total_num++;
//	}

//	printf("==============task list============== \r\n");
//	printf("list\ttask\t\tvalue\tduration  \r\n");
//	printf("------------------------------------- \r\n");
//	for(int i=0;i<8;i++){
//		printf(" %d \t %10s \t\t %.1f \t %.1f \t\r\n",i,Task_List[i].task,Task_List[i].task_val,Task_List[i].task_beat);
//	}

}

//�����������б������ر���
void handle_tasklists_about(mavlink_message_t* msg){
	mavlink_tasklists_about_t packet;
	mavlink_msg_tasklists_about_decode(msg, &packet);

	if(packet.task_return)	send_tasklists();//���������б�
	if(packet.task_run){
		if(sys_flag.relay_12V_enable){		
			send_statustext(MAV_SEVERITY_WARNING,"Motors are not powered!please enpower!");
		}
		else if(!sys_flag.motors_armed){		
			send_statustext(MAV_SEVERITY_WARNING,"Motors are disarmed!please arm!");
		}
		else if(!sys_flag.task_tasklists_recved){		
			send_statustext(MAV_SEVERITY_WARNING,"Without Tasklists,please send Tasklists!");
		}
		else if(vp.control_mode!=AUTO){		
			send_statustext(MAV_SEVERITY_WARNING,"Please switch to AUTO mode!");
		}
		else{
			sys_flag.task_run = true; //��ʼ����
		}
	}
}
//����ʱ����Ϣ
void handle_data_time(mavlink_message_t* msg){
	mavlink_data_time_t packet;
	mavlink_msg_data_time_decode(msg, &packet);
	if(!sys_flag.data_recv){	//ֻ����һ��UTCʱ�䣬Ȼ����RTC
		vp.sys_time.year = 2000+packet.year;
		vp.sys_time.month = packet.month;
		vp.sys_time.day = packet.day;
		vp.sys_time.hour = packet.hour;
		vp.sys_time.minute = packet.minute;
		vp.sys_time.second = packet.second;
		vp.sys_time.week = packet.week;
		sys_flag.data_recv = true;
		printf("[gcs->] received UTC time! \r\n");
	}
}

/*
���͵�������
*/
void handle_param_request_read(mavlink_message_t* msg){
	mavlink_param_request_read_t packet;
	mavlink_msg_param_request_read_decode(msg, &packet);
	printf("[gcs->] ask param %d return \r\n",packet.param_index);
	for(int i=0;i<3;i++){
		send_param_index(packet.param_index);	
		delay_ms(50);
	}
}
/*
����ȫ������
*/
void handle_param_request_list(mavlink_message_t* msg){	//������ȡ������
	mavlink_param_request_list_t packet;
	mavlink_msg_param_request_list_decode(msg,&packet);
	printf("[gcs->] send all param \r\n");
//	for(int i=0;i<3;i++) 
		send_all_param();
}
/*
�������ð�����

*/
void handle_param_set(mavlink_message_t* msg){	
	mavlink_param_set_t packet;
	mavlink_msg_param_set_decode(msg, &packet);
	for(int i=0;i<fp.param_num;i++){
		if(strcmp(packet.param_id,param_id_type_value[i].param_id)==0){	//strcmp�ж����ʱ����0
			printf("[gcs->] change %s value to %.2f \r\n",param_id_type_value[i].param_id,packet.param_value);
			char text[50];
			sprintf(text,"change %s value to %.2f",param_id_type_value[i].param_id,packet.param_value);
			send_statustext(MAV_SEVERITY_INFO,(const char*)text);
			//���ݽ��ղ������͵Ĳ�ͬʹ�ò�ͬ��ָ�룬ͬʱ�����޸ĵĲ�����Ȼ��д��flash
			switch(param_id_type_value[i].param_type){
				case UINT8:
					*(u8*)param_id_type_value[i].param_value = (u8)packet.param_value;
					flash_write_param((u8*)param_id_type_value[i].param_value,param_id_type_value[i].param_list);break;
				case INT8:
					*(s8*)param_id_type_value[i].param_value = (s8)packet.param_value;
					flash_write_param((u8*)param_id_type_value[i].param_value,param_id_type_value[i].param_list);break;
				case UINT16:
					*(u16*)param_id_type_value[i].param_value = (u16)packet.param_value;
					flash_write_param((u8*)param_id_type_value[i].param_value,param_id_type_value[i].param_list);break;
				case INT16:
					*(s16*)param_id_type_value[i].param_value = (s16)packet.param_value;
					flash_write_param((u8*)param_id_type_value[i].param_value,param_id_type_value[i].param_list);break;
				case UINT32:
					*(u32*)param_id_type_value[i].param_value = (u32)packet.param_value;
					flash_write_param((u8*)param_id_type_value[i].param_value,param_id_type_value[i].param_list);break;
				case INT32:
					*(s32*)param_id_type_value[i].param_value = (s32)packet.param_value;
					flash_write_param((u8*)param_id_type_value[i].param_value,param_id_type_value[i].param_list);break;
				case UINT64:
					*(unsigned long long*)param_id_type_value[i].param_value = (unsigned long long)packet.param_value;
					flash_write_param((u8*)param_id_type_value[i].param_value,param_id_type_value[i].param_list);break;
				case INT64:
					*(signed long long*)param_id_type_value[i].param_value = (signed long long)packet.param_value;
					flash_write_param((u8*)param_id_type_value[i].param_value,param_id_type_value[i].param_list);break;
				case REAL32:
					*(float*)param_id_type_value[i].param_value = (float)packet.param_value;
					flash_write_param((u8*)param_id_type_value[i].param_value,param_id_type_value[i].param_list);break;
				case REAL64:
					*(double*)param_id_type_value[i].param_value = (double)packet.param_value;
					flash_write_param((u8*)param_id_type_value[i].param_value,param_id_type_value[i].param_list);break;
			}
			//���޸ĺ�Ĳ����ٻش���QGC
			mavlink_msg_param_value_send(
				 MAVLINK_COMM_1, 										//ͨ��
				 param_id_type_value[i].param_id, 	//������
				 packet.param_value,								//����ֵ
				 param_id_type_value[i].param_type,	//��������
				 fp.param_num,											//ȫ����������
				 i);																//�˲����ڲ������еĴ���
			break;
		}
	}
}	

static bool handle_mission_item(mavlink_message_t* msg){
	MAV_MISSION_RESULT result = MAV_MISSION_ACCEPTED;
	Mission_Command cmd = {0};
	
	bool mission_is_complete = false;
	uint16_t seq=0;
//	uint16_t current = 0;
	
	if (msg->msgid == MAVLINK_MSG_ID_MISSION_ITEM) {	  
		mavlink_mission_item_t packet;	  
		mavlink_msg_mission_item_decode(msg, &packet);

//		printf("mission_item= seq=%d command=%d current=%d p1=%.2f p2=%.2f p3=%.2f p4=%.2f x=%.2f y=%.2f z=%.2f \r\n",
//			packet.seq,packet.command,packet.current,packet.param1,packet.param2,packet.param3,packet.param4,packet.x,packet.y,packet.z);
		//��mavlink���ݰ�ת��Ϊ����ָ��
		result = mavlink_to_mission_cmd(packet, &cmd);
		if (result != MAV_MISSION_ACCEPTED) {
			goto mission_ack;
		}
		
		seq = packet.seq;
//		current = packet.current;
	} 
	else {
		mavlink_mission_item_int_t packet;
		mavlink_msg_mission_item_int_decode(msg, &packet);

		result = mavlink_int_to_mission_cmd(packet, &cmd);
		if (result != MAV_MISSION_ACCEPTED) {
			goto mission_ack;
		}
		
		seq = packet.seq;
//		current = packet.current;
	}

	mission_cmd_to_bin(&cmd);

//	if (current == 2) { 											  
//		// current = 2 is a flag to tell us this is a "guided mode"
//		// waypoint and not for the mission
//		result = (handle_guided_request(cmd) ? MAV_MISSION_ACCEPTED
//											 : MAV_MISSION_ERROR) ;

//		// verify we received the command
//		goto mission_ack;
//	}

//	if (current == 3) {
//		//current = 3 is a flag to tell us this is a alt change only
//		// add home alt if needed
//		handle_change_alt_request(cmd);

//		// verify we recevied the command
//		result = MAV_MISSION_ACCEPTED;
//		goto mission_ack;
//	}

//	// Check if receiving waypoints (mission upload expected)
//	if (!waypoint_receiving) {
//		result = MAV_MISSION_ERROR;
//		goto mission_ack;
//	}

	// check if this is the requested waypoint
	if (seq != vp.waypoint_request_i) {
		result = MAV_MISSION_INVALID_SEQUENCE;
		goto mission_ack;
	}

//	// sanity check for DO_JUMP command
//	if (cmd.id == MAV_CMD_DO_JUMP) {
//		if ((cmd.content.jump.target >= mission.num_commands() && cmd.content.jump.target >= waypoint_request_last) || cmd.content.jump.target == 0) {
//			result = MAV_MISSION_ERROR;
//			goto mission_ack;
//		}
//	}
//	
//	// if command index is within the existing list, replace the command
//	if (seq < mission.num_commands()) {
//		if (mission.replace_cmd(seq,cmd)) {
//			result = MAV_MISSION_ACCEPTED;
//		}else{
//			result = MAV_MISSION_ERROR;
//			goto mission_ack;
//		}
//		// if command is at the end of command list, add the command
//	} else if (seq == mission.num_commands()) {
//		if (mission.add_cmd(cmd)) {
//			result = MAV_MISSION_ACCEPTED;
//		}else{
//			result = MAV_MISSION_ERROR;
//			goto mission_ack;
//		}
//		// if beyond the end of the command list, return an error
//	} else {
//		result = MAV_MISSION_ERROR;
//		goto mission_ack;
//	}
//	
//	// update waypoint receiving state machine
//	waypoint_timelast_receive = AP_HAL::millis();
	vp.waypoint_request_i++;
	
	if (vp.waypoint_request_i >= vp.waypoint_count) {
		mavlink_msg_mission_ack_send_buf(msg,MAVLINK_COMM_1,msg->sysid,msg->compid,MAV_MISSION_ACCEPTED);
		send_statustext(MAV_SEVERITY_INFO,"Flight plan received");
		sys_flag.waypoint_receiving = false;
		mission_is_complete = true;
		vp.waypoint_request_i = 0;
	} else {
		send_message(MSG_NEXT_WAYPOINT);	//Ҫ��QGC�����¸�����
	}
	return mission_is_complete;

mission_ack:
	// we are rejecting the mission/waypoint
	mavlink_msg_mission_ack_send_buf(msg,MAVLINK_COMM_1,msg->sysid,msg->compid,result);
	return mission_is_complete;

}

static void handle_mission_request(mavlink_message_t* msg){
	Mission_Command cmd = {0};
    if (msg->msgid == MAVLINK_MSG_ID_MISSION_REQUEST_INT) {  
        // decode
        mavlink_mission_request_int_t packet;
        mavlink_msg_mission_request_int_decode(msg, &packet);

        // retrieve mission from eeprom
        if (!mission_cmd_from_bin(packet.seq, &cmd)) {
            goto mission_item_send_failed;
        }

        mavlink_mission_item_int_t ret_packet;
        memset(&ret_packet, 0, sizeof(ret_packet));
        if (!mission_cmd_to_mavlink_int(&cmd, &ret_packet)) {
            goto mission_item_send_failed;
        }

        // set packet's current field to 1 if this is the command being executed
//        if (cmd.id == (uint16_t)mission.get_current_nav_cmd().index) {
//            ret_packet.current = 1;
//        } else {
//            ret_packet.current = 0;
//        }

        // set auto continue to 1
        ret_packet.autocontinue = 1;     // 1 (true), 0 (false)

        /*
          avoid the _send() function to save memory, as it avoids
          the stack usage of the _send() function by using the already
          declared ret_packet above
         */
        ret_packet.target_system = msg->sysid;
        ret_packet.target_component = msg->compid;
        ret_packet.seq = packet.seq;
        ret_packet.command = cmd.id;
		printf("[mission] send %d th waypoint \r\n",ret_packet.seq);

        _mav_finalize_message_chan_send(MAVLINK_COMM_1, 
                                        MAVLINK_MSG_ID_MISSION_ITEM_INT,
                                        (const char *)&ret_packet,
                                        MAVLINK_MSG_ID_MISSION_ITEM_MIN_LEN,
                                        MAVLINK_MSG_ID_MISSION_ITEM_INT_LEN,
                                        MAVLINK_MSG_ID_MISSION_ITEM_INT_CRC);
    } 
	else {
        // decode
        mavlink_mission_request_t packet;
        mavlink_msg_mission_request_decode(msg, &packet);

        if (packet.seq != 0 && // always allow HOME to be read
            packet.seq >= vp.waypoint_count) {
            // try to educate the GCS on the actual size of the mission:
            mavlink_msg_mission_count_send(MAVLINK_COMM_1,msg->sysid, msg->compid, vp.waypoint_count);
            goto mission_item_send_failed;
        }

		if (!mission_cmd_from_bin(packet.seq, &cmd)) {
			goto mission_item_send_failed;
		}

        mavlink_mission_item_t ret_packet;
        memset(&ret_packet, 0, sizeof(ret_packet));
        if (!mission_cmd_to_mavlink(&cmd, &ret_packet)) {
            goto mission_item_send_failed;
        }
            
        // set packet's current field to 1 if this is the command being executed
//        if (cmd.id == (uint16_t)mission.get_current_nav_cmd().index) {
//            ret_packet.current = 1;
//        } else {
//            ret_packet.current = 0;
//        }

        // set auto continue to 1
        ret_packet.autocontinue = 1;     // 1 (true), 0 (false)

        /*
          avoid the _send() function to save memory, as it avoids
          the stack usage of the _send() function by using the already
          declared ret_packet above
         */
        ret_packet.target_system = msg->sysid;
        ret_packet.target_component = msg->compid;
        ret_packet.seq = packet.seq;
        ret_packet.command = cmd.id;
		printf("[mission] send %d th waypoint \r\n",ret_packet.seq);

        _mav_finalize_message_chan_send(MAVLINK_COMM_1, 
                                        MAVLINK_MSG_ID_MISSION_ITEM,
                                        (const char *)&ret_packet,
                                        MAVLINK_MSG_ID_MISSION_ITEM_MIN_LEN,
                                        MAVLINK_MSG_ID_MISSION_ITEM_LEN,
                                        MAVLINK_MSG_ID_MISSION_ITEM_CRC);
    }

    return;

mission_item_send_failed:
    // send failure message
    mavlink_msg_mission_ack_send(MAVLINK_COMM_1, msg->sysid, msg->compid, MAV_MISSION_ERROR);
}

static void handle_mission_request_list(mavlink_message_t* msg){
	mavlink_mission_request_list_t packet;
	mavlink_msg_mission_request_list_decode(msg, &packet);

	mavlink_msg_mission_count_send(
		MAVLINK_COMM_1, 
		msg->sysid, 
		msg->compid, 
		vp.waypoint_count);

	sys_flag.waypoint_receiving = false;
	vp.waypoint_dest_sysid = msg->sysid;
	vp.waypoint_dest_compid = msg->compid;
}

static void handle_misssion_count(mavlink_message_t* msg){
	mavlink_mission_count_t packet;
	mavlink_msg_mission_count_decode(msg, &packet);
	printf("[gcs->] send %d waypoints \r\n",packet.count);

	vp.waypoint_count = packet.count;
	sys_flag.waypoint_receiving = true;

	send_message(MSG_NEXT_WAYPOINT);
}

/*
��������ȡ������
*/
void set_streamRates(int type, int freq){
	streamRates[type]  = freq;
}
void handle_request_data_stream(mavlink_message_t* msg){
	mavlink_request_data_stream_t packet;
	mavlink_msg_request_data_stream_decode(msg, &packet);
	u16 freq = packet.req_message_rate;

	if(packet.start_stop == 0){ //ֹͣ����
		for (u8 i=0; i<STREAM_PARAMS; i++) {
			streamRates[i] = 0;
		}
		printf("[gcs->] all stream stop return \r\n");
		return;
	}
	u8 _freq=0;
	switch (packet.req_stream_id) {
	    case MAV_DATA_STREAM_ALL:	//0
	    	_freq = fp.stream_freq_enable?fp.stream_ALL_freq:freq;
	        for (u8 i=0; i<STREAM_PARAMS; i++) {
	        	streamRates[i] = _freq;
	        }
			printf("[gcs->] all stream return at %d hz \r\n",_freq);
	        break;
	    case MAV_DATA_STREAM_RAW_SENSORS:	//1
			_freq = fp.stream_freq_enable?fp.stream_RAW_SENSORS_freq:freq;
			streamRates[STREAM_RAW_SENSORS]  = _freq;
			printf("[gcs->] STREAM_RAW_SENSORS return at %d hz \r\n",_freq);
	        break;
	    case MAV_DATA_STREAM_EXTENDED_STATUS:	//2
			_freq = fp.stream_freq_enable?fp.stream_EXTENDED_STATUS_freq:freq;
	        streamRates[STREAM_EXTENDED_STATUS]  = _freq;//freq;;
			printf("[gcs->] STREAM_EXTENDED_STATUS return at %d hz \r\n",_freq);
	        break;
	    case MAV_DATA_STREAM_RC_CHANNELS:	//3
			_freq = fp.stream_freq_enable?fp.stream_RC_CHANNELS_freq:freq;
	        streamRates[STREAM_RC_CHANNELS]  = _freq;//freq;;
			printf("[gcs->] STREAM_RC_CHANNELS return at %d hz \r\n",_freq);
	        break;
	    case MAV_DATA_STREAM_RAW_CONTROLLER:	//4
			_freq = fp.stream_freq_enable?fp.stream_RAW_CONTROLLER_freq:freq;
	        streamRates[STREAM_RAW_CONTROLLER]  = _freq;//freq;;
			printf("[gcs->] STREAM_RAW_CONTROLLER return at %d hz \r\n",_freq);
	        break;
	    case MAV_DATA_STREAM_POSITION:	//6
			_freq = fp.stream_freq_enable?fp.stream_POSITION_freq:freq;
	        streamRates[STREAM_POSITION]  = _freq;//freq;;
			printf("[gcs->] STREAM_POSITION return at %d hz \r\n",_freq);
	        break;
	    case MAV_DATA_STREAM_EXTRA1:	//10
			_freq = fp.stream_freq_enable?fp.stream_EXTRA1_freq:freq;
	        streamRates[STREAM_EXTRA1]  = _freq;//freq;;
	        printf("[gcs->] STREAM_EXTRA1 return at %d hz \r\n",_freq);
			break;
	    case MAV_DATA_STREAM_EXTRA2:	//11
			_freq = fp.stream_freq_enable?fp.stream_EXTRA2_freq:freq;
	        streamRates[STREAM_EXTRA2]  = FPDS_BIT_NUMBER;//freq;;
			printf("[gcs->] STREAM_EXTRA2 return at %d hz \r\n",_freq);
	        break;
	    case MAV_DATA_STREAM_EXTRA3:	//12
			_freq = fp.stream_freq_enable?fp.stream_EXTRA3_freq:freq;
	        streamRates[STREAM_EXTRA3]  = _freq;//freq;;
	        printf("[gcs->] STREAM_EXTRA3 return at %d hz \r\n",_freq);
			break;
    }
	sys_flag.request_data_stream = true;
}	
	
void handle_manual_control(mavlink_message_t* msg){	//�ֱ����ݰ�����
	mavlink_manual_control_t packet;
	mavlink_msg_manual_control_decode(msg, &packet);
	handle_joystick(packet.x,packet.y,packet.z,packet.r,packet.buttons);

}	
void handle_do_motor_test(mavlink_command_long_t command) {	//������԰�����
	if(!sys_flag.motors_armed){
//		gcs_send_text(MAV_SEVERITY_WARNING, "motors are disarmed!");
		send_statustext(MAV_SEVERITY_WARNING,"motors are disarmed!");
		return;
	}

	float motor_number = command.param1;
//	float throttle_type = command.param2;
	float throttle = command.param3;
	// float timeout_s = command.param4; // not used
	// float motor_count = command.param5;
//	float test_type = command.param6;
	
	u16 motor_test_pwm[MOTORS_MAX_NUMER];
	for(u8 i=0;i<MOTORS_MAX_NUMER;i++){
		motor_test_pwm[i] = 1000;
	}
	motor_test_pwm[(u8)motor_number] = (u16)throttle;
		//������������Ƹ˵�ֵ
//		printf("%d %d %d %d \r\n",motor_test.motor1_pwm,motor_test.motor2_pwm,motor_test.motor3_pwm,motor_test.motor4_pwm);
		for(u8 i=0;i<MOTORS_MAX_NUMER;i++){
			Moter_Set_PWM_Pulse(i,motor_test_pwm[i]);
	}
}

void handleMessage(mavlink_message_t* msg){
	 switch (msg->msgid) {
			case MAVLINK_MSG_ID_HEARTBEAT: {    // MAV ID: 0 ������
//				printf("receive heartbeat %02x \r\n",msg->seq);
				// We keep track of the last time we received a heartbeat from our GCS for failsafe purposes
				if (msg->sysid != fp.sysid_my_gcs) {
						break;
				}
//				send_statustext(MAV_SEVERITY_INFO,"QAUV has received Heartbeat!");
				sys_flag.mavlink_enable = true;
				handle_heartbeat(msg);
				vp.last_gcs_heartbeat_time = sys_time.one_second;//��¼������յ�����վ����������ʱ��
				break;
			}
			case MAVLINK_MSG_ID_SET_MODE: {     // MAV ID: 11 �޸�ģʽ
				handle_set_mode(msg);
				break;
			}
			case MAVLINK_MSG_ID_TASKLISTS:	// MAV ID: 12 ��ȡ�����б�
//				send_statustext(MAV_SEVERITY_INFO,"QAUV has received TaskLists!");
				sys_flag.task_tasklists_recved = true;
				vp.task_total_num = 0;
				handle_tasklists(msg);
				break;
			case MAVLINK_MSG_ID_TASKLISTS_ABOUT:	// MAV ID: 13 ���������б���ز������ش�������
				handle_tasklists_about(msg);
				break;
			case MAVLINK_MSG_ID_DATA_TIME:
				handle_data_time(msg);
				break;
			case MAVLINK_MSG_ID_PARAM_REQUEST_READ:{
				handle_param_request_read(msg);
				sys_flag.param_all_send = false;	//�յ�����������˵����λ��δ���յ��������ݣ��ݶ��������ݵķ���
				vp.mavlink_param_request_second = sys_time.one_second;	//����qgc��ȡ������ʱ��
				break;
			}
			case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: {       // MAV ID: 21 ��ȡ����
				//TODO ���͹̼��汾��
				sys_flag.param_all_send = false;	//ֻҪ���յ���ȡȫ������ָ��Ͱѱ�־λ��λ���Ӷ��������������ݣ�ֻ���Ͳ���
				handle_param_request_list(msg);
				break;
			}
			case MAVLINK_MSG_ID_PARAM_SET: {       // MAV ID: 23 �޸Ĳ���:PID,������,�̵���,��ȫ����
				handle_param_set(msg);
				break;
			}
			case MAVLINK_MSG_ID_MISSION_ITEM:	//MAV ID: 39 73 ����gcs�������б����� -> ������case����Ϊ���ݸĶ�ǰ���mavlinkЭ��
			case MAVLINK_MSG_ID_MISSION_ITEM_INT:
				handle_mission_item(msg);
				break;
			case MAVLINK_MSG_ID_MISSION_REQUEST:	//MAV ID: 40 51 �������б��͸�gcs
			case MAVLINK_MSG_ID_MISSION_REQUEST_INT:
//				printf("[->gcs] Send Mission to GCS \r\n");
				handle_mission_request(msg);
				break;
			case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:	//MAV ID: 43 
				handle_mission_request_list(msg);
				break;
			case MAVLINK_MSG_ID_MISSION_COUNT:	//MAV ID: 44 GCS���ͺ���ǰ�����ȷ��ͺ������
				handle_misssion_count(msg);
				break;
			case MAVLINK_MSG_ID_REQUEST_DATA_STREAM: {       // MAV ID: 66 ������������ȡ��,��λ����⵽pix�����ʼ���ͣ�����һ��ʱ���ֹͣ����
				handle_request_data_stream(msg);
				break;
			}
			case MAVLINK_MSG_ID_MANUAL_CONTROL: {       // MAV ID: 69 �����ֱ����룬����ҡ�˺Ͱ���
				if(sys_flag.ppm_connected) break;	//������յ�����ң������������usb�ֱ�
				sys_flag.joy_connected = true;
				handle_manual_control(msg);
				break;
			}
			case MAVLINK_MSG_ID_RADIO_STATUS:{
				mavlink_radio_status_t packet;
				mavlink_msg_radio_status_decode(msg, &packet);
//				printf("%d %d %d \r\n", packet.rxerrors, packet.fixed, packet.txbuf);
				break;
			}
			case MAVLINK_MSG_ID_COMMAND_LONG: {       // MAV ID: 76
				mavlink_command_long_t packet;
		        mavlink_msg_command_long_decode(msg, &packet);
				u8 result = MAV_RESULT_FAILED;
		        switch (packet.command) {
					case MAV_CMD_DO_MOTOR_TEST:	//CMD_ID=#209(0xd1)	������԰�,��Ҫ�Ƚ������
						handle_do_motor_test(packet);
						break;
					case MAV_CMD_COMPONENT_ARM_DISARM: //�������������
						printf("[gcs->] ARM command:%.1f \r\n",packet.param1);
						result = MAV_RESULT_ACCEPTED;
						if(is_equal(packet.param1, 1.0f)){	//����
							//TODO ����ѹ
							sys_flag.motors_armed = true;
//							sys_flag.relay_12V_enable = true;
						}
						else{
							sys_flag.motors_armed = false;
						}
						break;
					case MAV_CMD_MISSION_START:	//����ʼ
						printf("[gcs->] mission start \r\n");
						if(sys_flag.motors_armed && set_mode(AUTO)){
							result = MAV_RESULT_ACCEPTED;
						}
						break;
				}
				mavlink_msg_command_ack_send_buf(msg,  MAVLINK_COMM_1,  packet.command, result);
				break;
			}
		}
}

void gcs_update(){
	u16 nbytes =0;
	u8 *recv_buf = (u8*)mymalloc(SRAMIN, 2*1024);	//��̬����2K�ڴ�
	// receive new packets
	mavlink_message_t msg;
	mavlink_status_t status;
	status.packet_rx_drop_count = 0;
	static int recv_pac_num = 0;
	
	if(fp.ROV_mode_enable && sys_flag.relay_12V_enable){
		do_udp(recv_buf,&nbytes);
	}
	else{
		if(sys_flag.mavlink_recv_dma){
			if(!sys_flag.mavlink_recv) {
				myfree(SRAMIN, recv_buf);
				return;
			}
			sys_flag.mavlink_recv = false;
		}
		else if(mav_uart_available==0){
			myfree(SRAMIN, recv_buf);
			return;
		}
		nbytes = mav_uart_available;
		memcpy(recv_buf,mav_uart_buf,nbytes);
		mav_uart_available = 0;//��ͷ��ʼ���գ���ҪΪ�˽���dmaû��ʹ�����
	}
	for (u16 i=0; i<nbytes; i++){
		u8 c = *(recv_buf+i);
		if(sys_flag.show_mavlink_stream2){printf("%02x ",c);}
		// Try to get a new message
		//mavlink_parse_char�����Բ��Ͻ���һ���ֽڵ����ݣ����ڽ��յ�����һ������ʱ����1�����򷵻�0��������һ��mavlink_message_t���͵����ݡ�
		if (mavlink_parse_char(MAVLINK_COMM_1, c, &msg, &status)) {	
			if(sys_flag.show_mavlink_stream1){printf("\r\n[mavlink] seq=%02x mid=%02x \r\n",msg.seq,msg.msgid);}
			recv_pac_num++;
			if(msg.seq == 0xff){	//���㶪����
				vp.mavlink_recv_percentage = recv_pac_num*100/256;
				if(vp.mavlink_recv_percentage>100) vp.mavlink_recv_percentage /= (vp.mavlink_recv_percentage/100);	//û���յ�0xff,ȡƽ��
				recv_pac_num = 0;
			}
//			if (msg.sysid == mavlink_system.sysid && msg.compid == mavlink_system.compid) 
			{
				handleMessage(&msg);
			}
		}
	}
	if(sys_time.one_second-vp.mavlink_param_request_second >= 3){ //3s��û���յ���λ��Ҫ���Ͳ�����ָ��
		sys_flag.param_all_send = true;	//������������������
	}
	
	myfree(SRAMIN, recv_buf);
//	if(!sys_flag.waypoint_receiving) return;
//	u32 wp_recv_time = 1000;
//	u32 tnow = sys_time.one_mill;

//	//if(tnow - vp.waypoint_timelast_request)
//	if(sys_flag.waypoint_receiving && (tnow-vp.waypoint_timelast_request)>wp_recv_time){
//		vp.waypoint_timelast_request = tnow;
//		send_message(MSG_NEXT_WAYPOINT);
//	}
}
