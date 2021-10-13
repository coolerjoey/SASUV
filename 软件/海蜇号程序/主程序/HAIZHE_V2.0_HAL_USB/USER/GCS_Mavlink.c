#include "GCS_Mavlink.h"
#include "GCS_Common.h"
#include "parameter.h"

u32 recv_heartbeat_sec=0;

void gcs_send_heartbeat(){
	send_message(MSG_HEARTBEAT);
//	static u8 num = 0;
//	char text[50];
//	if(num==100) num=0;
//	sprintf(text,"hello GCS %d",num++);
//	gcs_send_text(MAV_SEVERITY_INFO,text);
}

//gcs设置回传数据流频率 -> 一般用在下位机断电后重连qgc，但qgc不会发送数据流回传指令的情况
void gcs_set_data_stream(){
	if(!sys_flag.health.gcs_heartbeat) return;	 //和上位机通信丢失，返回
	if(sys_flag.request_data_stream) return;	//若已经获取到数据流触发指令，退出
	if(sys_time.one_second-recv_heartbeat_sec>=3){	//接收到QGC心跳包3S内没有收到数据流触发指令，则设置为默认值
		set_streamRates(STREAM_RAW_SENSORS, fp.stream_RAW_SENSORS_freq);
		set_streamRates(STREAM_EXTENDED_STATUS, fp.stream_EXTENDED_STATUS_freq);
		set_streamRates(STREAM_RC_CHANNELS, fp.stream_RC_CHANNELS_freq);
		set_streamRates(STREAM_RAW_CONTROLLER, fp.stream_RAW_CONTROLLER_freq);
		set_streamRates(STREAM_POSITION, fp.stream_POSITION_freq);
		set_streamRates(STREAM_EXTRA1, fp.stream_EXTRA1_freq);
		set_streamRates(STREAM_EXTRA2, fp.stream_EXTRA2_freq);
		set_streamRates(STREAM_EXTRA3, fp.stream_EXTRA3_freq);	
		sys_flag.request_data_stream = true;	//标记设置完成
	}
}

//向地面站发送数据流
void gcs_data_stream_send(){
	if(!sys_flag.health.gcs_heartbeat){
		recv_heartbeat_sec = sys_time.one_second;
		return;	 //和上位机通信丢失，返回
	}
	if(sys_flag.waypoint_receiving){	//在航点接收期间不发送以下信息

		return;
	} 
	gcs_set_data_stream();
	sys_flag.gcs_out_of_time = false;
	if(sys_flag.gcs_out_of_time) return;
	if(!sys_flag.param_all_send) return;	//参数未发送完不发送以下数据
	if(stream_trigger(STREAM_RAW_SENSORS)){	//传感器原始数据
//		printf("[->gcs] send STREAM_RAW_SENSORS \r\n");
		send_message(MSG_RAW_IMU1);			//IMU原始数据
		send_message(MSG_RAW_IMU2);			//深度
	}
	if(sys_flag.gcs_out_of_time) return;
	if(stream_trigger(STREAM_EXTENDED_STATUS)){
		send_message(MSG_GPS_RAW);			//GPS
		send_message(MSG_EXTENDED_STATUS1);
	}
	if(sys_flag.gcs_out_of_time) return;
	if(stream_trigger(STREAM_RC_CHANNELS)){}
	if(sys_flag.gcs_out_of_time) return;
	if(stream_trigger(STREAM_RAW_CONTROLLER)){}
	if(sys_flag.gcs_out_of_time) return;
	if(stream_trigger(STREAM_POSITION)){}
	if(sys_flag.gcs_out_of_time) return;
	if(stream_trigger(STREAM_EXTRA1)){
		send_message(MSG_ATTITUDE);			//姿态数据
	}
	if(sys_flag.gcs_out_of_time) return;
	if(stream_trigger(STREAM_EXTRA2)){}
	if(sys_flag.gcs_out_of_time) return;
	if(stream_trigger(STREAM_EXTRA3)){}
	if(sys_flag.gcs_out_of_time) return;
	if(stream_trigger(STREAM_PARAMS)){}
	if(sys_flag.gcs_out_of_time) return;
	if(stream_trigger(STREAM_ADSB)){}
	if(sys_flag.gcs_out_of_time) return;
	if(stream_trigger(NUM_STREAMS)){}

}
//发送等待队列里的mavlink信息
void gcs_send_deferred()
{
   send_message(MSG_RETRY_DEFERRED);	
}
//发送文本信息
void gcs_send_text(MAV_SEVERITY severity, const char *str){
	vp.severity = severity;
	sprintf(vp.text,"%s",str);
	//ardusub里面没有使用MSG_STATUSTEXT?
//	send_statustext(severity,str);
	send_message(MSG_STATUSTEXT);
}

void gcs_send_msg(){
	send_message(MSG_STATUSTEXT);
}

void gcs_check_input(){
	gcs_update();
}

void gcs_send_message(enum mav_message id){
	send_message(id);
}

void gcs_send_allparams(){
	send_all_param();
}

