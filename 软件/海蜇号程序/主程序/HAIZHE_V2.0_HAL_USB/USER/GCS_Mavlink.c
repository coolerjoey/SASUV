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

//gcs���ûش�������Ƶ�� -> һ��������λ���ϵ������qgc����qgc���ᷢ���������ش�ָ������
void gcs_set_data_stream(){
	if(!sys_flag.health.gcs_heartbeat) return;	 //����λ��ͨ�Ŷ�ʧ������
	if(sys_flag.request_data_stream) return;	//���Ѿ���ȡ������������ָ��˳�
	if(sys_time.one_second-recv_heartbeat_sec>=3){	//���յ�QGC������3S��û���յ�����������ָ�������ΪĬ��ֵ
		set_streamRates(STREAM_RAW_SENSORS, fp.stream_RAW_SENSORS_freq);
		set_streamRates(STREAM_EXTENDED_STATUS, fp.stream_EXTENDED_STATUS_freq);
		set_streamRates(STREAM_RC_CHANNELS, fp.stream_RC_CHANNELS_freq);
		set_streamRates(STREAM_RAW_CONTROLLER, fp.stream_RAW_CONTROLLER_freq);
		set_streamRates(STREAM_POSITION, fp.stream_POSITION_freq);
		set_streamRates(STREAM_EXTRA1, fp.stream_EXTRA1_freq);
		set_streamRates(STREAM_EXTRA2, fp.stream_EXTRA2_freq);
		set_streamRates(STREAM_EXTRA3, fp.stream_EXTRA3_freq);	
		sys_flag.request_data_stream = true;	//����������
	}
}

//�����վ����������
void gcs_data_stream_send(){
	if(!sys_flag.health.gcs_heartbeat){
		recv_heartbeat_sec = sys_time.one_second;
		return;	 //����λ��ͨ�Ŷ�ʧ������
	}
	if(sys_flag.waypoint_receiving){	//�ں�������ڼ䲻����������Ϣ

		return;
	} 
	gcs_set_data_stream();
	sys_flag.gcs_out_of_time = false;
	if(sys_flag.gcs_out_of_time) return;
	if(!sys_flag.param_all_send) return;	//����δ�����겻������������
	if(stream_trigger(STREAM_RAW_SENSORS)){	//������ԭʼ����
//		printf("[->gcs] send STREAM_RAW_SENSORS \r\n");
		send_message(MSG_RAW_IMU1);			//IMUԭʼ����
		send_message(MSG_RAW_IMU2);			//���
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
		send_message(MSG_ATTITUDE);			//��̬����
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
//���͵ȴ��������mavlink��Ϣ
void gcs_send_deferred()
{
   send_message(MSG_RETRY_DEFERRED);	
}
//�����ı���Ϣ
void gcs_send_text(MAV_SEVERITY severity, const char *str){
	vp.severity = severity;
	sprintf(vp.text,"%s",str);
	//ardusub����û��ʹ��MSG_STATUSTEXT?
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

