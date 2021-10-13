#include "navigation.h"
#include "mission.h"
#include "parameter.h"
#include "sensor.h"
#include "position_vector.h"
#include "GCS_Mavlink.h"

//���㵽�¸�����ľ���ͺ���
void calc_wp_distance_and_bearing(){	
	vec3f curr,destination;
	static int num = 0;
	switch(vp.control_mode){
		case LOITER:
		case CIRCLE:
//			vp.wp_distance = wp_nav_get_loiter_distance_to_target();	//���˶��������и���
//			vp.wp_bearing = wp_nav_get_loiter_bearing_to_target();
			break;
		case AUTO:
		case RTL:
			pv_location_to_vector(Sensor_latest.position, curr);	//���㵱ǰλ�õ�home���λ��ʸ��
			pv_location_to_vector(wp_nav.destination, destination);	//����Ŀ�꺽�㵽home���λ��ʸ��
			vp.wp_distance = pv_get_horizontal_distance_cm(curr, destination);	//����ǰλ�õ�Ŀ�꺽��ľ���
			vp.wp_bearing = pv_get_bearing_cd(curr, destination);		//����ǰλ�õ�Ŀ�꺽��ķ���
			break;
		default:
			vp.wp_distance = 0;
			vp.wp_bearing = 0;
			break;
	}
	//������ͷ����͸���λ��
	if(sys_flag.GCS_show_WP_infor && num++==200){
		num=0;
		char text[50];
		sprintf(text,"[mission] dis=%.2f m, orient=%.2f deg \r\n",(float)vp.wp_distance/100.0f,(float)vp.wp_bearing/100.0f);
		printf(text);
		gcs_send_text(MAV_SEVERITY_INFO,(const char*)text);
	}
}

//���㵱ǰ��������ľ���ͺ���
void calc_home_distance_and_bearing(){
	if(sys_check_position()){
		vec3f home, curr;
		pv_location_to_vector(wp_nav.home, home);
		pv_location_to_vector(Sensor_latest.position, curr);
		vp.home_distance = pv_get_horizontal_distance_cm(curr, home);
		vp.home_bearing = pv_get_bearing_cd(curr, home);
	}

}

void run_nav_updates(){
	calc_wp_distance_and_bearing();
	calc_home_distance_and_bearing();

	if(vp.control_mode == AUTO){	//��AUTOģʽ�£����к�����л� -> �޸�vp.auto_mode
		mission_update();
	}
}


