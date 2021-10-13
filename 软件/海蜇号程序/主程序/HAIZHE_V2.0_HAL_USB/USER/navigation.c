#include "navigation.h"
#include "mission.h"
#include "parameter.h"
#include "sensor.h"
#include "position_vector.h"
#include "GCS_Mavlink.h"

//计算到下个航点的距离和航向
void calc_wp_distance_and_bearing(){	
	vec3f curr,destination;
	static int num = 0;
	switch(vp.control_mode){
		case LOITER:
		case CIRCLE:
//			vp.wp_distance = wp_nav_get_loiter_distance_to_target();	//在运动控制器中更新
//			vp.wp_bearing = wp_nav_get_loiter_bearing_to_target();
			break;
		case AUTO:
		case RTL:
			pv_location_to_vector(Sensor_latest.position, curr);	//计算当前位置到home点的位置矢量
			pv_location_to_vector(wp_nav.destination, destination);	//计算目标航点到home点的位置矢量
			vp.wp_distance = pv_get_horizontal_distance_cm(curr, destination);	//计算前位置到目标航点的距离
			vp.wp_bearing = pv_get_bearing_cd(curr, destination);		//计算前位置到目标航点的方向
			break;
		default:
			vp.wp_distance = 0;
			vp.wp_bearing = 0;
			break;
	}
	//将距离和方向发送给上位机
	if(sys_flag.GCS_show_WP_infor && num++==200){
		num=0;
		char text[50];
		sprintf(text,"[mission] dis=%.2f m, orient=%.2f deg \r\n",(float)vp.wp_distance/100.0f,(float)vp.wp_bearing/100.0f);
		printf(text);
		gcs_send_text(MAV_SEVERITY_INFO,(const char*)text);
	}
}

//计算当前到返航点的距离和航向
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

	if(vp.control_mode == AUTO){	//在AUTO模式下，进行航点间切换 -> 修改vp.auto_mode
		mission_update();
	}
}


