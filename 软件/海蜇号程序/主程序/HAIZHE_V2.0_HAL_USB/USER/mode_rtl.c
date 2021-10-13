#include "mode_rtl.h"
#include "GCS_Mavlink.h"
#include "mission.h"
#include "mymath.h"
#include "global_para.h"
#include "motors.h"
#include "attitude_controller.h"
#include "navigation.h"
#include "position_vector.h"
#include "commands_logic.h"


bool rtl_init(){
	//�жϵ�ǰ�Ƿ��ѵ���HOME��
	if(is_zero(wp_nav.home.lat) || !mission_reset()){
		printf("HOME didn't import! Please check waypoint update! \r\n");
		gcs_send_text(MAV_SEVERITY_WARNING, "HOME didn't import! Please check waypoint update!");
		return false;
	}
	memcpy(&wp_nav.destination, &wp_nav.home,sizeof(Location)); 	//��Ŀ�����ΪHOME��
	return true;
}

void rtl_run(){
	run_nav_updates();
	if(verify_command(&_nav_cmd)){	//����HOME�㣬����������л����ֶ�ģʽ
		printf("Return HOME successful! \r\ns");
		gcs_send_text(MAV_SEVERITY_INFO, "Return HOME successful!");
		sys_flag.motors_armed = false;
		vp.control_mode = MANUAL;
		return;
	}
	if(!sys_flag.motors_armed || sys_flag.battery_voltage_low){
		reset_att_controller(); 
		return;
	}
	
	att_controller_run(fp.attitude_init[0], fp.attitude_init[1], (float)vp.wp_bearing/100.0f);//������̬������

	set_forward(fp.forward_auto, INPUT_AUTO);
	set_lateral(0, INPUT_AUTO);
	set_throttle(0, INPUT_AUTO);	//��ˮ�淵��
}

