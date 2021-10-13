#include "mode_auto.h"
#include "global_para.h"
#include "motors.h"
#include "attitude_controller.h"
#include "position_controller.h"
#include "navigation.h"
#include "mission.h"
#include "position_vector.h"
#include "kf.h"
#include "model.h"

u8 auto_init(){
	if(!sys_check_ins() || !sys_check_position() || vp.waypoint_count<=1){	//检查ahrs
		printf("[chmod] change mode to \"AUTO\" Failed!\r\n ");
		return false;
	}
	if(fp.dr_mode == DR_MA || fp.dr_mode==DR_SINS_MA)
		model_init(Sensor_latest.ins.euler,O31,Sensor_latest.gpspos,0.01);	//动力学模型初始化
	if(fp.dr_mode!=DR_SINS && fp.dr_mode!=DR_MA && fp.dr_mode!=DR_GPS){	 	
		vec3f gpspos = {28.46886569, 118.54543111, 0 }; 
		KF_init(O31, gpspos, 0.01);	//不是单一传感器进行导航，就需要初始化滤波器
	}
	printf("[mission] start! \r\n");
	return(mission_start());	
}

//
void auto_submerge_start(const Location dest_loc){}
void auto_wp_start(const Location dest_loc){
	vp.auto_mode = Auto_WP;
	
	wp_nav.destination = dest_loc;	//设定下个目标航点 -> 用于计算角度和距离
	
}
void auto_wp_run(){
	if(!sys_flag.motors_armed || sys_flag.battery_voltage_low){
		reset_att_controller();	
		return;
	}

	if(!is_zero(channel.yaw)){	//有遥控输入
//		reset_att_controller();	
//		set_yaw(channel.yaw,INPUT_MANUAL);
	}
	
//	pos_z_controller_run(vp.screen_depth);//运行Z轴控制器 wp_nav.destination.alt
	att_controller_run(fp.attitude_init[0], fp.attitude_init[1], (float)vp.wp_bearing/100.0f);//运行姿态控制器

	set_forward(fp.forward_auto, INPUT_AUTO);
	set_lateral(0, INPUT_AUTO);
	
}

void auto_loiter_start(const Mission_Command *cmd){}
void auto_surface_start(){}
void auto_circle_start(const Mission_Command *cmd){}


void auto_submerge_run(){}
void auto_loiter_run(){}
void auto_surface_run(){}
void auto_circle_run(){}



//任务规划模式
void auto_run(){
	run_nav_updates();//航点切换 -> 修改auto_mode
	switch(vp.auto_mode){
		case Auto_Submerge: 
			auto_submerge_run();
			break;
		case Auto_WP: 
			auto_wp_run();
			break;
		case Auto_Loiter: 
			auto_loiter_run();
			break;
		case Auto_Surface: 
			auto_surface_run();
			break;
		case Auto_Circle: 
			auto_circle_run();
			break;
	}
}




