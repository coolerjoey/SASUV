#include "commands_logic.h"
#include "parameter.h"
#include "GCS_Mavlink.h"
#include "mode_auto.h"
#include "sensor.h"
#include "position_vector.h"
#include "mymath.h"

bool verifly_submerge(const Mission_Command *cmd){
	return true;
}

bool verifly_wp(const Mission_Command *cmd){
	if(wp_nav.flag.reached_destination) return true;
	if(vp.wp_distance <= fp.wp_radius_cm){
		wp_nav.flag.reached_destination = true;
		char msg[50];
		sprintf(msg,"Reached command #%d",cmd->index);
		gcs_send_text(MAV_SEVERITY_INFO, msg);
		printf("[mission] reached %d waypoint \r\n",cmd->index);
	}
	return wp_nav.flag.reached_destination;
}
bool verifly_loiter(){return true;}
bool verifly_surface(){return true;}
bool verifly_circle(){return true;}

//检测当前航点是否完成
bool verify_command(const Mission_Command *cmd){
	bool cmd_complete = false;
	switch(cmd->id){
		case MAV_CMD_NAV_WAYPOINT:
			cmd_complete = verifly_wp(cmd);break;
		case MAV_CMD_NAV_RETURN_TO_LAUNCH:
//				cmd_complete = verifly_wp(cmd);
			break;
		case MAV_CMD_NAV_LOITER_UNLIM: 
			cmd_complete = verifly_loiter();break;
		case MAV_CMD_NAV_LAND: 
			cmd_complete = verifly_surface();break;
		case MAV_CMD_NAV_LOITER_TURNS: 
			cmd_complete = verifly_circle();break;
		case MAV_CMD_NAV_TAKEOFF:
			cmd_complete = verifly_submerge(cmd);break;
	}
	if(cmd_complete){
		printf("[mission] %d th mission finish! \r\n",cmd->index);
		vp.mission_item_reached_index = cmd->index;
		gcs_send_message(MSG_MISSION_ITEM_REACHED);
		wp_nav.flag.reached_destination = false;
	}
	return cmd_complete;
}



void do_nav_wp(const Mission_Command *cmd){
	//判断cmd的经纬度是否为0
	Location target_loc = cmd->content.location;
	if(target_loc.lat==0 && target_loc.lng==0){
		target_loc.lat = Sensor_latest.position.lat;
		target_loc.lat = Sensor_latest.position.lng;
	}
	//[todo] 深度为0 -> 参考home点？
	if(target_loc.alt == 0){

	}
	
	auto_wp_start(target_loc);

}
void do_loiter_unlimited(const Mission_Command *cmd){
	auto_loiter_start(cmd);
}
void do_surface(const Mission_Command *cmd){
	if(cmd->content.location.lat!=0 || cmd->content.location.lng!=0){
		Location target_loc = cmd->content.location;
		auto_wp_start(target_loc);
		return;
	}
	auto_surface_start();
}
void do_circle(const Mission_Command *cmd){
	auto_circle_start(cmd);
}
void do_submerge(const Mission_Command *cmd){
	auto_submerge_start(cmd->content.location);	//设置下沉深度
}


//开始下个航点
bool start_command(const Mission_Command *cmd){
	switch(cmd->id){
		case MAV_CMD_NAV_WAYPOINT:
			do_nav_wp(cmd);break;
		case MAV_CMD_NAV_LOITER_UNLIM: 
			do_loiter_unlimited(cmd);break;
		case MAV_CMD_NAV_LAND: 
			do_surface(cmd);break;
		case MAV_CMD_NAV_LOITER_TURNS: 
			do_circle(cmd);break;
		case MAV_CMD_NAV_TAKEOFF:
			do_submerge(cmd);break;
	}
	printf("[mission] start %d th misson \r\n",cmd->index);
	return true;
}

//任务完成，退出
void exit_mission(){
	if(fp.exit_mission_rtl){	
		if(!set_mode(RTL)) set_mode(MANUAL);	//若设置返航失败，设置为MUNUAL模式
	}
	else{
		set_mode(MANUAL);
	}
}

