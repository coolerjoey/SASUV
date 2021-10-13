#include "mission.h"
#include "exfuns.h"
#include "parameter.h"
#include "defines.h"
#include "commands_logic.h"


/*
	mission���ղ���
	1. ����������� -> MAVLINK_MSG_ID_MISSION_COUNT

*/
Mission_Command _nav_cmd;	//�������еĵ�ǰ����
WP_NAV wp_nav={0};
char mission_file_name[20];

// return true when lat and lng are within range
bool check_lat_float(float lat){return fabsf(lat) <= 90;}   
bool check_lng_float(float lng){return fabsf(lng) <= 180;}
bool check_lat_int32(int32_t lat){return fabs(lat) <= 90*1e7;}
bool check_lng_int32(int32_t lng){return fabs(lng) <= 180*1e7;}
bool check_latlng_float(float lat, float lng){return check_lat_float(lat) && check_lng_float(lng);}
bool check_latlng_int32(int32_t lat, int32_t lng){return check_lat_int32(lat) && check_lng_int32(lng);}
bool check_latlng_loc(Location loc){return check_lat_int32(loc.lat) && check_lng_int32(loc.lng);}

//�����㱣��Ϊmission.bin�ļ�
void mission_cmd_to_bin(const Mission_Command *cmd){	
 //	FIL *file_root;		  //�ļ�1
	FRESULT res;
 	if(cmd->index == 0){ //��һ������
		
		wp_nav.home = cmd->content.location;	//����home��
	
		res = f_open(ftemp, mission_file_name, FA_CREATE_NEW | FA_WRITE); 
		if(res == FR_OK){
			printf("[mission] create %s \r\n",mission_file_name);
		}
		else if(res == FR_EXIST){	//����ͬ���ļ�����д
			f_unlink(mission_file_name);
			f_open(ftemp, mission_file_name, FA_CREATE_NEW | FA_WRITE); 
			printf("[mission] rewrite %s \r\n",mission_file_name);
		}
	}

	f_lseek(ftemp,f_size(ftemp));
	f_write(ftemp, &cmd->index, sizeof(cmd->id), &bw);
	f_write(ftemp, &cmd->id, sizeof(cmd->index), &bw);
	f_write(ftemp, &cmd->p1, sizeof(cmd->p1), &bw);
	f_write(ftemp, &cmd->content, sizeof(cmd->content), &bw);
	
	printf("[waypoint] seq=%d lat=%f long=%f alt=%.2fm \r\n",cmd->index,cmd->content.location.lat/1e7f,cmd->content.location.lng/1e7f,cmd->content.location.alt/100.0f);
	if(cmd->index == vp.waypoint_count-1){
		f_close(ftemp);
		printf("[mission] write %s finished !\r\n",mission_file_name);
	}
	
}

//�Ӻ����ļ��ж�ȡ
//bool mission_cmd_from_bin(u16 index, Mission_Command *cmd){
//	if(index > vp.waypoint_count) return false;
//	FRESULT res;
//	if(index == 0){	//������
//		res = f_open(ftemp, mission_file_name, FA_OPEN_EXISTING | FA_READ); 
//		if(res != FR_OK){
//			printf("[mission] open %s ERROR \r\n",mission_file_name);
//			return false;
//		}
//	}
//	u16 pos_in_bin = index * MISSION_BIN_COMMAND_SIZE;	//��ȡ��������waypoint.bin�ļ��е�λ��
//	res = f_lseek(ftemp, pos_in_bin);
//	if(res != FR_OK){
//		printf("[mission] seek %d bytes in %s ERROR \r\n",pos_in_bin,mission_file_name);
//		f_close(ftemp);
//		return false;
//	}
//	res = f_read(ftemp, cmd, MISSION_BIN_COMMAND_SIZE, &br);
//	if(res != FR_OK){
//		printf("[mission] read %s ERROR \r\n",mission_file_name);
//		f_close(ftemp);
//		return false;
//	}
//	printf("[mission] send %d th waypoint \r\n",index);
//	if(index == vp.waypoint_count-1){
//		f_close(ftemp);
//	}
//	return true;
//	
//}
bool mission_cmd_from_bin(u16 index, Mission_Command *cmd){
	if(index > vp.waypoint_count) return false;
	FRESULT res;
	res = f_open(ftemp, mission_file_name, FA_OPEN_EXISTING | FA_READ); 
	if(res != FR_OK){
		printf("[mission] open %s ERROR \r\n",mission_file_name);
		return false;
	}
	u16 pos_in_bin = index * MISSION_BIN_COMMAND_SIZE;	//��ȡ��������waypoint.bin�ļ��е�λ��
	res = f_lseek(ftemp, pos_in_bin);
	if(res != FR_OK){
		printf("[mission] seek %d bytes in %s ERROR \r\n",pos_in_bin,mission_file_name);
		f_close(ftemp);
		return false;
	}
	res = f_read(ftemp, cmd, MISSION_BIN_COMMAND_SIZE, &br);
	if(res != FR_OK){
		printf("[mission] read %s ERROR \r\n",mission_file_name);
		f_close(ftemp);
		return false;
	}

	f_close(ftemp);
	
	return true;
	
}


//��mavlink���ݰ�ת��Ϊ����ָ��
MAV_MISSION_RESULT mavlink_to_mission_cmd(const mavlink_mission_item_t packet, Mission_Command *cmd){
	mavlink_mission_item_int_t mav_cmd = {0};

    mav_cmd.param1 = packet.param1;
    mav_cmd.param2 = packet.param2;
    mav_cmd.param3 = packet.param3;
    mav_cmd.param4 = packet.param4;
    mav_cmd.z = packet.z;
    mav_cmd.seq = packet.seq;
    mav_cmd.command = packet.command;
    mav_cmd.target_system = packet.target_system;
    mav_cmd.target_component = packet.target_component;
    mav_cmd.frame = packet.frame;
    mav_cmd.current = packet.current;
    mav_cmd.autocontinue = packet.autocontinue;
    
    /*  
      the strategy for handling both MISSION_ITEM and MISSION_ITEM_INT
      is to pass the lat/lng in MISSION_ITEM_INT straight through, and
      for MISSION_ITEM multiply by 1e7 here. We need an exception for
      any commands which use the x and y fields not as
      latitude/longitude.
     */
    switch (packet.command) {
	    case MAV_CMD_DO_DIGICAM_CONTROL:
	    case MAV_CMD_DO_DIGICAM_CONFIGURE:
	        mav_cmd.x = packet.x;
	        mav_cmd.y = packet.y;
	        break;

	    default:
	        // all other commands use x and y as lat/lon. We need to
	        // multiply by 1e7 to convert to int32_t
	        if (!check_lat_float(packet.x)) {
	            return MAV_MISSION_INVALID_PARAM5_X;
	        }
	        if (!check_lng_float(packet.y)) {
	            return MAV_MISSION_INVALID_PARAM6_Y;
	        }
	        mav_cmd.x = packet.x * 1.0e7f;
	        mav_cmd.y = packet.y * 1.0e7f;
	        break;
    }
    
    MAV_MISSION_RESULT ans = mavlink_int_to_mission_cmd(mav_cmd, cmd);
    
    return ans;
}

MAV_MISSION_RESULT mavlink_int_to_mission_cmd(const mavlink_mission_item_int_t packet, Mission_Command *cmd){
	bool copy_location = false;
	bool copy_alt = false;

	// command's position in mission list and mavlink id
	cmd->index = packet.seq;
	cmd->id = packet.command;
	cmd->content.location.options = 0;

	// command specific conversions from mavlink packet to mission command
	switch (cmd->id) {		    
		case MAV_CMD_NAV_WAYPOINT:                          // MAV ID: 16
		{
		    copy_location = true;
		    cmd->p1 = packet.param1;		
//			printf("[waypoint] seq=%d cmd=%d cur=%d lat=%d long=%d alt=%.2f \r\n",packet.seq,packet.command,packet.current,packet.x,packet.y,packet.z);
		}
		break;
	}

		// copy location from mavlink to command
	if (copy_location || copy_alt) {

	    // sanity check location
	    if (copy_location) {
	        if (!check_lat_int32(packet.x)) {
	            return MAV_MISSION_INVALID_PARAM5_X;
	        }
	        if (!check_lng_int32(packet.y)) {
	            return MAV_MISSION_INVALID_PARAM6_Y;
	        }
	    }
	    if (fabsf(packet.z) >= LOCATION_DEPTH_MAX_M) {
	        return MAV_MISSION_INVALID_PARAM7;
	    }

	    switch (packet.frame) {

		    case MAV_FRAME_MISSION:
		    case MAV_FRAME_GLOBAL:
		        if (copy_location) {
		            cmd->content.location.lat = packet.x;
		            cmd->content.location.lng = packet.y;
		        }
		        cmd->content.location.alt = packet.z * 100.0f;       // convert packet's alt (m) to cmd alt (cm)
		        cmd->content.location.flags.relative_alt = 0;
		        break;

		    case MAV_FRAME_GLOBAL_RELATIVE_ALT:
		        if (copy_location) {
		            cmd->content.location.lat = packet.x;
		            cmd->content.location.lng = packet.y;
		        }
		        cmd->content.location.alt = packet.z * 100.0f;       // convert packet's alt (m) to cmd alt (cm)
		        cmd->content.location.flags.relative_alt = 1;
		        break;
		    default:
		        return MAV_MISSION_UNSUPPORTED_FRAME;
	    }
	}

	// if we got this far then it must have been successful
	return MAV_MISSION_ACCEPTED;
}

bool mission_cmd_to_mavlink(const Mission_Command *cmd, mavlink_mission_item_t *packet){
	mavlink_mission_item_int_t mav_cmd = {0};
	   
   bool ans = mission_cmd_to_mavlink_int(cmd, &mav_cmd);
   
   packet->param1 = mav_cmd.param1;
   packet->param2 = mav_cmd.param2;
   packet->param3 = mav_cmd.param3;
   packet->param4 = mav_cmd.param4;
   packet->z = mav_cmd.z;
   packet->seq = mav_cmd.seq;
   packet->command = mav_cmd.command;
   packet->target_system = mav_cmd.target_system;
   packet->target_component = mav_cmd.target_component;
   packet->frame = mav_cmd.frame;
   packet->current = mav_cmd.current;
   packet->autocontinue = mav_cmd.autocontinue;

   /*
	 the strategy for handling both MISSION_ITEM and MISSION_ITEM_INT
	 is to pass the lat/lng in MISSION_ITEM_INT straight through, and
	 for MISSION_ITEM multiply by 1e-7 here. We need an exception for
	 any commands which use the x and y fields not as
	 latitude/longitude.
	*/
   switch (packet->command) {
   case MAV_CMD_DO_DIGICAM_CONTROL:
   case MAV_CMD_DO_DIGICAM_CONFIGURE:
	   packet->x = mav_cmd.x;
	   packet->y = mav_cmd.y;
	   break;

   default:
	   // all other commands use x and y as lat/lon. We need to
	   // multiply by 1e-7 to convert to int32_t
	   packet->x = mav_cmd.x * 1.0e-7f;
	   packet->y = mav_cmd.y * 1.0e-7f;
	   break;
   }
   
   return ans;
}

//������ָ��ת��Ϊmavlink���ݽṹ
bool mission_cmd_to_mavlink_int(const Mission_Command *cmd, mavlink_mission_item_int_t *packet){
	bool copy_location = false;
	bool copy_alt = false;

	// command's position in mission list and mavlink id
	packet->seq = cmd->index;
	packet->command = cmd->id;

	// set defaults
	packet->current = 0; 	// 1 if we are passing back the mission command that is currently being executed
	packet->param1 = 0;
	packet->param2 = 0;
	packet->param3 = 0;
	packet->param4 = 0;
	packet->autocontinue = 1;

	// command specific conversions from mission command to mavlink packet
	switch (cmd->id) {
		case 0:
			// this is reserved for 16 bit command IDs
			return false;
			
		case MAV_CMD_NAV_WAYPOINT:							// MAV ID: 16
			copy_location = true;
			// delay at waypoint in seconds
			packet->param1 = cmd->p1;
			break;
	}

	// copy location from mavlink to command
	if (copy_location) {
		packet->x = cmd->content.location.lat;
		packet->y = cmd->content.location.lng;
	}
	if (copy_location || copy_alt) {
		packet->z = cmd->content.location.alt / 100.0f;	// cmd alt in cm to m
		if (cmd->content.location.flags.relative_alt) {
			packet->frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
		}else{
			packet->frame = MAV_FRAME_GLOBAL;
		}
#if AP_TERRAIN_AVAILABLE
		if (cmd->content.location.flags.terrain_alt) {
			// this is a above-terrain altitude
			if (!cmd->content.location.flags.relative_alt) {
				// refuse to return non-relative terrain mission
				// items. Internally we do have these, and they
				// have home.alt added, but we should never be
				// returning them to the GCS, as the GCS doesn't know
				// our home.alt, so it would have no way to properly
				// interpret it
				return false;
			}
			packet->z = cmd->content.location.alt * 0.01f;
			packet->frame = MAV_FRAME_GLOBAL_TERRAIN_ALT;
		}
#else
		// don't ever return terrain mission items if no terrain support
		if (cmd->content.location.flags.terrain_alt) {
			return false;
		}
#endif
	}

	// if we got this far then it must have been successful
	return true;

}

//����ִ�����
void mission_complete(){
	sys_flag.mission.state = MISSION_COMPLETE;	//��ǵ�������
	exit_mission();
}

//�ƽ����¸�����
bool mission_advance_current_nav_cmd(){
	Mission_Command cmd;

	if(sys_flag.mission.state != MISSION_RUNNING) return false;
	if(sys_flag.mission.nav_cmd_loaded) return false;	//��������Ѿ����룬�˳� -> ˵����δִ���굱ǰ���㣿
	if(_nav_cmd.index == vp.waypoint_count-1) return false;	//�Ѿ����е����һ������

	if(!mission_cmd_from_bin(_nav_cmd.index+1,&cmd)) return false;	//��ȡ��һ������
	_nav_cmd = cmd;
	sys_flag.mission.nav_cmd_loaded = true;
	
	start_command(&_nav_cmd);	//�����µĺ����ʼ��
	
	return true;
} 

void mission_update(){
	if(sys_flag.mission.state != MISSION_RUNNING || vp.waypoint_count == 0)
		return;
	if(!sys_flag.mission.nav_cmd_loaded){	//��δ���غ��� -> �տ�ʼ����������һ�������ѽ���
		if(!mission_advance_current_nav_cmd()){
			mission_complete();
			return;
		}
	}
	else{
		if(verify_command(&_nav_cmd)){//�жϵ�ǰ�����Ƿ����
			sys_flag.mission.nav_cmd_loaded = false;
			if(!mission_advance_current_nav_cmd()){
				mission_complete();
				return;
			}
		}
	}
}

//�������� -> ����HOME
bool mission_reset(){
	if(!mission_cmd_from_bin(0,&_nav_cmd)) return false;
	sys_flag.mission.state = MISSION_RUNNING;	//��־��ʼ���е�������
	sys_flag.mission.nav_cmd_loaded = true;		//��־���㵼�����
	return true;
}

//����ʼ -> �����һ������
bool mission_start(){
	if(!mission_cmd_from_bin(1,&_nav_cmd)) return false;
	if(!start_command(&_nav_cmd)) return false;
	sys_flag.mission.state = MISSION_RUNNING;	//��־��ʼ���е�������
	sys_flag.mission.nav_cmd_loaded = true;		//��־���㵼�����
	vp.mission_item_reached_index = 0;	
	return true;
}

//��ȡ��ǰ������
void mission_init(){
//	FRESULT res;
	switch(fp.wp_path){
		case PATH_LINE:	
			printf("[mission] waypoints path init: LINE \r\n");
			sprintf(mission_file_name,"mission_line.bin");
			break;	//ֱ��
		case PATH_PolyLine:
			printf("[mission] waypoints path init: PolyLine \r\n");
			sprintf(mission_file_name,"mission_polyline.bin");
			break;//����
		case PATH_Square: 
			printf("[mission] waypoints path init: Square \r\n");
			sprintf(mission_file_name,"mission_squre.bin");
			break;	//�ı���
		case PATH_Z:
			printf("[mission] waypoints path init: Z_Line \r\n");
			sprintf(mission_file_name,"mission_Z.bin");
			break;	//Z����
		case PATH_Circule: 
			printf("[mission] waypoints path init: Circule \r\n");
			sprintf(mission_file_name,"mission_circule.bin");
			break;	//Բ��
	}
	if(f_open(ftemp, mission_file_name, FA_OPEN_EXISTING | FA_READ) != FR_OK){
		printf("[Error] mission int Failed! \r\n");
		return;
	}
	vp.waypoint_count = f_size(ftemp)/MISSION_BIN_COMMAND_SIZE;
//	printf("%d \r\n",MISSION_BIN_COMMAND_SIZE);
	if(vp.waypoint_count>1){
		mission_cmd_from_bin(0,&_nav_cmd);	//��ȡhome��
		wp_nav.home = _nav_cmd.content.location;
	}
	printf("[OK] mission init complete. waypoints count: %d \r\n",vp.waypoint_count);
	f_close(ftemp);
}

