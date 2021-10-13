#include "mode_stabilize.h"
#include "global_para.h"
#include "motors.h"
#include "attitude_controller.h"
#include "position_controller.h"
#include "GCS_Mavlink.h"
#include "mymath.h"

bool stabilize_init(){
	if(!sys_check_ins()){	
		return false;
	}
	vp.last_pilot_heading = sys_flag.screen_heading?vp.screen_heading:Sensor_latest.ins.euler[2];//以系统当前朝向为定向方向(//屏幕设置优先)
	reset_att_controller();	
	return true;
}


void stabilize_run(){
	vp.last_pilot_heading = sys_flag.screen_heading?vp.screen_heading:vp.last_pilot_heading;	//若上位机定向角度改变，实时更改
	if(!sys_flag.motors_armed || !sys_check_BATT()){		
		vp.last_pilot_heading = sys_flag.screen_heading?vp.screen_heading:Sensor_latest.ins.euler[2];//更新最新航向
		reset_att_controller();	//PID运算清零
		return;
	}
	if(!is_zero(channel.yaw)){	//遥控转向
		reset_att_controller();	
		set_yaw(channel.yaw,INPUT_MANUAL);
		vp.last_pilot_yaw_input_ms = sys_time.one_mill;
	}
	else{
		if(sys_time.one_mill < vp.last_pilot_yaw_input_ms+250){	//手柄松开后给250ms恢复静止
			//[TODO] 运行航向角速度PID控制器
			if(sys_flag.follow_enable)	//随动时更新航向
				vp.last_pilot_heading = sys_flag.screen_heading?vp.screen_heading:Sensor_latest.ins.euler[2];
		}
		else{
			att_controller_run(fp.attitude_init[0], fp.attitude_init[1], vp.last_pilot_heading);
		}
	}

//	set_throttle(channel.throttle,INPUT_MANUAL);
	set_throttle(fp.throttle_hover,INPUT_MANUAL);
	set_forward(channel.forward+fp.speed_gain,INPUT_MANUAL);
	set_lateral(channel.lateral,INPUT_MANUAL);
}


