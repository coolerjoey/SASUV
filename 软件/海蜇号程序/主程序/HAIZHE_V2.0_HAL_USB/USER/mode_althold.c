#include "mode_althold.h"
#include "global_para.h"
#include "motors.h"
#include "attitude_controller.h"
#include "position_controller.h"
#include "GCS_Mavlink.h"
#include "mymath.h"

static u32 althode_test_start_ms;

bool althold_init(){
	if(!sys_check_ins() || !sys_check_baro() || !sys_check_BATT()){	
		return false;
	}
	vp.last_pilot_heading = sys_flag.screen_heading?vp.screen_heading:Sensor_latest.ins.euler[2];//以系统当前朝向为定向方向(//屏幕设置优先)
	vp.last_pilot_depth = sys_flag.screen_depth?vp.screen_depth:Sensor_latest.barometer.depth;//以系统当前深度为定深深度(//屏幕设置优先)
	reset_att_controller();	
	reset_pos_z_controller();

	althode_test_start_ms = sys_time.one_second;

	return true;
}

void althold_run_test(){
	if(!sys_flag.motors_armed){		
		althode_test_start_ms = sys_time.one_second;
		return;
	}
	if(sys_time.one_second > (althode_test_start_ms+5)){
		set_throttle(0,INPUT_MANUAL);
		set_mode(MANUAL);
		return;
	}
	set_throttle(fp.throttle_hover,INPUT_MANUAL);
}


//定深模式
void althold_run(){
	vp.last_pilot_heading = sys_flag.screen_heading?vp.screen_heading:vp.last_pilot_heading;	//若上位机定向角度改变，实时更改
	vp.last_pilot_depth = sys_flag.screen_depth?vp.screen_depth:vp.last_pilot_depth;
	if(!sys_flag.motors_armed){		
		vp.last_pilot_heading = sys_flag.screen_heading?vp.screen_heading:Sensor_latest.ins.euler[2];//以系统当前朝向为定向方向(//屏幕设置优先)
		vp.last_pilot_depth = sys_flag.screen_depth?vp.screen_depth:Sensor_latest.barometer.depth;//以系统当前深度为定深深度(//屏幕设置优先)
		reset_att_controller();	
		reset_pos_z_controller();
		return;
	}
	//航向控制
	if(!is_zero(yaw_in)){	//遥控转向
		set_yaw(channel.yaw,INPUT_MANUAL);
		reset_att_controller();	//pid重置
		vp.last_pilot_yaw_input_ms = sys_time.one_mill;
	}
	else{
		if(sys_time.one_mill < vp.last_pilot_yaw_input_ms+25){	//手柄松开后给250us恢复静止
			//[TODO] 运行航向角速度PID控制器
			if(sys_flag.follow_enable)
				vp.last_pilot_heading = sys_flag.screen_heading?vp.screen_heading:Sensor_latest.ins.euler[2];
		}
		else{
			att_controller_run(fp.attitude_init[0], fp.attitude_init[1], vp.last_pilot_heading);
		}
	}
	//深度控制
	if(fabs(channel.throttle)>0.05){	//遥控升沉
		reset_pos_z_controller();
		if(sys_flag.follow_enable)	//随动时更新深度
			vp.last_pilot_depth = sys_flag.screen_depth?vp.screen_depth:Sensor_latest.barometer.depth;
		set_throttle(channel.throttle,INPUT_MANUAL);
	}
	else{	//保持当前深度
		pos_z_controller_run(vp.last_pilot_depth);
	}

	set_forward(channel.forward,INPUT_MANUAL);
	set_lateral(channel.lateral,INPUT_MANUAL);

//	if(sys_time.one_second > (althode_test_start_ms+5)){
//		set_throttle(0,INPUT_MANUAL);
//		set_mode(MANUAL);
//		return;
//	}
//	set_throttle(fp.throttle_hover,INPUT_AUTO);
}




