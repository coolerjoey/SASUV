#include "mode_manual.h"
#include "global_para.h"
#include "motors.h"
#include "telem.h"
#include "barometer.h"

bool manual_init(){
	
	return true;
}

//水面遥控模式
void manual_run(){
	if(sys_flag.motors_armed != ENABLE){ //检查电机是否解锁
		set_roll(0,INPUT_MANUAL);
		set_pitch(0,INPUT_MANUAL);
		set_yaw(0,INPUT_MANUAL);
		set_throttle(0,INPUT_MANUAL);
		set_forward(0,INPUT_MANUAL);
		set_lateral(0,INPUT_MANUAL);	
		return ;
	}
	
	set_roll(channel.roll,INPUT_MANUAL);
	set_pitch(channel.pitch,INPUT_MANUAL);
	set_yaw(channel.yaw,INPUT_MANUAL);
	set_throttle(channel.throttle,INPUT_MANUAL);
	set_forward(channel.forward,INPUT_MANUAL);
	set_lateral(channel.lateral,INPUT_MANUAL);	
		
}




