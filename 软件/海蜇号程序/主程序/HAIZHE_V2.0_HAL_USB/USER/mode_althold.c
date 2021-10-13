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
	vp.last_pilot_heading = sys_flag.screen_heading?vp.screen_heading:Sensor_latest.ins.euler[2];//��ϵͳ��ǰ����Ϊ������(//��Ļ��������)
	vp.last_pilot_depth = sys_flag.screen_depth?vp.screen_depth:Sensor_latest.barometer.depth;//��ϵͳ��ǰ���Ϊ�������(//��Ļ��������)
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


//����ģʽ
void althold_run(){
	vp.last_pilot_heading = sys_flag.screen_heading?vp.screen_heading:vp.last_pilot_heading;	//����λ������Ƕȸı䣬ʵʱ����
	vp.last_pilot_depth = sys_flag.screen_depth?vp.screen_depth:vp.last_pilot_depth;
	if(!sys_flag.motors_armed){		
		vp.last_pilot_heading = sys_flag.screen_heading?vp.screen_heading:Sensor_latest.ins.euler[2];//��ϵͳ��ǰ����Ϊ������(//��Ļ��������)
		vp.last_pilot_depth = sys_flag.screen_depth?vp.screen_depth:Sensor_latest.barometer.depth;//��ϵͳ��ǰ���Ϊ�������(//��Ļ��������)
		reset_att_controller();	
		reset_pos_z_controller();
		return;
	}
	//�������
	if(!is_zero(yaw_in)){	//ң��ת��
		set_yaw(channel.yaw,INPUT_MANUAL);
		reset_att_controller();	//pid����
		vp.last_pilot_yaw_input_ms = sys_time.one_mill;
	}
	else{
		if(sys_time.one_mill < vp.last_pilot_yaw_input_ms+25){	//�ֱ��ɿ����250us�ָ���ֹ
			//[TODO] ���к�����ٶ�PID������
			if(sys_flag.follow_enable)
				vp.last_pilot_heading = sys_flag.screen_heading?vp.screen_heading:Sensor_latest.ins.euler[2];
		}
		else{
			att_controller_run(fp.attitude_init[0], fp.attitude_init[1], vp.last_pilot_heading);
		}
	}
	//��ȿ���
	if(fabs(channel.throttle)>0.05){	//ң������
		reset_pos_z_controller();
		if(sys_flag.follow_enable)	//�涯ʱ�������
			vp.last_pilot_depth = sys_flag.screen_depth?vp.screen_depth:Sensor_latest.barometer.depth;
		set_throttle(channel.throttle,INPUT_MANUAL);
	}
	else{	//���ֵ�ǰ���
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




