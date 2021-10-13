#include "joystick.h"
#include "global_para.h"
#include "control_mode.h"
#include "delay.h"
#include "GCS_Mavlink.h"
#include "mymath.h"
#include "relay.h"
#include "INS.h"

JOYSTICK joystic={0,0,0,0,0};

/*
ע�⣺USB�ֱ�������ң��������ֻ��ͬʱʹ��һ����������ͬʱ�ϵ磬�����ϵ�����ȼ���

��ҡ������ ǰ��/��ͷ
��ҡ������ ����/̧ͷ
��ҡ������ ����
��ҡ������ �Һ��
��ҡ������ �ϸ�
��ҡ������ �³�
��ҡ������ ��ת
��ҡ������ ��ת
*/

void handle_joystick(s16 x, s16 y, s16 z, s16 r, u16 buttons){

	float motor_gain = fp.motor_gain;

	joystic.left_lr = y;
	joystic.left_ud = x;
	joystic.right_lr = r;
	joystic.right_ud = z;
	joystic.button = buttons;

	static u16 buttons_prev = 0;
	if(sys_flag.rc_roll_pitch_enable){
		channel.roll = motor_gain * (float)joystic.left_lr/1000.0f;
		channel.pitch = -motor_gain * (float)joystic.left_ud/1000.0f;
		channel.forward = 0;
		channel.lateral = 0;
	}
	else{
		channel.roll = 0;
		channel.pitch = 0;
		channel.forward = motor_gain * (float)joystic.left_ud/1000.0f;
		channel.lateral = motor_gain * (float)joystic.left_lr/1000.0f;
	}

	channel.yaw = motor_gain * (float)joystic.right_lr/1000.0f;
	channel.throttle = motor_gain * (float)(joystic.right_ud-500)/500.0f;	//��0-1000������ֵӳ�䵽-1~1

	//���shift���Ƿ���
	bool shift = false;
	for(uint8_t i=0;i<16;i++){
		if((buttons&(1<<i)) && fp.btn_function[i] == k_shift){
			shift = true;
		}
	}
	
	for(u8 i=0;i<16;i++){
		if(buttons & (1<<i)){	//��i����ť���´������¼�
			bool held = (buttons_prev & (1 << i));
			handle_jsbutton_press(i,shift,held);
		}
		else if(buttons_prev & (1 << i)){	//�����i����ť�ɿ��¼�
			handle_jsbutton_release(i);
		}
	}
	buttons_prev = buttons;

}

void handle_jsbutton_press(u8 button, bool shift, bool held){
	u8 btn_func;
	if(!shift) btn_func = fp.btn_function[button];	//shiftû�а���
	else btn_func = fp.btn_sfunction[button];
	switch(btn_func){
		case k_arm_toggle:
			if(!held){
				if(sys_flag.motors_armed==true){
					printf("[joy] motor Disarmed! \r\n");
					sys_flag.motors_armed=false;
				}
				else{
					printf("[joy] motor Armed! \r\n");
					sys_flag.motors_armed=true;
				}
			}
			break;
		case k_baro_vibration:
			if(!held){
				sys_flag.baro_vibration = false;
				sys_flag.acc_vibration = false;
			}
			break;
		case k_att_toggle:
			if(!held){
				if(sys_flag.EC_att_enable==true){
					printf("[joy] change att resource to MPU! \r\n");
					gcs_send_text(MAV_SEVERITY_INFO,"Change att resource to MPU!");
					sys_flag.EC_att_enable=false;
				}
				else{
					printf("[joy] change att resource to TCM! \r\n");
					gcs_send_text(MAV_SEVERITY_INFO,"Change att resource to TCM!");
					sys_flag.EC_att_enable=true;
				}
			}
			break;
		case k_follow_toggle:
			if(!held){
				if(sys_flag.follow_enable==true){
					printf("[joy] Unfollow mode! \r\n");
					gcs_send_text(MAV_SEVERITY_INFO,"Unfollow mode!");
					sys_flag.follow_enable=false;
				}
				else{
					printf("[joy] Follow mode \r\n");
					gcs_send_text(MAV_SEVERITY_INFO,"Follow mode");
					sys_flag.follow_enable=true;
				}
			}
			break;
		case k_acc_vibration:
			if(!held){
				ins_calibration();
			}
			break;
		case k_mode_manual:
			set_mode(MANUAL);
			break; 
		case k_mode_auto:
			set_mode(AUTO);
			break;
		case k_mode_stabilize:
			set_mode(STABILIZE);
			break;
		case k_mode_depth_hold:
			set_mode(ALT_HOLD);
			break;
		case k_mode_rtl:
			if(!held){
				set_mode(RTL);
			}
			break;
		case k_relay_1_toggle:
			if(!held){
				sys_flag.relay_12V_enable = !sys_flag.relay_12V_enable;
				if(sys_flag.relay_12V_enable){
					printf("[joy] Relay ON! \r\n");
					relay_12V_check();
				}
				else printf("[joy] Relay OFF! \r\n");
			}
			break; 
		case k_return_boot:
			printf("\r\n**************\r\n");
			printf("NVIC_SystemReset\r\n");
			printf("\r\n**************\r\n");
			gcs_send_text(MAV_SEVERITY_INFO,"NVIC_SystemReset! Jump to BootLoader");
			delay_ms(1000);
			NVIC_SystemReset();
			break;	
		case k_roll_pitch_toggle:
			if(!held){
				sys_flag.rc_roll_pitch_enable = !sys_flag.rc_roll_pitch_enable;
				if(sys_flag.rc_roll_pitch_enable){ 
					printf("[joy] roll_pitch mode enable \r\n");
					gcs_send_text(MAV_SEVERITY_INFO, "[joy] roll_pitch mode enable");
				}
				else{
					printf("[joy] roll_pitch mode disable \r\n");
					gcs_send_text(MAV_SEVERITY_INFO, "[joy] roll_pitch mode disable!");
				}
			}
			break;
		case k_gain_dec:
			if(!held){	
				fp.motor_gain = constrain_float(fp.motor_gain-(fp.motor_maxGain-fp.motor_minGain)/(fp.motor_numGainSetting-1), fp.motor_minGain, fp.motor_maxGain);
				char text[50]="";
				sprintf(text,"Motor Gain is %d%%",(int)(fp.motor_gain*100));
				gcs_send_text(MAV_SEVERITY_INFO, text);
				printf("[js] %s \r\n",text);
			}
			break;
		case k_gain_inc:
			if(!held){	
				fp.motor_gain = constrain_float(fp.motor_gain+(fp.motor_maxGain-fp.motor_minGain)/(fp.motor_numGainSetting-1), fp.motor_minGain, fp.motor_maxGain);
				char text[50]="";
				sprintf(text,"Motor Gain is %d%%",(int)(fp.motor_gain*100));
				gcs_send_text(MAV_SEVERITY_INFO, text);
				printf("[js] %s \r\n",text);
			}
			break;
		case k_speed_gain_dec:
			if(!held){	
				fp.speed_gain = constrain_float(fp.speed_gain-(fp.speed_maxGain-fp.speed_minGain)/(fp.speed_numGainSetting-1), fp.speed_minGain, fp.speed_maxGain);
				char text[50]="";
				sprintf(text,"Speed Gain is %d%%",(int)(fp.speed_gain*100));
				gcs_send_text(MAV_SEVERITY_INFO, text);
				printf("[js] %s \r\n",text);
			}
			break;
		case k_speed_gain_inc:
			if(!held){	
				fp.speed_gain = constrain_float(fp.speed_gain+(fp.speed_maxGain-fp.speed_minGain)/(fp.speed_numGainSetting-1), fp.speed_minGain, fp.speed_maxGain);
				char text[50]="";
				sprintf(text,"Speed Gain is %d%%",(int)(fp.speed_gain*100));
				gcs_send_text(MAV_SEVERITY_INFO, text);
				printf("[js] %s \r\n",text);
			}
			break;
	}
}

void handle_jsbutton_release(u8 button){

}

//Ĭ�ϰ�������
u8 btn_default_func[16]={
	k_mode_manual,
	k_mode_stabilize,
	k_mode_depth_hold,
	k_mode_auto,

	k_arm_toggle,
	k_relay_1_toggle,
	k_shift,
	k_roll_pitch_toggle,

	k_return_boot,
	k_baro_vibration,
	k_gain_dec,
	k_gain_inc,

	k_none,
	k_none,
	k_none,
	k_none
};
u8 btn_default_sfunc[16]={
	k_mode_rtl,
	k_none,
	k_none,
	k_none,

	k_acc_vibration,
	k_follow_toggle,
	k_none,
	k_att_toggle,

	k_none,
	k_none,
	k_speed_gain_dec,
	k_speed_gain_inc,

	k_none,
	k_none,
	k_none,
	k_none
};

//�ظ��ֱ�Ĭ��ֵ
void set_default_buttons(){
	for(int i=0;i<16;i++){
		fp.btn_function[i] = btn_default_func[i];
		fp.btn_sfunction[i] = btn_default_sfunc[i];
	}
	flash_write_param(&fp.btn_function[0], btn0_function);
	flash_write_param(&fp.btn_function[1], btn1_function);
	flash_write_param(&fp.btn_function[2], btn2_function);
	flash_write_param(&fp.btn_function[3], btn3_function);
	flash_write_param(&fp.btn_function[4], btn4_function);
	flash_write_param(&fp.btn_function[5], btn5_function);
	flash_write_param(&fp.btn_function[6], btn6_function);
	flash_write_param(&fp.btn_function[7], btn7_function);
	flash_write_param(&fp.btn_function[8], btn8_function);
	flash_write_param(&fp.btn_function[9], btn9_function);
	flash_write_param(&fp.btn_function[10], btn10_function);
	flash_write_param(&fp.btn_function[11], btn11_function);
	flash_write_param(&fp.btn_function[12], btn12_function);
	flash_write_param(&fp.btn_function[13], btn13_function);
	flash_write_param(&fp.btn_function[14], btn14_function);
	flash_write_param(&fp.btn_function[15], btn15_function);

	flash_write_param(&fp.btn_sfunction[0], btn0_sfunction);
	flash_write_param(&fp.btn_sfunction[1], btn1_sfunction);
	flash_write_param(&fp.btn_sfunction[2], btn2_sfunction);
	flash_write_param(&fp.btn_sfunction[3], btn3_sfunction);
	flash_write_param(&fp.btn_sfunction[4], btn4_sfunction);
	flash_write_param(&fp.btn_sfunction[5], btn5_sfunction);
	flash_write_param(&fp.btn_sfunction[6], btn6_sfunction);
	flash_write_param(&fp.btn_sfunction[7], btn7_sfunction);
	flash_write_param(&fp.btn_sfunction[8], btn8_sfunction);
	flash_write_param(&fp.btn_sfunction[9], btn9_sfunction);
	flash_write_param(&fp.btn_sfunction[10], btn10_sfunction);
	flash_write_param(&fp.btn_sfunction[11], btn11_sfunction);
	flash_write_param(&fp.btn_sfunction[12], btn12_sfunction);
	flash_write_param(&fp.btn_sfunction[13], btn13_sfunction);
	flash_write_param(&fp.btn_sfunction[14], btn14_sfunction);
	flash_write_param(&fp.btn_sfunction[15], btn15_sfunction);
}
//�ֱ���ʼ��
void joystick_init(){
	for(int i=0;i<16;i++){
		fp.btn_function[i] = btn_default_func[i];
		fp.btn_sfunction[i] = btn_default_sfunc[i];
	}
	fp.motor_gain = 0.5;	//�������1fp.0
	fp.motor_maxGain = 1.0f;	//ң��ģʽ�µ���������
	fp.motor_minGain = 0.25;	//ң��ģʽ�µ����С����
	fp.motor_numGainSetting = 4;	//ң��ģʽ�µ�����浵λ
	fp.speed_gain = 0.0f;
	fp.speed_maxGain = 0.5;	//���ȡ�����ģʽ�µ���������
	fp.speed_minGain = 0;	//���ȡ�����ģʽ�µ����С����
	fp.speed_numGainSetting = 6; //����
	
	printf("[OK] joystick init complete! \r\n");
}
