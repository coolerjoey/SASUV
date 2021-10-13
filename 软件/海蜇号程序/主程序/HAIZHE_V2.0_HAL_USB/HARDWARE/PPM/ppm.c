#include "ppm.h"
#include "global_para.h"
#include "stdlib.h"
#include "telem.h"


u8 PPM_RX_STA=0;	//PPM接收状态
u8 PPM_RX_NUM=0;	//PPM通道
u16 PPM_VAL[PPM_CHNNEL_MAX];	//PPM通道值

void ppm_init(){
	PPM_CAP_Init();
}

static bool ppm_is_equal(int a,int b){
	if(abs(a-b)<2) return true;
	return false;
}

static s8 constrain(s8 val){
	val = val>100?100:val;
	val = val<-100?-100:val;
	return val;
}

//模拟通道数值检测
//static bool ppm_analog_val_check(u16 val,PPM_CHNEL* rc_chnnel){
//	if(val<rc_chnnel->chnnel_min || val>rc_chnnel->chnnel_max)
//		return false;
//	return true;
//}
//模拟通道数值上下限更ixn
static void ppm_analog_throttle_update(u16 val,PPM_CHNEL* rc_chnnel){
	if(val<rc_chnnel->chnnel_min) rc_chnnel->chnnel_min = val;
	else if(val>rc_chnnel->chnnel_max) rc_chnnel->chnnel_max = val;
//	rc_chnnel->chnnel_mid = (rc_chnnel->chnnel_max - rc_chnnel->chnnel_min)/2; 
}
	
//计算油门百分比
s8 percent_cal(u16 val,PPM_CHNEL* rc_chnnel){
	return 100*(float)(val-rc_chnnel->chnnel_mid)/((rc_chnnel->chnnel_max-rc_chnnel->chnnel_min)/2);
}
//判断当前模拟通道输入是否脱离死区
//返回值：false-在死区内 true-脱离死区
bool deadzone_check(s8 val){
	if(val<DEAD_ZONE && -DEAD_ZONE<val) return false;
	return true;
}

void ppm_update(){
	//如果接收到usb手柄数据，不进行ppm解析
	if(sys_flag.joy_connected) return;

	bool ppm_show = false;
	s8 PPM_Left_UP_DOWN_PENCENT =0;	//遥控器四个摇杆的百分比
	s8 PPM_Left_LEFT_RIGHT_PENCENT =0;
	s8 PPM_RIGHT_UP_DOWN_PENCENT =0;
	s8 PPM_RIGHT_LEFT_RIGHT_PENSENT=0;
	if(sys_flag.ppm_recv){	//成功读取到一帧ppm数据
		sys_flag.ppm_recv = false;
		if(ppm_is_equal(PPM_VAL[chnnel_3],fp.remoteCtrl.unconnect_val[chnnel_3])){	//当接收机和遥控器失去通信时，ppm第三个通道值为异常值	
			//通道归零，防止和遥控器断连后无法停止
			channel.roll = 0;
			channel.pitch = 0;
			channel.forward = 0;
			channel.lateral = 0;
			channel.throttle = 0;
			channel.yaw = 0;
			sys_flag.ppm_connected = false;
			return;
		} 
		
		sys_flag.ppm_connected = true;
		//动态检测接收机油门上下限
		ppm_analog_throttle_update(PPM_VAL[chnnel_1],&(fp.remoteCtrl.ppm_ch[chnnel_1])) ;
		ppm_analog_throttle_update(PPM_VAL[chnnel_2],&(fp.remoteCtrl.ppm_ch[chnnel_2]));
		ppm_analog_throttle_update(PPM_VAL[chnnel_3],&(fp.remoteCtrl.ppm_ch[chnnel_3])); 
		ppm_analog_throttle_update(PPM_VAL[chnnel_4],&(fp.remoteCtrl.ppm_ch[chnnel_4]));
		ppm_analog_throttle_update(PPM_VAL[chnnel_6],&(fp.remoteCtrl.ppm_ch[chnnel_6]));
		ppm_analog_throttle_update(PPM_VAL[chnnel_7],&(fp.remoteCtrl.ppm_ch[chnnel_7])); 
		ppm_analog_throttle_update(PPM_VAL[chnnel_8],&(fp.remoteCtrl.ppm_ch[chnnel_8]));
		
		
		//通道1-4 模拟通道百分比
		PPM_Left_UP_DOWN_PENCENT = constrain(percent_cal(PPM_VAL[chnnel_2],&(fp.remoteCtrl.ppm_ch[chnnel_2])));
		PPM_Left_LEFT_RIGHT_PENCENT = constrain(percent_cal(PPM_VAL[chnnel_4],&(fp.remoteCtrl.ppm_ch[chnnel_4])));
		PPM_RIGHT_UP_DOWN_PENCENT = constrain(percent_cal(PPM_VAL[chnnel_3],&(fp.remoteCtrl.ppm_ch[chnnel_3])));
		PPM_RIGHT_LEFT_RIGHT_PENSENT = constrain(percent_cal(PPM_VAL[chnnel_1],&(fp.remoteCtrl.ppm_ch[chnnel_1])));
		
		//通道5-拨动开关
		static u8 PPM_SWITCH_SB = 0;
		if(ppm_is_equal(PPM_VAL[chnnel_5],fp.remoteCtrl.ppm_ch[chnnel_5].chnnel_max)){PPM_SWITCH_SB=2;sys_flag.relay_12V_enable = false;}
		else if(ppm_is_equal(PPM_VAL[chnnel_5],fp.remoteCtrl.ppm_ch[chnnel_5].chnnel_mid)){PPM_SWITCH_SB=1;}
		else if(ppm_is_equal(PPM_VAL[chnnel_5],fp.remoteCtrl.ppm_ch[chnnel_5].chnnel_min)){PPM_SWITCH_SB=0;sys_flag.relay_12V_enable = true;}
		if(ppm_show) printf("CH5-%d \r\n",PPM_SWITCH_SB);
		
		//通道6->暂时用作模式切换
		s8 PPM_TWIST_CH_PENSENT = constrain(percent_cal(PPM_VAL[chnnel_6],&(fp.remoteCtrl.ppm_ch[chnnel_6])));
		if(PPM_TWIST_CH_PENSENT<-25){
			vp.control_mode = MANUAL;
		}
		else if(PPM_TWIST_CH_PENSENT>25){
			vp.control_mode = AUTO;
		}
		if(ppm_show) printf("CH6-%d%% ",PPM_TWIST_CH_PENSENT);
			
		
		//通道7-锁定切换
		s8 PPM_V2_CH_PENSENT = constrain(percent_cal(PPM_VAL[chnnel_7],&(fp.remoteCtrl.ppm_ch[chnnel_7])));
		if(ppm_show) printf("CH7-%d%% ",PPM_V2_CH_PENSENT);
		if(PPM_V2_CH_PENSENT<-50 && sys_flag.motors_armed){
			sys_flag.motors_armed = false;
			printf("[motor] Locked! \r\n");
		}
		else if(PPM_V2_CH_PENSENT>50 && !sys_flag.motors_armed){
			sys_flag.motors_armed = true;
			printf("[motor] Unlock \r\n");
		}	
		
		//输入死区检测
		bool _dz1 = deadzone_check(PPM_Left_UP_DOWN_PENCENT);
		bool _dz2 = deadzone_check(PPM_Left_LEFT_RIGHT_PENCENT);
		bool _dz3 = deadzone_check(PPM_RIGHT_UP_DOWN_PENCENT);
		bool _dz4 = deadzone_check(PPM_RIGHT_LEFT_RIGHT_PENSENT);
		if(!_dz1 && !_dz2 && !_dz3 && !_dz4){	//不考虑_dz3是因为遥控时_dz3不做输入通道，升沉
			if(ppm_show) printf("DeadZone \r\n");
			return;
		}
		sys_flag.ppm_enable = true;	//模拟通道输入脱离死区
		if(ppm_show){
			printf("左上下 %d%% ",PPM_Left_UP_DOWN_PENCENT);
			printf("左左右 %d%% ",PPM_Left_LEFT_RIGHT_PENCENT);
			printf("右上下 %d%% ",PPM_RIGHT_UP_DOWN_PENCENT);
			printf("右左右 %d%% \r\n",PPM_RIGHT_LEFT_RIGHT_PENSENT);	
		}

		float motor_gain = fp.motor_gain;

		if(sys_flag.rc_roll_pitch_enable){
			channel.roll = motor_gain * PPM_Left_LEFT_RIGHT_PENCENT/100.0f;
			channel.pitch = motor_gain * PPM_Left_UP_DOWN_PENCENT/100.0f;
			channel.forward = 0;
			channel.lateral = 0;
		}
		else{
			channel.roll = 0;
			channel.pitch = 0;
			channel.forward = motor_gain * PPM_Left_UP_DOWN_PENCENT/100.0f;
			channel.lateral = motor_gain * PPM_Left_LEFT_RIGHT_PENCENT/100.0f;
		}

		channel.throttle = motor_gain * PPM_RIGHT_UP_DOWN_PENCENT/100.0f;
		channel.yaw = motor_gain * PPM_RIGHT_LEFT_RIGHT_PENSENT/100.0f;
		
		sys_flag.ppm_recv=0;
	}
}



