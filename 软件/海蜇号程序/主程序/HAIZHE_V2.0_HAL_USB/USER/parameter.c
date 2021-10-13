#include "parameter.h"
#include "motors.h"
#include "attitude_controller.h"
#include "position_controller.h"
#include "flash.h"
#include "joystick.h"
//#include "global_para.h"
#include "string.h"

PARAM_ID_TYPE_VALUE param_id_type_value[]={
	{"SYS_PARAM_NUM",UINT16,&fp.param_num,param_num},
	{"SYSID_MYGCS",UINT16,&fp.sysid_my_gcs,sysid_my_gcs},
	{"SYS_FRAME",UINT8,&fp.frame_class,frame_class},
	{"POWER_MIN",REAL32,&fp.power_min,power_min},
	{"LOG_MODE",UINT8,&fp.log_mode,log_mode},
	{"ROV_ENABLE",UINT8,&fp.ROV_mode_enable,ROV_mode_enable},

	{"HEA_CHE_IMU",UINT8,&fp.health_chenable.imu,health_chenable_imu},
	{"HEA_CHE_BARO",UINT8,&fp.health_chenable.baro,health_chenable_baro},
	{"HEA_CHE_BATT",UINT8,&fp.health_chenable.batt,health_chenable_batt},
	{"HEA_CHE_JOY",UINT8,&fp.health_chenable.joystick,health_chenable_joystick},
	{"HEA_CHE_GPS",UINT8,&fp.health_chenable.gps,health_chenable_gps},
	{"HEA_CHE_SD",UINT8,&fp.health_chenable.sd,health_chenable_sd},
	{"HEA_CHE_FLASH",UINT8,&fp.health_chenable.flash,health_chenable_flash},
	
	{"UARTA_MPU",UINT32,&fp.uartA_baud,uartA_baud}, 	//MPU
	{"UARTB_WIFI",UINT32,&fp.uartB_baud,uartB_baud}, 	//WIFI
	{"UARTC_TELEM",UINT32,&fp.uartC_baud,uartC_baud},	//TELEM
	{"UARTD_GPS",UINT32,&fp.uartD_baud,uartD_baud},	//GPS
	{"UARTE_TCM",UINT32,&fp.uartE_baud,uartE_baud},	//UART
	{"UARTF_DEBUG",UINT32,&fp.uartF_baud,uartF_baud},	//DEBUG

	{"ATT_INIT_ROLL",REAL32,&fp.attitude_init[0],attitude_init_roll},
	{"ATT_INIT_PITCH",REAL32,&fp.attitude_init[1],attitude_init_pitch},
	{"ATT_INIT_YAW",REAL32,&fp.attitude_init[2],attitude_init_yaw},
	{"SINS_DB_X",REAL32,&fp.sins_db[0],sins_db_x},
	{"SINS_DB_Y",REAL32,&fp.sins_db[1],sins_db_y},
	{"SINS_DB_Z",REAL32,&fp.sins_db[2],sins_db_z},

	{"NAV_MODE",UINT8,&fp.dr_mode,dr_mode},
	{"NAV_PATH",UINT8,&fp.wp_path,wp_path},
	{"NAV_RADIUS",REAL32,&fp.wp_radius_cm,wp_radius_cm},	
	{"NAV_EXIT_RTL",UINT8,&fp.exit_mission_rtl,exit_mission_rtl},	
	{"NAV_FOR_THR",REAL32,&fp.forward_auto,wp_forward_auto},
	{"NAV_GPS_FB_INT",UINT8,&fp.gps_fb_interval,gps_fb_interval},

	{"MOT_THR_HOVER",REAL32,&fp.throttle_hover,throttle_hover},
	{"MOT_THR_SWERVER",REAL32,&fp.throttle_deadzone_swerve,throttle_deadzone_swerve},
	{"MOT_THR_STAB",REAL32,&fp.throttle_deadzone_stabilize,throttle_deadzone_stabilize},
	{"MOT_GAIN",REAL32,&fp.motor_gain,motor_gain},
	{"MOT_GAIN_MAX",REAL32,&fp.motor_maxGain,motor_maxGain},
	{"MOT_GAIN_MIN",REAL32,&fp.motor_minGain,motor_minGain},
	{"MOT_GAIN_NUM",REAL32,&fp.motor_numGainSetting,motor_numGainSetting},
	{"SPD_GAIN",REAL32,&fp.speed_gain,speed_gain},
	{"SPD_GAIN_MAX",REAL32,&fp.speed_maxGain,speed_maxGain},
	{"SPD_GAIN_MIN",REAL32,&fp.speed_minGain,speed_minGain},
	{"SPD_GAIN_NUM",REAL32,&fp.speed_numGainSetting,speed_numGainSetting},

	{"MOTOR1_RLL_FAC",REAL32,&fp.motor1_fac.roll_fac,motor1_roll_fac},
	{"MOTOR1_PIT_FAC",REAL32,&fp.motor1_fac.pitch_fac,motor1_pitch_fac},
	{"MOTOR1_YAW_FAC",REAL32,&fp.motor1_fac.yaw_fac,motor1_yaw_fac},
	{"MOTOR1_THR_FAC",REAL32,&fp.motor1_fac.throttle_fac,motor1_throttle_fac},
	{"MOTOR1_FOR_FAC",REAL32,&fp.motor1_fac.forward_fac,motor1_forward_fac},
	{"MOTOR1_LAT_FAC",REAL32,&fp.motor1_fac.lateral_fac,motor1_lateral_fac},
	{"MOTOR2_RLL_FAC",REAL32,&fp.motor2_fac.roll_fac,motor2_roll_fac},
	{"MOTOR2_PIT_FAC",REAL32,&fp.motor2_fac.pitch_fac,motor2_pitch_fac},
	{"MOTOR2_YAW_FAC",REAL32,&fp.motor2_fac.yaw_fac,motor2_yaw_fac},
	{"MOTOR2_THR_FAC",REAL32,&fp.motor2_fac.throttle_fac,motor2_throttle_fac},
	{"MOTOR2_FOR_FAC",REAL32,&fp.motor2_fac.forward_fac,motor2_forward_fac},
	{"MOTOR2_LAT_FAC",REAL32,&fp.motor2_fac.lateral_fac,motor2_lateral_fac},
	{"MOTOR3_RLL_FAC",REAL32,&fp.motor3_fac.roll_fac,motor3_roll_fac},
	{"MOTOR3_PIT_FAC",REAL32,&fp.motor3_fac.pitch_fac,motor3_pitch_fac},
	{"MOTOR3_YAW_FAC",REAL32,&fp.motor3_fac.yaw_fac,motor3_yaw_fac},
	{"MOTOR3_THR_FAC",REAL32,&fp.motor3_fac.throttle_fac,motor3_throttle_fac},
	{"MOTOR3_FOR_FAC",REAL32,&fp.motor3_fac.forward_fac,motor3_forward_fac},
	{"MOTOR3_LAT_FAC",REAL32,&fp.motor3_fac.lateral_fac,motor3_lateral_fac},
	{"MOTOR4_RLL_FAC",REAL32,&fp.motor4_fac.roll_fac,motor4_roll_fac},
	{"MOTOR4_PIT_FAC",REAL32,&fp.motor4_fac.pitch_fac,motor4_pitch_fac},
	{"MOTOR4_YAW_FAC",REAL32,&fp.motor4_fac.yaw_fac,motor4_yaw_fac},
	{"MOTOR4_THR_FAC",REAL32,&fp.motor4_fac.throttle_fac,motor4_throttle_fac},
	{"MOTOR4_FOR_FAC",REAL32,&fp.motor4_fac.forward_fac,motor4_forward_fac},
	{"MOTOR4_LAT_FAC",REAL32,&fp.motor4_fac.lateral_fac,motor4_lateral_fac},
	{"MOTOR5_RLL_FAC",REAL32,&fp.motor5_fac.roll_fac,motor5_roll_fac},
	{"MOTOR5_PIT_FAC",REAL32,&fp.motor5_fac.pitch_fac,motor5_pitch_fac},
	{"MOTOR5_YAW_FAC",REAL32,&fp.motor5_fac.yaw_fac,motor5_yaw_fac},
	{"MOTOR5_THR_FAC",REAL32,&fp.motor5_fac.throttle_fac,motor5_throttle_fac},
	{"MOTOR5_FOR_FAC",REAL32,&fp.motor5_fac.forward_fac,motor5_forward_fac},
	{"MOTOR5_LAT_FAC",REAL32,&fp.motor5_fac.lateral_fac,motor5_lateral_fac},
	{"MOTOR6_RLL_FAC",REAL32,&fp.motor6_fac.roll_fac,motor6_roll_fac},
	{"MOTOR6_PIT_FAC",REAL32,&fp.motor6_fac.pitch_fac,motor6_pitch_fac},
	{"MOTOR6_YAW_FAC",REAL32,&fp.motor6_fac.yaw_fac,motor6_yaw_fac},
	{"MOTOR6_THR_FAC",REAL32,&fp.motor6_fac.throttle_fac,motor6_throttle_fac},
	{"MOTOR6_FOR_FAC",REAL32,&fp.motor6_fac.forward_fac,motor6_forward_fac},
	{"MOTOR6_LAT_FAC",REAL32,&fp.motor6_fac.lateral_fac,motor6_lateral_fac},

	{"JS_TYPE",UINT8,&fp.remoteCtrl.type,remoteCtrl_type},
	{"JS_UNCON_CH1",UINT8,&fp.remoteCtrl.unconnect_val[0],remoteCtrl_unConVal_ch1},
	{"JS_UNCON_CH2",UINT8,&fp.remoteCtrl.unconnect_val[1],remoteCtrl_unConVal_ch2},
	{"JS_UNCON_CH3",UINT8,&fp.remoteCtrl.unconnect_val[2],remoteCtrl_unConVal_ch3},
	{"JS_UNCON_CH4",UINT8,&fp.remoteCtrl.unconnect_val[3],remoteCtrl_unConVal_ch4},
	{"JS_UNCON_CH5",UINT8,&fp.remoteCtrl.unconnect_val[4],remoteCtrl_unConVal_ch5},
	{"JS_UNCON_CH6",UINT8,&fp.remoteCtrl.unconnect_val[5],remoteCtrl_unConVal_ch6},
	{"JS_UNCON_CH7",UINT8,&fp.remoteCtrl.unconnect_val[6],remoteCtrl_unConVal_ch7},
	{"JS_UNCON_CH8",UINT8,&fp.remoteCtrl.unconnect_val[7],remoteCtrl_unConVal_ch8},
	{"JS_CON_CH1_MIN",UINT8,&fp.remoteCtrl.ppm_ch[chnnel_1].chnnel_min,remoteCtrl_ConVal_ch1_min},
	{"JS_CON_CH1_MID",UINT8,&fp.remoteCtrl.ppm_ch[chnnel_1].chnnel_mid,remoteCtrl_ConVal_ch1_mid},
	{"JS_CON_CH1_MAX",UINT8,&fp.remoteCtrl.ppm_ch[chnnel_1].chnnel_max,remoteCtrl_ConVal_ch1_max},
	{"JS_CON_CH2_MIN",UINT8,&fp.remoteCtrl.ppm_ch[chnnel_2].chnnel_min,remoteCtrl_ConVal_ch2_min},
	{"JS_CON_CH2_MID",UINT8,&fp.remoteCtrl.ppm_ch[chnnel_2].chnnel_mid,remoteCtrl_ConVal_ch2_mid},
	{"JS_CON_CH2_MAX",UINT8,&fp.remoteCtrl.ppm_ch[chnnel_2].chnnel_max,remoteCtrl_ConVal_ch2_max},
	{"JS_CON_CH3_MIN",UINT8,&fp.remoteCtrl.ppm_ch[chnnel_3].chnnel_min,remoteCtrl_ConVal_ch3_min},
	{"JS_CON_CH3_MID",UINT8,&fp.remoteCtrl.ppm_ch[chnnel_3].chnnel_mid,remoteCtrl_ConVal_ch3_mid},
	{"JS_CON_CH3_MAX",UINT8,&fp.remoteCtrl.ppm_ch[chnnel_3].chnnel_max,remoteCtrl_ConVal_ch3_max},
	{"JS_CON_CH4_MIN",UINT8,&fp.remoteCtrl.ppm_ch[chnnel_4].chnnel_min,remoteCtrl_ConVal_ch4_min},
	{"JS_CON_CH4_MID",UINT8,&fp.remoteCtrl.ppm_ch[chnnel_4].chnnel_mid,remoteCtrl_ConVal_ch4_mid},
	{"JS_CON_CH4_MAX",UINT8,&fp.remoteCtrl.ppm_ch[chnnel_4].chnnel_max,remoteCtrl_ConVal_ch4_max},
	{"JS_CON_CH5_MIN",UINT8,&fp.remoteCtrl.ppm_ch[chnnel_5].chnnel_min,remoteCtrl_ConVal_ch5_min},
	{"JS_CON_CH5_MID",UINT8,&fp.remoteCtrl.ppm_ch[chnnel_5].chnnel_mid,remoteCtrl_ConVal_ch5_mid},
	{"JS_CON_CH5_MAX",UINT8,&fp.remoteCtrl.ppm_ch[chnnel_5].chnnel_max,remoteCtrl_ConVal_ch5_max},
	{"JS_CON_CH6_MIN",UINT8,&fp.remoteCtrl.ppm_ch[chnnel_6].chnnel_min,remoteCtrl_ConVal_ch6_min},
	{"JS_CON_CH6_MID",UINT8,&fp.remoteCtrl.ppm_ch[chnnel_6].chnnel_mid,remoteCtrl_ConVal_ch6_mid},
	{"JS_CON_CH6_MAX",UINT8,&fp.remoteCtrl.ppm_ch[chnnel_6].chnnel_max,remoteCtrl_ConVal_ch6_max},
	{"JS_CON_CH7_MIN",UINT8,&fp.remoteCtrl.ppm_ch[chnnel_7].chnnel_min,remoteCtrl_ConVal_ch7_min},
	{"JS_CON_CH7_MID",UINT8,&fp.remoteCtrl.ppm_ch[chnnel_7].chnnel_mid,remoteCtrl_ConVal_ch7_mid},
	{"JS_CON_CH7_MAX",UINT8,&fp.remoteCtrl.ppm_ch[chnnel_7].chnnel_max,remoteCtrl_ConVal_ch7_max},
	{"JS_CON_CH8_MIN",UINT8,&fp.remoteCtrl.ppm_ch[chnnel_8].chnnel_min,remoteCtrl_ConVal_ch8_min},
	{"JS_CON_CH8_MID",UINT8,&fp.remoteCtrl.ppm_ch[chnnel_8].chnnel_mid,remoteCtrl_ConVal_ch8_mid},
	{"JS_CON_CH8_MAX",UINT8,&fp.remoteCtrl.ppm_ch[chnnel_8].chnnel_max,remoteCtrl_ConVal_ch8_max},

	{"BT0_FUNCTION",UINT8,&fp.btn_function[0],btn0_function},
	{"BT1_FUNCTION",UINT8,&fp.btn_function[1],btn1_function},
	{"BT2_FUNCTION",UINT8,&fp.btn_function[2],btn2_function},
	{"BT3_FUNCTION",UINT8,&fp.btn_function[3],btn3_function},
	{"BT4_FUNCTION",UINT8,&fp.btn_function[4],btn4_function},
	{"BT5_FUNCTION",UINT8,&fp.btn_function[5],btn5_function},
	{"BT6_FUNCTION",UINT8,&fp.btn_function[6],btn6_function},
	{"BT7_FUNCTION",UINT8,&fp.btn_function[7],btn7_function},
	{"BT8_FUNCTION",UINT8,&fp.btn_function[8],btn8_function},
	{"BT9_FUNCTION",UINT8,&fp.btn_function[9],btn9_function},
	{"BT10_FUNCTION",UINT8,&fp.btn_function[10],btn10_function},
	{"BT11_FUNCTION",UINT8,&fp.btn_function[11],btn11_function},
	{"BT12_FUNCTION",UINT8,&fp.btn_function[12],btn12_function},
	{"BT13_FUNCTION",UINT8,&fp.btn_function[13],btn13_function},
	{"BT14_FUNCTION",UINT8,&fp.btn_function[14],btn14_function},
	{"BT15_FUNCTION",UINT8,&fp.btn_function[15],btn15_function},
	{"BT0_SFUNCTION",UINT8,&fp.btn_sfunction[0],btn0_sfunction},
	{"BT1_SFUNCTION",UINT8,&fp.btn_sfunction[1],btn1_sfunction},
	{"BT2_SFUNCTION",UINT8,&fp.btn_sfunction[2],btn2_sfunction},
	{"BT3_SFUNCTION",UINT8,&fp.btn_sfunction[3],btn3_sfunction},
	{"BT4_SFUNCTION",UINT8,&fp.btn_sfunction[4],btn4_sfunction},
	{"BT5_SFUNCTION",UINT8,&fp.btn_sfunction[5],btn5_sfunction},
	{"BT6_SFUNCTION",UINT8,&fp.btn_sfunction[6],btn6_sfunction},
	{"BT7_SFUNCTION",UINT8,&fp.btn_sfunction[7],btn7_sfunction},
	{"BT8_SFUNCTION",UINT8,&fp.btn_sfunction[8],btn8_sfunction},
	{"BT9_SFUNCTION",UINT8,&fp.btn_sfunction[9],btn9_sfunction},
	{"BT10_SFUNCTION",UINT8,&fp.btn_sfunction[10],btn10_sfunction},
	{"BT11_SFUNCTION",UINT8,&fp.btn_sfunction[11],btn11_sfunction},
	{"BT12_SFUNCTION",UINT8,&fp.btn_sfunction[12],btn12_sfunction},
	{"BT13_SFUNCTION",UINT8,&fp.btn_sfunction[13],btn13_sfunction},
	{"BT14_SFUNCTION",UINT8,&fp.btn_sfunction[14],btn14_sfunction},
	{"BT15_SFUNCTION",UINT8,&fp.btn_sfunction[15],btn15_sfunction},

	{"ESC_TYPE",UINT8,&fp.esc_type,esc_type},

	{"STM_RAW_SENSORS",UINT8,&fp.stream_RAW_SENSORS_freq,stream_RAW_SENSORS_freq},
	{"STM_EXT_STATUS",UINT8,&fp.stream_EXTENDED_STATUS_freq,stream_EXTENDED_STATUS_freq},
	{"STM_RC_CHANNELS",UINT8,&fp.stream_RC_CHANNELS_freq,stream_RC_CHANNELS_freq},
	{"STM_RAW_CONT",UINT8,&fp.stream_RAW_CONTROLLER_freq,stream_RAW_CONTROLLER_freq},
	{"STM_POSITION",UINT8,&fp.stream_POSITION_freq,stream_POSITION_freq},
	{"STM_EXTRA1",UINT8,&fp.stream_EXTRA1_freq,stream_EXTRA1_freq},
	{"STM_EXTRA2",UINT8,&fp.stream_EXTRA2_freq,stream_EXTRA2_freq},
	{"STM_EXTRA3",UINT8,&fp.stream_EXTRA3_freq,stream_EXTRA3_freq},
	{"STM_ALL",UINT8,&fp.stream_ALL_freq,stream_ALL_freq},
	{"STM_FREQ_ENABLE",UINT8,&fp.stream_freq_enable,stream_freq_enable},

	{"LOG_TEST_RATE",UINT8,&fp.log_TEST_rate,log_TEST_rate},
	{"LOG_PARA_RATE",UINT8,&fp.log_PARAMETER_rate,log_PARAMETER_rate},	
	{"LOG_GPS_RATE",UINT8,&fp.log_GPS_rate,log_GPS_rate},			
	{"LOG_IMU_RATE",UINT8,&fp.log_IMU_rate,log_IMU_rate},			
	{"LOG_RCIN_RATE",UINT8,&fp.log_RCIN_rate,log_RCIN_rate},			
	{"LOG_RCOUT_RATE",UINT8,&fp.log_RCOUT_rate,log_RCOUT_rate},	
	{"LOG_BARO_RATE",UINT8,&fp.log_BARO_rate,log_BARO_rate},		
	{"LOG_POWR_RATE",UINT8,&fp.log_POWR_rate,log_POWR_rate},		
	{"LOG_ATT_RATE",UINT8,&fp.log_ATTITUDE_rate,log_ATTITUDE_rate},		
	{"LOG_MODE_RATE",UINT8,&fp.log_MODE_rate,log_MODE_rate},		
	{"LOG_POS_RATE",UINT8,&fp.log_POS_rate,log_POS_rate},		
	{"LOG_ATAR_RATE",UINT8,&fp.log_PID_ANG_RLL_rate,log_PID_ANG_RLL_rate},	
	{"LOG_ATAP_RATE",UINT8,&fp.log_PID_ANG_PIT_rate,log_PID_ANG_PIT_rate},
	{"LOG_ATAY_RATE",UINT8,&fp.log_PID_ANG_YAW_rate,log_PID_ANG_YAW_rate},	
	{"LOG_ATRR_RATE",UINT8,&fp.log_PID_RAT_RLL_rate,log_PID_RAT_RLL_rate},	
	{"LOG_ATRP_RATE",UINT8,&fp.log_PID_RAT_PIT_rate,log_PID_RAT_PIT_rate},	
	{"LOG_ATRY_RATE",UINT8,&fp.log_PID_RAT_YAW_rate,log_PID_RAT_YAW_rate},	
	{"LOG_PCAZ_RATE",UINT8,&fp.log_PID_ACC_Z_rate,log_PID_ACC_Z_rate},	
	{"LOG_PCVZ_RATE",UINT8,&fp.log_PID_VEL_Z_rate,log_PID_VEL_Z_rate},	
	{"LOG_PCPZ_RATE",UINT8,&fp.log_PID_POS_Z_rate,log_PID_POS_Z_rate},	
	{"LOG_DR_RATE",UINT8,&fp.log_DR_rate,log_DR_rate},		
	{"LOG_SINS_RATE",UINT8,&fp.log_SINS_rate,log_SINS_rate},
	{"LOG_WP_RATE",UINT8,&fp.log_WP_rate,log_WP_rate},			
	{"LOG_MDDEL_RATE",UINT8,&fp.log_MODEL_rate,log_MODEL_rate},
		
	{"ATC_R_ANG_P",REAL32,&fp.pidAngleRoll.kp,pidAngleRoll_kp},
	{"ATC_R_ANG_I",REAL32,&fp.pidAngleRoll.ki,pidAngleRoll_ki},
	{"ATC_R_ANG_D",REAL32,&fp.pidAngleRoll.kd,pidAngleRoll_kd},
	{"ATC_R_ANG_IMAX",REAL32,&fp.pidAngleRoll.iLimit,pidAngleRoll_iLimit},
	{"ATC_R_ANG_OMAX",REAL32,&fp.pidAngleRoll.outputLimit,pidAngleRoll_outputLimit},
	{"ATC_R_ANG_FF",REAL32,&fp.pidAngleRoll.feedfoward,pidAngleRoll_feedforward},
	{"ATC_P_ANG_P",REAL32,&fp.pidAnglePitch.kp,pidAnglePitch_kp},
	{"ATC_P_ANG_I",REAL32,&fp.pidAnglePitch.ki,pidAnglePitch_ki},
	{"ATC_P_ANG_D",REAL32,&fp.pidAnglePitch.kd,pidAnglePitch_kd},
	{"ATC_P_ANG_IMAX",REAL32,&fp.pidAnglePitch.iLimit,pidAnglePitch_iLimit},
	{"ATC_P_ANG_OMAX",REAL32,&fp.pidAnglePitch.outputLimit,pidAnglePitch_outputLimit},
	{"ATC_P_ANG_FF",REAL32,&fp.pidAnglePitch.feedfoward,pidAnglePitch_feedforward},
	{"ATC_Y_ANG_P",REAL32,&fp.pidAngleYaw.kp,pidAngleYaw_kp},
	{"ATC_Y_ANG_I",REAL32,&fp.pidAngleYaw.ki,pidAngleYaw_ki},
	{"ATC_Y_ANG_D",REAL32,&fp.pidAngleYaw.kd,pidAngleYaw_kd},
	{"ATC_Y_ANG_IMAX",REAL32,&fp.pidAngleYaw.iLimit,pidAngleYaw_iLimit},
	{"ATC_Y_ANG_OMAX",REAL32,&fp.pidAngleYaw.outputLimit,pidAngleYaw_outputLimit},
	{"ATC_Y_ANG_FF",REAL32,&fp.pidAngleYaw.feedfoward,pidAngleYaw_feedforward},
	{"ATC_R_RAT_P",REAL32,&fp.pidRateRoll.kp,pidRateRoll_kp},
	{"ATC_R_RAT_I",REAL32,&fp.pidRateRoll.ki,pidRateRoll_ki},
	{"ATC_R_RAT_D",REAL32,&fp.pidRateRoll.kd,pidRateRoll_kd},
	{"ATC_R_RAT_IMAX",REAL32,&fp.pidRateRoll.iLimit,pidRateRoll_iLimit},
	{"ATC_R_RAT_OMAX",REAL32,&fp.pidRateRoll.outputLimit,pidRateRoll_outputLimit},
	{"ATC_R_RAT_FF",REAL32,&fp.pidRateRoll.feedfoward,pidRateRoll_feedforward},
	{"ATC_P_RAT_P",REAL32,&fp.pidRatePitch.kp,pidRatePitch_kp},
	{"ATC_P_RAT_I",REAL32,&fp.pidRatePitch.ki,pidRatePitch_ki},
	{"ATC_P_RAT_D",REAL32,&fp.pidRatePitch.kd,pidRatePitch_kd},
	{"ATC_P_RAT_IMAX",REAL32,&fp.pidRatePitch.iLimit,pidRatePitch_iLimit},
	{"ATC_P_RAT_OMAX",REAL32,&fp.pidRatePitch.outputLimit,pidRatePitch_outputLimit},
	{"ATC_P_RAT_FF",REAL32,&fp.pidRatePitch.feedfoward,pidRatePitch_feedforward},
	{"ATC_Y_RAT_P",REAL32,&fp.pidRateYaw.kp,pidRateYaw_kp},
	{"ATC_Y_RAT_I",REAL32,&fp.pidRateYaw.ki,pidRateYaw_ki},
	{"ATC_Y_RAT_D",REAL32,&fp.pidRateYaw.kd,pidRateYaw_kd},
	{"ATC_Y_RAT_IMAX",REAL32,&fp.pidRateYaw.iLimit,pidRateYaw_iLimit},
	{"ATC_Y_RAT_OMAX",REAL32,&fp.pidRateYaw.outputLimit,pidRateYaw_outputLimit},
	{"ATC_Y_RAT_FF",REAL32,&fp.pidRateYaw.feedfoward,pidRateYaw_feedforward},
	{"POC_Z_POS_P",REAL32,&fp.pidPosZ.kp,pidPosZ_kp},
	{"POC_Z_POS_I",REAL32,&fp.pidPosZ.ki,pidPosZ_ki},
	{"POC_Z_POS_D",REAL32,&fp.pidPosZ.kd,pidPosZ_kd},
	{"POC_Z_POS_IMAX",REAL32,&fp.pidPosZ.iLimit,pidPosZ_iLimit},
	{"POC_Z_POS_OMAX",REAL32,&fp.pidPosZ.outputLimit,pidPosZ_outputLimit},
	{"POC_Z_POS_FF",REAL32,&fp.pidPosZ.feedfoward,pidPosZ_feedforward},
	{"POC_Z_VER_P",REAL32,&fp.pidVelZ.kp,pidVelZ_kp},
	{"POC_Z_VER_I",REAL32,&fp.pidVelZ.ki,pidVelZ_ki},
	{"POC_Z_VER_D",REAL32,&fp.pidVelZ.kd,pidVelZ_kd},
	{"POC_Z_VER_IMAX",REAL32,&fp.pidVelZ.iLimit,pidVelZ_iLimit},
	{"POC_Z_VER_OMAX",REAL32,&fp.pidVelZ.outputLimit,pidVelZ_outputLimit},
	{"POC_Z_VER_FF",REAL32,&fp.pidVelZ.feedfoward,pidVelZ_feedforward},
	{"POC_Z_ACC_P",REAL32,&fp.pidAccZ.kp,pidAccZ_kp},
	{"POC_Z_ACC_I",REAL32,&fp.pidAccZ.ki,pidAccZ_ki},
	{"POC_Z_ACC_D",REAL32,&fp.pidAccZ.kd,pidAccZ_kd},
	{"POC_Z_ACC_IMAX",REAL32,&fp.pidAccZ.iLimit,pidAccZ_iLimit},
	{"POC_Z_ACC_OMAX",REAL32,&fp.pidAccZ.outputLimit,pidAccZ_outputLimit},
	{"POC_Z_ACC_FF",REAL32,&fp.pidAccZ.feedfoward,pidAccZ_feedforward},

};

/*
规定：
控制量			正桨				反桨
------------------------
1000-1500	反转	/推力向下		正转/推力向上
1500-2000	正转/推力向上			反转/推力向下
*/
const float motor_fac[6][6]={
	//Roll_Factor 	Pitch_Factor	Yaw_Factor		Throttle_Factor 	Forward_Factor		Lateral_Factor	
	{-1.0,			1.0,			0.0,			1.0,				0.0,				0.0},
	{1.0,			1.0,			0.0,			1.0,				0.0,				0.0},
	{1.0,			-1.0,			0.0,			1.0,				0.0,				0.0},
	{-1.0,			-1.0,			0.0,			1.0,				0.0,				0.0},
	{0.0,			0.0,			-1.0,			0.0,				1.0,				0.0},
	{0.0,			0.0,			1.0,			0.0,				1.0,				0.0},
};

//使用默认参数赋值变量
PARA_FIXED fp;

PARA_VARYING vp={
	.sys_time={2020,1,1,12,0,0},
	.control_mode = MANUAL,	//初始控制模式为手动
	.time_available_max = 0,
	.time_available_min = INT16_MAX,
};
SYS_FLAGS sys_flag={
	.log_creat = false,
	.health.baro = true,
	.health.gps = false,
	.show_task_pass = false,
	.acc_vibration = true,	//标记已完成加计校准
};
SYS_TIME sys_time= {
	.ten_micro = 0,//系统10us
	.one_mill = 0,//系统ms
	.one_second = 0,//系统s
	.one_minute = 0,//系统min
	.loop_tick = 0,	//系统循环时钟
};

Channel channel = {
	.roll = 0,
	.pitch = 0,
	.yaw = 0,
	.throttle = 0,
	.forward = 0,
	.lateral = 0
};

//初始化电机矩阵 -> 为什么不能在初始化列表中赋值?
void motor_fac_init(){
	fp.motor1_fac.roll_fac = motor_fac[0][0];	//各电机自由度分配比例 motor_fac[0][0]
	fp.motor1_fac.pitch_fac = motor_fac[0][1];
	fp.motor1_fac.yaw_fac = motor_fac[0][2];
	fp.motor1_fac.throttle_fac = motor_fac[0][3];
	fp.motor1_fac.forward_fac = motor_fac[0][4];
	fp.motor1_fac.lateral_fac = motor_fac[0][5];
	fp.motor2_fac.roll_fac = motor_fac[1][0];	
	fp.motor2_fac.pitch_fac = motor_fac[1][1];
	fp.motor2_fac.yaw_fac = motor_fac[1][2];
	fp.motor2_fac.throttle_fac = motor_fac[1][3];
	fp.motor2_fac.forward_fac = motor_fac[1][4];
	fp.motor2_fac.lateral_fac = motor_fac[1][5];
	fp.motor3_fac.roll_fac = motor_fac[2][0]; 
	fp.motor3_fac.pitch_fac = motor_fac[2][1];
	fp.motor3_fac.yaw_fac = motor_fac[2][2];
	fp.motor3_fac.throttle_fac = motor_fac[2][3];
	fp.motor3_fac.forward_fac = motor_fac[2][4];
	fp.motor3_fac.lateral_fac = motor_fac[2][5];
	fp.motor4_fac.roll_fac = motor_fac[3][0];	
	fp.motor4_fac.pitch_fac = motor_fac[3][1];
	fp.motor4_fac.yaw_fac = motor_fac[3][2];
	fp.motor4_fac.throttle_fac = motor_fac[3][3];
	fp.motor4_fac.forward_fac = motor_fac[3][4];
	fp.motor4_fac.lateral_fac = motor_fac[3][5];
	fp.motor5_fac.roll_fac = motor_fac[4][0];	
	fp.motor5_fac.pitch_fac = motor_fac[4][1];
	fp.motor5_fac.yaw_fac = motor_fac[4][2];
	fp.motor5_fac.throttle_fac = motor_fac[4][3];
	fp.motor5_fac.forward_fac = motor_fac[4][4];
	fp.motor5_fac.lateral_fac = motor_fac[4][5];
	fp.motor6_fac.roll_fac = motor_fac[5][0];	
	fp.motor6_fac.pitch_fac = motor_fac[5][1];
	fp.motor6_fac.yaw_fac = motor_fac[5][2];
	fp.motor6_fac.throttle_fac = motor_fac[5][3];
	fp.motor6_fac.forward_fac = motor_fac[5][4];
	fp.motor6_fac.lateral_fac = motor_fac[5][5];
}

void param_default_init(){
	fp.param_num = sizeof(param_id_type_value)/sizeof(param_id_type_value[0]);
	//	fp.param_num = para_last-param_num+1; 
	//波特率
	fp.uartA_baud = 115200; 	//MPU9250
	fp.uartB_baud = 115200;	//WiFi
	fp.uartC_baud = 57600;	//TELEM
	fp.uartD_baud = 9600; 	//GPS
	fp.uartE_baud = 115200;	//SERIAL2
	fp.uartF_baud = 115200;	//DEBUG

	fp.frame_class = FRAME_HAIZHE1;	//海蜇号机架模型
	fp.sysid_my_gcs = 0xff;  //系统mavlink system id
	fp.power_min = 9.0;	//电池最小电压
	fp.log_mode = LOG_Armed;	//默认在电机解锁时记录日志
	fp.ROV_mode_enable = false;	//默认不使能rov模式

	fp.wp_radius_cm = 200,		//默认航点到达半径2m
	fp.dr_mode = DR_SINS_GPS,	//默认SINS/GPS组合导航模式
	fp.wp_path = PATH_LINE;		//默认路径是直线
	fp.gps_fb_interval = 10;	//默认在水下导航时每隔10秒通过GPS校准一次

	//使能所有安全检测
	fp.health_chenable.imu = true;
	fp.health_chenable.baro = true;
	fp.health_chenable.batt = true;
	fp.health_chenable.joystick = true;
	fp.health_chenable.gps = true;
	fp.health_chenable.sd = true;
	fp.health_chenable.flash = true;

	fp.throttle_deadzone_swerve = 0.1;	//转向死区油门
	fp.throttle_deadzone_stabilize = 0.1;	//水平姿态稳定油门死区
	fp.throttle_hover = 0.0;	//水下悬停油门(0fp.0-1fp.0)
	
	fp.attitude_init[0] = 0.0;	//初始姿态 roll
	fp.attitude_init[1] = 0.0;	//初始姿态 pitch
	fp.attitude_init[2] = 0.0;	//初始姿态 yaw
	fp.sins_db[0] = 0.0;	//x轴加计零漂
	fp.sins_db[1] = 0.0;	
	fp.sins_db[2] = 0.0;

	fp.exit_mission_rtl = false;	//任务结束是否返回home点
	fp.forward_auto = 0.2; 	//AUTO模式下前进油门

	//	fp.remoteCtrlfp.type = T8FB;	//默认遥控器
	//	fp.remoteCtrlfp.ppm_ch ={	{49;		100;		150};
	//							{49;		100;		150};
	//							{49;		97; 		150};
	//							{49;		100;		150};
	//							{49;		100;		150};
	//							{49;		100;		150};
	//							{49;		100;		150};
	//							{49;		100;		150};}; //默认归中油门
	//	fp.remoteCtrlfp.unconnect_val = {100;100;37;100;100;100;100;100};	//默认未连接油门
	fp.remoteCtrl.type = WFLY;	//默认遥控器
	PPM_CHNEL ppm_ch[8] = {	{70,		112,		155},
							{70,		112,		155},
							{79,		112,		144},
							{70,		112,		155},
							{70,		112,		155},
							{70,		112,		155},
							{70,		112,		155},
							{70,		112,		155}};
	memcpy(fp.remoteCtrl.ppm_ch,ppm_ch,sizeof(ppm_ch)/sizeof(u8));	//默认归中油门
	u8 unconnect_val[8] = {112,112,48,112,112,112,112,112};
	memcpy(fp.remoteCtrl.unconnect_val,unconnect_val,8);	//默认未连接油门
	
	joystick_init();//手柄按键功能默认设置
	
	fp.esc_type = TWO_WAY;	//默认双向电调
	
	motor_fac_init();

	//向qgc传输各类数据频率
	fp.stream_RAW_SENSORS_freq = 2;				
	fp.stream_EXTENDED_STATUS_freq = 2;
	fp.stream_RC_CHANNELS_freq = 2;
	fp.stream_RAW_CONTROLLER_freq = 2;
	fp.stream_POSITION_freq = 2;
	fp.stream_EXTRA1_freq = 10;
	fp.stream_EXTRA2_freq = 2;
	fp.stream_EXTRA3_freq = 2;
	fp.stream_ALL_freq = 2;

	//日志记录频率
	fp.log_TEST_rate = 1; 		
	fp.log_PARAMETER_rate = 10;	
	fp.log_GPS_rate = 1;			
	fp.log_IMU_rate = 100;			
	fp.log_RCIN_rate = 10;			
	fp.log_RCOUT_rate = 10;	
	fp.log_BARO_rate = 10;		
	fp.log_POWR_rate = 2;		
	fp.log_ATTITUDE_rate = 100;		
	fp.log_MODE_rate = 1;		
	fp.log_POS_rate = 2;		
	fp.log_PID_ANG_RLL_rate = 10;	
	fp.log_PID_ANG_PIT_rate = 10;
	fp.log_PID_ANG_YAW_rate = 10;	
	fp.log_PID_RAT_RLL_rate = 10;	
	fp.log_PID_RAT_PIT_rate = 10;	
	fp.log_PID_RAT_YAW_rate = 10;	
	fp.log_PID_ACC_Z_rate = 10;	
	fp.log_PID_VEL_Z_rate = 10;	
	fp.log_PID_POS_Z_rate = 10;	
	fp.log_DR_rate = 20;	
	fp.log_SINS_rate = 20;
	fp.log_WP_rate = 10;			
	fp.log_MODEL_rate = 20;	
	
	//pidAngleRoll
	fp.pidAngleRoll.kp = 1; 
	fp.pidAngleRoll.ki = 0; 
	fp.pidAngleRoll.kd = 0; 
	fp.pidAngleRoll.iLimit = PID_ANGLE_ROLL_OUTPUT_LIMIT; 
	fp.pidAngleRoll.outputLimit = PID_ANGLE_ROLL_OUTPUT_LIMIT; 
	fp.pidAngleRoll.feedfoward = 0;
	//pidAnglePitch
	fp.pidAnglePitch.kp = 1; 
	fp.pidAnglePitch.ki = 0; 
	fp.pidAnglePitch.kd = 0; 
	fp.pidAnglePitch.iLimit = PID_ANGLE_PITCH_OUTPUT_LIMIT; 
	fp.pidAnglePitch.outputLimit = PID_ANGLE_PITCH_OUTPUT_LIMIT; 
	fp.pidAnglePitch.feedfoward = 0;
	//pidAngleYaw
	fp.pidAngleYaw.kp = 1; 
	fp.pidAngleYaw.ki = 0; 
	fp.pidAngleYaw.kd = 0; 
	fp.pidAngleYaw.iLimit = PID_ANGLE_YAW_OUTPUT_LIMIT; 
	fp.pidAngleYaw.outputLimit = PID_ANGLE_YAW_OUTPUT_LIMIT; 
	fp.pidAngleYaw.feedfoward = 0;
	//pidRateRoll
	fp.pidRateRoll.kp = 1.0; 
	fp.pidRateRoll.ki = 0.5; 
	fp.pidRateRoll.kd = 0.01; 
	fp.pidRateRoll.iLimit = 50; 
	fp.pidRateRoll.outputLimit = PID_RATE_ROLL_OUTPUT_LIMIT; 
	fp.pidRateRoll.feedfoward = 0;
	//pidRatePitch
	fp.pidRatePitch.kp = 1.0; 
	fp.pidRatePitch.ki = 0.5; 
	fp.pidRatePitch.kd = 0.001; 
	fp.pidRatePitch.iLimit = 50; 
	fp.pidRatePitch.outputLimit = PID_RATE_PITCH_OUTPUT_LIMIT; 
	fp.pidRatePitch.feedfoward = 0;
	//pidRateYaw
	fp.pidRateYaw.kp = 1; 
	fp.pidRateYaw.ki = 0.5; 
	fp.pidRateYaw.kd = 0.02; 
	fp.pidRateYaw.iLimit = 50; 
	fp.pidRateYaw.outputLimit = PID_RATE_YAW_OUTPUT_LIMIT; 
	fp.pidRateYaw.feedfoward = 30;	//脱离推进器死区
	//pidPosZ
	fp.pidPosZ.kp = 1; 
	fp.pidPosZ.ki = 0; 
	fp.pidPosZ.kd = 0; 
	fp.pidPosZ.iLimit = 100; 
	fp.pidPosZ.outputLimit = PID_POS_Z_OUTPUT_LIMIT; 
	fp.pidPosZ.feedfoward = 0;
	//pidVelZ
	fp.pidVelZ.kp = 1; 
	fp.pidVelZ.ki = 0; 
	fp.pidVelZ.kd = 0; 
	fp.pidVelZ.iLimit = 100; 
	fp.pidVelZ.outputLimit = PID_VEL_Z_OUTPUT_LIMIT; 
	fp.pidVelZ.feedfoward = 0;
	//pidAccZ
	fp.pidAccZ.kp = 1; 
	fp.pidAccZ.ki = 0; 
	fp.pidAccZ.kd = 0; 
	fp.pidAccZ.iLimit = 100; 
	fp.pidAccZ.outputLimit = PID_ACC_Z_OUTPUT_LIMIT; 
	fp.pidAccZ.feedfoward = 0;
}

//检查param_id_type_value是否包含了全部参数
bool param_num_check(){
	u16 fp_size = para_last-param_num;	//计算flash_para_list中参数个数(即枚举值个数，前提枚举值连续)
	return (fp.param_num==fp_size);
}

