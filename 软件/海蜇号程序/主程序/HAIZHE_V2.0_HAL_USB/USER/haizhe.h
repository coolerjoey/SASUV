#ifndef _HAIZHE_H
#define _HAIZHE_H
	 
#include "sys.h"	 
#include "delay.h"
#include "stdio.h"
#include <string.h>
#include  <math.h>    //Keil library  	
#include "mymath.h"
#include "global_para.h"
#include "GCS_Mavlink.h"
#include "scheduler.h"
#include "parameter.h"	
#include "uart.h"
#include "adc.h"
#include "malloc.h"

#include "config.h"
#include "led.h"
#include "sdio_sdcard.h"
#include "malloc.h" 
#include "ff.h"  
#include "exfuns.h"    
#include "diskio.h"	
//#include "exti.h"
#include "servo.h"
#include "flash.h"	
#include "telem.h"
#include "relay.h"
//#include "esp8266.h"
#include "barometer.h"
#include "motors.h"
#include "sensor.h"
#include "attitude_controller.h"
#include "position_controller.h"
#include "atkp.h"
#include "log.h"
#include "test.h"
#include "ppm.h"
////#include "PID_control.h"
#include "gps.h"
#include "console.h"
#include "camera.h"

#include "control_mode.h"
#include "logger.h"
#include "rtc.h"
#include "model.h"
#include "model_verify.h"
#include "mission.h"

#include "w5500_conf.h"
#include "kf.h"
#include "kf_verify.h"

//#include "motors.h"
//#include "MAV_Haizhe.h"
//#include "vector.h"
//
//#include "pid.h"
//#include "openmv.h"

void fifty_hz_loop(void);
void sys_status_check(void);
void user_one_hz_task(void);	
void user_three_hz_task(void);
void user_ten_hz_task(void);
void user_twenty_hz_task(void);

void fast_loop(uint16_t);
void init_haizhe(void);
void system_init(void);
void load_default_param(void);


#endif
