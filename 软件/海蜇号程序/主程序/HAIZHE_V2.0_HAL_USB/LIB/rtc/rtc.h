#ifndef __RTC_H
#define __RTC_H
#include "sys.h"	

extern RTC_HandleTypeDef RTC_Handler;  //RTC句柄    
extern RTC_HandleTypeDef hrtc;

void rtc_init(void);
HAL_StatusTypeDef RTC_Set_Time(u8 hour,u8 min,u8 sec,u8 ampm);      //RTC时间设置
HAL_StatusTypeDef RTC_Set_Date(u8 year,u8 month,u8 date,u8 week);	//RTC日期设置
void RTC_Set_AlarmA(u8 week,u8 hour,u8 min,u8 sec); //设置闹钟时间(按星期闹铃,24小时制)
void RTC_Set_WakeUp(u32 wksel,u16 cnt);             //周期性唤醒定时器设置
void rtc_update(void);

#endif
