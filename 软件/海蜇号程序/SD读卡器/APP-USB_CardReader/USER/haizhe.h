#ifndef _HAIZHE_H
#define _HAIZHE_H
	 
#include "sys.h"	 
#include "delay.h"
#include "stdio.h"
#include <string.h>
#include  <math.h>    //Keil library  	
#include "uart.h"
#include "malloc.h"
#include "led.h"
#include "sdio_sdcard.h"
#include "malloc.h" 
#include "ff.h"  
#include "exfuns.h"    
#include "diskio.h"	
#include "usbd_msc_core.h"
#include "usbd_usr.h"
#include "usbd_desc.h"
#include "usb_conf.h"
#include "usbd_msc_bot.h"

#define BOOT_ENABLE true

#define CONSOLE_UART USART6
#define CONSOLE_UART_Init(uart,baud) UART_init(USART6,baud)
#define CONSOLE_uart_available USART6_RX_STA
#define CONSOLE_uart_buf USART6_RX_BUF

extern USB_OTG_CORE_HANDLE USB_OTG_dev;

void init_haizhe(void);
void system_init(void);

#endif
