#ifndef _GCS_COMMON_H
#define _GCS_COMMON_H

#include "sys.h"
#include "uart.h"
#include "GCS.h"

#define mav_uart_available 	UART7_RX_STA
#define mav_uart_buf 		UART7_RX_BUF

void send_all_param(void);
void send_message(enum mav_message id);
u8 stream_trigger(enum streams stream_num);
void gcs_update(void);

void send_statustext(u8 severity,const char text[50]);

void set_streamRates(int type, int freq);

#endif

