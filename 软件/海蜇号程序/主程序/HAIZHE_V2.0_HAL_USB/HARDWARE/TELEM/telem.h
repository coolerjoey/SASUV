#ifndef _TELEM_H
#define _TELEM_H

#include "sys.h"
#include "timer.h"
#include "telem_TTL_xG.h"

#define TELEM_TYPE TTL_xG
#define telem_uart_available	TTL_xG_UART_RX_STA
#define telem_uart_buf TTL_xG_UART_RX_BUF

#if TELEM_TYPE==TTL_xG
#define telem_send TTL_xG_Send
#define telem_write TTL_xG_write
//#elif
#endif

void telem_init(void);
#endif
