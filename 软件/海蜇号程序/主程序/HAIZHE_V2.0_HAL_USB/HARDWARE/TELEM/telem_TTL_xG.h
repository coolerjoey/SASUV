#ifndef _TELEM_TTL_xG_H
#define _TELEM_TTL_xG_H

#include "sys.h"
#include "uart.h"
#include "config.h"



#define TTL_xG_GPIO GPIOE
#define TTL_xG_CLK_ENABLE() __HAL_RCC_GPIOE_CLK_ENABLE()
#define TTL_xG_M0		GPIO_PIN_3
#define TTL_xG_M1		GPIO_PIN_10
#define TTL_xG_AUX	GPIO_PIN_12

void TTL_xG_init(void);

#endif
