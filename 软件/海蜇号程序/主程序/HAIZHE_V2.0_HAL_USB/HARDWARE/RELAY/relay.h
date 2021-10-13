#ifndef _RELAY_H
#define _RELAY_H

#include "sys.h"
#include "timer.h"

#define on 0
#define off 1

#define RELAY1_GPIO GPIOA
#define RELAY1_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()
#define RELAY1_PIN		GPIO_PIN_2
#define RELAY1 PAout(2)

#define RELAY2_GPIO GPIOA
#define RELAY2_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()
#define RELAY2_PIN		GPIO_PIN_3
#define RELAY2 PAout(3)

#define RELAY_12V_GPIO GPIOC
#define RELAY_12V_CLK_ENABLE() __HAL_RCC_GPIOC_CLK_ENABLE()
#define RELAY_12V_PIN		GPIO_PIN_5
#define RELAY_12V PCout(5)

void relay_init(void);
void relay_12V_check(void);
bool get_relay_12V_status(void);

#endif


