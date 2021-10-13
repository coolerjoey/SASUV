#ifndef _LED_H
#define _LED_H

#include "sys.h"
#include "timer.h"

#define DMA_WS2812_ENBALE 0	//是否开启DMA模式 1-开启 0-关闭

#define LED_DIODE_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()
#define LED_DIODE_GPIO	GPIOA
#define LED_DIODE_BE_PIN GPIO_PIN_8
#define LED_DIODE_ACT_PIN	GPIO_PIN_10
#define led_act PAout(10)
#define led_be	PAout(8)

#define LED_WS2812_GPIO_CLK_ENABLE()	__HAL_RCC_GPIOA_CLK_ENABLE()
#define LED_WS2812_GPIO	GPIOA
#define LED_WS2812_PIN	GPIO_PIN_15


#define WS2812_TIM_Handle htim2
#define WS2812_TIM_CHANNEL TIM_CHANNEL_1
#define WS2812_TIM_Init(arr,psc) TIM2_PWM_Init(arr,psc)
#define WS2813_TIM	TIM2
#define WS2813_TIM_DMA_Init() TIM2_DMA_Init()
#define WS2813_TIM_DMA_HANDLE hdma_tim2_ch1

#define ARR	28	//900
#define PSC 4//分频值 100000
#define TIMING_ONE  ARR*4/5	//表示1逻辑占空比50%？
#define TIMING_ZERO ARR*1/5	//表示0逻辑占空比25%

void led_init(void);
void LED_Doide_init(void);
void LED_WS2812_init(void);
void LED_RGB_Set(u8 red,u8 green,u8 blue);
void RGBLED_RED_twinkle(void);
void RGBLED_GREEN_twinkle(void);
void RGBLED_BLUE_twinkle(void);

#endif
