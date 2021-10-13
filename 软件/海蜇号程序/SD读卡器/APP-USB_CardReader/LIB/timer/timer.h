#ifndef __PWM_H
#define __PWM_H
#include "sys.h"

extern TIM_HandleTypeDef TIM3_Handler;      //定时器3PWM句柄 
extern TIM_OC_InitTypeDef TIM3_CH4Handler;  //定时器3通道4句柄
extern TIM_HandleTypeDef htim2;
extern DMA_HandleTypeDef  hdma_tim2_ch1;

void TIM1_PWM_Init(u16 arr,u16 psc);
void TIM2_PWM_Init(u16 arr,u16 psc);
void TIM2_DMA_Init(void);
void TIM2_CAP_Init(u16 arr,u16 psc);
void TIM3_Int_Init(u16 arr,u16 psc);
void TIM4_PWM_Init(u16 arr,u16 psc);
void TIM5_Int_Init(u16 arr,u16 psc);

void TIM_SetTIM3Compare4(u32 compare);
#endif

