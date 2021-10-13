#include "timer.h"
#include "led.h"


TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

TIM_OC_InitTypeDef TIM3_CH4Handler;	    //定时器3通道4句柄
TIM_HandleTypeDef TIM_Handler;      //定时器句柄 

//以下从cubemx生成项目中拷贝，需要做一定修改，修改原因:TODO
void TIM1_PWM_Init(u16 arr,u16 psc){

	 TIM_MasterConfigTypeDef sMasterConfig;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = psc;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = arr;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  HAL_TIM_PWM_Init(&htim1);
	
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig);

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = arr/2;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;

  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);
  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2);
  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3);
  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
//  HAL_TIM_MspPostInit(&htim1);
}

DMA_HandleTypeDef  hdma_tim2_ch1;
//void TIM2_PWM_Init(u16 arr,u16 psc){
//	__HAL_RCC_DMA1_CLK_ENABLE();//DMA1时钟使能
//	HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
//  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
//	
//  TIM_MasterConfigTypeDef sMasterConfig;
//  TIM_OC_InitTypeDef sConfigOC;

//  htim2.Instance = TIM2;
//  htim2.Init.Prescaler = psc;
//  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
//  htim2.Init.Period = arr;
//  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//	
//	

//	hdma_tim2_ch1.Instance = DMA1_Stream5;
//	hdma_tim2_ch1.Init.Channel = DMA_CHANNEL_3;
//	hdma_tim2_ch1.Init.Direction = DMA_MEMORY_TO_PERIPH;
//	hdma_tim2_ch1.Init.PeriphInc = DMA_PINC_DISABLE;
//	hdma_tim2_ch1.Init.MemInc = DMA_MINC_ENABLE;
//	hdma_tim2_ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
//	hdma_tim2_ch1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
//	hdma_tim2_ch1.Init.Mode = DMA_NORMAL;
//	hdma_tim2_ch1.Init.Priority = DMA_PRIORITY_MEDIUM;
//	hdma_tim2_ch1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
//	HAL_DMA_Init(&hdma_tim2_ch1);

//    __HAL_LINKDMA(&htim2,hdma[TIM_DMA_ID_CC1],hdma_tim2_ch1);
//		    __HAL_LINKDMA(&htim2,hdma[TIM_DMA_ID_TRIGGER],hdma_tim2_ch1);
//	
//	  HAL_TIM_PWM_Init(&htim2);

//  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//  HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);

//  sConfigOC.OCMode = TIM_OCMODE_PWM1;
//  sConfigOC.Pulse =  arr/2;
//  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
//  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
//  HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);

//  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);//开启PWM通道1
//}

void TIM2_PWM_Init(u16 arr,u16 psc){
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = psc;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = arr;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	
  HAL_TIM_PWM_Init(&htim2);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse =  arr/2;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);
	  
//	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);//开启PWM通道1


//  HAL_TIM_IC_Init(&htim2);	//初始化输入捕获时基参数
//	
//  TIM_IC_InitTypeDef sConfigIC;
//  sConfigIC.ICPolarity = TIM_ICPOLARITY_RISING;
//  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
//  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
//  sConfigIC.ICFilter = 0;
//  HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_4);//配置TIM2通道4

//  HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_4);   //开启TIM5的捕获通道1，并且开启捕获中断
//  __HAL_TIM_ENABLE_IT(&htim2,TIM_IT_UPDATE);   //使能更新中断
}

void TIM2_CAP_Init(u16 arr,u16 psc){
	TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = psc;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = arr;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_IC_Init(&htim2);	//初始化输入捕获时基参数
	
  sConfigIC.ICPolarity = TIM_ICPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_4);//配置TIM2通道4

  HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_4);   //开启TIM5的捕获通道1，并且开启捕获中断
  __HAL_TIM_ENABLE_IT(&htim2,TIM_IT_UPDATE);   //使能更新中断
}

//DMA_HandleTypeDef  hdma_tim2_ch1;
//void TIM2_DMA_Init(){


//	__HAL_RCC_DMA1_CLK_ENABLE();//DMA1时钟使能 
//	__HAL_LINKDMA(&htim2,hdma[TIM_DMA_ID_CC1],hdma_tim2_ch1); 
//	
//	hdma_tim2_ch1.Instance=DMA1_Stream5;                            //数据流选择
//	hdma_tim2_ch1.Init.Channel=DMA_CHANNEL_3;                                //通道选择
//	hdma_tim2_ch1.Init.Direction=DMA_MEMORY_TO_PERIPH;             //存储器到外设
//	hdma_tim2_ch1.Init.PeriphInc=DMA_PINC_DISABLE;                 //外设非增量模式
//	hdma_tim2_ch1.Init.MemInc=DMA_MINC_ENABLE;                     //存储器增量模式
//	hdma_tim2_ch1.Init.PeriphDataAlignment=DMA_PDATAALIGN_BYTE;    //外设数据长度:8位
//	hdma_tim2_ch1.Init.MemDataAlignment=DMA_MDATAALIGN_BYTE;       //存储器数据长度:8位
//	hdma_tim2_ch1.Init.Mode=DMA_NORMAL;                            //外设普通模式
//	hdma_tim2_ch1.Init.Priority=DMA_PRIORITY_MEDIUM;               //中等优先级
//	hdma_tim2_ch1.Init.FIFOMode=DMA_FIFOMODE_DISABLE;              
//	hdma_tim2_ch1.Init.FIFOThreshold=DMA_FIFO_THRESHOLD_FULL;      
//	hdma_tim2_ch1.Init.MemBurst=DMA_MBURST_SINGLE;                 //存储器突发单次传输
//	hdma_tim2_ch1.Init.PeriphBurst=DMA_PBURST_SINGLE;              //外设突发单次传输
//	
//	
//	HAL_DMA_DeInit(&hdma_tim2_ch1);   
//	HAL_DMA_Init(&hdma_tim2_ch1);


//  /* DMA interrupt init */
//  /* DMA1_Stream5_IRQn interrupt configuration */
//  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
//  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
//}
	

void DMA1_Stream5_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_tim2_ch1);
  /* USER CODE BEGIN DMA1_Stream5_IRQn 1 */

  /* USER CODE END DMA1_Stream5_IRQn 1 */
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	if(htim==(&htim2))
//		HAL_TIM_PWM_Stop_DMA(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
}

/* TIM3 init function */
void TIM3_Int_Init(u16 arr,u16 psc)
{
		htim3.Instance=TIM3;                          //通用定时器3
    htim3.Init.Prescaler=psc;                     //分频系数
    htim3.Init.CounterMode=TIM_COUNTERMODE_UP;    //向上计数器
    htim3.Init.Period=arr;                        //自动装载值
    htim3.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;//时钟分频因子
    HAL_TIM_Base_Init(&htim3);
    
    HAL_TIM_Base_Start_IT(&htim3); //使能定时器3和定时器3更新中断：TIM_IT_UPDATE									 
}

void TIM4_PWM_Init(u16 arr,u16 psc){
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = psc;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = arr;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_PWM_Init(&htim4);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = arr/2;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	
  HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1);
  HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2);
  HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3);
  HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);
/**/
	//经过测试，此处无法一起开启四个通道，而是需要依次开启
//  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1|TIM_CHANNEL_2|TIM_CHANNEL_3|TIM_CHANNEL_4);//开启PWM通道4
}

void TIM5_Int_Init(u16 arr,u16 psc){
	htim5.Instance=TIM5;                          //通用定时器5
	htim5.Init.Prescaler=psc;                     //分频系数
	htim5.Init.CounterMode=TIM_COUNTERMODE_UP;    //向上计数器
	htim5.Init.Period=arr;                        //自动装载值
	htim5.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;//时钟分频因子
	HAL_TIM_Base_Init(&htim5);

	HAL_TIM_Base_Start_IT(&htim5); //使能定时器5和定时器5更新中断：TIM_IT_UPDATE			

}

//定时器底册驱动，开启时钟，设置中断优先级
//此函数会被HAL_TIM_Base_Init()函数调用
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==TIM1){}
	else if(htim->Instance==TIM2){}	
  else if(htim->Instance==TIM3){
		__HAL_RCC_TIM3_CLK_ENABLE();            //使能TIM7时钟
		HAL_NVIC_SetPriority(TIM3_IRQn,0,2);    //设置中断优先级，抢占优先级0，子优先级1
		HAL_NVIC_EnableIRQ(TIM3_IRQn);          //开启ITM7中断   
	}	
	else if(htim->Instance==TIM4){}
	else if(htim->Instance==TIM5){
		__HAL_RCC_TIM5_CLK_ENABLE();            //使能TIM7时钟
		HAL_NVIC_SetPriority(TIM5_IRQn,0,0);    //设置中断优先级，抢占优先级0，子优先级1
		HAL_NVIC_EnableIRQ(TIM5_IRQn);          //开启ITM7中断   
	}		
}

//定时器底层驱动，时钟使能，引脚配置
//此函数会被HAL_TIM_PWM_Init()调用
//htim:定时器句柄
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
    GPIO_InitTypeDef GPIO_Initure;
	if(htim->Instance==TIM1){
		/**TIM1 GPIO Configuration    
    PE9     ------> TIM1_CH1
    PE11     ------> TIM1_CH2
    PE13     ------> TIM1_CH3
    PE14     ------> TIM1_CH4 
    */
		__HAL_RCC_TIM1_CLK_ENABLE();			
		__HAL_RCC_GPIOE_CLK_ENABLE();		

		GPIO_Initure.Pin=GPIO_PIN_9|GPIO_PIN_11|GPIO_PIN_13|GPIO_PIN_14;         
		GPIO_Initure.Mode=GPIO_MODE_AF_PP;  	
//		GPIO_Initure.Pull=GPIO_PULLUP;         
//		GPIO_Initure.Speed=GPIO_SPEED_HIGH;  
		GPIO_Initure.Pull = GPIO_NOPULL;
		GPIO_Initure.Speed = GPIO_SPEED_FREQ_LOW;		
		GPIO_Initure.Alternate= GPIO_AF1_TIM1;	
		HAL_GPIO_Init(GPIOE,&GPIO_Initure);
	}
	else if(htim->Instance==TIM2){
				__HAL_RCC_TIM2_CLK_ENABLE();		
		__HAL_RCC_GPIOA_CLK_ENABLE();			

		GPIO_Initure.Pin=GPIO_PIN_15;           
		GPIO_Initure.Mode=GPIO_MODE_AF_PP;  
		GPIO_Initure.Pull=GPIO_PULLUP;          
		GPIO_Initure.Speed=GPIO_SPEED_HIGH;     
		GPIO_Initure.Alternate= GPIO_AF1_TIM2;	
		HAL_GPIO_Init(GPIOA,&GPIO_Initure);
	}
	else if(htim->Instance==TIM3){}
	else if(htim->Instance==TIM4){
		/**TIM4 GPIO Configuration    
    PD12     ------> TIM4_CH1
    PD13     ------> TIM4_CH2
    PD14     ------> TIM4_CH3
    PD15     ------> TIM4_CH4 
    */
		__HAL_RCC_TIM4_CLK_ENABLE();			
		__HAL_RCC_GPIOD_CLK_ENABLE();	

		GPIO_Initure.Pin=GPIO_PIN_12|GPIO_PIN_15|GPIO_PIN_13|GPIO_PIN_14;           	
		GPIO_Initure.Mode=GPIO_MODE_AF_PP;  	
//		GPIO_Initure.Pull=GPIO_PULLUP;         
//		GPIO_Initure.Speed=GPIO_SPEED_HIGH; 
    GPIO_Initure.Pull = GPIO_NOPULL;
    GPIO_Initure.Speed = GPIO_SPEED_FREQ_LOW;		
		GPIO_Initure.Alternate= GPIO_AF2_TIM4;	
		HAL_GPIO_Init(GPIOD,&GPIO_Initure);
	}
	else if(htim->Instance==TIM5){}
}

//定时器5底层驱动，时钟使能，引脚配置
//此函数会被HAL_TIM_IC_Init()调用
//htim:定时器5句柄
void HAL_TIM_IC_MspInit(TIM_HandleTypeDef *htim)
{
   GPIO_InitTypeDef GPIO_Initure;
	if(htim->Instance==TIM2){
    __HAL_RCC_TIM2_CLK_ENABLE();            //使能TIM5时钟
    __HAL_RCC_GPIOA_CLK_ENABLE();			//开启GPIOA时钟
	
    GPIO_Initure.Pin=GPIO_PIN_3;            //PA0
    GPIO_Initure.Mode=GPIO_MODE_AF_PP;      //复用推挽输出
    GPIO_Initure.Pull=GPIO_PULLDOWN;        //下拉
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //高速
    GPIO_Initure.Alternate=GPIO_AF1_TIM2;   //PA0复用为TIM5通道1
    HAL_GPIO_Init(GPIOA,&GPIO_Initure);

    HAL_NVIC_SetPriority(TIM2_IRQn,2,0);    //设置中断优先级，抢占优先级2，子优先级0
    HAL_NVIC_EnableIRQ(TIM2_IRQn);          //开启ITM5中断通道  
	}
}

void TIM2_IRQHandler(void)
{ 
	HAL_TIM_IRQHandler(&htim2);
}

//定时器3中断服务程序		    
void TIM3_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&htim3);	
} 

void TIM5_IRQHandler(void)
{ 
	HAL_TIM_IRQHandler(&htim5);
}

u32 sys_timer_tick=0;		//系统循环时钟,10us计数
u32 sys_second=0;//系统运行秒数(以0.1s为单位)
u32 sys_minute=0;//系统运行分钟数(以1min为单位)
u32 sys_microtick=0;	//系统运行微秒数(以1us为单位)

//回调函数，定时器中断服务函数调用
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim==(&htim3)){
	}
	else if(htim==(&htim5)){
		static u16 sys_one_microtick=0;//1us计时一次
		static u16 sys_one_millitick=0;//1ms计时一次
		static u16 sys_hun_millitick=0;//100ms计时一次
		static u16 sys_one_secondtick=0;//1s计时一次
		static u16 sys_one_minutetick=0;//1min计时一次
		sys_timer_tick++;
		sys_one_microtick++;
		sys_microtick++;
		if(sys_microtick == UINT32_MAX)
			sys_microtick = 0;
		if(sys_one_microtick>=100){									//1ms
			sys_one_microtick = 0;
			sys_one_millitick++;
			if(sys_one_millitick>=100){					//0.1s
				sys_second++;
				sys_one_millitick = 0;
				sys_hun_millitick++;
				if(sys_hun_millitick>=10){				//1s
					sys_hun_millitick=0;
					sys_one_secondtick++;
					if(sys_one_secondtick>=60){			//1min
						sys_one_secondtick=0;
						sys_minute++;
						sys_one_minutetick++;
						if(sys_one_minutetick>=10)
							sys_one_minutetick = 0;				//10分钟后重新计数
					}
				}
			}
		}
	}
}


	