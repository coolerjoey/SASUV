#include "adc.h"
#include "delay.h"

ADC_HandleTypeDef hadc1;

void adc_init(){
	ADC1_Init();
	printf("[OK] ADC Init complete \r\n");
}
	
/* ADC1 init function */
void ADC1_Init(void){

  
  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = DISABLE;
  HAL_ADC_Init(&hadc1);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
//  sConfig.Channel = ADC_CHANNEL_10;
//  sConfig.Rank = 1;
//  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
//  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

//    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
//    */
//  sConfig.Channel = ADC_CHANNEL_11;
//  sConfig.Rank = 2;
//  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

//    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
//    */
//  sConfig.Channel = ADC_CHANNEL_12;
//  sConfig.Rank = 3;
//  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

//    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
//    */
//  sConfig.Channel = ADC_CHANNEL_13;
//  sConfig.Rank = 4;
//  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

}


void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(hadc->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspInit 0 */

  /* USER CODE END ADC1_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_ADC1_CLK_ENABLE();
  
    /**ADC1 GPIO Configuration    
    PC0     ------> ADC1_IN10
    PC1     ------> ADC1_IN11
    PC2     ------> ADC1_IN12
    PC3     ------> ADC1_IN13 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN ADC1_MspInit 1 */

  /* USER CODE END ADC1_MspInit 1 */
  }

}

//????ADC??
//ch: ?????? 0~16??????????????ADC_CHANNEL_0~ADC_CHANNEL_16
//??????:????????
u16 Get_Adc(u32 ch)   
{
	ADC_ChannelConfTypeDef ADC1_ChanConf;
	
	ADC1_ChanConf.Channel=ch;                                   //????
	ADC1_ChanConf.Rank=1;                                       //??1????????????1
	ADC1_ChanConf.SamplingTime=ADC_SAMPLETIME_480CYCLES;        //????????
	ADC1_ChanConf.Offset=0;                 
	HAL_ADC_ConfigChannel(&hadc1,&ADC1_ChanConf);        //????????

	HAL_ADC_Start(&hadc1);                               //????ADC

	HAL_ADC_PollForConversion(&hadc1,10);                //????????

	return (u16)HAL_ADC_GetValue(&hadc1);	        //????????????ADC1????????????????
}
//????????????????????????times??,???????? 
//times:????????
//??????:????ch??times????????????????
u16 Get_Adc_Average(u32 ch,u8 times)
{
	u32 temp_val=0;
	u8 t;
	for(t=0;t<times;t++)
	{
		temp_val+=Get_Adc(ch);
		delay_ms(5);
	}
	return temp_val/times;
} 

