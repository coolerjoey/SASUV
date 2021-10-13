#include "led.h"
#include "delay.h"
#include "uart.h"

//https://blog.csdn.net/ben392797097/article/details/78075699?utm_source=blogxgwz0
//https://blog.csdn.net/ben392797097/article/details/78075699?utm_source=blogxgwz0

//��ʼ��PB1Ϊ���.��ʹ��ʱ��	    
//LED IO��ʼ��
void led_init(){
	LED_Doide_init();
//	LED_WS2812_init();
	printf("[OK] LED Init complete \r\n");
}

void LED_Doide_init()
{
	GPIO_InitTypeDef GPIO_InitStruct;
	LED_DIODE_GPIO_CLK_ENABLE();           //����GPIOBʱ��
	
	  /*Configure GPIO pins : PA10 PA8 */
  GPIO_InitStruct.Pin = LED_DIODE_BE_PIN|LED_DIODE_ACT_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(LED_DIODE_GPIO, &GPIO_InitStruct);
	
	HAL_GPIO_WritePin(LED_DIODE_GPIO,LED_DIODE_BE_PIN,GPIO_PIN_SET);	//PB0��1 
	HAL_GPIO_WritePin(LED_DIODE_GPIO,LED_DIODE_ACT_PIN,GPIO_PIN_SET);	//PB1��1  
}

//�ο�http://bbs.eeworld.com.cn/thread-643893-1-1.html
void LED_WS2812_init(){
	
	//ʱ�ӳ�ʼ��������GPIO��ʼ����
	WS2812_TIM_Init(ARR-1,PSC-1);	//pwmƵ����Ҫ����Ϊ800khz
//	TM7812B_Init();
#if (DMA_WS2812_ENBALE==1)
//	WS2813_TIM_DMA_Init();	//DMA��ʼ��	
#else
//	WS2813_TIM->CR1 &= !(1<<1);    //����UDIS��������¼���UEV��,Ĭ�Ͽ�����
#endif
	
}

u16 LED_BYTE_Buffer[28];
void LED_RGB_Set(u8 red,u8 green,u8 blue){
	uint8_t i=0;
	uint16_t memaddr;
	uint16_t buffersize;
	buffersize = 28;//+43;	// number of bytes needed is #LEDs * 24 bytes + 42 trailing bytes
	memaddr = 0;				// reset buffer memory index
	LED_BYTE_Buffer[memaddr++] = 0;	 //�ȷ���3��0����ȶ�
	LED_BYTE_Buffer[memaddr++] =0;
	LED_BYTE_Buffer[memaddr++] =0;
	//����˳����GRB����λ���ȣ�
	for(i=0; i<8; i++) // GREEN data
	{
		LED_BYTE_Buffer[memaddr++] = ((green<<i) & 0x0080) ? TIMING_ONE:TIMING_ZERO;
	}
	for(i=0; i<8; i++) // RED
	{
			LED_BYTE_Buffer[memaddr++] = ((red<<i) & 0x0080) ? TIMING_ONE:TIMING_ZERO;
	}
	for(i=0; i<8; i++) // BLUE
	{
			LED_BYTE_Buffer[memaddr++] = ((blue<<i) & 0x0080) ? TIMING_ONE:TIMING_ZERO;
	}
	LED_BYTE_Buffer[memaddr++] = 0;	//�����һ��0����֤������pwm�����

	#if (DMA_WS2812_ENBALE==1)
		HAL_TIM_PWM_Start_DMA(&WS2812_TIM_Handle, WS2812_TIM_CHANNEL, (uint32_t *)&LED_BYTE_Buffer, buffersize);
	#else
		i= 0;	
		while(i<buffersize){		//ÿһ��pwm�����ĸ�������24����			
			while(!(WS2813_TIM->SR&0x0001));	
			WS2813_TIM->CCR1=LED_BYTE_Buffer[i++];
			WS2813_TIM->SR &= !(1<<0);
		}
	#endif
}


//��ɫ��˸
void RGBLED_RED_twinkle(){
	static bool i=true;
	if(i==true){
		LED_RGB_Set(0xff,0x00,0x00);
	}
	else{
		LED_RGB_Set(0x00,0x00,0x00);	
	} 
	i = !i;
}

//��ɫ��˸
void RGBLED_GREEN_twinkle(){
	static bool i=true;
	if(i==true){
		LED_RGB_Set(0x00,0xff,0x00);
	}
	else{
		LED_RGB_Set(0x00,0x00,0x00);	
	} 
	i = !i;
}

//��ɫ��˸
void RGBLED_BLUE_twinkle(){
	static bool i=true;
	if(i==true){
		LED_RGB_Set(0x00,0x00,0xff);
	}
	else{
		LED_RGB_Set(0x00,0x00,0x00);	
	} 
	i = !i;	
}

