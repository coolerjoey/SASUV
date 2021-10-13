#include "uart.h"

//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	  
//#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)	
#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{ 	
	while((USART6->SR&0X40)==0);//ѭ������,ֱ���������   
	USART6->DR = (u8) ch;      
	return ch;
}
#endif

__align(8) u8 USART1_TX_BUF[USART1_MAX_SEND_LEN]; 		  //���ڷ��ͻ����� 
u8 USART1_RX_BUF[USART1_MAX_RECV_LEN]; 					//���ڽ��ջ����� 	
u16 USART1_RX_STA=0; 

__align(8) u8 USART2_TX_BUF[USART2_MAX_SEND_LEN]; 		  //���ڷ��ͻ����� 
u8 USART2_RX_BUF[USART2_MAX_RECV_LEN]; 					//���ڽ��ջ����� 	
u16 USART2_RX_STA=0; 

__align(8) u8 USART3_TX_BUF[USART3_MAX_SEND_LEN]; 		  //���ڷ��ͻ����� 
u8 USART3_RX_BUF[USART3_MAX_RECV_LEN]; 					//���ڽ��ջ����� 	
u16 USART3_RX_STA=0; 

__align(8) u8 USART6_TX_BUF[USART6_MAX_SEND_LEN]; 		  //���ڷ��ͻ����� 
u8 USART6_RX_BUF[USART6_MAX_RECV_LEN]; 					//���ڽ��ջ����� 	
u16 USART6_RX_STA=0; 

__align(8) u8 UART7_TX_BUF[UART7_MAX_SEND_LEN]; 		  
u8 UART7_RX_BUF[UART7_MAX_RECV_LEN]; 		
u16 UART7_RX_STA=0; 

	
__align(8) u8 UART8_TX_BUF[UART8_MAX_SEND_LEN]; 		  
u8 UART8_RX_BUF[UART8_MAX_RECV_LEN]; 			
//ͨ���жϽ�������2���ַ�֮���ʱ������100ms�������ǲ���һ������������.
//���2���ַ����ռ������100ms,����Ϊ����1����������.Ҳ���ǳ���100msû�н��յ�
//�κ�����,���ʾ�˴ν������.
//���յ�������״̬
//[15]:0,û�н��յ�����;1,���յ���һ������.
//[14:0]:���յ������ݳ���
u16 UART8_RX_STA=0; 

UART_HandleTypeDef huart1,huart2,huart3,huart6,huart7,huart8;
UART_HandleTypeDef UART_Handler; //UART���,�趨��Ϊȫ�ֱ���

u8 u7_RxBuffer[RXBUFFERSIZE];//HAL��USART����Buffer
u8 u8_RxBuffer[RXBUFFERSIZE];
u8 u1_RxBuffer[RXBUFFERSIZE];//HAL��USART����Buffer
u8 u2_RxBuffer[RXBUFFERSIZE];
u8 u6_RxBuffer[RXBUFFERSIZE];

//����ֱ�Ӵ�cubemx���ɵ���Ŀ�п�������Ҫ���β�
/* UART7 init function */
void MX_UART7_Init(u32 baud)
{

  huart7.Instance = UART7;
  huart7.Init.BaudRate = baud;
  huart7.Init.WordLength = UART_WORDLENGTH_8B;
  huart7.Init.StopBits = UART_STOPBITS_1;
  huart7.Init.Parity = UART_PARITY_NONE;
  huart7.Init.Mode = UART_MODE_TX_RX;
  huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart7.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart7);
	
	HAL_UART_Receive_IT(&huart7, (u8 *)u7_RxBuffer, RXBUFFERSIZE);//�ú����Ὺ�������жϣ���־λUART_IT_RXNE���������ý��ջ����Լ����ջ���������������
}

/* UART8 init function */
void MX_UART8_Init(u32 baud)
{

  huart8.Instance = UART8;
  huart8.Init.BaudRate = baud;
  huart8.Init.WordLength = UART_WORDLENGTH_8B;
  huart8.Init.StopBits = UART_STOPBITS_1;
  huart8.Init.Parity = UART_PARITY_NONE;
  huart8.Init.Mode = UART_MODE_TX_RX;
  huart8.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart8.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart8);
	HAL_UART_Receive_IT(&huart8, (u8 *)u8_RxBuffer, RXBUFFERSIZE);//�ú����Ὺ�������жϣ���־λUART_IT_RXNE���������ý��ջ����Լ����ջ���������������

}

/* USART1 init function */
void MX_USART1_UART_Init(u32 baud)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = baud;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart1);

	
//	HAL_UART_Receive_IT(&huart1, (u8 *)u1_RxBuffer, RXBUFFERSIZE);//�ú����Ὺ�������жϣ���־λUART_IT_RXNE���������ý��ջ����Լ����ջ���������������
  __HAL_UART_ENABLE_IT(&huart1,UART_IT_RXNE); //������������ж�
}

/* USART2 init function */
void MX_USART2_UART_Init(u32 baud)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = baud;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart2);
	
	HAL_UART_Receive_IT(&huart2, (u8 *)u2_RxBuffer, RXBUFFERSIZE);
}

/* USART3 init function */
void MX_USART3_UART_Init(u32 baud)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = baud;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart3);

}

/* USART6 init function */
void MX_USART6_UART_Init(u32 baud)
{

  huart6.Instance = USART6;
  huart6.Init.BaudRate = baud;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart6);

}

void UART_init(USART_TypeDef* UART,u32 baud){
	//UART ��ʼ������
	if(UART == USART1)MX_USART1_UART_Init(baud);
	else if(UART == USART2)MX_USART2_UART_Init(baud);
	else if(UART == USART3)MX_USART3_UART_Init(baud);
	else if(UART == USART6)MX_USART6_UART_Init(baud);
	else if(UART == UART7)MX_UART7_Init(baud);
	else if(UART == UART8)MX_UART8_Init(baud);
}

//UART�ײ��ʼ����ʱ��ʹ�ܣ��������ã��ж�����
//�˺����ᱻHAL_UART_Init()����
//huart:���ھ��

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    //GPIO�˿�����
	GPIO_InitTypeDef GPIO_InitStruct;
	
	if(huart->Instance==USART1)//����Ǵ���1�����д���1 MSP��ʼ��
	{
		__HAL_RCC_GPIOB_CLK_ENABLE();			//ʹ��GPIOAʱ��
		__HAL_RCC_USART1_CLK_ENABLE();			//ʹ��USART1ʱ��
	
		/*Configure GPIO pins : PB6 PB7 */
		GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
					
#if EN_USART1_RX
		HAL_NVIC_EnableIRQ(USART1_IRQn);				//ʹ��USART1�ж�ͨ��
		HAL_NVIC_SetPriority(USART1_IRQn,0,2);			//��ռ���ȼ�3�������ȼ�3
#endif	
	}
	else if(huart->Instance==USART2){
				__HAL_RCC_GPIOD_CLK_ENABLE();			
		__HAL_RCC_USART2_CLK_ENABLE();	  
		/*Configure GPIO pins : PD3 PD4 PD5 PD6 */
		GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
		HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
#if EN_USART2_RX
		HAL_NVIC_EnableIRQ(USART2_IRQn);				//ʹ��USART1�ж�ͨ��
		HAL_NVIC_SetPriority(USART2_IRQn,2,3);			//��ռ���ȼ�3�������ȼ�3
#endif
	}
	else if(huart->Instance==USART3){
				__HAL_RCC_GPIOD_CLK_ENABLE();			
		__HAL_RCC_USART3_CLK_ENABLE();	
		  /*Configure GPIO pins : PD8 PD9 PD11 */
		GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_11;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
		HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
		#if EN_USART2_RX
		HAL_NVIC_EnableIRQ(USART2_IRQn);				//ʹ��USART1�ж�ͨ��
		HAL_NVIC_SetPriority(USART2_IRQn,3,0);			//��ռ���ȼ�3�������ȼ�3
#endif
	
	}
	else if(huart->Instance==UART4){
	}
	else if(huart->Instance==UART5){
	}
	else if(huart->Instance==USART6){
		__HAL_RCC_GPIOC_CLK_ENABLE();			
		__HAL_RCC_USART6_CLK_ENABLE();			
				/*Configure GPIO pins : PC6 PC7 */
		GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
		HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
#if EN_USART6_RX
		__HAL_UART_ENABLE_IT(&huart6,UART_IT_RXNE); 
		HAL_NVIC_EnableIRQ(USART6_IRQn);				//ʹ��USART1�ж�ͨ��
		HAL_NVIC_SetPriority(USART6_IRQn,3,1);			//��ռ���ȼ�3�������ȼ�3
#endif
	}
	else if(huart->Instance==UART7){
				__HAL_RCC_GPIOE_CLK_ENABLE();			
		__HAL_RCC_UART7_CLK_ENABLE();	
		
		  /*Configure GPIO pins : PE7 PE8 */
		GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF8_UART7;
		HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
		#if EN_UART7_RX
		HAL_NVIC_EnableIRQ(UART7_IRQn);				//ʹ��USART1�ж�ͨ��
		HAL_NVIC_SetPriority(UART7_IRQn,0,0);			//��ռ���ȼ�3�������ȼ�3
#endif
	}
	else if(huart->Instance==UART8){
		__HAL_RCC_GPIOE_CLK_ENABLE();			
		__HAL_RCC_UART8_CLK_ENABLE();	
		
		 /*Configure GPIO pins : PE0 PE1 */
		GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF8_UART8;
		HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
#if EN_UART8_RX
		HAL_NVIC_EnableIRQ(UART8_IRQn);				//ʹ��USART1�ж�ͨ��
		HAL_NVIC_SetPriority(UART8_IRQn,0,1);			//��ռ���ȼ�3�������ȼ�3
#endif
	}
}

void u1_printf(char* fmt,...)  
{  
	u16 i,j; 
	va_list ap; 
	va_start(ap,fmt);
	vsprintf((char*)USART1_TX_BUF,fmt,ap);
	va_end(ap);
	i=strlen((const char*)USART1_TX_BUF);		//�˴η������ݵĳ���
	for(j=0;j<i;j++)							//ѭ����������
	{
		while((USART1->SR&0X40)==0);			//ѭ������,ֱ���������   
		USART1->DR=USART1_TX_BUF[j];  
	} 
}

void u2_printf(char* fmt,...)  
{  
	u16 i,j; 
	va_list ap; 
	va_start(ap,fmt);
	vsprintf((char*)USART2_TX_BUF,fmt,ap);
	va_end(ap);
	i=strlen((const char*)USART2_TX_BUF);		//�˴η������ݵĳ���
	for(j=0;j<i;j++)							//ѭ����������
	{
		while((USART2->SR&0X40)==0);			//ѭ������,ֱ���������   
		USART2->DR=USART2_TX_BUF[j];  
	} 
}

void u3_printf(char* fmt,...)  
{  
	u16 i,j; 
	va_list ap; 
	va_start(ap,fmt);
	vsprintf((char*)USART3_TX_BUF,fmt,ap);
	va_end(ap);
	i=strlen((const char*)USART3_TX_BUF);		//�˴η������ݵĳ���
	for(j=0;j<i;j++)							//ѭ����������
	{
		while((USART3->SR&0X40)==0);			//ѭ������,ֱ���������   
		USART3->DR=USART3_TX_BUF[j];  
	} 
}

void u6_printf(char* fmt,...)  
{  
	u16 i,j; 
	va_list ap; 
	va_start(ap,fmt);
	vsprintf((char*)USART6_TX_BUF,fmt,ap);
	va_end(ap);
	i=strlen((const char*)USART6_TX_BUF);		//�˴η������ݵĳ���
	for(j=0;j<i;j++)							//ѭ����������
	{
		while((USART6->SR&0X40)==0);			//ѭ������,ֱ���������   
		USART6->DR=USART6_TX_BUF[j];  
	} 
}

void u7_printf(char* fmt,...)  
{  
	u16 i,j; 
	va_list ap; 
	va_start(ap,fmt);
	vsprintf((char*)UART7_TX_BUF,fmt,ap);
	va_end(ap);
	i=strlen((const char*)UART7_TX_BUF);		//�˴η������ݵĳ���
	for(j=0;j<i;j++)							//ѭ����������
	{
		while((UART7->SR&0X40)==0);			//ѭ������,ֱ���������   
		UART7->DR=UART7_TX_BUF[j];  
	} 
}

//����8,printf ����
//ȷ��һ�η������ݲ�����UART8_MAX_SEND_LEN�ֽ�
void u8_printf(char* fmt,...)  
{  
	u16 i,j; 
	va_list ap; 
	va_start(ap,fmt);
	vsprintf((char*)UART8_TX_BUF,fmt,ap);
	va_end(ap);
	i=strlen((const char*)UART8_TX_BUF);		//�˴η������ݵĳ���
	for(j=0;j<i;j++)							//ѭ����������
	{
		while((UART8->SR&0X40)==0);			//ѭ������,ֱ���������   
		UART8->DR=UART8_TX_BUF[j];  
	} 
}

//
void uart_write(USART_TypeDef* UART,char* buf,int size){	
	for(int i=0;i<size;i++){
		while((UART->SR&0X40)==0){}; 	//�жϷ������
		UART->DR = buf[i];   //��䷢�ͼĴ���
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
//	if(huart->Instance==USART1)//����Ǵ���1
//	{
//		for(u8 i=0;i<RXBUFFERSIZE;i++){
//			USART1_RX_BUF[USART1_RX_STA++]=u1_RxBuffer[i] ;
//			if(USART1_RX_STA == USART1_MAX_RECV_LEN){	//��ֹ���ջ���û�ж�ȡ�����ѭ��д��
//				USART1_RX_STA = 0;
//			}
//		}
//	}
	if(huart->Instance==USART2)//����Ǵ���2
	{
		for(u8 i=0;i<RXBUFFERSIZE;i++){
			USART2_RX_BUF[USART2_RX_STA++]=u2_RxBuffer[i] ;
			if(USART2_RX_STA == USART2_MAX_RECV_LEN){	//��ֹ���ջ���û�ж�ȡ�����ѭ��д��
				USART2_RX_STA = 0;
			}
		}
	}
	if(huart->Instance==USART6)//����Ǵ���2
	{
		for(u8 i=0;i<RXBUFFERSIZE;i++){
			USART6_RX_BUF[USART6_RX_STA++]=u6_RxBuffer[i] ;
			if(USART6_RX_STA == USART6_MAX_RECV_LEN){	//��ֹ���ջ���û�ж�ȡ�����ѭ��д��
				USART6_RX_STA = 0;
			}
		}
	}
	if(huart->Instance==UART7)//����Ǵ���1
	{
		for(u8 i=0;i<RXBUFFERSIZE;i++){
			UART7_RX_BUF[UART7_RX_STA++]=u7_RxBuffer[i] ;
			if(UART7_RX_STA == UART7_MAX_RECV_LEN){	//��ֹ���ջ���û�ж�ȡ�����ѭ��д��
				UART7_RX_STA = 0;
			}
		}
	}
	if(huart->Instance==UART8)//����Ǵ���1
	{
		for(u8 i=0;i<RXBUFFERSIZE;i++){
			UART8_RX_BUF[UART8_RX_STA++]=u8_RxBuffer[i] ;
			if(UART8_RX_STA == UART8_MAX_RECV_LEN){	//��ֹ���ջ���û�ж�ȡ�����ѭ��д��
				UART8_RX_STA = 0;
			}
		}
	}
}

//����1�жϷ������
void USART1_IRQHandler(void)                	
{ 	
	u8 Res;
	if((__HAL_UART_GET_FLAG(&huart1,UART_FLAG_RXNE)!=RESET))  //�����ж�(���յ������ݱ�����0x0d 0x0a��β)
	{
		HAL_UART_Receive(&huart1,&Res,1,1000); 
		if(USART1_RX_STA>(USART1_MAX_RECV_LEN-1))USART1_RX_STA=0;//�������ݴ���,���¿�ʼ����	
		USART1_RX_BUF[USART1_RX_STA++]=Res ;			 
	}
	HAL_UART_IRQHandler(&huart1);	
	
//	HAL_UART_IRQHandler(&huart1);	//����HAL���жϴ����ú���
//	HAL_UART_Receive_IT(&huart1, (u8 *)u1_RxBuffer, RXBUFFERSIZE);//�ú����Ὺ�������жϣ���־λUART_IT_RXNE���������ý��ջ����Լ����ջ���������������
}

void USART2_IRQHandler(void)                	
{ 	
	HAL_UART_IRQHandler(&huart2);	//����HAL���жϴ����ú���
	HAL_UART_Receive_IT(&huart2, (u8 *)u2_RxBuffer, RXBUFFERSIZE);
}

void USART6_IRQHandler(void)                	
{ 	
//	HAL_UART_IRQHandler(&huart6);	//����HAL���жϴ����ú���
//	HAL_UART_Receive_IT(&huart6, (u8 *)u6_RxBuffer, RXBUFFERSIZE);
	u8 Res;
	if((__HAL_UART_GET_FLAG(&huart6,UART_FLAG_RXNE)!=RESET))  //�����ж�(���յ������ݱ�����0x0d 0x0a��β)
	{
		HAL_UART_Receive(&huart6,&Res,1,1000); 
		if((USART6_RX_STA&0x8000)==0)//����δ���
		{
			if(USART6_RX_STA&0x4000)//���յ���0x0a
			{
				if(Res!=0x0a)USART6_RX_STA=0;//���մ���,���¿�ʼ
				else {
					USART6_RX_STA|=0x8000;	//��������� 
					if(strcmp((char*)USART6_RX_BUF,"reset")==0){
						printf("\r\n**************\r\n");
						printf("NVIC_SystemReset\r\n");
						printf("\r\n**************\r\n");
						NVIC_SystemReset();
					}

				}
			}
			else //��û�յ�0X0D
			{	
				if(Res==0x0d)USART6_RX_STA|=0x4000;
				else
				{
					USART6_RX_BUF[USART6_RX_STA&0X3FFF]=Res ;
					USART6_RX_STA++;
					if(USART6_RX_STA>(USART6_MAX_RECV_LEN-1))USART6_RX_STA=0;//�������ݴ���,���¿�ʼ����	  
				}		 
			}
		}		 
	}
}

void UART7_IRQHandler(void)                	
{ 
	HAL_UART_IRQHandler(&huart7);	//����HAL���жϴ����ú���
	HAL_UART_Receive_IT(&huart7, (u8 *)u7_RxBuffer, RXBUFFERSIZE);//�ú����Ὺ�������жϣ���־λUART_IT_RXNE���������ý��ջ����Լ����ջ���������������

}

void UART8_IRQHandler(void){
	HAL_UART_IRQHandler(&huart8);	//����HAL���жϴ����ú���
	HAL_UART_Receive_IT(&huart8, (u8 *)u8_RxBuffer, RXBUFFERSIZE);			 		
}
