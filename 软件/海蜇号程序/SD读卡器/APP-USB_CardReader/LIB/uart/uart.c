#include "uart.h"

//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
//#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)	
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{ 	
	while((USART6->SR&0X40)==0);//循环发送,直到发送完毕   
	USART6->DR = (u8) ch;      
	return ch;
}
#endif

__align(8) u8 USART1_TX_BUF[USART1_MAX_SEND_LEN]; 		  //串口发送缓存区 
u8 USART1_RX_BUF[USART1_MAX_RECV_LEN]; 					//串口接收缓存区 	
u16 USART1_RX_STA=0; 

__align(8) u8 USART2_TX_BUF[USART2_MAX_SEND_LEN]; 		  //串口发送缓存区 
u8 USART2_RX_BUF[USART2_MAX_RECV_LEN]; 					//串口接收缓存区 	
u16 USART2_RX_STA=0; 

__align(8) u8 USART3_TX_BUF[USART3_MAX_SEND_LEN]; 		  //串口发送缓存区 
u8 USART3_RX_BUF[USART3_MAX_RECV_LEN]; 					//串口接收缓存区 	
u16 USART3_RX_STA=0; 

__align(8) u8 USART6_TX_BUF[USART6_MAX_SEND_LEN]; 		  //串口发送缓存区 
u8 USART6_RX_BUF[USART6_MAX_RECV_LEN]; 					//串口接收缓存区 	
u16 USART6_RX_STA=0; 

__align(8) u8 UART7_TX_BUF[UART7_MAX_SEND_LEN]; 		  
u8 UART7_RX_BUF[UART7_MAX_RECV_LEN]; 		
u16 UART7_RX_STA=0; 

	
__align(8) u8 UART8_TX_BUF[UART8_MAX_SEND_LEN]; 		  
u8 UART8_RX_BUF[UART8_MAX_RECV_LEN]; 			
//通过判断接收连续2个字符之间的时间差不大于100ms来决定是不是一次连续的数据.
//如果2个字符接收间隔超过100ms,则认为不是1次连续数据.也就是超过100ms没有接收到
//任何数据,则表示此次接收完毕.
//接收到的数据状态
//[15]:0,没有接收到数据;1,接收到了一批数据.
//[14:0]:接收到的数据长度
u16 UART8_RX_STA=0; 

UART_HandleTypeDef huart1,huart2,huart3,huart6,huart7,huart8;
UART_HandleTypeDef UART_Handler; //UART句柄,需定义为全局变量

u8 u7_RxBuffer[RXBUFFERSIZE];//HAL库USART接收Buffer
u8 u8_RxBuffer[RXBUFFERSIZE];
u8 u1_RxBuffer[RXBUFFERSIZE];//HAL库USART接收Buffer
u8 u2_RxBuffer[RXBUFFERSIZE];
u8 u6_RxBuffer[RXBUFFERSIZE];

//以下直接从cubemx生成的项目中拷贝，需要改形参
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
	
	HAL_UART_Receive_IT(&huart7, (u8 *)u7_RxBuffer, RXBUFFERSIZE);//该函数会开启接收中断：标志位UART_IT_RXNE，并且设置接收缓冲以及接收缓冲接收最大数据量
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
	HAL_UART_Receive_IT(&huart8, (u8 *)u8_RxBuffer, RXBUFFERSIZE);//该函数会开启接收中断：标志位UART_IT_RXNE，并且设置接收缓冲以及接收缓冲接收最大数据量

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

	
//	HAL_UART_Receive_IT(&huart1, (u8 *)u1_RxBuffer, RXBUFFERSIZE);//该函数会开启接收中断：标志位UART_IT_RXNE，并且设置接收缓冲以及接收缓冲接收最大数据量
  __HAL_UART_ENABLE_IT(&huart1,UART_IT_RXNE); //开启接收完成中断
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
	//UART 初始化设置
	if(UART == USART1)MX_USART1_UART_Init(baud);
	else if(UART == USART2)MX_USART2_UART_Init(baud);
	else if(UART == USART3)MX_USART3_UART_Init(baud);
	else if(UART == USART6)MX_USART6_UART_Init(baud);
	else if(UART == UART7)MX_UART7_Init(baud);
	else if(UART == UART8)MX_UART8_Init(baud);
}

//UART底层初始化，时钟使能，引脚配置，中断配置
//此函数会被HAL_UART_Init()调用
//huart:串口句柄

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    //GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStruct;
	
	if(huart->Instance==USART1)//如果是串口1，进行串口1 MSP初始化
	{
		__HAL_RCC_GPIOB_CLK_ENABLE();			//使能GPIOA时钟
		__HAL_RCC_USART1_CLK_ENABLE();			//使能USART1时钟
	
		/*Configure GPIO pins : PB6 PB7 */
		GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
					
#if EN_USART1_RX
		HAL_NVIC_EnableIRQ(USART1_IRQn);				//使能USART1中断通道
		HAL_NVIC_SetPriority(USART1_IRQn,0,2);			//抢占优先级3，子优先级3
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
		HAL_NVIC_EnableIRQ(USART2_IRQn);				//使能USART1中断通道
		HAL_NVIC_SetPriority(USART2_IRQn,2,3);			//抢占优先级3，子优先级3
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
		HAL_NVIC_EnableIRQ(USART2_IRQn);				//使能USART1中断通道
		HAL_NVIC_SetPriority(USART2_IRQn,3,0);			//抢占优先级3，子优先级3
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
		HAL_NVIC_EnableIRQ(USART6_IRQn);				//使能USART1中断通道
		HAL_NVIC_SetPriority(USART6_IRQn,3,1);			//抢占优先级3，子优先级3
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
		HAL_NVIC_EnableIRQ(UART7_IRQn);				//使能USART1中断通道
		HAL_NVIC_SetPriority(UART7_IRQn,0,0);			//抢占优先级3，子优先级3
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
		HAL_NVIC_EnableIRQ(UART8_IRQn);				//使能USART1中断通道
		HAL_NVIC_SetPriority(UART8_IRQn,0,1);			//抢占优先级3，子优先级3
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
	i=strlen((const char*)USART1_TX_BUF);		//此次发送数据的长度
	for(j=0;j<i;j++)							//循环发送数据
	{
		while((USART1->SR&0X40)==0);			//循环发送,直到发送完毕   
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
	i=strlen((const char*)USART2_TX_BUF);		//此次发送数据的长度
	for(j=0;j<i;j++)							//循环发送数据
	{
		while((USART2->SR&0X40)==0);			//循环发送,直到发送完毕   
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
	i=strlen((const char*)USART3_TX_BUF);		//此次发送数据的长度
	for(j=0;j<i;j++)							//循环发送数据
	{
		while((USART3->SR&0X40)==0);			//循环发送,直到发送完毕   
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
	i=strlen((const char*)USART6_TX_BUF);		//此次发送数据的长度
	for(j=0;j<i;j++)							//循环发送数据
	{
		while((USART6->SR&0X40)==0);			//循环发送,直到发送完毕   
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
	i=strlen((const char*)UART7_TX_BUF);		//此次发送数据的长度
	for(j=0;j<i;j++)							//循环发送数据
	{
		while((UART7->SR&0X40)==0);			//循环发送,直到发送完毕   
		UART7->DR=UART7_TX_BUF[j];  
	} 
}

//串口8,printf 函数
//确保一次发送数据不超过UART8_MAX_SEND_LEN字节
void u8_printf(char* fmt,...)  
{  
	u16 i,j; 
	va_list ap; 
	va_start(ap,fmt);
	vsprintf((char*)UART8_TX_BUF,fmt,ap);
	va_end(ap);
	i=strlen((const char*)UART8_TX_BUF);		//此次发送数据的长度
	for(j=0;j<i;j++)							//循环发送数据
	{
		while((UART8->SR&0X40)==0);			//循环发送,直到发送完毕   
		UART8->DR=UART8_TX_BUF[j];  
	} 
}

//
void uart_write(USART_TypeDef* UART,char* buf,int size){	
	for(int i=0;i<size;i++){
		while((UART->SR&0X40)==0){}; 	//判断发送完毕
		UART->DR = buf[i];   //填充发送寄存器
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
//	if(huart->Instance==USART1)//如果是串口1
//	{
//		for(u8 i=0;i<RXBUFFERSIZE;i++){
//			USART1_RX_BUF[USART1_RX_STA++]=u1_RxBuffer[i] ;
//			if(USART1_RX_STA == USART1_MAX_RECV_LEN){	//防止接收缓存没有读取溢出，循环写入
//				USART1_RX_STA = 0;
//			}
//		}
//	}
	if(huart->Instance==USART2)//如果是串口2
	{
		for(u8 i=0;i<RXBUFFERSIZE;i++){
			USART2_RX_BUF[USART2_RX_STA++]=u2_RxBuffer[i] ;
			if(USART2_RX_STA == USART2_MAX_RECV_LEN){	//防止接收缓存没有读取溢出，循环写入
				USART2_RX_STA = 0;
			}
		}
	}
	if(huart->Instance==USART6)//如果是串口2
	{
		for(u8 i=0;i<RXBUFFERSIZE;i++){
			USART6_RX_BUF[USART6_RX_STA++]=u6_RxBuffer[i] ;
			if(USART6_RX_STA == USART6_MAX_RECV_LEN){	//防止接收缓存没有读取溢出，循环写入
				USART6_RX_STA = 0;
			}
		}
	}
	if(huart->Instance==UART7)//如果是串口1
	{
		for(u8 i=0;i<RXBUFFERSIZE;i++){
			UART7_RX_BUF[UART7_RX_STA++]=u7_RxBuffer[i] ;
			if(UART7_RX_STA == UART7_MAX_RECV_LEN){	//防止接收缓存没有读取溢出，循环写入
				UART7_RX_STA = 0;
			}
		}
	}
	if(huart->Instance==UART8)//如果是串口1
	{
		for(u8 i=0;i<RXBUFFERSIZE;i++){
			UART8_RX_BUF[UART8_RX_STA++]=u8_RxBuffer[i] ;
			if(UART8_RX_STA == UART8_MAX_RECV_LEN){	//防止接收缓存没有读取溢出，循环写入
				UART8_RX_STA = 0;
			}
		}
	}
}

//串口1中断服务程序
void USART1_IRQHandler(void)                	
{ 	
	u8 Res;
	if((__HAL_UART_GET_FLAG(&huart1,UART_FLAG_RXNE)!=RESET))  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
	{
		HAL_UART_Receive(&huart1,&Res,1,1000); 
		if(USART1_RX_STA>(USART1_MAX_RECV_LEN-1))USART1_RX_STA=0;//接收数据错误,重新开始接收	
		USART1_RX_BUF[USART1_RX_STA++]=Res ;			 
	}
	HAL_UART_IRQHandler(&huart1);	
	
//	HAL_UART_IRQHandler(&huart1);	//调用HAL库中断处理公用函数
//	HAL_UART_Receive_IT(&huart1, (u8 *)u1_RxBuffer, RXBUFFERSIZE);//该函数会开启接收中断：标志位UART_IT_RXNE，并且设置接收缓冲以及接收缓冲接收最大数据量
}

void USART2_IRQHandler(void)                	
{ 	
	HAL_UART_IRQHandler(&huart2);	//调用HAL库中断处理公用函数
	HAL_UART_Receive_IT(&huart2, (u8 *)u2_RxBuffer, RXBUFFERSIZE);
}

void USART6_IRQHandler(void)                	
{ 	
//	HAL_UART_IRQHandler(&huart6);	//调用HAL库中断处理公用函数
//	HAL_UART_Receive_IT(&huart6, (u8 *)u6_RxBuffer, RXBUFFERSIZE);
	u8 Res;
	if((__HAL_UART_GET_FLAG(&huart6,UART_FLAG_RXNE)!=RESET))  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
	{
		HAL_UART_Receive(&huart6,&Res,1,1000); 
		if((USART6_RX_STA&0x8000)==0)//接收未完成
		{
			if(USART6_RX_STA&0x4000)//接收到了0x0a
			{
				if(Res!=0x0a)USART6_RX_STA=0;//接收错误,重新开始
				else {
					USART6_RX_STA|=0x8000;	//接收完成了 
					if(strcmp((char*)USART6_RX_BUF,"reset")==0){
						printf("\r\n**************\r\n");
						printf("NVIC_SystemReset\r\n");
						printf("\r\n**************\r\n");
						NVIC_SystemReset();
					}

				}
			}
			else //还没收到0X0D
			{	
				if(Res==0x0d)USART6_RX_STA|=0x4000;
				else
				{
					USART6_RX_BUF[USART6_RX_STA&0X3FFF]=Res ;
					USART6_RX_STA++;
					if(USART6_RX_STA>(USART6_MAX_RECV_LEN-1))USART6_RX_STA=0;//接收数据错误,重新开始接收	  
				}		 
			}
		}		 
	}
}

void UART7_IRQHandler(void)                	
{ 
	HAL_UART_IRQHandler(&huart7);	//调用HAL库中断处理公用函数
	HAL_UART_Receive_IT(&huart7, (u8 *)u7_RxBuffer, RXBUFFERSIZE);//该函数会开启接收中断：标志位UART_IT_RXNE，并且设置接收缓冲以及接收缓冲接收最大数据量

}

void UART8_IRQHandler(void){
	HAL_UART_IRQHandler(&huart8);	//调用HAL库中断处理公用函数
	HAL_UART_Receive_IT(&huart8, (u8 *)u8_RxBuffer, RXBUFFERSIZE);			 		
}
