#include "uart.h"
#include "malloc.h"
#include "delay.h"
#include "parameter.h"
#include "udp_demo.h"

//串口接收中断使能
#define EN_USART1_RX 			0		//使能（1）/禁止（0）串口1接收
#define EN_USART2_RX 			1
#define EN_USART3_RX 			1
#define EN_USART4_RX 			0
#define EN_USART6_RX      		1
#define EN_UART7_RX 			0
#define EN_UART8_RX 			0
//串口DMA接收中断使能
#define EN_USART1_RX_DMA		!EN_USART1_RX
#define EN_USART2_RX_DMA		!EN_USART2_RX
#define EN_USART3_RX_DMA		!EN_USART3_RX
#define EN_UART7_RX_DMA			!EN_UART7_RX


//#define UART1_RX_PrePriority	1	//mpu
//#define UART1_RX_SubPriority	1
//#define UART2_RX_PrePriority	1	//gps
//#define UART2_RX_SubPriority	2
//#define UART3_RX_PrePriority	1	//tcm
//#define UART3_RX_SubPriority	0
//#define UART6_RX_PrePriority	2	//debug
//#define UART6_RX_SubPriority	0
//#define UART7_RX_PrePriority	1	//telem
//#define UART7_RX_SubPriority	3
//#define UART8_RX_PrePriority	2	//wifi
//#define UART8_RX_SubPriority	1

#define UART1_RX_PrePriority	3	//mpu
#define UART1_RX_SubPriority	1
#define UART2_RX_PrePriority	3	//gps
#define UART2_RX_SubPriority	2
#define UART3_RX_PrePriority	3	//tcm
#define UART3_RX_SubPriority	1
#define UART6_RX_PrePriority	3	//debug
#define UART6_RX_SubPriority	3
#define UART7_RX_PrePriority	3	//telem
#define UART7_RX_SubPriority	1
#define UART8_RX_PrePriority	3	//wifi
#define UART8_RX_SubPriority	3


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
//__use_no_semihosting was requested, but _ttywrch was 
void _ttywrch(int ch)
{
    ch = ch;
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
u16 UART8_RX_STA=0; 

UART_HandleTypeDef huart1,huart2,huart3,huart6,huart7,huart8;//UART句柄,需定义为全局变量

DMA_HandleTypeDef hdma_usart1_rx, hdma_usart2_rx, hdma_usart3_rx, hdma_uart7_rx, hdma_uart7_tx;

u8 u7_RxBuffer[RXBUFFERSIZE];//HAL库USART接收Buffer
u8 u8_RxBuffer[RXBUFFERSIZE];
u8 u1_RxBuffer[RXBUFFERSIZE];//HAL库USART接收Buffer
u8 u2_RxBuffer[RXBUFFERSIZE];
u8 u3_RxBuffer[RXBUFFERSIZE];
u8 u6_RxBuffer[RXBUFFERSIZE];

int DMA_FIFO_SIZE=0;
int delta_size=512;
int dma_fifo_len=0;

u8 temp[20]={0};
void mavsend(const uint8_t *buf, uint16_t len){
	if(fp.ROV_mode_enable && sys_flag.relay_12V_enable){	//若载波模块未上电，取消网络发送，切换到无线电台模式，否则会导致发送TIME_OUT
		UDP_send(buf,len);
		return;
	}

	static char *DMA_FIFO=NULL;
	static bool start_flag = false;	//开始传输标志，防止无法进入dma传输
//	if(sys_flag.param_send){for(int j=0;j<len;j++){while((UART7->SR&0X40)==0);UART7->DR=buf[j];} return;}

//	for(int i=0;i<len;i++) printf("%02x ",buf[i]);//printf("\r\n");
	if(sys_flag.param_all_send && DMA_FIFO_SIZE > (fp.uartC_baud*3/10)){	//若积压的数据量超过3s，清空fifo
		myfree(SRAMIN, DMA_FIFO);
		DMA_FIFO = mymalloc(SRAMIN,delta_size);//清空fifo,重新进行空间分配，防止内存占用过大
		DMA_FIFO_SIZE = delta_size;
		dma_fifo_len = 0;
		printf("[telem] DMA_FIFO oversize! \r\n");
	}

	sys_flag.send_empty = (dma_fifo_len==0)?true:false;
	if(len==0 && dma_fifo_len==0) return;	//若当前数据都发送完毕，直接退出
	if(!start_flag){	//第一次传输，给fifo申请内存
		DMA_FIFO = mymalloc(SRAMIN,delta_size);
		DMA_FIFO_SIZE = delta_size;
	}

	if(dma_fifo_len+len > DMA_FIFO_SIZE){ //fifo内存不够(一般发生在参数发送)，每次增加delta_size字节的内存申请
		DMA_FIFO = myrealloc(SRAMIN,DMA_FIFO,DMA_FIFO_SIZE+delta_size);
		DMA_FIFO_SIZE += delta_size;
		printf("[telem] DMA_FIFO_SIZE=%d bytes \r\n",DMA_FIFO_SIZE);
	}
		
	memcpy(DMA_FIFO+dma_fifo_len,buf,len);	//追加到fifo数组
	dma_fifo_len += len; //fifo数组长度更新

	if(start_flag && !__HAL_DMA_GET_FLAG(&hdma_uart7_tx,DMA_FLAG_TCIF1_5)){	//上一次传输还未完成
		return;
	}
	start_flag = true;	//标记开始传输

	__HAL_DMA_CLEAR_FLAG(&hdma_uart7_tx,DMA_FLAG_TCIF1_5);	//上一次传输完成，清除相关标志位
	huart7.Instance->CR3 &= ~USART_CR3_DMAT;
//	huart7.Instance->CR3 &= ~USART_CR3_DMAR;
	HAL_DMA_Abort(huart7.hdmatx);
	if(huart7.State==HAL_UART_STATE_BUSY_TX) huart7.State = HAL_UART_STATE_READY;
	if(huart7.State==HAL_UART_STATE_BUSY_TX_RX) huart7.State = HAL_UART_STATE_BUSY_RX;

//	HAL_UART_DMAStop(&huart7);      //传输完成以后关闭串口DMA -> 恢复串口状态

//	printf("fifo size=%d \r\n",dma_fifo_len);
	u8 *pdate = (u8 *)mymalloc(SRAMIN, dma_fifo_len);
	memcpy(pdate,DMA_FIFO,dma_fifo_len);	//不能直接传输dma_fifo，否则会覆盖，需要先拷贝
//	for(int i=0;i<dma_fifo_len;i++) printf("%02x ",*(pdate+i));
	HAL_StatusTypeDef res = HAL_UART_Transmit_DMA(&huart7, pdate, dma_fifo_len);	//启动dma传输
	if(DMA_FIFO_SIZE > delta_size){	//当前fifo数据全部发送完且fifo大小超过一个单位
		myfree(SRAMIN, DMA_FIFO);
		DMA_FIFO = mymalloc(SRAMIN,delta_size);//清空fifo,重新进行空间分配，防止内存占用过大
		DMA_FIFO_SIZE = delta_size;
	}
	dma_fifo_len = 0;
	myfree(SRAMIN, pdate);

}

void dma_config(){
	 /* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();
#if EN_UART7_RX_DMA
    hdma_uart7_rx.Instance = DMA1_Stream3;
    hdma_uart7_rx.Init.Channel = DMA_CHANNEL_5;
    hdma_uart7_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_uart7_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_uart7_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_uart7_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_uart7_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_uart7_rx.Init.Mode = DMA_CIRCULAR;
    hdma_uart7_rx.Init.Priority = DMA_PRIORITY_MEDIUM;
    hdma_uart7_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    HAL_DMA_Init(&hdma_uart7_rx);

    __HAL_LINKDMA(&huart7,hdmarx,hdma_uart7_rx);
	/* DMA1_Stream1_IRQn interrupt configuration */
//	HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
//	HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
#endif

    hdma_uart7_tx.Instance = DMA1_Stream1;
    hdma_uart7_tx.Init.Channel = DMA_CHANNEL_5;
    hdma_uart7_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_uart7_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_uart7_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_uart7_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_uart7_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_uart7_tx.Init.Mode = DMA_NORMAL;
    hdma_uart7_tx.Init.Priority = DMA_PRIORITY_MEDIUM;
    hdma_uart7_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    HAL_DMA_Init(&hdma_uart7_tx);

    __HAL_LINKDMA(&huart7,hdmatx,hdma_uart7_tx);

}
void DMA1_Stream3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream1_IRQn 0 */

  /* USER CODE END DMA1_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart7_rx);
  /* USER CODE BEGIN DMA1_Stream1_IRQn 1 */

  /* USER CODE END DMA1_Stream1_IRQn 1 */
}

void DMA2_Stream5_IRQHandler(void){
	HAL_DMA_IRQHandler(&hdma_usart1_rx);
}


//以下直接从cubemx生成的项目中拷贝，需要改形参
/* UART7 init function */
void MX_UART7_Init(u32 baud)
{
//	__HAL_RCC_DMA1_CLK_ENABLE();
//	HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
//	HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);

	huart7.Instance = UART7;
	huart7.Init.BaudRate = baud;
	huart7.Init.WordLength = UART_WORDLENGTH_8B;
	huart7.Init.StopBits = UART_STOPBITS_1;
	huart7.Init.Parity = UART_PARITY_NONE;
	huart7.Init.Mode = UART_MODE_TX_RX;
	huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart7.Init.OverSampling = UART_OVERSAMPLING_16;
	HAL_UART_Init(&huart7);

	//注意：dma接收需要在串口初始化完成之后
#if EN_UART7_RX_DMA
	sys_flag.mavlink_recv_dma = true;
	HAL_UART_Receive_DMA(&huart7, UART7_RX_BUF, RXBUFFERSIZE);	//绑定dma相关回掉函数，开启dma接收
#endif

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
	
	#if EN_UART8_RX
//	HAL_UART_Receive_IT(&huart8, (u8 *)u8_RxBuffer, RXBUFFERSIZE);//该函数会开启接收中断：标志位UART_IT_RXNE，并且设置接收缓冲以及接收缓冲接收最大数据量
	#endif

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

#if EN_USART1_RX_DMA
	sys_flag.imu_recv_dma = true;
	HAL_UART_Receive_DMA(&huart1, USART1_RX_BUF, RXBUFFERSIZE);	
#endif

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
	
#if EN_USART2_RX_DMA
	sys_flag.gps_recv_dma = true;
	HAL_UART_Receive_DMA(&huart2, USART2_RX_BUF, RXBUFFERSIZE);	
#endif

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

#if EN_USART3_RX_DMA
	sys_flag.ec_recv_dma = true;
	HAL_UART_Receive_DMA(&huart3, USART3_RX_BUF, RXBUFFERSIZE);	
#endif

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
	
	#if EN_USART6_RX
//	__HAL_UART_ENABLE_IT(&huart6,UART_IT_RXNE);
//	HAL_UART_Receive_IT(&huart6, (u8 *)u6_RxBuffer, RXBUFFERSIZE);
	#endif
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
		/* DMA controller clock enable */
		
#if EN_USART1_RX_DMA
		__HAL_RCC_DMA2_CLK_ENABLE();
		hdma_usart1_rx.Instance = DMA2_Stream5;
		hdma_usart1_rx.Init.Channel = DMA_CHANNEL_4;
		hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
		hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
		hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
		hdma_usart1_rx.Init.Mode = DMA_CIRCULAR;
		hdma_usart1_rx.Init.Priority = DMA_PRIORITY_MEDIUM;
		hdma_usart1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
		HAL_DMA_Init(&hdma_usart1_rx);

		__HAL_LINKDMA(&huart1,hdmarx,hdma_usart1_rx);
		/* DMA1_Stream1_IRQn interrupt configuration */
//		HAL_NVIC_SetPriority(DMA2_Stream5_IRQn, UART1_RX_PrePriority, UART1_RX_SubPriority);
//		HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);
		HAL_NVIC_SetPriority(USART1_IRQn,UART1_RX_PrePriority,UART1_RX_SubPriority);
		HAL_NVIC_EnableIRQ(USART1_IRQn);	
		__HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);	//开启串口空闲中断
#else
		HAL_NVIC_SetPriority(USART1_IRQn,UART1_RX_PrePriority,UART1_RX_SubPriority);
		HAL_NVIC_EnableIRQ(USART1_IRQn);			
		__HAL_UART_ENABLE_IT(&huart1,UART_IT_RXNE); //开启接收完成中断
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
#if EN_USART2_RX_DMA
		__HAL_RCC_DMA1_CLK_ENABLE();
		hdma_usart2_rx.Instance = DMA1_Stream5;
		hdma_usart2_rx.Init.Channel = DMA_CHANNEL_4;
		hdma_usart2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
		hdma_usart2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_usart2_rx.Init.MemInc = DMA_MINC_ENABLE;
		hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		hdma_usart2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
		hdma_usart2_rx.Init.Mode = DMA_CIRCULAR;
		hdma_usart2_rx.Init.Priority = DMA_PRIORITY_MEDIUM;
		hdma_usart2_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
		HAL_DMA_Init(&hdma_usart2_rx);

		__HAL_LINKDMA(&huart2,hdmarx,hdma_usart2_rx);
		/* DMA1_Stream1_IRQn interrupt configuration */
		__HAL_UART_ENABLE_IT(&huart2,UART_IT_IDLE); //开启串口空闲中断
#else
		__HAL_UART_ENABLE_IT(&huart2,UART_IT_RXNE); //开启接收完成中断
#endif	

		HAL_NVIC_EnableIRQ(USART2_IRQn);				//使能USART1中断通道
		HAL_NVIC_SetPriority(USART2_IRQn,UART2_RX_PrePriority,UART2_RX_SubPriority);			//抢占优先级3，子优先级3
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
#if EN_USART3_RX_DMA
		__HAL_RCC_DMA1_CLK_ENABLE();
		hdma_usart3_rx.Instance = DMA1_Stream1;
		hdma_usart3_rx.Init.Channel = DMA_CHANNEL_4;
		hdma_usart3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
		hdma_usart3_rx.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_usart3_rx.Init.MemInc = DMA_MINC_ENABLE;
		hdma_usart3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		hdma_usart3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
		hdma_usart3_rx.Init.Mode = DMA_CIRCULAR;
		hdma_usart3_rx.Init.Priority = DMA_PRIORITY_MEDIUM;
		hdma_usart3_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
		HAL_DMA_Init(&hdma_usart3_rx);

		__HAL_LINKDMA(&huart3,hdmarx,hdma_usart3_rx);
		__HAL_UART_ENABLE_IT(&huart3,UART_IT_IDLE); //开启串口空闲中断
#else
		__HAL_UART_ENABLE_IT(&huart3,UART_IT_RXNE); //开启接收完成中断
#endif	
		HAL_NVIC_EnableIRQ(USART3_IRQn);			
		HAL_NVIC_SetPriority(USART3_IRQn,UART3_RX_PrePriority,UART3_RX_SubPriority);			

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
		HAL_NVIC_SetPriority(USART6_IRQn,UART6_RX_PrePriority,UART6_RX_SubPriority);			//抢占优先级3，子优先级3
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

		/* DMA controller clock enable */
		__HAL_RCC_DMA1_CLK_ENABLE();
#if EN_UART7_RX_DMA
		hdma_uart7_rx.Instance = DMA1_Stream3;
		hdma_uart7_rx.Init.Channel = DMA_CHANNEL_5;
		hdma_uart7_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
		hdma_uart7_rx.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_uart7_rx.Init.MemInc = DMA_MINC_ENABLE;
		hdma_uart7_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		hdma_uart7_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
		hdma_uart7_rx.Init.Mode = DMA_CIRCULAR;
		hdma_uart7_rx.Init.Priority = DMA_PRIORITY_MEDIUM;
		hdma_uart7_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
		HAL_DMA_Init(&hdma_uart7_rx);

		__HAL_LINKDMA(&huart7,hdmarx,hdma_uart7_rx);
		/* DMA1_Stream1_IRQn interrupt configuration */
		__HAL_UART_ENABLE_IT(&huart7,UART_IT_IDLE);	//开启串口空闲中断
#else
		__HAL_UART_ENABLE_IT(&huart7,UART_IT_RXNE);
#endif

		HAL_NVIC_EnableIRQ(UART7_IRQn); 			//使能USART1中断通道
		HAL_NVIC_SetPriority(UART7_IRQn,UART7_RX_PrePriority,UART7_RX_SubPriority); 		//抢占优先级3，子优先级3


		hdma_uart7_tx.Instance = DMA1_Stream1;
		hdma_uart7_tx.Init.Channel = DMA_CHANNEL_5;
		hdma_uart7_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
		hdma_uart7_tx.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_uart7_tx.Init.MemInc = DMA_MINC_ENABLE;
		hdma_uart7_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		hdma_uart7_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
		hdma_uart7_tx.Init.Mode = DMA_NORMAL;
		hdma_uart7_tx.Init.Priority = DMA_PRIORITY_VERY_HIGH;
		hdma_uart7_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
		HAL_DMA_Init(&hdma_uart7_tx);

		__HAL_LINKDMA(&huart7,hdmatx,hdma_uart7_tx);

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
		__HAL_UART_ENABLE_IT(&huart8,UART_IT_RXNE);

		HAL_NVIC_EnableIRQ(UART8_IRQn);				//使能USART1中断通道
		HAL_NVIC_SetPriority(UART8_IRQn,UART8_RX_PrePriority,UART8_RX_SubPriority);			//抢占优先级3，子优先级3
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

void u7_printf(const char* fmt,...)  
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
void uart_write(USART_TypeDef* UART,const char* buf,int size){	
	for(int i=0;i<size;i++){
		while((UART->SR&0X40)==0){}; 	//判断发送完毕
		UART->DR = buf[i];   //填充发送寄存器
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==USART1)//如果是串口1
	{
	}
//	if(huart->Instance==USART2)//如果是串口2
//	{
//		for(u8 i=0;i<RXBUFFERSIZE;i++){
//			USART2_RX_BUF[USART2_RX_STA++]=u2_RxBuffer[i] ;
//			if(USART2_RX_STA == USART2_MAX_RECV_LEN){	//防止接收缓存没有读取溢出，循环写入
//				USART2_RX_STA = 0;
//			}
//		}
//	}
	if(huart->Instance==USART6)//如果是串口2
	{
	}
//	if(huart->Instance==UART7)//如果是串口1
//	{
//		for(u8 i=0;i<RXBUFFERSIZE;i++){
//			//printf("%02x",u7_RxBuffer[i]);
//			UART7_RX_BUF[UART7_RX_STA++]=u7_RxBuffer[i] ;
//			if(UART7_RX_STA == UART7_MAX_RECV_LEN){	//防止接收缓存没有读取溢出，循环写入
//				UART7_RX_STA = 0;
//			}
//		}
//		//HAL_UART_Receive_DMA(&huart7, u7_RxBuffer, RXBUFFERSIZE); //dma配置为normal模式需要再次开启
//	}
	if(huart->Instance==UART8)//如果是串口1
	{
	}
}

//串口1中断服务程序
void USART1_IRQHandler(void)                	
{ 	
	u8 Res;
	if((huart1.Instance->CR1&USART_CR1_RXNEIE) && (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE)!=RESET))  
	{
		HAL_UART_Receive(&huart1,&Res,1,1000); 
		if(USART1_RX_STA>(USART1_MAX_RECV_LEN-1))USART1_RX_STA=0;
		USART1_RX_BUF[USART1_RX_STA++]=Res ;
		if(sys_flag.uart1_dictect_trans) {
			while((USART6->SR&0X40)==0);//循环发送,直到发送完毕   
			USART6->DR = (u8) Res;  
		}
	}	
	else if((huart1.Instance->CR1&USART_CR1_IDLEIE) && (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE)) == SET){
		//接收数据       
		USART1_RX_STA = RXBUFFERSIZE-__HAL_DMA_GET_COUNTER(&hdma_usart1_rx); /* 获取接收到的数据长度 单位为字节*/
		__HAL_DMA_DISABLE(&hdma_usart1_rx);  /* 暂时关闭dma，防止干扰，数据尚未处理 */
		__HAL_DMA_CLEAR_FLAG(&hdma_usart1_rx,DMA_FLAG_TCIF2_6);/* 清DMA标志位 */
		__HAL_DMA_SET_COUNTER(&hdma_usart1_rx,RXBUFFERSIZE);/* 重新赋值计数值，必须大于等于最大可能接收到的数据帧数目 */
		__HAL_DMA_ENABLE(&hdma_usart1_rx);
		sys_flag.imu_recv = true;  /* 标记接收完成*/
//		printf("recv mpu package %d \r\n",USART1_RX_STA);
		Res = huart1.Instance->SR;
		Res = huart1.Instance->DR;
	}

}

void USART2_IRQHandler(void)                	
{ 	
	u8 Res;
	if((huart2.Instance->CR1&USART_CR1_RXNEIE) && (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE)!=RESET))  
	{
		HAL_UART_Receive(&huart2,&Res,1,1000); 
		if(USART2_RX_STA>(USART2_MAX_RECV_LEN-1))USART2_RX_STA=0;
		USART2_RX_BUF[USART2_RX_STA++]=Res ;	
		if(sys_flag.uart2_dictect_trans) {
			while((USART6->SR&0X40)==0);//循环发送,直到发送完毕   
			USART6->DR = (u8) Res;  
		}
	}	
	if((huart2.Instance->CR1&USART_CR1_IDLEIE) && (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE)) == SET){
		//接收数据		 
		USART2_RX_STA = RXBUFFERSIZE-__HAL_DMA_GET_COUNTER(&hdma_usart2_rx); /* 获取接收到的数据长度 单位为字节*/
		__HAL_DMA_DISABLE(&hdma_usart2_rx);  /* 暂时关闭dma，防止干扰，数据尚未处理 */
		__HAL_DMA_CLEAR_FLAG(&hdma_usart2_rx,DMA_FLAG_TCIF1_5);/* 清DMA标志位 */
		__HAL_DMA_SET_COUNTER(&hdma_usart2_rx,RXBUFFERSIZE);/* 重新赋值计数值，必须大于等于最大可能接收到的数据帧数目 */
		__HAL_DMA_ENABLE(&hdma_usart2_rx);
//		sys_flag.gps_recv = true;  /* 标记接收完成*/
//		printf("recv gps package %d \r\n",USART2_RX_STA);
		Res = huart2.Instance->SR;
		Res = huart2.Instance->DR;
	}

}
void USART3_IRQHandler(void)                	
{ 	
	u8 Res;
	if((huart3.Instance->CR1&USART_CR1_RXNEIE) && (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_RXNE)!=RESET))  
	{
		HAL_UART_Receive(&huart3,&Res,1,1000); 
		if(USART3_RX_STA>(USART3_MAX_RECV_LEN-1))USART3_RX_STA=0;
		USART3_RX_BUF[USART3_RX_STA++]=Res ;		
		__HAL_UART_CLEAR_FLAG(&huart3, UART_FLAG_RXNE);
		if(sys_flag.uart3_showrecv) printf("%02x ",Res);
		if(sys_flag.uart3_dictect_trans) {
			while((USART6->SR&0X40)==0);//循环发送,直到发送完毕   
			USART6->DR = (u8) Res;  
		}
	}	
	else if((huart1.Instance->CR1&USART_CR1_IDLEIE) && (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE)) == SET){
		//接收数据		 
		USART3_RX_STA = RXBUFFERSIZE-__HAL_DMA_GET_COUNTER(&hdma_usart3_rx); /* 获取接收到的数据长度 单位为字节*/
		__HAL_DMA_DISABLE(&hdma_usart3_rx);  /* 暂时关闭dma，防止干扰，数据尚未处理 */
		__HAL_DMA_CLEAR_FLAG(&hdma_usart3_rx,DMA_FLAG_TCIF1_5);/* 清DMA标志位 */
		__HAL_DMA_SET_COUNTER(&hdma_usart3_rx,RXBUFFERSIZE);/* 重新赋值计数值，必须大于等于最大可能接收到的数据帧数目 */
		__HAL_DMA_ENABLE(&hdma_usart3_rx);
		sys_flag.ec_recv = true;  /* 标记接收完成*/
//		printf("recv tcm package %d \r\n",USART3_RX_STA);
		Res = huart3.Instance->SR;
		Res = huart3.Instance->DR;
	}
}

void USART6_IRQHandler(void)                	
{ 	
	u8 Res;
	if((__HAL_UART_GET_FLAG(&huart6,UART_FLAG_RXNE)!=RESET))  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
	{
		HAL_UART_Receive(&huart6,&Res,1,1000); 
		//串口透传
		if(sys_flag.uart1_dictect_trans){
			while((USART1->SR&0X40)==0); USART1->DR=Res; 
			return;
		}
		if(sys_flag.uart2_dictect_trans){
			while((USART2->SR&0X40)==0); USART2->DR=Res; 
			return;
		}
		if(sys_flag.uart3_dictect_trans){
			while((USART3->SR&0X40)==0); USART3->DR=Res; 
			return;
		}
		if(sys_flag.uart7_dictect_trans){
			while((UART7->SR&0X40)==0); UART7->DR=Res; 
			return;
		}
		if(sys_flag.uart8_dictect_trans){
			while((UART8->SR&0X40)==0); UART8->DR=Res; 
			return;
		}
		if((USART6_RX_STA&0x8000)==0)//接收未完成
		{
			if(USART6_RX_STA&0x4000)//接收到了0x0d
			{
				if(Res!=0x0a)USART6_RX_STA=0;//接收错误,重新开始
				else {
					USART6_RX_STA|=0x8000;	//接收完成了
					if(vp.control_mode==AUTO && strcmp((char*)USART6_RX_BUF,"reset")==0)
						NVIC_SystemReset();
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
	u8 Res;
	if((huart7.Instance->CR1&USART_CR1_RXNEIE) && (__HAL_UART_GET_FLAG(&huart7, UART_FLAG_RXNE)))  { //
		HAL_UART_Receive(&huart7,&Res,1,1000); 
		if(UART7_RX_STA>(UART7_MAX_RECV_LEN-1))UART7_RX_STA=0;
		UART7_RX_BUF[UART7_RX_STA++]=Res ;			
		__HAL_UART_CLEAR_FLAG(&huart7, UART_FLAG_RXNE);
		if(sys_flag.uart7_dictect_trans) {
			while((USART6->SR&0X40)==0);//循环发送,直到发送完毕   
			USART6->DR = (u8) Res;  
		}
	}
	else if((huart7.Instance->CR1&USART_CR1_IDLEIE) && (__HAL_UART_GET_FLAG(&huart7, UART_FLAG_IDLE))){	
		//接收数据       
		UART7_RX_STA =  RXBUFFERSIZE-__HAL_DMA_GET_COUNTER(&hdma_uart7_rx); /* 获取接收到的数据长度 单位为字节*/
		__HAL_DMA_DISABLE(&hdma_uart7_rx);  /* 暂时关闭dma，防止干扰，数据尚未处理 */
		__HAL_DMA_CLEAR_FLAG(&hdma_uart7_rx,DMA_FLAG_TCIF3_7);/* 清DMA标志位 */
		__HAL_DMA_SET_COUNTER(&hdma_uart7_rx,RXBUFFERSIZE);/* 重新赋值计数值，必须大于等于最大可能接收到的数据帧数目 */
		__HAL_DMA_ENABLE(&hdma_uart7_rx);
		sys_flag.mavlink_recv = true;  /* 标记接收完成*/
//		printf("recv mavlink package %d \r\n",UART7_RX_STA);
		//注意：需要先读入SR寄存器，然后读入DR寄存器，才能清除IDLE标志位
		Res = huart7.Instance->SR;
		Res = huart7.Instance->DR;
		//防止在某个地方阻塞无法退出（需要在没有心跳的时候处理）
		if(!sys_flag.health.gcs_heartbeat && strcmp((char*)UART7_RX_BUF,"reset")==0)	
			NVIC_SystemReset();
//		__HAL_UART_CLEAR_FLAG(&huart7, UART_FLAG_IDLE);
			
	}

}

void UART8_IRQHandler(void){
	u8 Res;
	if((__HAL_UART_GET_FLAG(&huart8,UART_FLAG_RXNE)!=RESET)){  //接收中断
		HAL_UART_Receive(&huart8,&Res,1,1000); 
		UART8_RX_BUF[UART8_RX_STA&0X3FFF]=Res ;
		UART8_RX_STA++;					
	}	 		
//	HAL_UART_IRQHandler(&huart8);
	if((__HAL_UART_GET_FLAG(&huart8, UART_FLAG_ORE)) == SET){
			__HAL_UART_CLEAR_FLAG(&huart8, UART_FLAG_ORE);
		}

}
