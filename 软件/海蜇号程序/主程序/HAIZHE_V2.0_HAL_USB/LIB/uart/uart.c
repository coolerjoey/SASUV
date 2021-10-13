#include "uart.h"
#include "malloc.h"
#include "delay.h"
#include "parameter.h"
#include "udp_demo.h"

//���ڽ����ж�ʹ��
#define EN_USART1_RX 			0		//ʹ�ܣ�1��/��ֹ��0������1����
#define EN_USART2_RX 			1
#define EN_USART3_RX 			1
#define EN_USART4_RX 			0
#define EN_USART6_RX      		1
#define EN_UART7_RX 			0
#define EN_UART8_RX 			0
//����DMA�����ж�ʹ��
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
//__use_no_semihosting was requested, but _ttywrch was 
void _ttywrch(int ch)
{
    ch = ch;
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
u16 UART8_RX_STA=0; 

UART_HandleTypeDef huart1,huart2,huart3,huart6,huart7,huart8;//UART���,�趨��Ϊȫ�ֱ���

DMA_HandleTypeDef hdma_usart1_rx, hdma_usart2_rx, hdma_usart3_rx, hdma_uart7_rx, hdma_uart7_tx;

u8 u7_RxBuffer[RXBUFFERSIZE];//HAL��USART����Buffer
u8 u8_RxBuffer[RXBUFFERSIZE];
u8 u1_RxBuffer[RXBUFFERSIZE];//HAL��USART����Buffer
u8 u2_RxBuffer[RXBUFFERSIZE];
u8 u3_RxBuffer[RXBUFFERSIZE];
u8 u6_RxBuffer[RXBUFFERSIZE];

int DMA_FIFO_SIZE=0;
int delta_size=512;
int dma_fifo_len=0;

u8 temp[20]={0};
void mavsend(const uint8_t *buf, uint16_t len){
	if(fp.ROV_mode_enable && sys_flag.relay_12V_enable){	//���ز�ģ��δ�ϵ磬ȡ�����緢�ͣ��л������ߵ�̨ģʽ������ᵼ�·���TIME_OUT
		UDP_send(buf,len);
		return;
	}

	static char *DMA_FIFO=NULL;
	static bool start_flag = false;	//��ʼ�����־����ֹ�޷�����dma����
//	if(sys_flag.param_send){for(int j=0;j<len;j++){while((UART7->SR&0X40)==0);UART7->DR=buf[j];} return;}

//	for(int i=0;i<len;i++) printf("%02x ",buf[i]);//printf("\r\n");
	if(sys_flag.param_all_send && DMA_FIFO_SIZE > (fp.uartC_baud*3/10)){	//����ѹ������������3s�����fifo
		myfree(SRAMIN, DMA_FIFO);
		DMA_FIFO = mymalloc(SRAMIN,delta_size);//���fifo,���½��пռ���䣬��ֹ�ڴ�ռ�ù���
		DMA_FIFO_SIZE = delta_size;
		dma_fifo_len = 0;
		printf("[telem] DMA_FIFO oversize! \r\n");
	}

	sys_flag.send_empty = (dma_fifo_len==0)?true:false;
	if(len==0 && dma_fifo_len==0) return;	//����ǰ���ݶ�������ϣ�ֱ���˳�
	if(!start_flag){	//��һ�δ��䣬��fifo�����ڴ�
		DMA_FIFO = mymalloc(SRAMIN,delta_size);
		DMA_FIFO_SIZE = delta_size;
	}

	if(dma_fifo_len+len > DMA_FIFO_SIZE){ //fifo�ڴ治��(һ�㷢���ڲ�������)��ÿ������delta_size�ֽڵ��ڴ�����
		DMA_FIFO = myrealloc(SRAMIN,DMA_FIFO,DMA_FIFO_SIZE+delta_size);
		DMA_FIFO_SIZE += delta_size;
		printf("[telem] DMA_FIFO_SIZE=%d bytes \r\n",DMA_FIFO_SIZE);
	}
		
	memcpy(DMA_FIFO+dma_fifo_len,buf,len);	//׷�ӵ�fifo����
	dma_fifo_len += len; //fifo���鳤�ȸ���

	if(start_flag && !__HAL_DMA_GET_FLAG(&hdma_uart7_tx,DMA_FLAG_TCIF1_5)){	//��һ�δ��仹δ���
		return;
	}
	start_flag = true;	//��ǿ�ʼ����

	__HAL_DMA_CLEAR_FLAG(&hdma_uart7_tx,DMA_FLAG_TCIF1_5);	//��һ�δ�����ɣ������ر�־λ
	huart7.Instance->CR3 &= ~USART_CR3_DMAT;
//	huart7.Instance->CR3 &= ~USART_CR3_DMAR;
	HAL_DMA_Abort(huart7.hdmatx);
	if(huart7.State==HAL_UART_STATE_BUSY_TX) huart7.State = HAL_UART_STATE_READY;
	if(huart7.State==HAL_UART_STATE_BUSY_TX_RX) huart7.State = HAL_UART_STATE_BUSY_RX;

//	HAL_UART_DMAStop(&huart7);      //��������Ժ�رմ���DMA -> �ָ�����״̬

//	printf("fifo size=%d \r\n",dma_fifo_len);
	u8 *pdate = (u8 *)mymalloc(SRAMIN, dma_fifo_len);
	memcpy(pdate,DMA_FIFO,dma_fifo_len);	//����ֱ�Ӵ���dma_fifo������Ḳ�ǣ���Ҫ�ȿ���
//	for(int i=0;i<dma_fifo_len;i++) printf("%02x ",*(pdate+i));
	HAL_StatusTypeDef res = HAL_UART_Transmit_DMA(&huart7, pdate, dma_fifo_len);	//����dma����
	if(DMA_FIFO_SIZE > delta_size){	//��ǰfifo����ȫ����������fifo��С����һ����λ
		myfree(SRAMIN, DMA_FIFO);
		DMA_FIFO = mymalloc(SRAMIN,delta_size);//���fifo,���½��пռ���䣬��ֹ�ڴ�ռ�ù���
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


//����ֱ�Ӵ�cubemx���ɵ���Ŀ�п�������Ҫ���β�
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

	//ע�⣺dma������Ҫ�ڴ��ڳ�ʼ�����֮��
#if EN_UART7_RX_DMA
	sys_flag.mavlink_recv_dma = true;
	HAL_UART_Receive_DMA(&huart7, UART7_RX_BUF, RXBUFFERSIZE);	//��dma��ػص�����������dma����
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
//	HAL_UART_Receive_IT(&huart8, (u8 *)u8_RxBuffer, RXBUFFERSIZE);//�ú����Ὺ�������жϣ���־λUART_IT_RXNE���������ý��ջ����Լ����ջ���������������
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
		__HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);	//�������ڿ����ж�
#else
		HAL_NVIC_SetPriority(USART1_IRQn,UART1_RX_PrePriority,UART1_RX_SubPriority);
		HAL_NVIC_EnableIRQ(USART1_IRQn);			
		__HAL_UART_ENABLE_IT(&huart1,UART_IT_RXNE); //������������ж�
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
		__HAL_UART_ENABLE_IT(&huart2,UART_IT_IDLE); //�������ڿ����ж�
#else
		__HAL_UART_ENABLE_IT(&huart2,UART_IT_RXNE); //������������ж�
#endif	

		HAL_NVIC_EnableIRQ(USART2_IRQn);				//ʹ��USART1�ж�ͨ��
		HAL_NVIC_SetPriority(USART2_IRQn,UART2_RX_PrePriority,UART2_RX_SubPriority);			//��ռ���ȼ�3�������ȼ�3
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
		__HAL_UART_ENABLE_IT(&huart3,UART_IT_IDLE); //�������ڿ����ж�
#else
		__HAL_UART_ENABLE_IT(&huart3,UART_IT_RXNE); //������������ж�
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
		HAL_NVIC_EnableIRQ(USART6_IRQn);				//ʹ��USART1�ж�ͨ��
		HAL_NVIC_SetPriority(USART6_IRQn,UART6_RX_PrePriority,UART6_RX_SubPriority);			//��ռ���ȼ�3�������ȼ�3
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
		__HAL_UART_ENABLE_IT(&huart7,UART_IT_IDLE);	//�������ڿ����ж�
#else
		__HAL_UART_ENABLE_IT(&huart7,UART_IT_RXNE);
#endif

		HAL_NVIC_EnableIRQ(UART7_IRQn); 			//ʹ��USART1�ж�ͨ��
		HAL_NVIC_SetPriority(UART7_IRQn,UART7_RX_PrePriority,UART7_RX_SubPriority); 		//��ռ���ȼ�3�������ȼ�3


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

		HAL_NVIC_EnableIRQ(UART8_IRQn);				//ʹ��USART1�ж�ͨ��
		HAL_NVIC_SetPriority(UART8_IRQn,UART8_RX_PrePriority,UART8_RX_SubPriority);			//��ռ���ȼ�3�������ȼ�3
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

void u7_printf(const char* fmt,...)  
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
void uart_write(USART_TypeDef* UART,const char* buf,int size){	
	for(int i=0;i<size;i++){
		while((UART->SR&0X40)==0){}; 	//�жϷ������
		UART->DR = buf[i];   //��䷢�ͼĴ���
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==USART1)//����Ǵ���1
	{
	}
//	if(huart->Instance==USART2)//����Ǵ���2
//	{
//		for(u8 i=0;i<RXBUFFERSIZE;i++){
//			USART2_RX_BUF[USART2_RX_STA++]=u2_RxBuffer[i] ;
//			if(USART2_RX_STA == USART2_MAX_RECV_LEN){	//��ֹ���ջ���û�ж�ȡ�����ѭ��д��
//				USART2_RX_STA = 0;
//			}
//		}
//	}
	if(huart->Instance==USART6)//����Ǵ���2
	{
	}
//	if(huart->Instance==UART7)//����Ǵ���1
//	{
//		for(u8 i=0;i<RXBUFFERSIZE;i++){
//			//printf("%02x",u7_RxBuffer[i]);
//			UART7_RX_BUF[UART7_RX_STA++]=u7_RxBuffer[i] ;
//			if(UART7_RX_STA == UART7_MAX_RECV_LEN){	//��ֹ���ջ���û�ж�ȡ�����ѭ��д��
//				UART7_RX_STA = 0;
//			}
//		}
//		//HAL_UART_Receive_DMA(&huart7, u7_RxBuffer, RXBUFFERSIZE); //dma����Ϊnormalģʽ��Ҫ�ٴο���
//	}
	if(huart->Instance==UART8)//����Ǵ���1
	{
	}
}

//����1�жϷ������
void USART1_IRQHandler(void)                	
{ 	
	u8 Res;
	if((huart1.Instance->CR1&USART_CR1_RXNEIE) && (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE)!=RESET))  
	{
		HAL_UART_Receive(&huart1,&Res,1,1000); 
		if(USART1_RX_STA>(USART1_MAX_RECV_LEN-1))USART1_RX_STA=0;
		USART1_RX_BUF[USART1_RX_STA++]=Res ;
		if(sys_flag.uart1_dictect_trans) {
			while((USART6->SR&0X40)==0);//ѭ������,ֱ���������   
			USART6->DR = (u8) Res;  
		}
	}	
	else if((huart1.Instance->CR1&USART_CR1_IDLEIE) && (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE)) == SET){
		//��������       
		USART1_RX_STA = RXBUFFERSIZE-__HAL_DMA_GET_COUNTER(&hdma_usart1_rx); /* ��ȡ���յ������ݳ��� ��λΪ�ֽ�*/
		__HAL_DMA_DISABLE(&hdma_usart1_rx);  /* ��ʱ�ر�dma����ֹ���ţ�������δ���� */
		__HAL_DMA_CLEAR_FLAG(&hdma_usart1_rx,DMA_FLAG_TCIF2_6);/* ��DMA��־λ */
		__HAL_DMA_SET_COUNTER(&hdma_usart1_rx,RXBUFFERSIZE);/* ���¸�ֵ����ֵ��������ڵ��������ܽ��յ�������֡��Ŀ */
		__HAL_DMA_ENABLE(&hdma_usart1_rx);
		sys_flag.imu_recv = true;  /* ��ǽ������*/
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
			while((USART6->SR&0X40)==0);//ѭ������,ֱ���������   
			USART6->DR = (u8) Res;  
		}
	}	
	if((huart2.Instance->CR1&USART_CR1_IDLEIE) && (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE)) == SET){
		//��������		 
		USART2_RX_STA = RXBUFFERSIZE-__HAL_DMA_GET_COUNTER(&hdma_usart2_rx); /* ��ȡ���յ������ݳ��� ��λΪ�ֽ�*/
		__HAL_DMA_DISABLE(&hdma_usart2_rx);  /* ��ʱ�ر�dma����ֹ���ţ�������δ���� */
		__HAL_DMA_CLEAR_FLAG(&hdma_usart2_rx,DMA_FLAG_TCIF1_5);/* ��DMA��־λ */
		__HAL_DMA_SET_COUNTER(&hdma_usart2_rx,RXBUFFERSIZE);/* ���¸�ֵ����ֵ��������ڵ��������ܽ��յ�������֡��Ŀ */
		__HAL_DMA_ENABLE(&hdma_usart2_rx);
//		sys_flag.gps_recv = true;  /* ��ǽ������*/
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
			while((USART6->SR&0X40)==0);//ѭ������,ֱ���������   
			USART6->DR = (u8) Res;  
		}
	}	
	else if((huart1.Instance->CR1&USART_CR1_IDLEIE) && (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE)) == SET){
		//��������		 
		USART3_RX_STA = RXBUFFERSIZE-__HAL_DMA_GET_COUNTER(&hdma_usart3_rx); /* ��ȡ���յ������ݳ��� ��λΪ�ֽ�*/
		__HAL_DMA_DISABLE(&hdma_usart3_rx);  /* ��ʱ�ر�dma����ֹ���ţ�������δ���� */
		__HAL_DMA_CLEAR_FLAG(&hdma_usart3_rx,DMA_FLAG_TCIF1_5);/* ��DMA��־λ */
		__HAL_DMA_SET_COUNTER(&hdma_usart3_rx,RXBUFFERSIZE);/* ���¸�ֵ����ֵ��������ڵ��������ܽ��յ�������֡��Ŀ */
		__HAL_DMA_ENABLE(&hdma_usart3_rx);
		sys_flag.ec_recv = true;  /* ��ǽ������*/
//		printf("recv tcm package %d \r\n",USART3_RX_STA);
		Res = huart3.Instance->SR;
		Res = huart3.Instance->DR;
	}
}

void USART6_IRQHandler(void)                	
{ 	
	u8 Res;
	if((__HAL_UART_GET_FLAG(&huart6,UART_FLAG_RXNE)!=RESET))  //�����ж�(���յ������ݱ�����0x0d 0x0a��β)
	{
		HAL_UART_Receive(&huart6,&Res,1,1000); 
		//����͸��
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
		if((USART6_RX_STA&0x8000)==0)//����δ���
		{
			if(USART6_RX_STA&0x4000)//���յ���0x0d
			{
				if(Res!=0x0a)USART6_RX_STA=0;//���մ���,���¿�ʼ
				else {
					USART6_RX_STA|=0x8000;	//���������
					if(vp.control_mode==AUTO && strcmp((char*)USART6_RX_BUF,"reset")==0)
						NVIC_SystemReset();
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
	u8 Res;
	if((huart7.Instance->CR1&USART_CR1_RXNEIE) && (__HAL_UART_GET_FLAG(&huart7, UART_FLAG_RXNE)))  { //
		HAL_UART_Receive(&huart7,&Res,1,1000); 
		if(UART7_RX_STA>(UART7_MAX_RECV_LEN-1))UART7_RX_STA=0;
		UART7_RX_BUF[UART7_RX_STA++]=Res ;			
		__HAL_UART_CLEAR_FLAG(&huart7, UART_FLAG_RXNE);
		if(sys_flag.uart7_dictect_trans) {
			while((USART6->SR&0X40)==0);//ѭ������,ֱ���������   
			USART6->DR = (u8) Res;  
		}
	}
	else if((huart7.Instance->CR1&USART_CR1_IDLEIE) && (__HAL_UART_GET_FLAG(&huart7, UART_FLAG_IDLE))){	
		//��������       
		UART7_RX_STA =  RXBUFFERSIZE-__HAL_DMA_GET_COUNTER(&hdma_uart7_rx); /* ��ȡ���յ������ݳ��� ��λΪ�ֽ�*/
		__HAL_DMA_DISABLE(&hdma_uart7_rx);  /* ��ʱ�ر�dma����ֹ���ţ�������δ���� */
		__HAL_DMA_CLEAR_FLAG(&hdma_uart7_rx,DMA_FLAG_TCIF3_7);/* ��DMA��־λ */
		__HAL_DMA_SET_COUNTER(&hdma_uart7_rx,RXBUFFERSIZE);/* ���¸�ֵ����ֵ��������ڵ��������ܽ��յ�������֡��Ŀ */
		__HAL_DMA_ENABLE(&hdma_uart7_rx);
		sys_flag.mavlink_recv = true;  /* ��ǽ������*/
//		printf("recv mavlink package %d \r\n",UART7_RX_STA);
		//ע�⣺��Ҫ�ȶ���SR�Ĵ�����Ȼ�����DR�Ĵ������������IDLE��־λ
		Res = huart7.Instance->SR;
		Res = huart7.Instance->DR;
		//��ֹ��ĳ���ط������޷��˳�����Ҫ��û��������ʱ����
		if(!sys_flag.health.gcs_heartbeat && strcmp((char*)UART7_RX_BUF,"reset")==0)	
			NVIC_SystemReset();
//		__HAL_UART_CLEAR_FLAG(&huart7, UART_FLAG_IDLE);
			
	}

}

void UART8_IRQHandler(void){
	u8 Res;
	if((__HAL_UART_GET_FLAG(&huart8,UART_FLAG_RXNE)!=RESET)){  //�����ж�
		HAL_UART_Receive(&huart8,&Res,1,1000); 
		UART8_RX_BUF[UART8_RX_STA&0X3FFF]=Res ;
		UART8_RX_STA++;					
	}	 		
//	HAL_UART_IRQHandler(&huart8);
	if((__HAL_UART_GET_FLAG(&huart8, UART_FLAG_ORE)) == SET){
			__HAL_UART_CLEAR_FLAG(&huart8, UART_FLAG_ORE);
		}

}
