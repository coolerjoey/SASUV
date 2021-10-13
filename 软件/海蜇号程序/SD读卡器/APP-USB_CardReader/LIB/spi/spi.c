#include "spi.h"
SPI_HandleTypeDef SPI_Handler;
SPI_HandleTypeDef hspi1,hspi2,hspi3,hspi4,hspi5,hspi6;

//以下是SPI模块的初始化代码，配置成主机模式 						  
//SPI口初始化
//这里针是对SPI5的初始化
void SPI_Init(SPI_TypeDef *SPI){
	SPI_Handler.Instance=SPI;
	SPI_Handler.Init.Mode=SPI_MODE_MASTER;             //设置SPI工作模式，设置为主模式
	SPI_Handler.Init.Direction=SPI_DIRECTION_2LINES;   //设置SPI单向或者双向的数据模式:SPI设置为双线模式
	SPI_Handler.Init.DataSize=SPI_DATASIZE_8BIT;       //设置SPI的数据大小:SPI发送接收8位帧结构
	SPI_Handler.Init.CLKPolarity=SPI_POLARITY_HIGH;    //串行同步时钟的空闲状态为高电平
	SPI_Handler.Init.CLKPhase=SPI_PHASE_2EDGE;         //串行同步时钟的第二个跳变沿（上升或下降）数据被采样
	SPI_Handler.Init.NSS=SPI_NSS_SOFT;                 //NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制
	SPI_Handler.Init.BaudRatePrescaler=SPI_BAUDRATEPRESCALER_256;//定义波特率预分频的值:波特率预分频值为256
	SPI_Handler.Init.FirstBit=SPI_FIRSTBIT_MSB;        //指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始
	SPI_Handler.Init.TIMode=SPI_TIMODE_DISABLE;        //关闭TI模式
	SPI_Handler.Init.CRCCalculation=SPI_CRCCALCULATION_DISABLE;//关闭硬件CRC校验
	SPI_Handler.Init.CRCPolynomial=7;                  //CRC值计算的多项式
	HAL_SPI_Init(&SPI_Handler);//初始化
	
	__HAL_SPI_ENABLE(&SPI_Handler);                    //使能SPI

	SPI_ReadWriteByte(SPI,0Xff);                           //启动传输,这句话最大的作用就是维持MOSI为高电平，而且这句话也不是必须的，可以去掉。
}

//SPI底层驱动，时钟使能，引脚配置
//此函数会被HAL_SPI_Init()调用
//hspi:SPI句柄
void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
    GPIO_InitTypeDef GPIO_InitStruct;
		if(hspi->Instance==SPI1){
			hspi1 = *hspi;
		}
		else if(hspi->Instance==SPI2){
			hspi2 = *hspi;
			__HAL_RCC_GPIOB_CLK_ENABLE();
			__HAL_RCC_SPI2_CLK_ENABLE();
						/*Configure GPIO pins : PB13 PB14 PB15 */
			GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
			GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
			GPIO_InitStruct.Pull = GPIO_PULLUP;
			GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
			GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
			HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
		}
		else if(hspi->Instance==SPI3){
			hspi3 = *hspi;
		}
		else if(hspi->Instance==SPI4){
			hspi4 = *hspi;
		}
    else if(hspi->Instance==SPI5){
			hspi5 = *hspi;
			__HAL_RCC_GPIOF_CLK_ENABLE();       //使能GPIOF时钟
			__HAL_RCC_SPI5_CLK_ENABLE();        //使能SPI5时钟
			
			//PF7,8,9
			GPIO_InitStruct.Pin=GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
			GPIO_InitStruct.Mode=GPIO_MODE_AF_PP;              //复用推挽输出
			GPIO_InitStruct.Pull=GPIO_PULLUP;                  //上拉
			GPIO_InitStruct.Speed=GPIO_SPEED_FAST;             //快速            
			GPIO_InitStruct.Alternate=GPIO_AF5_SPI5;           //复用为SPI5
		    HAL_GPIO_Init(GPIOF,&GPIO_InitStruct);
		}	
		else if(hspi->Instance==SPI6){
			hspi6 = *hspi;
		}
}

//SPI速度设置函数
//SPI速度=fAPB1/分频系数
//@ref SPI_BaudRate_Prescaler:SPI_BAUDRATEPRESCALER_2~SPI_BAUDRATEPRESCALER_2 256
//fAPB1时钟一般为45Mhz：
void SPI_SetSpeed(SPI_TypeDef *SPI,u8 SPI_BaudRatePrescaler)
{
	SPI_Handler.Instance=SPI;
	assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_BaudRatePrescaler));//判断有效性
	__HAL_SPI_DISABLE(&SPI_Handler);            //关闭SPI
	SPI_Handler.Instance->CR1&=0XFFC7;          //位3-5清零，用来设置波特率
	SPI_Handler.Instance->CR1|=SPI_BaudRatePrescaler;//设置SPI速度
	__HAL_SPI_ENABLE(&SPI_Handler);             //使能SPI
}


//SPI5 读写一个字节
//TxData:要写入的字节
//返回值:读取到的字节
u8 SPI_ReadWriteByte(SPI_TypeDef *SPI,u8 TxData)
{
	SPI_Handler.Instance=SPI;
	u8 Rxdata;
	HAL_SPI_TransmitReceive(&SPI_Handler,&TxData,&Rxdata,1, 1000);       
 	return Rxdata;          		    //返回收到的数据		
}
