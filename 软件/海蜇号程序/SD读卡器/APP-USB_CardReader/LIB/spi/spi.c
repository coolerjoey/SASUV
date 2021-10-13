#include "spi.h"
SPI_HandleTypeDef SPI_Handler;
SPI_HandleTypeDef hspi1,hspi2,hspi3,hspi4,hspi5,hspi6;

//������SPIģ��ĳ�ʼ�����룬���ó�����ģʽ 						  
//SPI�ڳ�ʼ��
//�������Ƕ�SPI5�ĳ�ʼ��
void SPI_Init(SPI_TypeDef *SPI){
	SPI_Handler.Instance=SPI;
	SPI_Handler.Init.Mode=SPI_MODE_MASTER;             //����SPI����ģʽ������Ϊ��ģʽ
	SPI_Handler.Init.Direction=SPI_DIRECTION_2LINES;   //����SPI�������˫�������ģʽ:SPI����Ϊ˫��ģʽ
	SPI_Handler.Init.DataSize=SPI_DATASIZE_8BIT;       //����SPI�����ݴ�С:SPI���ͽ���8λ֡�ṹ
	SPI_Handler.Init.CLKPolarity=SPI_POLARITY_HIGH;    //����ͬ��ʱ�ӵĿ���״̬Ϊ�ߵ�ƽ
	SPI_Handler.Init.CLKPhase=SPI_PHASE_2EDGE;         //����ͬ��ʱ�ӵĵڶ��������أ��������½������ݱ�����
	SPI_Handler.Init.NSS=SPI_NSS_SOFT;                 //NSS�ź���Ӳ����NSS�ܽţ����������ʹ��SSIλ������:�ڲ�NSS�ź���SSIλ����
	SPI_Handler.Init.BaudRatePrescaler=SPI_BAUDRATEPRESCALER_256;//���岨����Ԥ��Ƶ��ֵ:������Ԥ��ƵֵΪ256
	SPI_Handler.Init.FirstBit=SPI_FIRSTBIT_MSB;        //ָ�����ݴ����MSBλ����LSBλ��ʼ:���ݴ����MSBλ��ʼ
	SPI_Handler.Init.TIMode=SPI_TIMODE_DISABLE;        //�ر�TIģʽ
	SPI_Handler.Init.CRCCalculation=SPI_CRCCALCULATION_DISABLE;//�ر�Ӳ��CRCУ��
	SPI_Handler.Init.CRCPolynomial=7;                  //CRCֵ����Ķ���ʽ
	HAL_SPI_Init(&SPI_Handler);//��ʼ��
	
	__HAL_SPI_ENABLE(&SPI_Handler);                    //ʹ��SPI

	SPI_ReadWriteByte(SPI,0Xff);                           //��������,��仰�������þ���ά��MOSIΪ�ߵ�ƽ��������仰Ҳ���Ǳ���ģ�����ȥ����
}

//SPI�ײ�������ʱ��ʹ�ܣ���������
//�˺����ᱻHAL_SPI_Init()����
//hspi:SPI���
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
			__HAL_RCC_GPIOF_CLK_ENABLE();       //ʹ��GPIOFʱ��
			__HAL_RCC_SPI5_CLK_ENABLE();        //ʹ��SPI5ʱ��
			
			//PF7,8,9
			GPIO_InitStruct.Pin=GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
			GPIO_InitStruct.Mode=GPIO_MODE_AF_PP;              //�����������
			GPIO_InitStruct.Pull=GPIO_PULLUP;                  //����
			GPIO_InitStruct.Speed=GPIO_SPEED_FAST;             //����            
			GPIO_InitStruct.Alternate=GPIO_AF5_SPI5;           //����ΪSPI5
		    HAL_GPIO_Init(GPIOF,&GPIO_InitStruct);
		}	
		else if(hspi->Instance==SPI6){
			hspi6 = *hspi;
		}
}

//SPI�ٶ����ú���
//SPI�ٶ�=fAPB1/��Ƶϵ��
//@ref SPI_BaudRate_Prescaler:SPI_BAUDRATEPRESCALER_2~SPI_BAUDRATEPRESCALER_2 256
//fAPB1ʱ��һ��Ϊ45Mhz��
void SPI_SetSpeed(SPI_TypeDef *SPI,u8 SPI_BaudRatePrescaler)
{
	SPI_Handler.Instance=SPI;
	assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_BaudRatePrescaler));//�ж���Ч��
	__HAL_SPI_DISABLE(&SPI_Handler);            //�ر�SPI
	SPI_Handler.Instance->CR1&=0XFFC7;          //λ3-5���㣬�������ò�����
	SPI_Handler.Instance->CR1|=SPI_BaudRatePrescaler;//����SPI�ٶ�
	__HAL_SPI_ENABLE(&SPI_Handler);             //ʹ��SPI
}


//SPI5 ��дһ���ֽ�
//TxData:Ҫд����ֽ�
//����ֵ:��ȡ�����ֽ�
u8 SPI_ReadWriteByte(SPI_TypeDef *SPI,u8 TxData)
{
	SPI_Handler.Instance=SPI;
	u8 Rxdata;
	HAL_SPI_TransmitReceive(&SPI_Handler,&TxData,&Rxdata,1, 1000);       
 	return Rxdata;          		    //�����յ�������		
}
