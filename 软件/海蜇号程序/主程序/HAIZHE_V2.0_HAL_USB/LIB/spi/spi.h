#ifndef __SPI_H
#define __SPI_H
#include "sys.h" 	

extern SPI_HandleTypeDef SPI_Handler;  //SPI¾ä±ú

void SPI_Init(SPI_TypeDef *SPI);
u8 SPI_ReadWriteByte(SPI_TypeDef *SPI,u8 TxData);
void SPI_SetSpeed(SPI_TypeDef *SPI,u8 SPI_BaudRatePrescaler);

#endif
