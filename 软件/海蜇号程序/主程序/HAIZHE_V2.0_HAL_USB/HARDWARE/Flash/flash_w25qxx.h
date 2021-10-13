#ifndef __W25QXX_H
#define __W25QXX_H
#include "sys.h"

/*
W25Q128将16M的容量分为256个块（Block），每个块大小为64K字节，
每个块又分为16个扇区（Sector），每个扇区4K个字节。
W25Q128的最小擦除单位为一个扇区，也就是每次必须擦除4K个字节。
这样我们需要给W25Q128开辟一个至少4K的缓存区，
这样对SRAM要求比较高，要求芯片必须有4K以上SRAM才能很好的操作。

W25Q256及以上型号有32位地址，其他型号只有24位地址
因为 32Mbytes=2^25bytes	16Mbytes=2^24bytes
*/

//W25X系列/Q系列芯片列表	   
//W25Q80  ID  0XEF13
//W25Q16  ID  0XEF14
//W25Q32  ID  0XEF15
//W25Q64  ID  0XEF16	
//W25Q128 ID  0XEF17	
//W25Q256 ID  0XEF18
#define W25Q80 	0XEF13 	
#define W25Q16 	0XEF14
#define W25Q32 	0XEF15
#define W25Q64 	0XEF16
#define W25Q128	0XEF17	//128Mbits=16Mbytes
#define W25Q256 0XEF18	

#define W25QXX_Sector_Size 4*1024	//扇区大小4Kbytes

extern u16 W25QXX_TYPE;					//定义W25QXX芯片型号		   

#define W25Q_USE W25Q128
#define W25QXX_SPI_init() SPI_Init(SPI2)
#define W25QXX_SPI_SetSpeed(baudrate)	SPI_SetSpeed(SPI2,baudrate)
#define W25QXX_SPI_ReadWriteByte(data) SPI_ReadWriteByte(SPI2,data)
#define	W25QXX_CS_CLK_ENABLE() __HAL_RCC_GPIOC_CLK_ENABLE();           //使能GPIOF时钟
#define	W25QXX_CS_GPIO		GPIOC
#define	W25QXX_CS_PIN 		GPIO_PIN_4 
#define	W25QXX_CS 				PCout(4)  		//W25QXX的片选信号

////////////////////////////////////////////////////////////////////////////////// 
//指令表
#define W25X_WriteEnable		0x06 
#define W25X_WriteDisable		0x04 
#define W25X_ReadStatusReg1		0x05 
#define W25X_ReadStatusReg2		0x35 
#define W25X_ReadStatusReg3		0x15 
#define W25X_WriteStatusReg1    0x01 
#define W25X_WriteStatusReg2    0x31 
#define W25X_WriteStatusReg3    0x11 
#define W25X_ReadData			0x03 
#define W25X_FastReadData		0x0B 
#define W25X_FastReadDual		0x3B 
#define W25X_PageProgram		0x02 
#define W25X_BlockErase			0xD8 
#define W25X_SectorErase		0x20 
#define W25X_ChipErase			0xC7 
#define W25X_PowerDown			0xB9 
#define W25X_ReleasePowerDown	0xAB 
#define W25X_DeviceID			0xAB 
#define W25X_ManufactDeviceID	0x90 
#define W25X_JedecDeviceID		0x9F 
#define W25X_Enable4ByteAddr    0xB7
#define W25X_Exit4ByteAddr      0xE9

void W25QXX_Init(void);
u16  W25QXX_ReadID(void);  	    		//读取FLASH ID
u8 W25QXX_ReadSR(u8 regno);             //读取状态寄存器 
void W25QXX_4ByteAddr_Enable(void);     //使能4字节地址模式
void W25QXX_Write_SR(u8 regno,u8 sr);   //写状态寄存器
void W25QXX_Write_Enable(void);  		//写使能 
void W25QXX_Write_Disable(void);		//写保护
void W25QXX_Write_NoCheck(u8* pBuffer,u32 WriteAddr,u16 NumByteToWrite);
void W25QXX_Read(u8* pBuffer,u32 ReadAddr,u16 NumByteToRead);   //读取flash
void W25QXX_Write(u8* pBuffer,u32 WriteAddr,u16 NumByteToWrite);//写入flash
void W25QXX_Erase_Chip(void);    	  	//整片擦除
void W25QXX_Erase_Sector(u32 Dst_Addr);	//扇区擦除
void W25QXX_Wait_Busy(void);           	//等待空闲
void W25QXX_PowerDown(void);        	//进入掉电模式
void W25QXX_WAKEUP(void);				//唤醒

#endif
