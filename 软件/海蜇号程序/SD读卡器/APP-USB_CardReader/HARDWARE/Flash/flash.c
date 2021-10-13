#include "flash.h"
#include "string.h"

void flash_ext_init(){
#if FLASHTYPE==W25Q_xx
		W25QXX_Init();

//#elif
	
#endif	
}

//读取SPI FLASH  
//在指定地址开始读取指定长度的数据
//pBuffer:数据存储区
//ReadAddr:开始读取的地址(24bit),即开始读取的第几个字节数
//NumByteToRead:要读取的字节数(最大65535)
void flash_read(u8* pBuffer,u32 ReadAddr,u16 NumByteToRead){
#if FLASHTYPE==W25Q_xx
		W25QXX_Read(pBuffer,ReadAddr,NumByteToRead);
//#elif
	
#endif
}

//写SPI FLASH  
//在指定地址开始写入指定长度的数据
//该函数带擦除操作!
//pBuffer:数据存储区
//WriteAddr:开始写入的地址(24bit)						
//NumByteToWrite:要写入的字节数(最大65535)    
void flash_write(u8* pBuffer,u32 WriteAddr,u16 NumByteToWrite){
#if FLASHTYPE==W25Q_xx
		W25QXX_Write(pBuffer,WriteAddr,NumByteToWrite);
//#elif
	
#endif

}

		
u8 data[4];
u8* flash_read_param(flash_para_list paralist){
#if FLASHTYPE==W25Q_xx
	flash_read(data,paralist*W25QXX_Sector_Size,4);
#endif
	return data;
}

void flash_write_param(u8* param_value,flash_para_list paralist){
	#if FLASHTYPE==W25Q_xx
		flash_write(param_value,paralist*W25QXX_Sector_Size,4);
	#endif
}


