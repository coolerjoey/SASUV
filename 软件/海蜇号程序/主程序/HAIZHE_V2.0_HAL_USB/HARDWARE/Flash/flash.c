#include "flash.h"
#include "global_para.h"
#include "parameter.h"
#include "string.h"
#include "delay.h"
#include "led.h"

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
		printf("[flash] save %d th parameter \r\n",paralist);
	#endif
}



/*flash保存参数  ->  需要
功能：1.板子第一次初始化时写入默认参数,第一次读出来的数是27750(即0xffffffff的int值？)
		  2.当前spi flash记录的参数个数和列表中的不同，则表明修改了列表，需要把全部参数重新写入->更新：使用下位机命令来覆盖flash
注意：重新写入的为默认参数，所以在修改参数列表前先修改默认参数！！！
*/
void flash_write_core(){
	param_default_init();	//还原当前参数
	//修改flash第一个地址记录的数据
	printf("[OK] Parameters To_Be saved nums: %d \r\n",fp.param_num);
	for(int i=0;i<fp.param_num;i++){
		flash_write_param((u8*)param_id_type_value[i].param_value,param_id_type_value[i].param_list);
		led_be = !led_be;
	}
	printf("[OK] Parameters Save Finish \r\n");
}

void flash_parameter_write(){
	u16 _param_num = 0;
	memcpy(&_param_num,flash_read_param(param_num),4);
	printf("[OK] Current parameters saved : %d \r\n",(u16)fp.param_num);
	if(_param_num > PARAM_SAVED_MAX || _param_num != fp.param_num){	//初次上电(一般读取出的param_num相当大)或者参数改变时
		flash_write_core();
		return;
	}
	printf("[OK] Dont't need to Resave Parameters \r\n");
}
	
//读取flash保存的系统参数
void flash_parameter_read(){
	printf("[OK] Load Parameters From SPI Flash \r\n");
	for(int i=0;i<fp.param_num;i++){
		memcpy(param_id_type_value[i].param_value,flash_read_param(param_id_type_value[i].param_list),4);
	}	
	printf("[OK] Load Parameters Finish \r\n");
}


