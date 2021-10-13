#include "flash.h"
#include "string.h"

void flash_ext_init(){
#if FLASHTYPE==W25Q_xx
		W25QXX_Init();

//#elif
	
#endif	
}

//��ȡSPI FLASH  
//��ָ����ַ��ʼ��ȡָ�����ȵ�����
//pBuffer:���ݴ洢��
//ReadAddr:��ʼ��ȡ�ĵ�ַ(24bit),����ʼ��ȡ�ĵڼ����ֽ���
//NumByteToRead:Ҫ��ȡ���ֽ���(���65535)
void flash_read(u8* pBuffer,u32 ReadAddr,u16 NumByteToRead){
#if FLASHTYPE==W25Q_xx
		W25QXX_Read(pBuffer,ReadAddr,NumByteToRead);
//#elif
	
#endif
}

//дSPI FLASH  
//��ָ����ַ��ʼд��ָ�����ȵ�����
//�ú�������������!
//pBuffer:���ݴ洢��
//WriteAddr:��ʼд��ĵ�ַ(24bit)						
//NumByteToWrite:Ҫд����ֽ���(���65535)    
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


