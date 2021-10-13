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
		printf("[flash] save %d th parameter \r\n",paralist);
	#endif
}



/*flash�������  ->  ��Ҫ
���ܣ�1.���ӵ�һ�γ�ʼ��ʱд��Ĭ�ϲ���,��һ�ζ�����������27750(��0xffffffff��intֵ��)
		  2.��ǰspi flash��¼�Ĳ����������б��еĲ�ͬ��������޸����б���Ҫ��ȫ����������д��->���£�ʹ����λ������������flash
ע�⣺����д���ΪĬ�ϲ������������޸Ĳ����б�ǰ���޸�Ĭ�ϲ���������
*/
void flash_write_core(){
	param_default_init();	//��ԭ��ǰ����
	//�޸�flash��һ����ַ��¼������
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
	if(_param_num > PARAM_SAVED_MAX || _param_num != fp.param_num){	//�����ϵ�(һ���ȡ����param_num�൱��)���߲����ı�ʱ
		flash_write_core();
		return;
	}
	printf("[OK] Dont't need to Resave Parameters \r\n");
}
	
//��ȡflash�����ϵͳ����
void flash_parameter_read(){
	printf("[OK] Load Parameters From SPI Flash \r\n");
	for(int i=0;i<fp.param_num;i++){
		memcpy(param_id_type_value[i].param_value,flash_read_param(param_id_type_value[i].param_list),4);
	}	
	printf("[OK] Load Parameters Finish \r\n");
}


