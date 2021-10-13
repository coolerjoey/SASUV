#include "flash_w25qxx.h"
#include "spi.h"
#include "delay.h"
#include "stm32f4xx_hal_gpio.h"
#include "global_para.h"

u16 W25QXX_TYPE;	//Ĭ����W25Q256

//4KbytesΪһ������Sector
//16������Ϊ1��Block
//W25Q256 ����Ϊ256Mbits,32Mbytes,����512��Block,8192��Sector 
//��С������λΪ1�������������ÿ�������洢һ�����������ɴ�8192������

//��ʼ��SPI FLASH��IO��
void W25QXX_Init(void)
{ 
	u8 temp;
	GPIO_InitTypeDef GPIO_Initure;
	
	W25QXX_CS_CLK_ENABLE();           
	
	//PF6
	GPIO_Initure.Pin=W25QXX_CS_PIN;            //PF6
	GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;  //�������
	GPIO_Initure.Pull=GPIO_PULLUP;          //����
	GPIO_Initure.Speed=GPIO_SPEED_FAST;     //����         
	HAL_GPIO_Init(W25QXX_CS_GPIO,&GPIO_Initure);     //��ʼ��
    
	W25QXX_CS=1;			                //SPI FLASH��ѡ��
	W25QXX_SPI_init(); //��ʼ��SPI
	W25QXX_SPI_SetSpeed(SPI_BAUDRATEPRESCALER_2); //����Ϊ45Mʱ��,����ģʽ
	W25QXX_TYPE=W25QXX_ReadID();	        //��ȡFLASH ID.
	// W25Q256�������ͺ���32λ��ַ�������ͺ�ֻ��24λ��ַ
	//һλ��ַָ��1byte�Ĵ洢�ռ�
	//24λ��ַ��2^24byte=16*1024*1024byte = 16Mbytes
	switch(W25QXX_TYPE){
		case W25Q256: //SPI FLASHΪW25Q256
			printf("[OK] SPI Flash W25Q2xx Detected  \r\n");
			temp=W25QXX_ReadSR(3);              //��ȡ״̬�Ĵ���3���жϵ�ַģʽ
			if((temp&0X01)==0)			        //�������4�ֽڵ�ַģʽ,�����4�ֽڵ�ַģʽ
			{
				W25QXX_CS=0; 			        //ѡ��
				W25QXX_SPI_ReadWriteByte(W25X_Enable4ByteAddr);//���ͽ���4�ֽڵ�ַģʽָ��   
				W25QXX_CS=1;       		        //ȡ��Ƭѡ   
			}
			sys_flag.flash_check=1;
			break;
		//�����ͺŲ���Ҫ������
		case W25Q128:
		case W25Q64:
		case W25Q32:
			sys_flag.flash_check=1;
			printf("[OK] SPI Flash W25Q1xx Detected \r\n");
			break;
		default:
			sys_flag.flash_check=0;
			printf("[ERROR] SPI Flash Undetected!  \r\n");
			break;
	}
		
}  

//��ȡW25QXX��״̬�Ĵ�����W25QXXһ����3��״̬�Ĵ���
//״̬�Ĵ���1��
//BIT7  6   5   4   3   2   1   0
//SPR   RV  TB BP2 BP1 BP0 WEL BUSY
//SPR:Ĭ��0,״̬�Ĵ�������λ,���WPʹ��
//TB,BP2,BP1,BP0:FLASH����д��������
//WEL:дʹ������
//BUSY:æ���λ(1,æ;0,����)
//Ĭ��:0x00
//״̬�Ĵ���2��
//BIT7  6   5   4   3   2   1   0
//SUS   CMP LB3 LB2 LB1 (R) QE  SRP1
//״̬�Ĵ���3��
//BIT7      6    5    4   3   2   1   0
//HOLD/RST  DRV1 DRV0 (R) (R) WPS ADP ADS
//regno:״̬�Ĵ����ţ���:1~3
//����ֵ:״̬�Ĵ���ֵ
u8 W25QXX_ReadSR(u8 regno)   
{  
	u8 byte=0,command=0; 
    switch(regno)
    {
        case 1:
            command=W25X_ReadStatusReg1;    //��״̬�Ĵ���1ָ��
            break;
        case 2:
            command=W25X_ReadStatusReg2;    //��״̬�Ĵ���2ָ��
            break;
        case 3:
            command=W25X_ReadStatusReg3;    //��״̬�Ĵ���3ָ��
            break;
        default:
            command=W25X_ReadStatusReg1;    
            break;
    }    
	W25QXX_CS=0;                            //ʹ������   
	W25QXX_SPI_ReadWriteByte(command);            //���Ͷ�ȡ״̬�Ĵ�������    
	byte=W25QXX_SPI_ReadWriteByte(0Xff);          //��ȡһ���ֽ�  
	W25QXX_CS=1;                            //ȡ��Ƭѡ     
	return byte;   
} 
//дW25QXX״̬�Ĵ���
void W25QXX_Write_SR(u8 regno,u8 sr)   
{   
    u8 command=0;
    switch(regno)
    {
        case 1:
            command=W25X_WriteStatusReg1;    //д״̬�Ĵ���1ָ��
            break;
        case 2:
            command=W25X_WriteStatusReg2;    //д״̬�Ĵ���2ָ��
            break;
        case 3:
            command=W25X_WriteStatusReg3;    //д״̬�Ĵ���3ָ��
            break;
        default:
            command=W25X_WriteStatusReg1;    
            break;
    }   
	W25QXX_CS=0;                            //ʹ������   
	W25QXX_SPI_ReadWriteByte(command);            //����дȡ״̬�Ĵ�������    
	W25QXX_SPI_ReadWriteByte(sr);                 //д��һ���ֽ�  
	W25QXX_CS=1;                            //ȡ��Ƭѡ     	      
}   
//W25QXXдʹ��	
//��WEL��λ   
void W25QXX_Write_Enable(void)   
{
	W25QXX_CS=0;                            //ʹ������   
    W25QXX_SPI_ReadWriteByte(W25X_WriteEnable);   //����дʹ��  
	W25QXX_CS=1;                            //ȡ��Ƭѡ     	      
} 
//W25QXXд��ֹ	
//��WEL����  
void W25QXX_Write_Disable(void)   
{  
	W25QXX_CS=0;                            //ʹ������   
    W25QXX_SPI_ReadWriteByte(W25X_WriteDisable);  //����д��ָֹ��    
	W25QXX_CS=1;                            //ȡ��Ƭѡ     	      
} 

//��ȡоƬID
//����ֵ����:				   
//0XEF13,��ʾоƬ�ͺ�ΪW25Q80  
//0XEF14,��ʾоƬ�ͺ�ΪW25Q16    
//0XEF15,��ʾоƬ�ͺ�ΪW25Q32  
//0XEF16,��ʾоƬ�ͺ�ΪW25Q64 
//0XEF17,��ʾоƬ�ͺ�ΪW25Q128 	  
//0XEF18,��ʾоƬ�ͺ�ΪW25Q256
u16 W25QXX_ReadID(void)
{
	u16 Temp = 0;	  
	W25QXX_CS=0;				    
	W25QXX_SPI_ReadWriteByte(0x90);//���Ͷ�ȡID����	    
	W25QXX_SPI_ReadWriteByte(0x00); 	    
	W25QXX_SPI_ReadWriteByte(0x00); 	    
	W25QXX_SPI_ReadWriteByte(0x00); 	 			   
	Temp|=W25QXX_SPI_ReadWriteByte(0xFF)<<8;  
	Temp|=W25QXX_SPI_ReadWriteByte(0xFF);	 
	W25QXX_CS=1;				    
	return Temp;
}   		    
//��ȡSPI FLASH  
//��ָ����ַ��ʼ��ȡָ�����ȵ�����
//pBuffer:���ݴ洢��
//ReadAddr:��ʼ��ȡ�ĵ�ַ(24bit)
//NumByteToRead:Ҫ��ȡ���ֽ���(���65535)
//ע�⣺���ָ����ַָ���Ĵ洢�ռ仹û��д�����ݣ�����������0xff
void W25QXX_Read(u8* pBuffer,u32 ReadAddr,u16 NumByteToRead)   
{ 
 	u16 i;   										    
	W25QXX_CS=0;                            //ʹ������   
    W25QXX_SPI_ReadWriteByte(W25X_ReadData);      //���Ͷ�ȡ����  
    if(W25QXX_TYPE==W25Q256)                //�����W25Q256�Ļ���ַΪ4�ֽڵģ�Ҫ�������8λ
    {
        W25QXX_SPI_ReadWriteByte((u8)((ReadAddr)>>24));    
    }
    W25QXX_SPI_ReadWriteByte((u8)((ReadAddr)>>16));   //����24bit��ַ    
    W25QXX_SPI_ReadWriteByte((u8)((ReadAddr)>>8));   
    W25QXX_SPI_ReadWriteByte((u8)ReadAddr);   
    for(i=0;i<NumByteToRead;i++)
	{ 
        pBuffer[i]=W25QXX_SPI_ReadWriteByte(0XFF);    //ѭ������  
    }
	W25QXX_CS=1;  				    	      
}  
//SPI��һҳ(0~65535)��д������256���ֽڵ�����
//��ָ����ַ��ʼд�����256�ֽڵ�����
//pBuffer:���ݴ洢��
//WriteAddr:��ʼд��ĵ�ַ(24bit)
//NumByteToWrite:Ҫд����ֽ���(���256),������Ӧ�ó�����ҳ��ʣ���ֽ���!!!	 
void W25QXX_Write_Page(u8* pBuffer,u32 WriteAddr,u16 NumByteToWrite)
{
 	u16 i;  
    W25QXX_Write_Enable();                  //SET WEL 
	W25QXX_CS=0;                            //ʹ������   
    W25QXX_SPI_ReadWriteByte(W25X_PageProgram);   //����дҳ����   
    if(W25QXX_TYPE==W25Q256)                //�����W25Q256�Ļ���ַΪ4�ֽڵģ�Ҫ�������8λ
    {
        W25QXX_SPI_ReadWriteByte((u8)((WriteAddr)>>24)); 
    }
    W25QXX_SPI_ReadWriteByte((u8)((WriteAddr)>>16)); //����24bit��ַ    
    W25QXX_SPI_ReadWriteByte((u8)((WriteAddr)>>8));   
    W25QXX_SPI_ReadWriteByte((u8)WriteAddr);   
    for(i=0;i<NumByteToWrite;i++)W25QXX_SPI_ReadWriteByte(pBuffer[i]);//ѭ��д��  
	W25QXX_CS=1;                            //ȡ��Ƭѡ 
	W25QXX_Wait_Busy();					   //�ȴ�д�����
} 
//�޼���дSPI FLASH 
//����ȷ����д�ĵ�ַ��Χ�ڵ�����ȫ��Ϊ0XFF,�����ڷ�0XFF��д������ݽ�ʧ��!
//�����Զ���ҳ���� 
//��ָ����ַ��ʼд��ָ�����ȵ�����,����Ҫȷ����ַ��Խ��!
//pBuffer:���ݴ洢��
//WriteAddr:��ʼд��ĵ�ַ(24bit)
//NumByteToWrite:Ҫд����ֽ���(���65535)
//CHECK OK
void W25QXX_Write_NoCheck(u8* pBuffer,u32 WriteAddr,u16 NumByteToWrite)   
{ 			 		 
	u16 pageremain;	   
	pageremain=256-WriteAddr%256; //��ҳʣ����ֽ���		 	    
	if(NumByteToWrite<=pageremain)pageremain=NumByteToWrite;//������256���ֽ�
	while(1)
	{	   
		W25QXX_Write_Page(pBuffer,WriteAddr,pageremain);
		if(NumByteToWrite==pageremain)break;//д�������
	 	else //NumByteToWrite>pageremain
		{
			pBuffer+=pageremain;
			WriteAddr+=pageremain;	

			NumByteToWrite-=pageremain;			  //��ȥ�Ѿ�д���˵��ֽ���
			if(NumByteToWrite>256)pageremain=256; //һ�ο���д��256���ֽ�
			else pageremain=NumByteToWrite; 	  //����256���ֽ���
		}
	};	    
} 
//дSPI FLASH  
//��ָ����ַ��ʼд��ָ�����ȵ�����
//�ú�������������!
//pBuffer:���ݴ洢��
//WriteAddr:��ʼд��ĵ�ַ(24bit)						
//NumByteToWrite:Ҫд����ֽ���(���65535)   
u8 W25QXX_BUFFER[4096];		 
void W25QXX_Write(u8* pBuffer,u32 WriteAddr,u16 NumByteToWrite)   
{ 
	u32 secpos;
	u16 secoff;
	u16 secremain;	   
 	u16 i;    
	u8 * W25QXX_BUF;	  
   	W25QXX_BUF=W25QXX_BUFFER;	  
	/*
														
															[ secremain ]
	...|_____sector1_____|__sector2_________|...
											 /		 /
										scepos	/
													secoff	
				
	*/	
 	secpos=WriteAddr/4096;//������ַ  
	secoff=WriteAddr%4096;//�������ڵ�ƫ��
	secremain=4096-secoff;//����ʣ��ռ��С   
 	//printf("ad:%X,nb:%X\r\n",WriteAddr,NumByteToWrite);//������
 	if(NumByteToWrite<=secremain)secremain=NumByteToWrite;//������4096���ֽ�
	while(1) 
	{	
		W25QXX_Read(W25QXX_BUF,secpos*4096,4096);//������������������
		for(i=0;i<secremain;i++)//У������
		{
			if(W25QXX_BUF[secoff+i]!=0XFF)break;//Ĭ�ϱ���0xff�������Ҫд��Ŀռ����Ѿ�������д�룬�Ȳ�����������  	  
		}
		if(i<secremain)//��Ҫ����
		{
			W25QXX_Erase_Sector(secpos);//�����������
			for(i=0;i<secremain;i++)	   //����
			{
				W25QXX_BUF[i+secoff]=pBuffer[i];	  
			}
			W25QXX_Write_NoCheck(W25QXX_BUF,secpos*4096,4096);//д����������  

		}
		else W25QXX_Write_NoCheck(pBuffer,WriteAddr,secremain);//д�Ѿ������˵�,ֱ��д������ʣ������. 				   
		if(NumByteToWrite==secremain)break;//д�������
		else//д��δ����	//Ҫд����ֽ�������Ҫд�������ڲ�ʣ��ռ�Ĵ�С
		{
			secpos++;//������ַ��1
			secoff=0;//ƫ��λ��Ϊ0 	 

		   	pBuffer+=secremain;  //ָ��ƫ��
			WriteAddr+=secremain;//д��ַƫ��	   
		   	NumByteToWrite-=secremain;				//�ֽ����ݼ�
			if(NumByteToWrite>4096)secremain=4096;	//��һ����������д����
			else secremain=NumByteToWrite;			//��һ����������д����
		}	 
	};	 
}
//��������оƬ		  
//�ȴ�ʱ�䳬��...
void W25QXX_Erase_Chip(void)   
{                                   
    W25QXX_Write_Enable();                  //SET WEL 
    W25QXX_Wait_Busy();   
  	W25QXX_CS=0;                            //ʹ������   
    W25QXX_SPI_ReadWriteByte(W25X_ChipErase);        //����Ƭ��������  
	W25QXX_CS=1;                            //ȡ��Ƭѡ     	      
	W25QXX_Wait_Busy();   				   //�ȴ�оƬ��������
}   
//����һ������
//Dst_Addr:������ַ ����ʵ����������
//����һ������������ʱ��:150ms
void W25QXX_Erase_Sector(u32 Dst_Addr)   
{  
	//����falsh�������,������   
 	//printf("fe:%x\r\n",Dst_Addr);	  
 	Dst_Addr*=4096;
    W25QXX_Write_Enable();                  //SET WEL 	 
    W25QXX_Wait_Busy();   
  	W25QXX_CS=0;                            //ʹ������   
    W25QXX_SPI_ReadWriteByte(W25X_SectorErase);   //������������ָ�� 
    if(W25QXX_TYPE==W25Q256)                //�����W25Q256�Ļ���ַΪ4�ֽڵģ�Ҫ�������8λ
    {
        W25QXX_SPI_ReadWriteByte((u8)((Dst_Addr)>>24)); 
    }
    W25QXX_SPI_ReadWriteByte((u8)((Dst_Addr)>>16));  //����24bit��ַ    
    W25QXX_SPI_ReadWriteByte((u8)((Dst_Addr)>>8));   
    W25QXX_SPI_ReadWriteByte((u8)Dst_Addr);  
	W25QXX_CS=1;                            //ȡ��Ƭѡ     	      
    W25QXX_Wait_Busy();   				    //�ȴ��������
}  
//�ȴ�����
void W25QXX_Wait_Busy(void)   
{   
	while((W25QXX_ReadSR(1)&0x01)==0x01);   // �ȴ�BUSYλ���
}  
//�������ģʽ
void W25QXX_PowerDown(void)   
{ 
  	W25QXX_CS=0;                            //ʹ������   
    W25QXX_SPI_ReadWriteByte(W25X_PowerDown);     //���͵�������  
	W25QXX_CS=1;                            //ȡ��Ƭѡ     	      
    delay_us(3);                            //�ȴ�TPD  
}   
//����
void W25QXX_WAKEUP(void)   
{  
  	W25QXX_CS=0;                                //ʹ������   
    W25QXX_SPI_ReadWriteByte(W25X_ReleasePowerDown);  //  send W25X_PowerDown command 0xAB    
	W25QXX_CS=1;                                //ȡ��Ƭѡ     	      
    delay_us(3);                                //�ȴ�TRES1
}   
