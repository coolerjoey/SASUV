#include "haizhe.h"
	
extern vu8 USB_STATUS_REG;		//USB״̬
extern vu8 bDeviceState;		//USB���� ���

int main(void)
{		
	u8 offline_cnt=0;
	u8 tct=0;
	u8 USB_STA;
	u8 Divece_STA;
	system_init();
	init_haizhe();//�豸��ʼ��
	while(1){
	 delay_ms(1);				  
		if(USB_STA!=USB_STATUS_REG)//״̬�ı��� 
		{	 						   
			if(USB_STATUS_REG&0x01)//����д		  
			{
				led_act=0;
//				printf("USB Writing... \r\n");//��ʾUSB����д������	 
			}
			if(USB_STATUS_REG&0x02)//���ڶ�
			{
				led_act=0;
//				printf("USB Reading... \r\n");//��ʾUSB���ڶ�������  		 
			}	 										  
			if(USB_STATUS_REG&0x04)printf("USB Write Err \r\n");//��ʾд�����
			if(USB_STATUS_REG&0x08)printf("USB Read  Err \r\n");//��ʾ��������
			USB_STA=USB_STATUS_REG;//��¼����״̬
		}
		if(Divece_STA!=bDeviceState) 
		{
			if(bDeviceState==1)printf("USB Connected   \r\n ");//��ʾUSB�����Ѿ�����
			else printf("USB DisConnected \r\n");//��ʾUSB���γ���
			Divece_STA=bDeviceState;
		}
		tct++;
		if(tct==200)
		{
			tct=0;
			led_act=1;
			led_be=!led_be;//��ʾϵͳ������
			if(USB_STATUS_REG&0x10)
			{
				offline_cnt=0;//USB������,�����offline������
				bDeviceState=1;
			}else//û�еõ���ѯ 
			{
				offline_cnt++;  
				if(offline_cnt>10)bDeviceState=0;//2s��û�յ����߱��,����USB���γ���
			}
			USB_STATUS_REG=0;
		} 
	}

}

								    
