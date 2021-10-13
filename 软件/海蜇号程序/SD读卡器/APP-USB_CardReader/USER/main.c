#include "haizhe.h"
	
extern vu8 USB_STATUS_REG;		//USB状态
extern vu8 bDeviceState;		//USB连接 情况

int main(void)
{		
	u8 offline_cnt=0;
	u8 tct=0;
	u8 USB_STA;
	u8 Divece_STA;
	system_init();
	init_haizhe();//设备初始化
	while(1){
	 delay_ms(1);				  
		if(USB_STA!=USB_STATUS_REG)//状态改变了 
		{	 						   
			if(USB_STATUS_REG&0x01)//正在写		  
			{
				led_act=0;
//				printf("USB Writing... \r\n");//提示USB正在写入数据	 
			}
			if(USB_STATUS_REG&0x02)//正在读
			{
				led_act=0;
//				printf("USB Reading... \r\n");//提示USB正在读出数据  		 
			}	 										  
			if(USB_STATUS_REG&0x04)printf("USB Write Err \r\n");//提示写入错误
			if(USB_STATUS_REG&0x08)printf("USB Read  Err \r\n");//提示读出错误
			USB_STA=USB_STATUS_REG;//记录最后的状态
		}
		if(Divece_STA!=bDeviceState) 
		{
			if(bDeviceState==1)printf("USB Connected   \r\n ");//提示USB连接已经建立
			else printf("USB DisConnected \r\n");//提示USB被拔出了
			Divece_STA=bDeviceState;
		}
		tct++;
		if(tct==200)
		{
			tct=0;
			led_act=1;
			led_be=!led_be;//提示系统在运行
			if(USB_STATUS_REG&0x10)
			{
				offline_cnt=0;//USB连接了,则清除offline计数器
				bDeviceState=1;
			}else//没有得到轮询 
			{
				offline_cnt++;  
				if(offline_cnt>10)bDeviceState=0;//2s内没收到在线标记,代表USB被拔出了
			}
			USB_STATUS_REG=0;
		} 
	}

}

								    
