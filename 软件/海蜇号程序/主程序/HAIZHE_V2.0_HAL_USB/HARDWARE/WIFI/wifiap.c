#include "common.h"

//ATK-ESP8266 WIFI AP测试
//用于测试TCP/UDP连接
//返回值:0,正常
//    其他,错误代码
u8 atk_8266_wifiap_test(void)
{
	u8 timex=0; 
	u8 index;
	u8 i=0;
	u8 ipbuf[16]; 	//IP缓存
	u8 *p;
	u16 t=999;		//加速第一次获取链接状态
	u8 res=0;
	u16 rlen=0;
	u8 constate=0;	//连接状态
	p=mymalloc(SRAMIN,32);							//申请32字节内存
	
PRESTA:
//	netpro=atk_8266_netpro_sel(50,30,(u8*)ATK_ESP8266_CWMODE_TBL[1]);	//选择网络模式

	//TCP Server 配置
	atk_8266_send_cmd("AT+CIPMUX=1","OK",20);   //0：单连接，1：多连接
	sprintf((char*)p,"AT+CIPSERVER=1,%s",(u8*)portnum);
	atk_8266_send_cmd(p,"OK",20);     //开启Server模式，端口号为8086
		
	atk_8266_get_wanip(ipbuf);//服务器模式,获取WAN IP
	sprintf((char*)p,"IP地址:%s 端口:%s \r\n",ipbuf,(u8*)portnum);
	printf(p);				//显示IP地址和端口	

	USART2_RX_STA=0;
	while(1)
	{
		t++;
		delay_ms(10);
		if(USART2_RX_STA&0X8000)		//接收到一次数据了
		{ 
			rlen=USART2_RX_STA&0X7FFF;	//得到本次接收到的数据长度
			USART2_RX_BUF[rlen]=0;		//添加结束符 
			if(USART2_RX_BUF[2]=='+'){   //防止WiFi连接和断开时误接收
				if(rlen>20) index = 10;//当接收到10个以上字符时，包括回车换行符
				else index = 9;
				rlen -= index; //真实接受字节数
				p = &USART2_RX_BUF[++index];
				while(*p++ != '\0'){
					USART2_RX_BUF[i++] = *p;
				}
				i =0;
				sprintf((char*)p,"收到%d字节,内容如下:\r\n",rlen-2);//接收到的字节数,还要除去\r\n
				printf(p); 			//显示接收到的数据长度	
				printf("%s",USART2_RX_BUF);	//发送到串口
				printf("\r\n");	
			}			 
			USART2_RX_STA=0;
			if(constate!='+')t=1000;		//状态为还未连接,立即更新连接状态
			else t=0;                   //状态为已经连接了,10秒后再检查
		}  
		if(t==1000)//连续10秒钟没有收到任何数据,检查连接是不是还存在.
		{
			constate=atk_8266_consta_check();//得到连接状态
			if(constate=='+')printf("设备连接成功,等待接收数据 \r\n");  //连接状态
			else printf("无设备连接 \r\n"); 	 
			t=0;
		}
		if((t%20)==0)LED=!LED;
		atk_8266_at_response(1);
		
	}
	myfree(SRAMIN,p);		//释放内存 
	return res;		
} 







