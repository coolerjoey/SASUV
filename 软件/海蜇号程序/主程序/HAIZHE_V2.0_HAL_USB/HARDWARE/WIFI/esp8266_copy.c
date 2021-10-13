
/*
esp8266配置为AP+tcp server模式
*/
#include "esp8266.h"

/////////////////////////////////////////////////////////////////////////////////////////////////////////// 
//用户配置区

//连接端口号:8086,可自行修改为其他端口.
const u8* portnum="8086";		 

//WIFI AP模式,模块对外的无线参数,可自行修改.
const u8* wifiap_ssid="ATK-ESP8266";			//对外SSID号
const u8* wifiap_encryption="wpawpa2_aes";	//wpa/wpa2 aes加密方式
const u8* wifiap_password="12345678"; 		//连接密码 

/////////////////////////////////////////////////////////////////////////////////////////////////////////// 
//4个网络模式
const u8 *ATK_ESP8266_CWMODE_TBL[3]={"STA模式 ","AP模式 ","AP&STA模式 "};	//ATK-ESP8266,3种网络模式,默认为路由器(ROUTER)模式 
//4种工作模式
const u8 *ATK_ESP8266_WORKMODE_TBL[3]={"TCP服务器","TCP客户端"," UDP 模式"};	//ATK-ESP8266,4种工作模式
//5种加密方式
const u8 *ATK_ESP8266_ECN_TBL[5]={"OPEN","WEP","WPA_PSK","WPA2_PSK","WPA_WAP2_PSK"};
/////////////////////////////////////////////////////////////////////////////////////////////////////////// 

//usmart支持部分
//将收到的AT指令应答数据返回给电脑串口
//mode:0,不清零USART2_RX_STA;
//     1,清零USART2_RX_STA;
void atk_8266_at_response(u8 mode)
{
	if(USART2_RX_STA&0X8000)		//接收到一次数据了
	{ 
		USART2_RX_BUF[USART2_RX_STA&0X7FFF]=0;//添加结束符
//		printf("%s",USART2_RX_BUF);	//发送到串口
		if(mode)USART2_RX_STA=0;
	} 
}
//ATK-ESP8266发送命令后,检测接收到的应答
//str:期待的应答结果
//返回值:0,没有得到期待的应答结果
//    其他,期待应答结果的位置(str的位置)
u8* atk_8266_check_cmd(u8 *str)
{
	
	char *strx=0;
	if(USART2_RX_STA&0X8000)		//接收到一次数据了
	{ 
		USART2_RX_BUF[USART2_RX_STA&0X7FFF]=0;//添加结束符
		strx=strstr((const char*)USART2_RX_BUF,(const char*)str);
//		USART2_RX_STA =0;
	} 
	return (u8*)strx;
}
//向ATK-ESP8266发送命令
//cmd:发送的命令字符串
//ack:期待的应答结果,如果为空,则表示不需要等待应答
//waittime:等待时间(单位:10ms)
//返回值:0,发送成功(得到了期待的应答结果)
//       1,发送失败
u8 atk_8266_send_cmd(u8 *cmd,u8 *ack,u16 waittime)
{
	u8 res=0; 
	USART2_RX_STA=0;
	u2_printf("%s\r\n",cmd);	//发送命令
	if(ack&&waittime)		//需要等待应答
	{
		while(--waittime)	//等待倒计时
		{
			delay_ms(10);
			if(USART2_RX_STA&0X8000)//接收到期待的应答结果
			{
				if(atk_8266_check_cmd(ack))
				{
//					printf("ack:%s\r\n",(u8*)ack);
					break;//得到有效数据 
				}
					USART2_RX_STA=0;
			} 
		}
		if(waittime==0)res=1; 
	}
	return res;
} 
//向ATK-ESP8266发送指定数据
//data:发送的数据(不需要添加回车了)
//ack:期待的应答结果,如果为空,则表示不需要等待应答
//waittime:等待时间(单位:10ms)
//返回值:0,发送成功(得到了期待的应答结果)luojian
u8 atk_8266_send_data(u8 *data,u8 *ack,u16 waittime)
{
	u8 res=0; 
	USART2_RX_STA=0;
	u2_printf("%s",data);	//发送命令
	if(ack&&waittime)		//需要等待应答
	{
		while(--waittime)	//等待倒计时
		{
			delay_ms(10);
			if(USART2_RX_STA&0X8000)//接收到期待的应答结果
			{
				if(atk_8266_check_cmd(ack))break;//得到有效数据 
				USART2_RX_STA=0;
			} 
		}
		if(waittime==0)res=1; 
	}
	return res;
}
//ATK-ESP8266退出透传模式
//返回值:0,退出成功;
//       1,退出失败
u8 atk_8266_quit_trans(void)
{
	while((USART2->SR&0X40)==0);	//等待发送空
	USART2->DR='+';      
	delay_ms(15);					//大于串口组帧时间(10ms)
	while((USART2->SR&0X40)==0);	//等待发送空
	USART2->DR='+';      
	delay_ms(15);					//大于串口组帧时间(10ms)
	while((USART2->SR&0X40)==0);	//等待发送空
	USART2->DR='+';      
	delay_ms(500);					//等待500ms
	return atk_8266_send_cmd("AT","OK",20);//退出透传判断.
}

//获取ATK-ESP8266模块的连接状态
//返回值:0,未连接;1,连接成功.
u8 atk_8266_consta_check(void)
{
	u8 *p;
	u8 res;
	if(atk_8266_quit_trans())return 0;			//退出透传 
	atk_8266_send_cmd("AT+CIPSTATUS",":",50);	//发送AT+CIPSTATUS指令,查询连接状态
	p=atk_8266_check_cmd("+CIPSTATUS:"); 
	res=*p;									//得到连接状态	
	return res;
}

//获取server ip地址
//ipbuf:ip地址输出缓存区
void atk_8266_get_wanip(u8* ipbuf)
{
	u8 *p,*p1;
		if(atk_8266_send_cmd("AT+CIFSR","OK",50))//获取WAN IP地址失败
		{
			ipbuf[0]=0;
			return;
		}		
		p=atk_8266_check_cmd("\"");
		p1=(u8*)strstr((const char*)(p+1),"\"");
		*p1=0;
		sprintf((char*)ipbuf,"%s",p+1);	
}

//ATK-ESP8266模块测试主函数
void atk_8266_test(void)
{
	u8 timex;

	while(1){
		delay_ms(10); 
		atk_8266_at_response(1);//检查ATK-ESP8266模块发送过来的数据,及时上传给电脑

		printf("ATK-ESP WIFI-AP 测试 \r\n"); 
		printf("正在配置ATK-ESP模块,请稍等...\r\n");
		atk_8266_wifiap_mode();	//WIFI AP测试
	 
		if((timex%20)==0)LED=!LED;//200ms闪烁 
		timex++;	 
	} 
}


//ATK-ESP8266 WIFI AP测试
//用于测试TCP/UDP连接
//返回值:0,正常
//    其他,错误代码
u8 atk_8266_wifiap_mode(void){
	u8 ipbuf[16]; 	//IP缓存
	u8 *p;
	u8 res=0;

	p=mymalloc(SRAMIN,32);							//申请32字节内存
	//TCP Server 配置
	atk_8266_send_cmd("AT+CIPMUX=1","OK",20);   //0：单连接，1：多连接
	sprintf((char*)p,"AT+CIPSERVER=1,%s",(u8*)portnum);
	atk_8266_send_cmd(p,"OK",20);     //开启Server模式，端口号为8086
		
	atk_8266_get_wanip(ipbuf);//服务器模式,获取WAN IP
	sprintf((char*)p,"IP地址:%s 端口:%s \r\n",ipbuf,(u8*)portnum);
	printf(p);				//显示IP地址和端口	

	USART2_RX_STA=0;

	myfree(SRAMIN,p);		//释放内存 
	return res;		
}


u16 t=999;		//加速第一次获取链接状态，需作为全局变量，否则每次循环调用清零
u8 set_flag; //手机app参数设定标志位
u8 singlerun_flag; //测试运行标志
u8 taskrun_flag;  //任务运行标志
float depth,pitch,roll,yaw;
float depth_copy,pitch_copy,roll_copy,yaw_copy;
u16 runtime; //接收app设定运行时间
u8 message[100],move[20],run[10];

//APP通过WiFi发送数据回显	
uint8_t WiFi_echo(){  
	u8 index;
	u8 i=0;
	u8 *p;	
	u8 res=0;
	u16 rlen=0;
	u8 constate=0;	//连接状态
		
	t++;
	delay_ms(10);
	if(USART2_RX_STA&0X8000){		//接收到一次数据了
	 
		rlen=USART2_RX_STA&0X7FFF;	//得到本次接收到的数据长度
		USART2_RX_BUF[rlen]=0;		//添加结束符 
		if(USART2_RX_BUF[2]=='+'){   //防止WiFi连接和断开时误接收
			res =1;  //接收到WiFi数据
			if(rlen>20) index = 10;//当接收到10个以上字符时，包括回车换行符
			else index = 9;
			rlen -= index; //真实接受字节数
			p = &USART2_RX_BUF[index+1];  
			while(*p++ != '\0'){  //删除多余部分，得到真实接收字符串
				USART2_RX_BUF[i++] = *p;
			}
			i =0;
			sprintf((char*)p,"收到%d字节,内容如下:\r\n",rlen-2);//接收到的字节数,还要除去\r\n
			printf(p); 			//显示接收到的数据长度	
			printf("%s",USART2_RX_BUF);	//发送到串口
			printf("\r\n");	
			
			switch (USART2_RX_BUF[0]){  //WiFi收到信息数据解析
				case 's':set_flag=ENABLE;
								sscanf((char *)&USART2_RX_BUF[1],"%f %f %f %f %d ",&depth,&pitch,&roll,&yaw,&runtime);
								depth_copy=depth;pitch_copy=pitch;roll_copy=roll;yaw_copy=yaw; //保留副本
								sprintf(message,"Set目标: rol:%.1f pit:%.1f yaw:%.1f dep:%.1f time:%d \r\n",roll,pitch,yaw,depth,runtime);		
								printf(message); 
								break;
				case 'K':printf("3 \r\n");break;   //bootloader跳转到主程序
				case 'k':printf("E 1 \r\n");break;	//开继电器
				case 'g':printf("E 0 \r\n");break;	//关继电器
				case 'f':printf("E 2 \r\n");break;	//复位
			 default:if(set_flag==DISABLE)
									printf("请先进行参数设定 \r\n");
								break ;
			}
			if(set_flag==ENABLE){  //app参数设定完成
				switch (USART2_RX_BUF[0]){
					case 'S':singlerun_flag=ENABLE;taskrun_flag=DISABLE;roll=roll_copy;pitch=pitch_copy;yaw=yaw_copy;depth=depth_copy;break; //运行 
					case 'q':singlerun_flag=ENABLE;taskrun_flag=DISABLE;roll=roll_copy;pitch=0;yaw=0;depth=0;break; //前进
					case 'h':singlerun_flag=ENABLE;taskrun_flag=DISABLE;roll=-roll_copy;pitch=0;yaw=0;depth=0;break; //后退
					case 'z':singlerun_flag=ENABLE;taskrun_flag=DISABLE;roll=0;pitch=0;yaw=yaw_copy;depth=0;break; //左转
					case 'y':singlerun_flag=ENABLE;taskrun_flag=DISABLE;roll=0;pitch=0;yaw=-yaw_copy;depth=0;break; //右转
					case 'x':singlerun_flag=ENABLE;taskrun_flag=DISABLE;roll=0;pitch=0;yaw=0;depth=depth_copy;break; //下潜
					case '1':singlerun_flag=DISABLE;taskrun_flag=1;break; //动作一
					case '2':singlerun_flag=DISABLE;taskrun_flag=2;break; //动作二
					case '3':singlerun_flag=DISABLE;taskrun_flag=3;break; //
				}
				if(singlerun_flag==ENABLE){  //执行单个动作
					sprintf(move,"S %.2f %.2f %.2f %.2f \r\n",roll,pitch,yaw,depth);
					printf(move);  //发送设定参数
					delay_ms(100);
					sprintf(run,"Q %d \r\n",runtime*100);
					printf(run);   //发送运行指令
					memset(move,0,20*sizeof(char));
					memset(run,0,10*sizeof(char));
				}
				else if(taskrun_flag != DISABLE){  //执行四边形任务
					switch (taskrun_flag){
						case 1:printf("K 4 \r\n"); 
										delay_ms(100);
									 printf("move_1\r\n");
										break;
						case 2:printf("K 5 \r\n"); 
										delay_ms(100);
									 printf("move_2\r\n");
										break;
						case 3:printf("K 8 \r\n"); 
										delay_ms(100);
									 printf("move_3\r\n");
										break;
					}
					delay_ms(100);
					 printf("1 \r\n"); //开始运行任务
					 taskrun_flag = 0;
				}
			}
			
		}			
		res =0; //没接收到WiFi数据
		USART2_RX_STA=0;
		if(constate!='+')t=1000;		//状态为还未连接,立即更新连接状态
		else t=0;                   //状态为已经连接了,10秒后再检查
	}
	res =0; //没接收到WiFi数据
	if(t==1000){//连续10秒钟没有收到任何数据,检查连接是不是还存在.
		constate=atk_8266_consta_check();//得到连接状态
		if(constate=='+')printf("设备连接成功,等待接收数据 \r\n");  //连接状态
		else printf("无设备连接 \r\n"); 	 
		t=0;
	}
	if((t%20)==0)LED=!LED;
	atk_8266_at_response(1);
	return res;
}



