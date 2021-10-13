
/*
esp8266配置为AP+tcp server模式
*/
#include "esp8266.h"
#include "timer.h"
#include "malloc.h"
#include "delay.h"
 
/////////////////////////////////////////////////////////////////////////////////////////////////////////// 
//用户配置区

//连接端口号:8086,可自行修改为其他端口.
const u16 wifi_portnum=8086;		 

//UDP传输的远端IP和端口
//IP地址也为广播地址，端口必须得指定

const u8* PC_ip="192.168.4.255";	//广播地址：192.168.4.255
const u16 PC_portnum=5000;

//WiFi AP模式,模块对外的无线参数,可自行修改.
const u8* wifiap_ssid="HAIZHE_V2";			//对外SSID号
const u8* wifiap_encryption="wpawpa2_aes";	//wpa/wpa2 aes加密方式
const u8* wifiap_password="12345678"; 		//连接密码 

const u8 mode_STA=1;
const u8 mode_AP=2;
const u8 mode_STA_AP=3;

/////////////////////////////////////////////////////////////////////////////////////////////////////////// 
//4个网络模式
const u8 *ATK_ESP8266_CWMODE_TBL[3]={"STA模式 ","AP模式 ","AP&STA模式 "};	//ATK-ESP8266,3种网络模式,默认为路由器(ROUTER)模式 
//4种工作模式
const u8 *ATK_ESP8266_WORKMODE_TBL[3]={"TCP服务器","TCP客户端"," UDP 模式"};	//ATK-ESP8266,4种工作模式
//5种加密方式
const u8 *ATK_ESP8266_ECN_TBL[5]={"OPEN","WEP","WPA_PSK","WPA2_PSK","WPA_WAP2_PSK"};
/////////////////////////////////////////////////////////////////////////////////////////////////////////// 
u8 WiFi_recvflag=0;  //WiFi接收标志位

//usmart支持部分
//将收到的AT指令应答数据返回给电脑串口
//mode:0,不清零WIFI_RX_STA;
//     1,清零WIFI_RX_STA;
void esp8266_at_response(u8 mode)
{
	if(WIFI_RX_STA&0X8000)		//接收到一次数据了
	{ 
		WIFI_RX_BUF[WIFI_RX_STA&0X7FFF]=0;//添加结束符
//		printf("%s",WIFI_RX_BUF);	//发送到串口
		if(mode)WIFI_RX_STA=0;
	} 
}
//ATK-ESP8266发送命令后,检测接收到的应答
//str:期待的应答结果
//返回值:0,没有得到期待的应答结果
//    其他,期待应答结果的位置(str的位置)
u8* esp8266_check_cmd(u8 *str)
{
	
	char *strx=0;
//	if(WIFI_RX_STA&0X8000)		//接收到一次数据了
	if(WIFI_RX_STA)
	{ 
//		WIFI_RX_BUF[WIFI_RX_STA&0X7FFF]=0;//添加结束符
		strx=strstr((const char*)WIFI_RX_BUF,(const char*)str);
//		WIFI_RX_STA =0;
	} 
	return (u8*)strx;
}
//向ATK-ESP8266发送命令
//cmd:发送的命令字符串
//ack:期待的应答结果,如果为空,则表示不需要等待应答
//waittime:等待时间(单位:10ms)
//返回值:0,发送成功(得到了期待的应答结果)
//       1,发送失败
u8 esp8266_send_cmd(u8 *cmd,u8 *ack,u16 waittime)
{
	u8 res=0; 
	WIFI_RX_STA=0;
	wifi_printf("%s\r\n",cmd);	//发送命令
	memset(WIFI_RX_BUF,0,sizeof(WIFI_RX_BUF)/sizeof(char));
	if(ack&&waittime)		//需要等待应答
	{
		while(--waittime)	//等待倒计时
		{
			delay_ms(10);
			if(esp8266_check_cmd(ack)){
//				printf("ack:%s\r\n",(u8*)ack);
				WIFI_RX_STA = 0;
				break;//得到有效数据 
			}
			
//			if(WIFI_RX_STA&0X8000)//接收到期待的应答结果
//			{
//				if(esp8266_check_cmd(ack))
//				{
//					printf("ack:%s\r\n",(u8*)ack);
//					break;//得到有效数据 
//				}
//					WIFI_RX_STA=0;
//			} 
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
u8 esp8266_send_data(u8 *data,u8 *ack,u16 waittime)
{
	u8 res=0; 
	WIFI_RX_STA=0;
	wifi_printf("%s",data);	//发送命令
	if(ack&&waittime)		//需要等待应答
	{
		while(--waittime)	//等待倒计时
		{
			delay_ms(10);
			if(WIFI_RX_STA&0X8000)//接收到期待的应答结果
			{
				if(esp8266_check_cmd(ack))break;//得到有效数据 
				WIFI_RX_STA=0;
			} 
		}
		if(waittime==0)res=1; 
	}
	return res;
}
//ATK-ESP8266退出透传模式
//返回值:0,退出成功;
//       1,退出失败
u8 esp8266_quit_trans(void)
{
	while((WIFI_UART->SR&0X40)==0);	//等待发送空
	WIFI_UART->DR='+';      
	delay_ms(15);					//大于串口组帧时间(10ms)
	while((WIFI_UART->SR&0X40)==0);	//等待发送空
	WIFI_UART->DR='+';      
	delay_ms(15);					//大于串口组帧时间(10ms)
	while((WIFI_UART->SR&0X40)==0);	//等待发送空
	WIFI_UART->DR='+';      
	delay_ms(500);					//等待500ms
	return esp8266_send_cmd("AT","OK",20);//退出透传判断.
}

//获取ATK-ESP8266模块的连接状态
//返回值:0,未连接;1,连接成功.
u8 esp8266_consta_check(void)
{
	u8 *p;
	u8 res;
	if(esp8266_quit_trans())return 0;			//退出透传 
	esp8266_send_cmd("AT+CIPSTATUS",":",50);	//发送AT+CIPSTATUS指令,查询连接状态
	p=esp8266_check_cmd("+CIPSTATUS:"); 
	res=*p;									//得到连接状态	
	return res;
}

//获取server ip地址
//ipbuf:ip地址输出缓存区
void esp8266_get_wanip(u8* ipbuf)
{
	u8 *p,*p1;
	if(esp8266_send_cmd("AT+CIFSR","OK",50))//获取WAN IP地址失败
	{
		ipbuf[0]=0;
		printf("get wanip fail \r\n");
		return;
	}		
//		printf(WIFI_RX_BUF);
//	p=esp8266_check_cmd("\"");	
	p=(u8*)strstr((const char*)WIFI_RX_BUF,"\"");//查找第一个引号的位置
//	printf("%d \r\n",*p);
	p1=(u8*)strstr((const char*)(p+1),"\""); //查找第二个引号的位置
	*p1=0;
	sprintf((char*)ipbuf,"%s",p+1);	

}

//ATK-ESP8266模块测试主函数
void esp8266_test(void)
{
	u8 timex;

	while(1){
		delay_ms(10); 
		esp8266_at_response(1);//检查ATK-ESP8266模块发送过来的数据,及时上传给电脑

		printf("ATK-ESP WiFi-AP 测试 \r\n"); 
		printf("正在配置ATK-ESP模块,请稍等...\r\n");
		esp8266_wifiap_mode();	//WiFi AP测试
	 
//		if((timex%20)==0)LED0=!LED0;//200ms闪烁 
		timex++;	 
	} 
}

//设置esp8266工作在udp透传模式，具体设置方法可查看"ESP8266 AT指令使用示例"
//返回0说明设置成功
u8 ESP8266_UDP_Transparent_Trans(){
	u8 ipbuf[16]; 	//IP缓存
	u8 *p;
	u8 res=0;
	p=mymalloc(SRAMIN,64);							//申请32字节内存
	sprintf((char*)p,"AT+CWMODE=%d",mode_STA_AP);    //配置wifi工作在STA_AP模式
	if(esp8266_send_cmd(p,"OK",20)){
		printf("[ERROR] WiFi set STA_AP mode failed! \r\n");
		return 1;
	};
	printf("[OK] WiFi STA_AP mode \r\n");
	sprintf((char*)p,"AT+CWSAP=\"%s\",\"%s\",1,4",wifiap_ssid,wifiap_password);    //配置模块无线参数
	if(esp8266_send_cmd(p,"OK",1000)){
		printf("[ERROR] WiFi set CWSAP failed! \r\n");
		return 1;
	}
	printf("[OK] WiFi AP ssid:%s passwd:%s \r\n",wifiap_ssid,wifiap_password);	
	if(esp8266_send_cmd("AT+CIPMUX=0","OK",20)){ //0：单连接，1：多连接
		printf("[ERROR] WiFi set CIPMUX failed! \r\n");
		return 1;
	}  
	esp8266_get_wanip(ipbuf);//服务器模式,获取WAN IP
	sprintf((char*)p,"[OK] IP adress:%s port:%d \r\n",ipbuf,wifi_portnum);
	printf(p);				//显示IP地址和端口	
	sprintf((char*)p,"AT+CIPSTART=\"UDP\",\"%s\",%d,%d,0",(u8*)PC_ip,PC_portnum,wifi_portnum);
	if(esp8266_send_cmd(p,"OK",20)){	//esp8266与对应连接设备建立固定对端的UDP传输
		printf("[ERROR] WiFi UDP setup failed! \r\n");
		return 1;
	}
	printf("[OK] WiFi UDP setup,PC_adress:%s PC_port:%d \r\n",(u8*)PC_ip,PC_portnum);
	if(esp8266_send_cmd("AT+CIPMODE=1","OK",20)){	//使能透传模式
		printf("[ERROR] WiFi transparent transfer mode enable failed! \r\n");
		return 1;
	}	
	printf("[OK] WiFi transparent transfer mode enabled \r\n");
	if(esp8266_send_cmd("AT+CIPSEND","OK",20)){	//开启透传
		printf("[ERROR] WiFi start transparent transfer failed! \r\n");
		return 1;
	}	
	printf("[OK] WiFi start transparent transfer \r\n");
	WIFI_RX_STA=0;
	
	myfree(SRAMIN,p);		//释放内存 
	return res;		
}

//ATK-ESP8266 WiFi AP测试
//用于测试TCP/UDP连接
//返回值:0,正常
//    其他,错误代码
u8 esp8266_wifiap_mode(void){
	u8 ipbuf[16]; 	//IP缓存
	u8 *p;
	u8 res=0;
		//TCP Server 配置
	p=mymalloc(SRAMIN,64);							//申请32字节内存
	sprintf((char*)p,"AT+CWMODE=%d",mode_AP);    //配置wifi工作在AP模式
	esp8266_send_cmd(p,"OK",20);
	printf("WiFi AP mode \r\n");
	sprintf((char*)p,"AT+CWSAP=\"%s\",\"%s\",1,4",wifiap_ssid,wifiap_password);    //配置模块AP模式无线参数
	esp8266_send_cmd(p,"OK",1000);
	esp8266_send_cmd("AT+CIPMUX=1","OK",20);   //0：单连接，1：多连接
	sprintf((char*)p,"AT+CIPSERVER=1,%d",wifi_portnum);
	esp8266_send_cmd(p,"OK",20);     //开启Server模式，端口号为8086
	printf("WiFi ap ssid:%s passwd:%s \r\n",wifiap_ssid,wifiap_password);	
	esp8266_get_wanip(ipbuf);//服务器模式,获取WAN IP
	sprintf((char*)p,"ip adress:%s port:%d \r\n",ipbuf,wifi_portnum);
	printf(p);				//显示IP地址和端口	

	WIFI_RX_STA=0;

	myfree(SRAMIN,p);		//释放内存 
	return res;		
}


u16 t=999;		//加速第一次获取链接状态，需作为全局变量，否则每次循环调用清零
u8 set_flag; //手机app参数设定标志位
u8 app_singlerun_flag; //app发送单个动作命令标志位
u8 app_taskrun_flag;  //app发送四边形动作命令标志位
u16 app_runtime; //接收app设定运行时间	


char app_task[200];


//WiFi进入透明传输模式,不等待
u8 wifi_transparent_trans_mode(u16 len){
	uint16_t waittime=10000;
	wifi_printf("AT+CIPSEND=0,%d\r\n",len);	//发送命令
	while(waittime--);//等待ESP8266进入透传模式，不读取回传信息是为了节省时间
//	while(!(WIFI_RX_STA&0X8000));
//	WIFI_RX_STA = 0;
}
//WiFi进入透明传输模式,不等待
u8 wifi_quit_trans(){
	uint32_t waittime=100000;
	wifi_printf("AT\r\n");	//发送命令
	while(waittime--);//等待ESP8266进入透传模式，不读取回传信息是为了节省时间
	while(!(WIFI_RX_STA&0X8000));
	WIFI_RX_STA = 0;
}

//WiFi回传函数
u8 WiFi_frint(u8* const data){
	u16 len=0;	
	u8 *p=data;

//	len = sizeof(data)/sizeof(u8);
	while(*p++ != '\0') len++; //获取回传字符串长度，可小于，不可超出实际长度
//	printf("len=%d\r\n",len);
	wifi_transparent_trans_mode(len);//重新接收WiFi串口发送过来的数据
	wifi_printf("%s \r\n\r\n\r\n",data);	//多发送几个换行回车，确保esp8266发送完透传数据
//	wifi_quit_trans();
	esp8266_send_cmd("AT","OK",20);
}


//u8 WiFi_frint_MSG(u8* const data,u8 msg_taye){
//	u8 com[20];
//	u8 len=0;
//	u8 *p;
//	u8 res=0;
//	MAV_MSG mavmsg;
//	static uint16_t seq=0;//发送mav数据序列
//	char mav_send_msg[100];
//		
//	strcpy(mavmsg.msg_head ,"fe");
//	strcpy(mavmsg.msg_tail ,"ff");
//	mavmsg.seq = seq++;
//	mavmsg.mid = MAV_MID_MSG;	//消息回传包
//	sprintf(mav_send_msg,"%s %d %d %d %s %s",mavmsg.msg_head,mavmsg.seq,mavmsg.mid,msg_taye,data,mavmsg.msg_tail);//打包发送
//	
//	p = mav_send_msg;
//	while(*p++ != '\0') 	len++; //获取回传字符串长度，可小于，不可超出实际长度
//	wifi_transparent_trans_mode(len);

//	wifi_printf(mav_send_msg);

//	esp8266_send_cmd("AT","OK",20); //使esp8266推出透传

//	return res;

//}


//检测WiFi模块
void WiFi_init(){
	WIFI_UART_Init(WIFI_UART,WIFI_UART_Baud);
	printf("[OK] WIFI init baud %d \r\n",WIFI_UART_Baud);
	WIFI_PWR_Init();	//开启wifi电源使能
	u8 check_num=0;
	while(esp8266_send_cmd("AT","OK",20)){//检查WIFI模块是否在线
		esp8266_quit_trans();//退出透传
		esp8266_send_cmd("AT+CIPMODE=0","OK",200);  //关闭透传模式	
		printf("[ERROR] WiFi Undetected! ");
		delay_ms(100);
		printf("Try Detected WiFi...%d \r\n",check_num); 
		if(check_num++>2) return;
	} 
	while(esp8266_send_cmd("ATE0","OK",20));	//关闭回显
//	esp8266_wifiap_mode();		//设置ATK_ESP8266IP和端口号
	if(ESP8266_UDP_Transparent_Trans()){	//模块进入udp透传模式，传输mavlink数据
		printf("[ERROR] WiFi init failed! \r\n");
		return;
	}
	sys_flag.mavlink_enable = true;
	printf("[OK] WiFi init successful \r\n");
}

void WIFI_PWR_Init(){
	GPIO_InitTypeDef GPIO_InitStruct;
	
	WIFI_PWR_CLK_ENABLE();

  GPIO_InitStruct.Pin = WIFI_PWR_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(WIFI_PWR_GPIO, &GPIO_InitStruct);

	HAL_GPIO_WritePin(WIFI_PWR_GPIO, WIFI_PWR_PIN,GPIO_PIN_SET);
}
