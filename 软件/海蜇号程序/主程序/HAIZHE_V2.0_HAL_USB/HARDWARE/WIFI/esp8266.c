
/*
esp8266����ΪAP+tcp serverģʽ
*/
#include "esp8266.h"
#include "timer.h"
#include "malloc.h"
#include "delay.h"
 
/////////////////////////////////////////////////////////////////////////////////////////////////////////// 
//�û�������

//���Ӷ˿ں�:8086,�������޸�Ϊ�����˿�.
const u16 wifi_portnum=8086;		 

//UDP�����Զ��IP�Ͷ˿�
//IP��ַҲΪ�㲥��ַ���˿ڱ����ָ��

const u8* PC_ip="192.168.4.255";	//�㲥��ַ��192.168.4.255
const u16 PC_portnum=5000;

//WiFi APģʽ,ģ���������߲���,�������޸�.
const u8* wifiap_ssid="HAIZHE_V2";			//����SSID��
const u8* wifiap_encryption="wpawpa2_aes";	//wpa/wpa2 aes���ܷ�ʽ
const u8* wifiap_password="12345678"; 		//�������� 

const u8 mode_STA=1;
const u8 mode_AP=2;
const u8 mode_STA_AP=3;

/////////////////////////////////////////////////////////////////////////////////////////////////////////// 
//4������ģʽ
const u8 *ATK_ESP8266_CWMODE_TBL[3]={"STAģʽ ","APģʽ ","AP&STAģʽ "};	//ATK-ESP8266,3������ģʽ,Ĭ��Ϊ·����(ROUTER)ģʽ 
//4�ֹ���ģʽ
const u8 *ATK_ESP8266_WORKMODE_TBL[3]={"TCP������","TCP�ͻ���"," UDP ģʽ"};	//ATK-ESP8266,4�ֹ���ģʽ
//5�ּ��ܷ�ʽ
const u8 *ATK_ESP8266_ECN_TBL[5]={"OPEN","WEP","WPA_PSK","WPA2_PSK","WPA_WAP2_PSK"};
/////////////////////////////////////////////////////////////////////////////////////////////////////////// 
u8 WiFi_recvflag=0;  //WiFi���ձ�־λ

//usmart֧�ֲ���
//���յ���ATָ��Ӧ�����ݷ��ظ����Դ���
//mode:0,������WIFI_RX_STA;
//     1,����WIFI_RX_STA;
void esp8266_at_response(u8 mode)
{
	if(WIFI_RX_STA&0X8000)		//���յ�һ��������
	{ 
		WIFI_RX_BUF[WIFI_RX_STA&0X7FFF]=0;//��ӽ�����
//		printf("%s",WIFI_RX_BUF);	//���͵�����
		if(mode)WIFI_RX_STA=0;
	} 
}
//ATK-ESP8266���������,�����յ���Ӧ��
//str:�ڴ���Ӧ����
//����ֵ:0,û�еõ��ڴ���Ӧ����
//    ����,�ڴ�Ӧ������λ��(str��λ��)
u8* esp8266_check_cmd(u8 *str)
{
	
	char *strx=0;
//	if(WIFI_RX_STA&0X8000)		//���յ�һ��������
	if(WIFI_RX_STA)
	{ 
//		WIFI_RX_BUF[WIFI_RX_STA&0X7FFF]=0;//��ӽ�����
		strx=strstr((const char*)WIFI_RX_BUF,(const char*)str);
//		WIFI_RX_STA =0;
	} 
	return (u8*)strx;
}
//��ATK-ESP8266��������
//cmd:���͵������ַ���
//ack:�ڴ���Ӧ����,���Ϊ��,���ʾ����Ҫ�ȴ�Ӧ��
//waittime:�ȴ�ʱ��(��λ:10ms)
//����ֵ:0,���ͳɹ�(�õ����ڴ���Ӧ����)
//       1,����ʧ��
u8 esp8266_send_cmd(u8 *cmd,u8 *ack,u16 waittime)
{
	u8 res=0; 
	WIFI_RX_STA=0;
	wifi_printf("%s\r\n",cmd);	//��������
	memset(WIFI_RX_BUF,0,sizeof(WIFI_RX_BUF)/sizeof(char));
	if(ack&&waittime)		//��Ҫ�ȴ�Ӧ��
	{
		while(--waittime)	//�ȴ�����ʱ
		{
			delay_ms(10);
			if(esp8266_check_cmd(ack)){
//				printf("ack:%s\r\n",(u8*)ack);
				WIFI_RX_STA = 0;
				break;//�õ���Ч���� 
			}
			
//			if(WIFI_RX_STA&0X8000)//���յ��ڴ���Ӧ����
//			{
//				if(esp8266_check_cmd(ack))
//				{
//					printf("ack:%s\r\n",(u8*)ack);
//					break;//�õ���Ч���� 
//				}
//					WIFI_RX_STA=0;
//			} 
		}
		if(waittime==0)res=1; 
	}
	return res;
} 
//��ATK-ESP8266����ָ������
//data:���͵�����(����Ҫ��ӻس���)
//ack:�ڴ���Ӧ����,���Ϊ��,���ʾ����Ҫ�ȴ�Ӧ��
//waittime:�ȴ�ʱ��(��λ:10ms)
//����ֵ:0,���ͳɹ�(�õ����ڴ���Ӧ����)luojian
u8 esp8266_send_data(u8 *data,u8 *ack,u16 waittime)
{
	u8 res=0; 
	WIFI_RX_STA=0;
	wifi_printf("%s",data);	//��������
	if(ack&&waittime)		//��Ҫ�ȴ�Ӧ��
	{
		while(--waittime)	//�ȴ�����ʱ
		{
			delay_ms(10);
			if(WIFI_RX_STA&0X8000)//���յ��ڴ���Ӧ����
			{
				if(esp8266_check_cmd(ack))break;//�õ���Ч���� 
				WIFI_RX_STA=0;
			} 
		}
		if(waittime==0)res=1; 
	}
	return res;
}
//ATK-ESP8266�˳�͸��ģʽ
//����ֵ:0,�˳��ɹ�;
//       1,�˳�ʧ��
u8 esp8266_quit_trans(void)
{
	while((WIFI_UART->SR&0X40)==0);	//�ȴ����Ϳ�
	WIFI_UART->DR='+';      
	delay_ms(15);					//���ڴ�����֡ʱ��(10ms)
	while((WIFI_UART->SR&0X40)==0);	//�ȴ����Ϳ�
	WIFI_UART->DR='+';      
	delay_ms(15);					//���ڴ�����֡ʱ��(10ms)
	while((WIFI_UART->SR&0X40)==0);	//�ȴ����Ϳ�
	WIFI_UART->DR='+';      
	delay_ms(500);					//�ȴ�500ms
	return esp8266_send_cmd("AT","OK",20);//�˳�͸���ж�.
}

//��ȡATK-ESP8266ģ�������״̬
//����ֵ:0,δ����;1,���ӳɹ�.
u8 esp8266_consta_check(void)
{
	u8 *p;
	u8 res;
	if(esp8266_quit_trans())return 0;			//�˳�͸�� 
	esp8266_send_cmd("AT+CIPSTATUS",":",50);	//����AT+CIPSTATUSָ��,��ѯ����״̬
	p=esp8266_check_cmd("+CIPSTATUS:"); 
	res=*p;									//�õ�����״̬	
	return res;
}

//��ȡserver ip��ַ
//ipbuf:ip��ַ���������
void esp8266_get_wanip(u8* ipbuf)
{
	u8 *p,*p1;
	if(esp8266_send_cmd("AT+CIFSR","OK",50))//��ȡWAN IP��ַʧ��
	{
		ipbuf[0]=0;
		printf("get wanip fail \r\n");
		return;
	}		
//		printf(WIFI_RX_BUF);
//	p=esp8266_check_cmd("\"");	
	p=(u8*)strstr((const char*)WIFI_RX_BUF,"\"");//���ҵ�һ�����ŵ�λ��
//	printf("%d \r\n",*p);
	p1=(u8*)strstr((const char*)(p+1),"\""); //���ҵڶ������ŵ�λ��
	*p1=0;
	sprintf((char*)ipbuf,"%s",p+1);	

}

//ATK-ESP8266ģ�����������
void esp8266_test(void)
{
	u8 timex;

	while(1){
		delay_ms(10); 
		esp8266_at_response(1);//���ATK-ESP8266ģ�鷢�͹���������,��ʱ�ϴ�������

		printf("ATK-ESP WiFi-AP ���� \r\n"); 
		printf("��������ATK-ESPģ��,���Ե�...\r\n");
		esp8266_wifiap_mode();	//WiFi AP����
	 
//		if((timex%20)==0)LED0=!LED0;//200ms��˸ 
		timex++;	 
	} 
}

//����esp8266������udp͸��ģʽ���������÷����ɲ鿴"ESP8266 ATָ��ʹ��ʾ��"
//����0˵�����óɹ�
u8 ESP8266_UDP_Transparent_Trans(){
	u8 ipbuf[16]; 	//IP����
	u8 *p;
	u8 res=0;
	p=mymalloc(SRAMIN,64);							//����32�ֽ��ڴ�
	sprintf((char*)p,"AT+CWMODE=%d",mode_STA_AP);    //����wifi������STA_APģʽ
	if(esp8266_send_cmd(p,"OK",20)){
		printf("[ERROR] WiFi set STA_AP mode failed! \r\n");
		return 1;
	};
	printf("[OK] WiFi STA_AP mode \r\n");
	sprintf((char*)p,"AT+CWSAP=\"%s\",\"%s\",1,4",wifiap_ssid,wifiap_password);    //����ģ�����߲���
	if(esp8266_send_cmd(p,"OK",1000)){
		printf("[ERROR] WiFi set CWSAP failed! \r\n");
		return 1;
	}
	printf("[OK] WiFi AP ssid:%s passwd:%s \r\n",wifiap_ssid,wifiap_password);	
	if(esp8266_send_cmd("AT+CIPMUX=0","OK",20)){ //0�������ӣ�1��������
		printf("[ERROR] WiFi set CIPMUX failed! \r\n");
		return 1;
	}  
	esp8266_get_wanip(ipbuf);//������ģʽ,��ȡWAN IP
	sprintf((char*)p,"[OK] IP adress:%s port:%d \r\n",ipbuf,wifi_portnum);
	printf(p);				//��ʾIP��ַ�Ͷ˿�	
	sprintf((char*)p,"AT+CIPSTART=\"UDP\",\"%s\",%d,%d,0",(u8*)PC_ip,PC_portnum,wifi_portnum);
	if(esp8266_send_cmd(p,"OK",20)){	//esp8266���Ӧ�����豸�����̶��Զ˵�UDP����
		printf("[ERROR] WiFi UDP setup failed! \r\n");
		return 1;
	}
	printf("[OK] WiFi UDP setup,PC_adress:%s PC_port:%d \r\n",(u8*)PC_ip,PC_portnum);
	if(esp8266_send_cmd("AT+CIPMODE=1","OK",20)){	//ʹ��͸��ģʽ
		printf("[ERROR] WiFi transparent transfer mode enable failed! \r\n");
		return 1;
	}	
	printf("[OK] WiFi transparent transfer mode enabled \r\n");
	if(esp8266_send_cmd("AT+CIPSEND","OK",20)){	//����͸��
		printf("[ERROR] WiFi start transparent transfer failed! \r\n");
		return 1;
	}	
	printf("[OK] WiFi start transparent transfer \r\n");
	WIFI_RX_STA=0;
	
	myfree(SRAMIN,p);		//�ͷ��ڴ� 
	return res;		
}

//ATK-ESP8266 WiFi AP����
//���ڲ���TCP/UDP����
//����ֵ:0,����
//    ����,�������
u8 esp8266_wifiap_mode(void){
	u8 ipbuf[16]; 	//IP����
	u8 *p;
	u8 res=0;
		//TCP Server ����
	p=mymalloc(SRAMIN,64);							//����32�ֽ��ڴ�
	sprintf((char*)p,"AT+CWMODE=%d",mode_AP);    //����wifi������APģʽ
	esp8266_send_cmd(p,"OK",20);
	printf("WiFi AP mode \r\n");
	sprintf((char*)p,"AT+CWSAP=\"%s\",\"%s\",1,4",wifiap_ssid,wifiap_password);    //����ģ��APģʽ���߲���
	esp8266_send_cmd(p,"OK",1000);
	esp8266_send_cmd("AT+CIPMUX=1","OK",20);   //0�������ӣ�1��������
	sprintf((char*)p,"AT+CIPSERVER=1,%d",wifi_portnum);
	esp8266_send_cmd(p,"OK",20);     //����Serverģʽ���˿ں�Ϊ8086
	printf("WiFi ap ssid:%s passwd:%s \r\n",wifiap_ssid,wifiap_password);	
	esp8266_get_wanip(ipbuf);//������ģʽ,��ȡWAN IP
	sprintf((char*)p,"ip adress:%s port:%d \r\n",ipbuf,wifi_portnum);
	printf(p);				//��ʾIP��ַ�Ͷ˿�	

	WIFI_RX_STA=0;

	myfree(SRAMIN,p);		//�ͷ��ڴ� 
	return res;		
}


u16 t=999;		//���ٵ�һ�λ�ȡ����״̬������Ϊȫ�ֱ���������ÿ��ѭ����������
u8 set_flag; //�ֻ�app�����趨��־λ
u8 app_singlerun_flag; //app���͵������������־λ
u8 app_taskrun_flag;  //app�����ı��ζ��������־λ
u16 app_runtime; //����app�趨����ʱ��	


char app_task[200];


//WiFi����͸������ģʽ,���ȴ�
u8 wifi_transparent_trans_mode(u16 len){
	uint16_t waittime=10000;
	wifi_printf("AT+CIPSEND=0,%d\r\n",len);	//��������
	while(waittime--);//�ȴ�ESP8266����͸��ģʽ������ȡ�ش���Ϣ��Ϊ�˽�ʡʱ��
//	while(!(WIFI_RX_STA&0X8000));
//	WIFI_RX_STA = 0;
}
//WiFi����͸������ģʽ,���ȴ�
u8 wifi_quit_trans(){
	uint32_t waittime=100000;
	wifi_printf("AT\r\n");	//��������
	while(waittime--);//�ȴ�ESP8266����͸��ģʽ������ȡ�ش���Ϣ��Ϊ�˽�ʡʱ��
	while(!(WIFI_RX_STA&0X8000));
	WIFI_RX_STA = 0;
}

//WiFi�ش�����
u8 WiFi_frint(u8* const data){
	u16 len=0;	
	u8 *p=data;

//	len = sizeof(data)/sizeof(u8);
	while(*p++ != '\0') len++; //��ȡ�ش��ַ������ȣ���С�ڣ����ɳ���ʵ�ʳ���
//	printf("len=%d\r\n",len);
	wifi_transparent_trans_mode(len);//���½���WiFi���ڷ��͹���������
	wifi_printf("%s \r\n\r\n\r\n",data);	//�෢�ͼ������лس���ȷ��esp8266������͸������
//	wifi_quit_trans();
	esp8266_send_cmd("AT","OK",20);
}


//u8 WiFi_frint_MSG(u8* const data,u8 msg_taye){
//	u8 com[20];
//	u8 len=0;
//	u8 *p;
//	u8 res=0;
//	MAV_MSG mavmsg;
//	static uint16_t seq=0;//����mav��������
//	char mav_send_msg[100];
//		
//	strcpy(mavmsg.msg_head ,"fe");
//	strcpy(mavmsg.msg_tail ,"ff");
//	mavmsg.seq = seq++;
//	mavmsg.mid = MAV_MID_MSG;	//��Ϣ�ش���
//	sprintf(mav_send_msg,"%s %d %d %d %s %s",mavmsg.msg_head,mavmsg.seq,mavmsg.mid,msg_taye,data,mavmsg.msg_tail);//�������
//	
//	p = mav_send_msg;
//	while(*p++ != '\0') 	len++; //��ȡ�ش��ַ������ȣ���С�ڣ����ɳ���ʵ�ʳ���
//	wifi_transparent_trans_mode(len);

//	wifi_printf(mav_send_msg);

//	esp8266_send_cmd("AT","OK",20); //ʹesp8266�Ƴ�͸��

//	return res;

//}


//���WiFiģ��
void WiFi_init(){
	WIFI_UART_Init(WIFI_UART,WIFI_UART_Baud);
	printf("[OK] WIFI init baud %d \r\n",WIFI_UART_Baud);
	WIFI_PWR_Init();	//����wifi��Դʹ��
	u8 check_num=0;
	while(esp8266_send_cmd("AT","OK",20)){//���WIFIģ���Ƿ�����
		esp8266_quit_trans();//�˳�͸��
		esp8266_send_cmd("AT+CIPMODE=0","OK",200);  //�ر�͸��ģʽ	
		printf("[ERROR] WiFi Undetected! ");
		delay_ms(100);
		printf("Try Detected WiFi...%d \r\n",check_num); 
		if(check_num++>2) return;
	} 
	while(esp8266_send_cmd("ATE0","OK",20));	//�رջ���
//	esp8266_wifiap_mode();		//����ATK_ESP8266IP�Ͷ˿ں�
	if(ESP8266_UDP_Transparent_Trans()){	//ģ�����udp͸��ģʽ������mavlink����
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
