
/*
esp8266����ΪAP+tcp serverģʽ
*/
#include "esp8266.h"

/////////////////////////////////////////////////////////////////////////////////////////////////////////// 
//�û�������

//���Ӷ˿ں�:8086,�������޸�Ϊ�����˿�.
const u8* portnum="8086";		 

//WIFI APģʽ,ģ���������߲���,�������޸�.
const u8* wifiap_ssid="ATK-ESP8266";			//����SSID��
const u8* wifiap_encryption="wpawpa2_aes";	//wpa/wpa2 aes���ܷ�ʽ
const u8* wifiap_password="12345678"; 		//�������� 

/////////////////////////////////////////////////////////////////////////////////////////////////////////// 
//4������ģʽ
const u8 *ATK_ESP8266_CWMODE_TBL[3]={"STAģʽ ","APģʽ ","AP&STAģʽ "};	//ATK-ESP8266,3������ģʽ,Ĭ��Ϊ·����(ROUTER)ģʽ 
//4�ֹ���ģʽ
const u8 *ATK_ESP8266_WORKMODE_TBL[3]={"TCP������","TCP�ͻ���"," UDP ģʽ"};	//ATK-ESP8266,4�ֹ���ģʽ
//5�ּ��ܷ�ʽ
const u8 *ATK_ESP8266_ECN_TBL[5]={"OPEN","WEP","WPA_PSK","WPA2_PSK","WPA_WAP2_PSK"};
/////////////////////////////////////////////////////////////////////////////////////////////////////////// 

//usmart֧�ֲ���
//���յ���ATָ��Ӧ�����ݷ��ظ����Դ���
//mode:0,������USART2_RX_STA;
//     1,����USART2_RX_STA;
void atk_8266_at_response(u8 mode)
{
	if(USART2_RX_STA&0X8000)		//���յ�һ��������
	{ 
		USART2_RX_BUF[USART2_RX_STA&0X7FFF]=0;//��ӽ�����
//		printf("%s",USART2_RX_BUF);	//���͵�����
		if(mode)USART2_RX_STA=0;
	} 
}
//ATK-ESP8266���������,�����յ���Ӧ��
//str:�ڴ���Ӧ����
//����ֵ:0,û�еõ��ڴ���Ӧ����
//    ����,�ڴ�Ӧ������λ��(str��λ��)
u8* atk_8266_check_cmd(u8 *str)
{
	
	char *strx=0;
	if(USART2_RX_STA&0X8000)		//���յ�һ��������
	{ 
		USART2_RX_BUF[USART2_RX_STA&0X7FFF]=0;//��ӽ�����
		strx=strstr((const char*)USART2_RX_BUF,(const char*)str);
//		USART2_RX_STA =0;
	} 
	return (u8*)strx;
}
//��ATK-ESP8266��������
//cmd:���͵������ַ���
//ack:�ڴ���Ӧ����,���Ϊ��,���ʾ����Ҫ�ȴ�Ӧ��
//waittime:�ȴ�ʱ��(��λ:10ms)
//����ֵ:0,���ͳɹ�(�õ����ڴ���Ӧ����)
//       1,����ʧ��
u8 atk_8266_send_cmd(u8 *cmd,u8 *ack,u16 waittime)
{
	u8 res=0; 
	USART2_RX_STA=0;
	u2_printf("%s\r\n",cmd);	//��������
	if(ack&&waittime)		//��Ҫ�ȴ�Ӧ��
	{
		while(--waittime)	//�ȴ�����ʱ
		{
			delay_ms(10);
			if(USART2_RX_STA&0X8000)//���յ��ڴ���Ӧ����
			{
				if(atk_8266_check_cmd(ack))
				{
//					printf("ack:%s\r\n",(u8*)ack);
					break;//�õ���Ч���� 
				}
					USART2_RX_STA=0;
			} 
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
u8 atk_8266_send_data(u8 *data,u8 *ack,u16 waittime)
{
	u8 res=0; 
	USART2_RX_STA=0;
	u2_printf("%s",data);	//��������
	if(ack&&waittime)		//��Ҫ�ȴ�Ӧ��
	{
		while(--waittime)	//�ȴ�����ʱ
		{
			delay_ms(10);
			if(USART2_RX_STA&0X8000)//���յ��ڴ���Ӧ����
			{
				if(atk_8266_check_cmd(ack))break;//�õ���Ч���� 
				USART2_RX_STA=0;
			} 
		}
		if(waittime==0)res=1; 
	}
	return res;
}
//ATK-ESP8266�˳�͸��ģʽ
//����ֵ:0,�˳��ɹ�;
//       1,�˳�ʧ��
u8 atk_8266_quit_trans(void)
{
	while((USART2->SR&0X40)==0);	//�ȴ����Ϳ�
	USART2->DR='+';      
	delay_ms(15);					//���ڴ�����֡ʱ��(10ms)
	while((USART2->SR&0X40)==0);	//�ȴ����Ϳ�
	USART2->DR='+';      
	delay_ms(15);					//���ڴ�����֡ʱ��(10ms)
	while((USART2->SR&0X40)==0);	//�ȴ����Ϳ�
	USART2->DR='+';      
	delay_ms(500);					//�ȴ�500ms
	return atk_8266_send_cmd("AT","OK",20);//�˳�͸���ж�.
}

//��ȡATK-ESP8266ģ�������״̬
//����ֵ:0,δ����;1,���ӳɹ�.
u8 atk_8266_consta_check(void)
{
	u8 *p;
	u8 res;
	if(atk_8266_quit_trans())return 0;			//�˳�͸�� 
	atk_8266_send_cmd("AT+CIPSTATUS",":",50);	//����AT+CIPSTATUSָ��,��ѯ����״̬
	p=atk_8266_check_cmd("+CIPSTATUS:"); 
	res=*p;									//�õ�����״̬	
	return res;
}

//��ȡserver ip��ַ
//ipbuf:ip��ַ���������
void atk_8266_get_wanip(u8* ipbuf)
{
	u8 *p,*p1;
		if(atk_8266_send_cmd("AT+CIFSR","OK",50))//��ȡWAN IP��ַʧ��
		{
			ipbuf[0]=0;
			return;
		}		
		p=atk_8266_check_cmd("\"");
		p1=(u8*)strstr((const char*)(p+1),"\"");
		*p1=0;
		sprintf((char*)ipbuf,"%s",p+1);	
}

//ATK-ESP8266ģ�����������
void atk_8266_test(void)
{
	u8 timex;

	while(1){
		delay_ms(10); 
		atk_8266_at_response(1);//���ATK-ESP8266ģ�鷢�͹���������,��ʱ�ϴ�������

		printf("ATK-ESP WIFI-AP ���� \r\n"); 
		printf("��������ATK-ESPģ��,���Ե�...\r\n");
		atk_8266_wifiap_mode();	//WIFI AP����
	 
		if((timex%20)==0)LED=!LED;//200ms��˸ 
		timex++;	 
	} 
}


//ATK-ESP8266 WIFI AP����
//���ڲ���TCP/UDP����
//����ֵ:0,����
//    ����,�������
u8 atk_8266_wifiap_mode(void){
	u8 ipbuf[16]; 	//IP����
	u8 *p;
	u8 res=0;

	p=mymalloc(SRAMIN,32);							//����32�ֽ��ڴ�
	//TCP Server ����
	atk_8266_send_cmd("AT+CIPMUX=1","OK",20);   //0�������ӣ�1��������
	sprintf((char*)p,"AT+CIPSERVER=1,%s",(u8*)portnum);
	atk_8266_send_cmd(p,"OK",20);     //����Serverģʽ���˿ں�Ϊ8086
		
	atk_8266_get_wanip(ipbuf);//������ģʽ,��ȡWAN IP
	sprintf((char*)p,"IP��ַ:%s �˿�:%s \r\n",ipbuf,(u8*)portnum);
	printf(p);				//��ʾIP��ַ�Ͷ˿�	

	USART2_RX_STA=0;

	myfree(SRAMIN,p);		//�ͷ��ڴ� 
	return res;		
}


u16 t=999;		//���ٵ�һ�λ�ȡ����״̬������Ϊȫ�ֱ���������ÿ��ѭ����������
u8 set_flag; //�ֻ�app�����趨��־λ
u8 singlerun_flag; //�������б�־
u8 taskrun_flag;  //�������б�־
float depth,pitch,roll,yaw;
float depth_copy,pitch_copy,roll_copy,yaw_copy;
u16 runtime; //����app�趨����ʱ��
u8 message[100],move[20],run[10];

//APPͨ��WiFi�������ݻ���	
uint8_t WiFi_echo(){  
	u8 index;
	u8 i=0;
	u8 *p;	
	u8 res=0;
	u16 rlen=0;
	u8 constate=0;	//����״̬
		
	t++;
	delay_ms(10);
	if(USART2_RX_STA&0X8000){		//���յ�һ��������
	 
		rlen=USART2_RX_STA&0X7FFF;	//�õ����ν��յ������ݳ���
		USART2_RX_BUF[rlen]=0;		//��ӽ����� 
		if(USART2_RX_BUF[2]=='+'){   //��ֹWiFi���ӺͶϿ�ʱ�����
			res =1;  //���յ�WiFi����
			if(rlen>20) index = 10;//�����յ�10�������ַ�ʱ�������س����з�
			else index = 9;
			rlen -= index; //��ʵ�����ֽ���
			p = &USART2_RX_BUF[index+1];  
			while(*p++ != '\0'){  //ɾ�����ಿ�֣��õ���ʵ�����ַ���
				USART2_RX_BUF[i++] = *p;
			}
			i =0;
			sprintf((char*)p,"�յ�%d�ֽ�,��������:\r\n",rlen-2);//���յ����ֽ���,��Ҫ��ȥ\r\n
			printf(p); 			//��ʾ���յ������ݳ���	
			printf("%s",USART2_RX_BUF);	//���͵�����
			printf("\r\n");	
			
			switch (USART2_RX_BUF[0]){  //WiFi�յ���Ϣ���ݽ���
				case 's':set_flag=ENABLE;
								sscanf((char *)&USART2_RX_BUF[1],"%f %f %f %f %d ",&depth,&pitch,&roll,&yaw,&runtime);
								depth_copy=depth;pitch_copy=pitch;roll_copy=roll;yaw_copy=yaw; //��������
								sprintf(message,"SetĿ��: rol:%.1f pit:%.1f yaw:%.1f dep:%.1f time:%d \r\n",roll,pitch,yaw,depth,runtime);		
								printf(message); 
								break;
				case 'K':printf("3 \r\n");break;   //bootloader��ת��������
				case 'k':printf("E 1 \r\n");break;	//���̵���
				case 'g':printf("E 0 \r\n");break;	//�ؼ̵���
				case 'f':printf("E 2 \r\n");break;	//��λ
			 default:if(set_flag==DISABLE)
									printf("���Ƚ��в����趨 \r\n");
								break ;
			}
			if(set_flag==ENABLE){  //app�����趨���
				switch (USART2_RX_BUF[0]){
					case 'S':singlerun_flag=ENABLE;taskrun_flag=DISABLE;roll=roll_copy;pitch=pitch_copy;yaw=yaw_copy;depth=depth_copy;break; //���� 
					case 'q':singlerun_flag=ENABLE;taskrun_flag=DISABLE;roll=roll_copy;pitch=0;yaw=0;depth=0;break; //ǰ��
					case 'h':singlerun_flag=ENABLE;taskrun_flag=DISABLE;roll=-roll_copy;pitch=0;yaw=0;depth=0;break; //����
					case 'z':singlerun_flag=ENABLE;taskrun_flag=DISABLE;roll=0;pitch=0;yaw=yaw_copy;depth=0;break; //��ת
					case 'y':singlerun_flag=ENABLE;taskrun_flag=DISABLE;roll=0;pitch=0;yaw=-yaw_copy;depth=0;break; //��ת
					case 'x':singlerun_flag=ENABLE;taskrun_flag=DISABLE;roll=0;pitch=0;yaw=0;depth=depth_copy;break; //��Ǳ
					case '1':singlerun_flag=DISABLE;taskrun_flag=1;break; //����һ
					case '2':singlerun_flag=DISABLE;taskrun_flag=2;break; //������
					case '3':singlerun_flag=DISABLE;taskrun_flag=3;break; //
				}
				if(singlerun_flag==ENABLE){  //ִ�е�������
					sprintf(move,"S %.2f %.2f %.2f %.2f \r\n",roll,pitch,yaw,depth);
					printf(move);  //�����趨����
					delay_ms(100);
					sprintf(run,"Q %d \r\n",runtime*100);
					printf(run);   //��������ָ��
					memset(move,0,20*sizeof(char));
					memset(run,0,10*sizeof(char));
				}
				else if(taskrun_flag != DISABLE){  //ִ���ı�������
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
					 printf("1 \r\n"); //��ʼ��������
					 taskrun_flag = 0;
				}
			}
			
		}			
		res =0; //û���յ�WiFi����
		USART2_RX_STA=0;
		if(constate!='+')t=1000;		//״̬Ϊ��δ����,������������״̬
		else t=0;                   //״̬Ϊ�Ѿ�������,10����ټ��
	}
	res =0; //û���յ�WiFi����
	if(t==1000){//����10����û���յ��κ�����,��������ǲ��ǻ�����.
		constate=atk_8266_consta_check();//�õ�����״̬
		if(constate=='+')printf("�豸���ӳɹ�,�ȴ��������� \r\n");  //����״̬
		else printf("���豸���� \r\n"); 	 
		t=0;
	}
	if((t%20)==0)LED=!LED;
	atk_8266_at_response(1);
	return res;
}



