#include "common.h"

//ATK-ESP8266 WIFI AP����
//���ڲ���TCP/UDP����
//����ֵ:0,����
//    ����,�������
u8 atk_8266_wifiap_test(void)
{
	u8 timex=0; 
	u8 index;
	u8 i=0;
	u8 ipbuf[16]; 	//IP����
	u8 *p;
	u16 t=999;		//���ٵ�һ�λ�ȡ����״̬
	u8 res=0;
	u16 rlen=0;
	u8 constate=0;	//����״̬
	p=mymalloc(SRAMIN,32);							//����32�ֽ��ڴ�
	
PRESTA:
//	netpro=atk_8266_netpro_sel(50,30,(u8*)ATK_ESP8266_CWMODE_TBL[1]);	//ѡ������ģʽ

	//TCP Server ����
	atk_8266_send_cmd("AT+CIPMUX=1","OK",20);   //0�������ӣ�1��������
	sprintf((char*)p,"AT+CIPSERVER=1,%s",(u8*)portnum);
	atk_8266_send_cmd(p,"OK",20);     //����Serverģʽ���˿ں�Ϊ8086
		
	atk_8266_get_wanip(ipbuf);//������ģʽ,��ȡWAN IP
	sprintf((char*)p,"IP��ַ:%s �˿�:%s \r\n",ipbuf,(u8*)portnum);
	printf(p);				//��ʾIP��ַ�Ͷ˿�	

	USART2_RX_STA=0;
	while(1)
	{
		t++;
		delay_ms(10);
		if(USART2_RX_STA&0X8000)		//���յ�һ��������
		{ 
			rlen=USART2_RX_STA&0X7FFF;	//�õ����ν��յ������ݳ���
			USART2_RX_BUF[rlen]=0;		//��ӽ����� 
			if(USART2_RX_BUF[2]=='+'){   //��ֹWiFi���ӺͶϿ�ʱ�����
				if(rlen>20) index = 10;//�����յ�10�������ַ�ʱ�������س����з�
				else index = 9;
				rlen -= index; //��ʵ�����ֽ���
				p = &USART2_RX_BUF[++index];
				while(*p++ != '\0'){
					USART2_RX_BUF[i++] = *p;
				}
				i =0;
				sprintf((char*)p,"�յ�%d�ֽ�,��������:\r\n",rlen-2);//���յ����ֽ���,��Ҫ��ȥ\r\n
				printf(p); 			//��ʾ���յ������ݳ���	
				printf("%s",USART2_RX_BUF);	//���͵�����
				printf("\r\n");	
			}			 
			USART2_RX_STA=0;
			if(constate!='+')t=1000;		//״̬Ϊ��δ����,������������״̬
			else t=0;                   //״̬Ϊ�Ѿ�������,10����ټ��
		}  
		if(t==1000)//����10����û���յ��κ�����,��������ǲ��ǻ�����.
		{
			constate=atk_8266_consta_check();//�õ�����״̬
			if(constate=='+')printf("�豸���ӳɹ�,�ȴ��������� \r\n");  //����״̬
			else printf("���豸���� \r\n"); 	 
			t=0;
		}
		if((t%20)==0)LED=!LED;
		atk_8266_at_response(1);
		
	}
	myfree(SRAMIN,p);		//�ͷ��ڴ� 
	return res;		
} 







