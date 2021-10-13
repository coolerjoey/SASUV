#ifndef __COMMON_H__
#define __COMMON_H__	 

#include "sys.h"	
#include "string.h"
#include "uart.h"
#include "timer.h"
#include "led.h"
#include "global_para.h"

extern PARA_FIXED fp;

#define WIFI_UART UART8
#define WIFI_UART_Baud fp.uartB_baud
#define WIFI_UART_Init(uart,baud) UART_init(uart,baud)
#define WIFI_RX_STA UART8_RX_STA
#define WIFI_RX_BUF UART8_RX_BUF
#define wifi_printf u8_printf

#define WIFI_PWR_CLK_ENABLE() __HAL_RCC_GPIOC_CLK_ENABLE()
#define WIFI_PWR_GPIO GPIOC
#define WIFI_PWR_PIN GPIO_PIN_13
#define	WIFI_PWR PCout(13)

/////////////////////////////////////////////////////////////////////////////////////////////////////////// 
void esp8266_at_response(u8 mode);
u8* esp8266_check_cmd(u8 *str);
u8 esp8266_send_cmd(u8 *cmd,u8 *ack,u16 waittime);
u8 esp8266_send_data(u8 *data,u8 *ack,u16 waittime);
u8 esp8266_quit_trans(void);
u8 esp8266_consta_check(void);
void esp8266_get_wanip(u8* ipbuf);
void esp8266_test(void);
u8 esp8266_wifiap_mode(void);	//WiFi AP����
void WiFi_echo(void);
u8 WiFi_frint(u8* const );
void WiFi_init(void);
u8 wifi_transparent_trans_mode(u16);
u8 WiFi_frint_MSG(u8* const ,u8 );
void WIFI_PWR_Init(void);
//�û����ò���
extern const u16 wifi_portnum;			//���Ӷ˿�
 
extern const u8* wifista_ssid;		//WiFi STA SSID
extern const u8* wifista_encryption;//WiFi STA ���ܷ�ʽ
extern const u8* wifista_password; 	//WiFi STA ����

extern const u8* wifiap_ssid;		//WiFi AP SSID
extern const u8* wifiap_encryption;	//WiFi AP ���ܷ�ʽ
extern const u8* wifiap_password; 	//WiFi AP ����

extern const u8* ATK_ESP8266_CWMODE_TBL[3];
extern const u8* ATK_ESP8266_WORKMODE_TBL[3];
extern const u8* ATK_ESP8266_ECN_TBL[5];
extern u8 WiFi_recvflag;  //WiFi���ձ�־λ
extern u8 app_singlerun_flag; //app���͵������������־λ
extern u8 app_taskrun_flag;  //app�����ı��ζ��������־λ
extern u16 app_runtime; //����app�趨����ʱ��
extern char app_task[200]; //�����б�



#endif





