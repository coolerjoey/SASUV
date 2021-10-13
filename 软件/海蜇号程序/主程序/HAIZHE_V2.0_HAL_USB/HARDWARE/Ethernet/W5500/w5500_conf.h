#ifndef _W5500_CONF_H_
#define _W5500_CONF_H_

#include "stdio.h"
#include "types.h"
#include "sys.h"
#define __GNUC__

typedef  void (*pFunction)(void);

#define W5500_SPI_ReadWriteByte(data) SPI_ReadWriteByte(SPI4,data)


extern uint8  	remote_ip[4];															/*Զ��IP��ַ*/
extern uint16 	remote_port;															/*Զ�˶˿ں�*/
extern uint16		local_port;																/*���屾�ض˿�*/
extern uint8  	use_dhcp;																	/*�Ƿ�ʹ��DHCP��ȡIP*/
extern uint8  	use_eeprom;																/*�Ƿ�ʹ��EEPROM�е�IP������Ϣ*/

/*����SPI1��ΪW5500��Ӳ���ӿ�*/
#define W5500_SPI_init() SPI_Init(SPI4)
#define W5500_SPI_SetSpeed(baudrate)	SPI_SetSpeed(SPI4,baudrate)
#define W5500_SPI_ReadWriteByte(data) 	SPI_ReadWriteByte(SPI4,data)
/*����W5500��Ƭѡ�ܽ�*/
#define	W5500_CS_CLK_ENABLE() 			__HAL_RCC_GPIOA_CLK_ENABLE()         
#define	W5500_CS_GPIO					GPIOA
#define	W5500_CS_PIN 					GPIO_PIN_3	
#define	W5500_CS 						PAout(3)  	
/*����W5500��RESET�ܽ�*/
#define	W5500_RESET_CLK_ENABLE() 		__HAL_RCC_GPIOA_CLK_ENABLE()         
#define	W5500_RESET_GPIO				GPIOA
#define	W5500_RESET_PIN 				GPIO_PIN_1
#define	W5500_RESET 					PAout(1)  	
/*����W5500��INT�ܽ�*/
#define	W5500_INT_CLK_ENABLE() 			__HAL_RCC_GPIOA_CLK_ENABLE()         
#define	W5500_INT_GPIO					GPIOA
#define	W5500_INT_PIN 					GPIO_PIN_2	
#define	W5500_INT 						PAout(2)  	


/*ʹ��Ұ��ISO��׼������*/
#define STM32F103ZET6

#ifdef  STM32F103ZET6 

	
#endif

/*ʹ��Ұ��MINI������*/
//#define STM32F103VET6

#ifdef STM32F103VET6    

	#define WIZ_RESET							GPIO_Pin_6								/*����W5500��RESET�ܽ�*/
	#define WIZ_INT								GPIO_Pin_5								/*����W5500��INT�ܽ�*/
#endif

#define FW_VER_HIGH  						1               
#define FW_VER_LOW    					0
#define ON	                 		1
#define OFF	                 		0
#define HIGH	           	 			1
#define LOW		             			0

#define MAX_BUF_SIZE		 				1460											 /*����ÿ�����ݰ��Ĵ�С*/
#define KEEP_ALIVE_TIME	     		30	// 30sec
#define TX_RX_MAX_BUF_SIZE      2048							 
#define EEPROM_MSG_LEN        	sizeof(EEPROM_MSG)

#define IP_FROM_DEFINE	           	 			0                /*ʹ�ó�ʼ�����IP��Ϣ*/
#define IP_FROM_DHCP	              			1                /*ʹ��DHCP��ȡIP��Ϣ*/
#define IP_FROM_EEPROM	                  2								 /*ʹ��EEPROM�����IP��Ϣ*/
extern uint8	ip_from;											               /*ѡ��IP��Ϣ����Դ*/
#pragma pack(1)
/*�˽ṹ�嶨����W5500�ɹ����õ���Ҫ����*/
typedef struct _CONFIG_MSG											
{
  uint8 mac[6];																							/*MAC��ַ*/
  uint8 lip[4];																							/*local IP����IP��ַ*/
  uint8 sub[4];																							/*��������*/
  uint8 gw[4];																							/*����*/	
  uint8 dns[4];																							/*DNS��������ַ*/
  uint8 rip[4];																							/*remote IPԶ��IP��ַ*/
	uint8 sw_ver[2];																					/*����汾��*/

}CONFIG_MSG;
#pragma pack()

#pragma pack(1)
/*�˽ṹ�嶨����eepromд��ļ����������ɰ����޸�*/
typedef struct _EEPROM_MSG	                    
{
	uint8 mac[6];																							/*MAC��ַ*/
  uint8 lip[4];																							/*local IP����IP��ַ*/
  uint8 sub[4];																							/*��������*/
  uint8 gw[4];																							/*����*/
}EEPROM_MSG_STR;
#pragma pack()

extern EEPROM_MSG_STR EEPROM_MSG;
extern CONFIG_MSG  	ConfigMsg;
extern uint8 dhcp_ok;																				/*DHCP��ȡ�ɹ�*/
extern uint32	dhcp_time;																		/*DHCP���м���*/
extern vu8	ntptimer;																				/*NPT�����*/

/*MCU������غ���*/
void gpio_for_w5500_config(void);														/*SPI�ӿ�reset ���ж�����*/
void timer2_init(void);																			/*STM32��ʱ��2��ʼ��*/
void timer2_isr(void);																			/*��ʱ���ж�ִ�еĺ���*/
void reboot(void);																					/*STM32��λ*/
void write_config_to_eeprom(void);													/*д������Ϣ��EEPROM��*/
void read_config_from_eeprom(void);													/*��EEPROM�ж�����Ϣ*/

/*W5500SPI��غ���*/
void IINCHIP_WRITE( uint32 addrbsb,  uint8 data);						/*д��һ��8λ���ݵ�W5500*/
uint8 IINCHIP_READ(uint32 addrbsb);													/*��W5500����һ��8λ����*/
uint16 wiz_write_buf(uint32 addrbsb,uint8* buf,uint16 len);	/*��W5500д��len�ֽ�����*/
uint16 wiz_read_buf(uint32 addrbsb, uint8* buf,uint16 len);	/*��W5500����len�ֽ�����*/

/*W5500����������غ���*/
void reset_w5500(void);																			/*Ӳ��λW5500*/
void set_w5500_mac(void);																		/*����W5500��MAC��ַ*/
void set_w5500_ip(void);																		/*����W5500��IP��ַ*/
void w5500_init(void);

/*��Ҫ�ö�ʱ�ĵ�Ӧ�ú���*/
void dhcp_timer_init(void);																	/*dhcp�õ��Ķ�ʱ����ʼ��*/
void ntp_timer_init(void);																	/*npt�õ��Ķ�ʱ����ʼ��*/


#endif
