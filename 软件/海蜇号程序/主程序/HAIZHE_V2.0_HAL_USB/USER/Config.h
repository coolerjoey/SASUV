#ifndef _CONFIG_H
#define _CONFIG_H

#include "sys.h"
#include "timer.h"
#include "parameter.h"
#include "uart.h"

#define BOOT_ENABLE true	//true-��bootloader��ת��false-��boot ��ע�⣺Ϊtrueʱ��Ҫ�޸�ROM��ַ��	

#define QGC_ENABLE true	//true-ʹ��qgc��λ����false-ʹ������������λ��

#define USB_MAVLINK_EN 	true	//USB�����mavlink����

#define REMOTE_TYPE T8FB_RC	//ң�����ͺ�

#define systick_init()	TIM5_Int_Init(10-1,96-1)	//90/10/90 = 100Khz

//�������ͺ�
#define IMU_TYPE JY901	//IMU
#define EC_TYPE  DDM350B 	//��������
#define TELEM_USE TTL_xG			//������̨
#define BARO_USE BARO_AV			//��ȼ�		//BARO_MS5803 


#define MAVLINK_UART UART7	//mavlink��Ϣ������ں궨��

//IMU�ӿ�
#define IMU_UART USART1 
#define IMU_UART_Baud fp.uartA_baud
#define IMU_UART_Init(uart,baud) UART_init(uart,baud)
#define IMU_UART_write(buf,size) uart_write(USART1,buf,size)
#define IMU_uart_available USART1_RX_STA
#define IMU_uart_buf USART1_RX_BUF


//�������̽ӿ�
#define EC_UART USART3
#define EC_UART_Baud fp.uartE_baud
#define EC_UART_Init(uart,baud) UART_init(uart,baud)
#define EC_UART_write(buf,size) uart_write(USART3,buf,size)
#define EC_uart_available USART3_RX_STA
#define EC_uart_buf USART3_RX_BUF


#define GPS_UART USART2
#define GPS_UART_Baud fp.uartD_baud
#define GPS_UART_Init(uart,baud) UART_init(uart,baud)
#define GPS_UART_write(buf,size) UART_write(GPS_UART,buf,size)
#define GPS_UART_available USART2_RX_STA
#define GPS_UART_BUF USART2_RX_BUF

#define CONSOLE_UART USART6
#define CONSOLE_UART_Init(uart,baud) UART_init(USART6,baud)
#define CONSOLE_uart_available USART6_RX_STA
#define CONSOLE_uart_buf USART6_RX_BUF


#if TELEM_USE==TTL_xG
	#define TTL_xG_UART UART7
	#define TTL_xG_UART_Baud fp.uartC_baud
	#define TTL_xG_UART_Init(uart,baud) UART_init(uart,baud)
	#define TTL_xG_Send u7_printf
	#define TTL_xG_write(buf,size) uart_write(UART7,buf,size)
	#define TTL_xG_UART_RX_STA	UART7_RX_STA
	#define TTL_xG_UART_RX_BUF	UART7_RX_BUF
#endif




#endif
