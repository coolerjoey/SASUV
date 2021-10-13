#ifndef _UART_H
#define _UART_H

#include "sys.h"
#include "stdarg.h"	 	 
#include "stdio.h"	 	 
#include "string.h"	
#include "timer.h"


//����1��غ궨��
#define USART1_MAX_RECV_LEN		800					//�����ջ����ֽ���
#define USART1_MAX_SEND_LEN		800					//����ͻ����ֽ���
extern u8  USART1_RX_BUF[USART1_MAX_RECV_LEN];
extern u16 USART1_RX_STA;

//����2��غ궨��
#define USART2_MAX_RECV_LEN		800					//�����ջ����ֽ���
#define USART2_MAX_SEND_LEN		800					//����ͻ����ֽ���
extern u8  USART2_RX_BUF[USART2_MAX_RECV_LEN];
extern u16 USART2_RX_STA;

//����3��غ궨��
#define USART3_MAX_RECV_LEN		800					//�����ջ����ֽ���
#define USART3_MAX_SEND_LEN		800					//����ͻ����ֽ���
extern u8  USART3_RX_BUF[USART3_MAX_RECV_LEN];
extern u16 USART3_RX_STA;

//����6��غ궨��
#define USART6_MAX_RECV_LEN		80					//�����ջ����ֽ���
#define USART6_MAX_SEND_LEN		800					//����ͻ����ֽ���
extern u8  USART6_RX_BUF[USART6_MAX_RECV_LEN];
extern u16 USART6_RX_STA;

//����7��غ궨��
#define UART7_MAX_RECV_LEN		800					//�����ջ����ֽ���
#define UART7_MAX_SEND_LEN		800					//����ͻ����ֽ���
extern u8  UART7_RX_BUF[UART7_MAX_RECV_LEN];
extern u16 UART7_RX_STA;

//����8��غ궨��
#define UART8_MAX_RECV_LEN		800					//�����ջ����ֽ���
#define UART8_MAX_SEND_LEN		800					//����ͻ����ֽ���
#define UART8_TIM				TIM3
#define UART8_TIM_Init TIM3_Int_Init
extern u8  UART8_RX_BUF[UART8_MAX_RECV_LEN];
extern u16 UART8_RX_STA;

//���ڽ����ж�ʹ��
#define EN_USART1_RX 			1		//ʹ�ܣ�1��/��ֹ��0������1����
#define EN_USART2_RX 			1
#define EN_USART3_RX 			0
#define EN_USART4_RX 			0
#define EN_USART6_RX      1
#define EN_UART7_RX 			1
#define EN_UART8_RX 			1
	  	
extern UART_HandleTypeDef UART1_Handler; //UART���

#define RXBUFFERSIZE   1 //�����С
extern u8 u1_RxBuffer[RXBUFFERSIZE];//HAL��USART����Buffer

void UART_init(USART_TypeDef* UART,u32 baud);
void u1_printf(char* fmt,...);  
void u2_printf(char* fmt,...); 
void u3_printf(char* fmt,...); 
void u6_printf(char* fmt,...);
void u7_printf(char* fmt,...);  
void u8_printf(char* fmt,...);  
void uart_write(USART_TypeDef* UART,char* buf,int num);

#endif
