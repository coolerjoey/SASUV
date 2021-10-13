#ifndef _UART_H
#define _UART_H

#include "sys.h"
#include "stdarg.h"	 	 
#include "stdio.h"	 	 
#include "string.h"	

#include "timer.h"

#define RXBUFFERSIZE   100 //缓存大小


//串口1相关宏定义
#define USART1_MAX_RECV_LEN		800					//最大接收缓存字节数
#define USART1_MAX_SEND_LEN		800					//最大发送缓存字节数
extern u8  USART1_RX_BUF[USART1_MAX_RECV_LEN];
extern u16 USART1_RX_STA;

//串口2相关宏定义
#define USART2_MAX_RECV_LEN		800					//最大接收缓存字节数
#define USART2_MAX_SEND_LEN		800					//最大发送缓存字节数
extern u8  USART2_RX_BUF[USART2_MAX_RECV_LEN];
extern u16 USART2_RX_STA;

//串口3相关宏定义
#define USART3_MAX_RECV_LEN		80					//最大接收缓存字节数
#define USART3_MAX_SEND_LEN		80					//最大发送缓存字节数
extern u8  USART3_RX_BUF[USART3_MAX_RECV_LEN];
extern u16 USART3_RX_STA;

//串口6相关宏定义
#define USART6_MAX_RECV_LEN		80					//最大接收缓存字节数
#define USART6_MAX_SEND_LEN		80					//最大发送缓存字节数
extern u8  USART6_RX_BUF[USART6_MAX_RECV_LEN];
extern u16 USART6_RX_STA;

//串口7相关宏定义
#define UART7_MAX_RECV_LEN		800					//最大接收缓存字节数
#define UART7_MAX_SEND_LEN		800					//最大发送缓存字节数
extern u8  UART7_RX_BUF[UART7_MAX_RECV_LEN];
extern u16 UART7_RX_STA;

//串口8相关宏定义
#define UART8_MAX_RECV_LEN		800					//最大接收缓存字节数
#define UART8_MAX_SEND_LEN		800					//最大发送缓存字节数
#define UART8_TIM				TIM3
#define UART8_TIM_Init TIM3_Int_Init
extern u8  UART8_RX_BUF[UART8_MAX_RECV_LEN];
extern u16 UART8_RX_STA;

extern DMA_HandleTypeDef hdma_uart7_rx, hdma_uart7_tx;
extern UART_HandleTypeDef huart1,huart2,huart3,huart6,huart7,huart8;

void UART_init(USART_TypeDef* UART,u32 baud);
void u1_printf(char* fmt,...);  
void u2_printf(char* fmt,...); 
void u3_printf(char* fmt,...); 
void u6_printf(char* fmt,...);
void u7_printf(const char* fmt,...);  
void u8_printf(char* fmt,...);  
void uart_write(USART_TypeDef* UART,const char* buf,int num);

void mavsend(const uint8_t *buf, uint16_t len);


#endif
