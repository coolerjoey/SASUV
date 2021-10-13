/** @file mavlink_avoid_errors.h
* @��飺���ļ�����624668529��ӣ�����ͳһ���mavlink������Ϣ
* @see QQ624668529
*/
#ifndef MAVLINK_AVOID_ERRORS_H
#define MAVLINK_AVOID_ERRORS_H
/*���..\MAVLINK\common\../mavlink_types.h(53): error: #20: identifier "pack" is undefined*/
#define MAVPACKED( __Declaration__ ) __Declaration__
/*���..\MAVLINK\common\../mavlink_types.h(53): error: #3092: anonymous unions are only supported in --gnu mode, or when enabled with #pragma anon_unions*/
#pragma anon_unions
#define inline __INLINE
#ifndef memset//��624668529��� 2018-08-24
static inline void* memset(void*dest,int data,size_t length){
	uint32_t i;
	int*point = dest;
	for(i=0; i<length; i++) point[i]= data;
	return dest;
}
#endif
#ifndef memcpy//��624668529��� 2018-08-24
void*memcpy(void*dest,const void*src,size_t n)
{
	unsigned char* pout =(unsigned char*)dest;
	unsigned char* pin =(unsigned char*)src;
	while(n-->0)*pout++=*pin++;
	return dest;
}
#include"mavlink_types.h"
#include "sys.h"
#include "config.h"
#define MAVLINK_USE_CONVENIENCE_FUNCTIONS
#define MAVLINK_SEPARATE_HELPERS
//mavlink_system_t mavlink_system = {0,0};
mavlink_system_t mavlink_system ={
1,
1
};// System ID, 1-255, Component/Subsystem ID, 1-255
void comm_send_ch(mavlink_channel_t chan,uint8_t buf)
{
	chan=chan;
	while((MAVLINK_UART->SR&0X40)==0);			//ѭ������,ֱ���������   
	MAVLINK_UART->DR=buf;  
//	if(buf == 0xfe) printf("\r\n");
//	printf("%x ",buf);
}
#endif
#include "mavlink.h"
#include "mavlink_helpers.h"
#endif	//AVLINK_AVOID_ERRORS_H

