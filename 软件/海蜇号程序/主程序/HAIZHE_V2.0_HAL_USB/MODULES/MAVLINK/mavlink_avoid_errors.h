/** @file mavlink_avoid_errors.h
* @简介：本文件是由624668529添加，用来统一解决mavlink报错信息
* @see QQ624668529
*/
#ifndef MAVLINK_AVOID_ERRORS_H
#define MAVLINK_AVOID_ERRORS_H

#define MAVLINK_USE_CONVENIENCE_FUNCTIONS
#define MAVLINK_SEPARATE_HELPERS

#define MAVLINK_SEND_UART_BYTES

/*解决..\MAVLINK\common\../mavlink_types.h(53): error: #20: identifier "pack" is undefined*/
#define MAVPACKED( __Declaration__ ) __Declaration__
/*解决..\MAVLINK\common\../mavlink_types.h(53): error: #3092: anonymous unions are only supported in --gnu mode, or when enabled with #pragma anon_unions*/
#pragma anon_unions
#define inline __INLINE

//#ifndef memset//由624668529添加 2018-08-24
//static inline void* memset(void*dest,int data,size_t length){
//	uint32_t i;
//	int*point = dest;
//	for(i=0; i<length; i++) point[i]= data;
//	return dest;
//}
//#endif
//#ifndef memcpy//由624668529添加 2018-08-24
//void*memcpy(void*dest,const void*src,size_t n)
//{
//	unsigned char* pout =(unsigned char*)dest;
//	unsigned char* pin =(unsigned char*)src;
//	while(n-->0)*pout++=*pin++;
//	return dest;
//}
//#endif

#include"mavlink_types.h"
#include "sys.h"
#include "config.h"
#include "string.h"
#include "parameter.h"

//mavlink_system_t mavlink_system = {0,0};
mavlink_system_t mavlink_system ={
	0x01,
	0x01
};// System ID, 1-255, Component/Subsystem ID, 1-255
void comm_send_ch(mavlink_channel_t chan,uint8_t buf)
{
	chan=chan;
	while((MAVLINK_UART->SR&0X40)==0);			//循环发送,直到发送完毕   
	MAVLINK_UART->DR=buf;
	vp.data_size_one_second++;
//	if(buf == 0xfe) printf("\r\n");
//	printf("%x ",buf);
//	VCP_DataTx(buf); 
}

void comm_send(mavlink_channel_t chan, const uint8_t *buf, uint16_t len){
	chan=chan;
	vp.data_size_one_second += len;	//数据量累积
	mavsend(buf,len);
}

#include "mavlink.h"
#include "mavlink_helpers.h"

#endif	//AVLINK_AVOID_ERRORS_H

