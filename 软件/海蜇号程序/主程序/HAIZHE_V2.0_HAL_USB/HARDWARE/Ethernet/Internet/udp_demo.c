/**
******************************************************************************
* @file   		udp_demo.c
* @author  		WIZnet Software Team 
* @version 		V1.0
* @date    		2015-02-14
* @brief   		UDP演示函数
******************************************************************************
**/
#include <stdio.h>
#include <string.h>
#include "w5500_conf.h"
#include "w5500.h"
#include "socket.h"
#include "utility.h"
#include "udp_demo.h"
#include "delay.h"
#include "malloc.h"
#include "parameter.h"

/**
*@brief		UDP测试程序
*@param		无
*@return	无
*/
bool create_UDP_socket(){
	u8 status = getSn_SR(SOCK_UDPS);
	if(status == SOCK_UDP){
//		printf("UDP socket allready created \r\n");
		return true;
	}
	u8 res=socket(SOCK_UDPS,Sn_MR_UDP,local_port,0);							  /*初始化socket*/
	if(res) printf("[OK] UDP socket create successful \r\n");
	else printf("[Error] UDP socket create failed! \r\n");
	return res;
}

void do_udp(u8 *buf, u16 *length)
{                                                              
	uint16 len=0;
	uint8 buff[2048];                                                          /*定义一个2KB的缓存*/	
	static u8 status_last = 0;
	u8 status_latest = getSn_SR(SOCK_UDPS);
	vp.sock_status = status_latest;
	create_UDP_socket();	//超时中断后socket会自动关闭，需要重启 -> 为什么会超时？
	if(status_latest != status_last){
		printf("network status : %d \r\n",status_latest);
		status_last = status_latest;
	}
	if(status_latest == SOCK_UDP){
		if(getSn_IR(SOCK_UDPS) & Sn_IR_RECV)
		{
			setSn_IR(SOCK_UDPS, Sn_IR_RECV);									 /*清接收中断*/
		}
		if((len=getSn_RX_RSR(SOCK_UDPS))>0) 								   /*接收到数据*/
		{
			recvfrom(SOCK_UDPS,buff, len, remote_ip,&remote_port);				 /*W5500接收计算机发送来的数据*/
			buff[len-8]=0x00;													 /*添加字符串结束符*/
//			for(int i=0;i<len-8;++i)printf("%02x ",buff[i]);printf("\r\n");
//			sendto(SOCK_UDPS,buff,len-8, remote_ip, remote_port);				 /*W5500把接收到的数据发送给Remote*/
			memcpy(buf,buff,len-8);
			*length = len-8;
		}
	}
}

void UDP_send(const u8 *buff,int len){
	u8 buf[50]="";
	memcpy(buf,buff,len);
	if(len==0) return;
	if(vp.sock_status != SOCK_UDP) return;
	u8 *p = mymalloc(SRAMIN, len);
	memcpy(p,buff,len);
	sendto(SOCK_UDPS,p,len, remote_ip, remote_port);
	myfree(SRAMIN,p);
}

