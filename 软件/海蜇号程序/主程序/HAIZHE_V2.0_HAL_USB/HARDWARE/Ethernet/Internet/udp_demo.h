#ifndef __UDP_DEMO_H
#define __UDP_DEMO_H

#include "sys.h"
#include "Types.h"
extern uint16 udp_port;/*����UDP��һ���˿ڲ���ʼ��*/
void do_udp(u8 *buf, u16 *length);
void UDP_send(const u8 *buff,int len);
bool create_UDP_socket(void);

#endif 


