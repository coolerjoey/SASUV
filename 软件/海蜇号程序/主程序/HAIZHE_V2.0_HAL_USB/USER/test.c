#include "haizhe.h"

//flash test
void flash_test(){
	int adress = 4096;
	static u8 loop_num = 0;
	u8 p[100];
	sprintf((char*)p,"flash_test %d \r\n",loop_num++);
	if(loop_num == 100) loop_num = 0;
	flash_write(p,4094*adress,100);
	flash_read(p,4094*adress,100);
	printf("%s",p);
}

void rgbled_test(){
	u8 red = 10;
	for(u8 i=0;i<24;i++)
	{
		red += 10;
		LED_RGB_Set(red,0,0);
		delay_ms(50);
	}
	for(u8 i=0;i<24;i++)
	{
		red -= 10;
		LED_RGB_Set(red,0,0);
		delay_ms(50);
	}	
}
