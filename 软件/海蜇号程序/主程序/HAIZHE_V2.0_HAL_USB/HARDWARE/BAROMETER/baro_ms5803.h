#ifndef _BARO_MS5803_H_
#define _BARO_MS5803_H_

#include "sys.h"
#include "delay.h"
#include "ms5803iic.h"

//MS5803地址配置	
#define MS5803_ADDR 0x76  //在 CSP 接高电平时，地址为 0x76(1110110 b)，而 CSB 接低电平时，地址为 0x77

//MS5803命令配置
#define MS_RESET		0x1E
#define CONVERT_D11	0x40
#define CONVERT_D12	0x42
#define CONVERT_D13	0x44
#define CONVERT_D14 0x46
#define CONVERT_D15	0x48
#define CONVERT_D21	0x50
#define CONVERT_D22	0x52
#define CONVERT_D23	0x54
#define CONVERT_D24	0x56
#define CONVERT_D25	0x58
#define ADC_READ		0x00
#define	MS5803_PROM_C0    0xA0	//
#define	MS5803_PROM_C1    0xA2	//MS5803 Pressure sensitivity | SENS T1
#define	MS5803_PROM_C2    0xA4	//MS5803 Pressure offset | OFF T1
#define	MS5803_PROM_C3    0xA6	//MS5803 Temperature coefficient of pressure sensitivity | TCS
#define	MS5803_PROM_C4    0xA8	//MS5803 Temperature coefficient of pressure offset | TCO
#define	MS5803_PROM_C5    0xAA	//MS5803 Reference temperature | T REF
#define	MS5803_PROM_C6    0xAC	//MS5803 Temperature coefficient of the temperature | TEMPSENS

//extern	u16 C0,C1,C2,C3,C4,C5,C6;
//extern	u32 D1,D2;
//extern	u32 dT,TEMP,P,T2;
//extern	int64_t OFF,SENS,OFF2,SENS2;
extern	float Dep;


void MS5803_Init(void);
u8 MS5803_Write_Byte(u8 reg,u8 data);	
u8 MS5803_Read_Byte(u8 reg);
u8 MS5803_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf);
u32 MS5803_Read_ADC(u8 addr);
u16 MS5803_Read_PROM(u8 addr);
void MS5803_Get_data(float* dep);

#endif
