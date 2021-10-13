#include "baro_ms5803.h"
#include <stdio.h>


	u16 C0,C1,C2,C3,C4,C5,C6;
	u32 D1,D2;
	u32 dT,TEMP,P,T2;
	int64_t OFF,SENS,OFF2,SENS2;
	float deep;
	float Dep;


u8 MS5803_Write_Byte(u8 reg,u8 data) 				 
{ 
  MS5803_IIC_Start(); 
	MS5803_IIC_Send_Byte((MS5803_ADDR<<1)|0);//����������ַ+д����	
	if(MS5803_IIC_Wait_Ack())	//�ȴ�Ӧ��
	{
		MS5803_IIC_Stop();		 
		return 1;		
	}
    MS5803_IIC_Send_Byte(reg);	//д�Ĵ�����ַ
    MS5803_IIC_Wait_Ack();		//�ȴ�Ӧ�� 
	MS5803_IIC_Send_Byte(data);//��������
	if(MS5803_IIC_Wait_Ack())	//�ȴ�ACK
	{
		MS5803_IIC_Stop();	 
		return 1;		 
	}		 
    MS5803_IIC_Stop();	 
	return 0;
}

u8 MS5803_Read_Byte(u8 reg)
{
	u8 res;
			MS5803_IIC_Start(); 
	MS5803_IIC_Send_Byte((MS5803_ADDR<<1)|0);		//����������ַ+д����	
	MS5803_IIC_Wait_Ack();			//�ȴ�Ӧ�� 
			MS5803_IIC_Send_Byte(reg);		//д�Ĵ�����ַ
			MS5803_IIC_Wait_Ack();			//�ȴ�Ӧ��
			MS5803_IIC_Start();
	MS5803_IIC_Send_Byte((MS5803_ADDR<<1)|1);	//����������ַ+������	
			MS5803_IIC_Wait_Ack();			//�ȴ�Ӧ�� 
	res=MS5803_IIC_Read_Byte(0);		//��ȡ����,����nACK 
			MS5803_IIC_Stop();				//����һ��ֹͣ���� 
	return res;		
}

u8 MS5803_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{ 
 	MS5803_IIC_Start(); 
	MS5803_IIC_Send_Byte((addr<<1)|0);//����������ַ+д����	
	if(MS5803_IIC_Wait_Ack())	//�ȴ�Ӧ��
	{
		MS5803_IIC_Stop();		 
		return 1;		
	}
    MS5803_IIC_Send_Byte(reg);	//д�Ĵ�����ַ
    MS5803_IIC_Wait_Ack();		//�ȴ�Ӧ��
    MS5803_IIC_Start();
	MS5803_IIC_Send_Byte((addr<<1)|1);//����������ַ+������	
    MS5803_IIC_Wait_Ack();		//�ȴ�Ӧ�� 
	while(len)
	{
		if(len==1)*buf=MS5803_IIC_Read_Byte(0);//������,����nACK 
		else *buf=MS5803_IIC_Read_Byte(1);		//������,����ACK  
		len--;
		buf++; 
	}    
    MS5803_IIC_Stop();	//����һ��ֹͣ���� 
	return 0;	
}

u16 MS5803_Read_PROM(u8 addr){
	u8 buf[2]; 
	u16 res;
	MS5803_Read_Len(MS5803_ADDR,addr,2,buf);
	res=((u16)buf[0]<<8)|buf[1]; 	
	return res;

}


u32 MS5803_Read_ADC(u8 addr){
		u8 buf[3]; 
		u32 res;
		//MS5803_Write_Byte(MS5803_ADDR,addr);
		MS5803_IIC_Start(); 
		MS5803_IIC_Send_Byte((MS5803_ADDR<<1)|0);//����������ַ+д����	
		if(MS5803_IIC_Wait_Ack())	//�ȴ�Ӧ��
		{
			MS5803_IIC_Stop();		 
			return 1;		
		}
		MS5803_IIC_Send_Byte(addr);	//д�Ĵ�����ַ
		MS5803_IIC_Wait_Ack();
    MS5803_IIC_Stop();	 
		delay_ms(10);
		MS5803_Read_Len(MS5803_ADDR,ADC_READ,3,buf);
		res=((u32)buf[0]<<16)|((u16)buf[1]<<8)|buf[2]; 	
	return res;
	
}

void MS5803_Get_data(float* dep){
		D1=MS5803_Read_ADC(CONVERT_D14);//ѹ��ֵ����
		D2=MS5803_Read_ADC(CONVERT_D24);//�¶�����
		
//		printf("D1Ϊ: %d \r\n",D1);
//		printf("D2Ϊ: %d \r\n",D2);
		
		
		
		dT  = D2-C5*256;
		TEMP= 2000+dT*C6/8388608;
		OFF = C2*65536+(C4*dT)/128;
		SENS= C1*32768+(C3*dT)/256;
		P   = ( D1*(SENS>>21) - OFF )/32768;
		
//		dT  = D2 - (C5<<8);
//		TEMP= 2000 + dT*C6 >> 23;
//		OFF = (C2<<16) + ((C4*dT)>>7);
//		SENS= (C2<<15) + ((C3*dT)>>8);		
//		P   = ( D1*(SENS>>21) - OFF )>>15;
		
		
//		printf("dTΪ: %d \r\n",dT);
//		printf("TEMPΪ: %d \r\n",TEMP);
//		printf("OFFΪ: %lld \r\n",OFF);
//		printf("SENSΪ: %lld \r\n",SENS);
//		printf("PΪ: %d \r\n",P);
//		
		
		if(TEMP<2000)
		{
			T2=(3*dT*dT)/8589934592;
			OFF2=3*(TEMP-2000)*(TEMP-2000)/2;
			SENS2=5*(TEMP-2000)*(TEMP-2000)/8;
			if(TEMP<-1500)
			{
				OFF2=OFF2+7*(TEMP+1500)*(TEMP+1500);
				SENS2=SENS2+4*(TEMP+1500)*(TEMP+1500);
			}
		}else
		{
			T2=7*dT*dT/137438953472;
			OFF2=1*(TEMP-2000)*(TEMP-2000)/16;
			SENS2=0;
		}		
		
		
//		if(TEMP<2000)
//		{
//			T2   = (3*dT*dT)>>33;
//			OFF2 = (3*(TEMP-2000)*(TEMP-2000))>>1;
//			SENS2= (5*(TEMP-2000)*(TEMP-2000))>>3;
//			if(TEMP<-1500)
//			{
//				OFF2=OFF2+7*(TEMP+1500)*(TEMP+1500);
//				SENS2=SENS2+4*(TEMP+1500)*(TEMP+1500);
//			}
//		}else
//		{
//			T2   = (7*dT*dT)>>37;
//			OFF2 = (1*(TEMP-2000)*(TEMP-2000))>>4;
//			SENS2=0;
//		}		
		
		TEMP = TEMP - T2;
		OFF  = OFF - OFF2;
		SENS = SENS - SENS2;
		P    = ((D1*(SENS>>21)-OFF)>>15);
		P = 1.0125*P - 100009*1.0125;
		deep = P/(1000*9.8);
		*dep = (-0.5235) * deep*deep*deep + 1.247*deep*deep + (-0.01609)*deep + (-0.03725);
		
//		printf("SENSΪ: %lld \r\n",SENS);
//		printf("OFFΪ: %lld \r\n",OFF);
//		printf("detaPΪ��%d \r\n", P);
//		printf("DepΪ: %.5f \r\n",*dep);
//		printf("TEMPΪ: %d \r\n\r\n",TEMP);

	

}


void MS5803_Init(void)
{
	MS5803_IIC_Init();
	MS5803_Write_Byte(MS5803_ADDR,MS_RESET);
	delay_ms(100);
	//��ȡУ׼ֵ
	//	C1=46546; // ��Щ�ǵ���ֵ(���������ֲ�)�������Լ������ݲ�𲻴�˵��һ��OK
//	C2=42845;
//	C3=29751;
//	C4=29457;
//	C5=32745;
//	C6=29059;
	C1 = MS5803_Read_PROM(MS5803_PROM_C1);/*C1 ѹ��������*/
	C2 = MS5803_Read_PROM(MS5803_PROM_C2);/*C2 ѹ������ֵ*/
	C3 = MS5803_Read_PROM(MS5803_PROM_C3);/*C3 ѹ���������¶�ϵ��*/
	C4 = MS5803_Read_PROM(MS5803_PROM_C4);/*C4 ѹ�������¶�ϵ��*/
	C5 = MS5803_Read_PROM(MS5803_PROM_C5);/*C5 �ο��¶�*/
	C6 = MS5803_Read_PROM(MS5803_PROM_C6);/*C6 �¶ȴ������¶�ϵ��*/
	printf("The MS5803 Calibration Data is :%d, %d, %d, %d, %d, %d\r\n",C1,C2,C3,C4,C5,C6);
}




