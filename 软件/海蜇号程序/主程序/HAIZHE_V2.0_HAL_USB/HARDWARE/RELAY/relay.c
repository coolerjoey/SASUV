#include "relay.h"
#include "parameter.h"
#include "motors.h"


extern SYS_FLAGS sys_flag;

void relay_init(){
	
	 GPIO_InitTypeDef GPIO_InitStruct;
	
	RELAY1_CLK_ENABLE();
//	RELAY2_CLK_ENABLE();
	RELAY_12V_CLK_ENABLE();

  GPIO_InitStruct.Pin = RELAY1_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RELAY1_GPIO, &GPIO_InitStruct);

//	  GPIO_InitStruct.Pin = RELAY2_PIN;
//  HAL_GPIO_Init(RELAY2_GPIO, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = RELAY_12V_PIN;
  HAL_GPIO_Init(RELAY_12V_GPIO, &GPIO_InitStruct);
	
	HAL_GPIO_WritePin(RELAY1_GPIO, RELAY1_PIN,GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(RELAY2_GPIO, RELAY2_PIN,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RELAY_12V_GPIO, RELAY_12V_PIN,GPIO_PIN_SET);

}

//12V�̵����ϵ���
void relay_12V_check(){
	if(sys_flag.relay_12V_enable){
		RELAY_12V = on;
		if(!sys_flag.esc_init_complete){	//�̵�����һ��������Ҫ���ص��
			Motor_ESC_Mount();	//���ص��
		}
	}
	else{
		sys_flag.esc_init_complete = false;
		RELAY_12V = off;
	}
}

bool get_relay_12V_status(){
	return( HAL_GPIO_ReadPin(RELAY_12V_GPIO, RELAY_12V_PIN));
}
