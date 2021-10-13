#include "telem_TTL_xG.h"

void TTL_xG_init(){
	 GPIO_InitTypeDef GPIO_InitStruct;
	
	TTL_xG_CLK_ENABLE();
	
  /*Configure GPIO pins : PE3 PE10 PE12 */
  GPIO_InitStruct.Pin = TTL_xG_M0|TTL_xG_M1|TTL_xG_AUX;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TTL_xG_GPIO, &GPIO_InitStruct);
	
	/*Configure GPIO pin Output Level */
	//模式0：M0=0 m1=0 定频模式  串口打开,无线打开,透明传输 接收方必须是模式 0、2
  HAL_GPIO_WritePin(TTL_xG_GPIO, TTL_xG_M0|TTL_xG_M1,GPIO_PIN_RESET);
//  HAL_GPIO_WritePin(TTL_xG_GPIO, TTL_xG_AUX,GPIO_PIN_SET);
	
	TTL_xG_UART_Init(TTL_xG_UART,TTL_xG_UART_Baud);
	sys_flag.telem_enable=true;
	printf("[OK] Telem_xG init baud %d \r\n",TTL_xG_UART_Baud);

}

