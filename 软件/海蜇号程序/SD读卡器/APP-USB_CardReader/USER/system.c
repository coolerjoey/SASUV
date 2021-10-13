#include "haizhe.h"

USB_OTG_CORE_HANDLE USB_OTG_dev;

void init_haizhe(){
	/*	*/
	delay_init(192);  //��ʼ����ʱ����
	my_mem_init(SRAMIN);		    //��ʼ���ڲ��ڴ��
	my_mem_init(SRAMCCM);		    //��ʼ��CCM�ڴ��
	led_init();//led��ʼ��
	SD_FATFS_Init();	//sd����fat�ļ�ϵͳ��ʼ��
//	printf("USB Connecting...\r\n");	//��ʾ���ڽ������� 	
	MSC_BOT_Data=mymalloc(SRAMIN,MSC_MEDIA_PACKET);			//�����ڴ�
	USBD_Init(&USB_OTG_dev,USB_OTG_FS_CORE_ID,&USR_desc,&USBD_MSC_cb,&USR_cb);
}


/** System Clock Configuration
 * ����stm32f4��Ƶ��192Mhz����Ƶ����Ϊ��4��Ƶ����48Mhz��USBʱ��Ƶ��
 * 192Mhz��Ƶ�޷�ͨ��cubemx���õõ�
**/

void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 24;
  //196Mhz��Ƶ����
	RCC_OscInitStruct.PLL.PLLN = 384;
	
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  HAL_PWREx_EnableOverDrive();

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

}

void system_init(){
	#if BOOT_ENABLE==true
	SCB->VTOR = FLASH_BASE | 0x100000;//����flashƫ����
	#endif
	HAL_Init(); //��ʼ��HAL��   
	SystemClock_Config();	//����ʱ��,192Mhz��ʹ��cubemx����
	CONSOLE_UART_Init(CONSOLE_UART,115200);//debug uart��ʼ��
	
	printf("====================================================================\r\n");
	printf("     __   _____        ________________     __   _________ \r\n");
	printf("    / /  / / _ \\      /___  ___/____  /    / /  / / _____/ \r\n");
	printf("   / /__/ / /_\\ \\        / /       / /    / /__/ / /____ 	 \r\n");
	printf("  / ___  / _____ \\  	/ /       / /    / ___  / _____/   \r\n");
	printf(" / /  / / /     \\ \\  __/ /____   / /____/ /  / / /_____    \r\n");
	printf("/_/  /_/_/       \\_\\/________/  /______/_/  /_/_______/    \r\n");
	printf("*HAIZHE ROBOT USB DISK Version:V1.0 \r\n");
	printf("*Author:cooler \r\n");
	printf("====================================================================\r\n\r\n");
	printf("[OK] System Clock Config \r\n");

}

