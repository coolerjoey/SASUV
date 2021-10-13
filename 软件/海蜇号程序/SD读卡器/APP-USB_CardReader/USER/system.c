#include "haizhe.h"

USB_OTG_CORE_HANDLE USB_OTG_dev;

void init_haizhe(){
	/*	*/
	delay_init(192);  //初始化延时函数
	my_mem_init(SRAMIN);		    //初始化内部内存池
	my_mem_init(SRAMCCM);		    //初始化CCM内存池
	led_init();//led初始化
	SD_FATFS_Init();	//sd卡和fat文件系统初始化
//	printf("USB Connecting...\r\n");	//提示正在建立连接 	
	MSC_BOT_Data=mymalloc(SRAMIN,MSC_MEDIA_PACKET);			//申请内存
	USBD_Init(&USB_OTG_dev,USB_OTG_FS_CORE_ID,&USR_desc,&USBD_MSC_cb,&USR_cb);
}


/** System Clock Configuration
 * 提升stm32f4主频至192Mhz（超频），为了4分频产生48Mhz的USB时钟频率
 * 192Mhz主频无法通过cubemx配置得到
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
  //196Mhz超频设置
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
	SCB->VTOR = FLASH_BASE | 0x100000;//设置flash偏移量
	#endif
	HAL_Init(); //初始化HAL库   
	SystemClock_Config();	//设置时钟,192Mhz。使用cubemx配置
	CONSOLE_UART_Init(CONSOLE_UART,115200);//debug uart初始化
	
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

