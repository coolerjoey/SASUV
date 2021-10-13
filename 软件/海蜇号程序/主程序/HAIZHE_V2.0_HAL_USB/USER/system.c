#include "haizhe.h"

//USB_OTG_CORE_HANDLE    USB_OTG_dev;

void init_haizhe(){
	/*	*/
	delay_init(180);  //初始化延时函数
	my_mem_init(SRAMIN);		    //初始化内部内存池
	my_mem_init(SRAMCCM);		    //初始化CCM内存池
	//USBD_Init(&USB_OTG_dev,USB_OTG_FS_CORE_ID,&USR_desc,&USBD_CDC_cb,&USR_cb);	//USB接口初始化（运行mavlink协议）
	adc_init();//ADC初始化
	//TODO:外部中断初始化
	//TODO:IIC初始化
	//TODO:SPI初始化
	//TODO:CAN初始化
	//TODO:DAC初始化
//	w5500_init();//spi网口初始化
	telem_init();//数传电台初始化
	rtc_init(); //RTC初始化
	//WiFi_init();//WiFi初始化
	ins_init();//AHRS初始化
	gps_init();	//GPS初始化
	baro_init();//深度计初始化
	SD_FATFS_Init();	//sd卡和fat文件系统初始化
	//TODO:servo_init();//云台初始化
	relay_init();//继电器初始化
	//ppm_init();	//遥控接收PPM接口初始化
	camera_init();	//相机初始化
	attitude_Controller_Init(1.0/loop_rate_hz);//姿态PID控制器初始化[注意：需要使用1.0]
	position_Controller_Init(1.0/loop_rate_hz);//位置PID控制器初始化 
	printf("[OK] PID controller init complete \r\n");
	setup_motors();//海蜇号电机模型初始化
	mission_init();	//读取航点数
	//TODO:向openmv发送当前日期
	printf("[OK] HaiZhe V2.0 init compelete! \r\n");
	printf("[OK] timer tick open \r\n");
	systick_init();//开启系统定时器(100Khz)		
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
	//192Mhz超频设置
//  RCC_OscInitStruct.PLL.PLLM = 24;
//  RCC_OscInitStruct.PLL.PLLN = 384;
	//180Mhz最大频率设置
	RCC_OscInitStruct.PLL.PLLM = 24;
	RCC_OscInitStruct.PLL.PLLN = 360;
	
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
	SCB->VTOR = FLASH_BASE | 0x10000;//设置flash偏移量
	#endif
	HAL_Init(); //初始化HAL库   
	SystemClock_Config();	//设置时钟,180Mhz。使用cubemx配置
	CONSOLE_UART_Init(CONSOLE_UART,115200);//debug uart初始化
	led_init();//led初始化
	
	printf("=============================================================\r\n");
	printf("     __   _____        ________________     __   _________ \r\n");
	printf("    / /  / / _ \\      /___  ___/____  /    / /  / / _____/ \r\n");
	printf("   / /__/ / /_\\ \\        / /       / /    / /__/ / /____ 	 \r\n");
	printf("  / ___  / _____ \\  	/ /       / /    / ___  / _____/   \r\n");
	printf(" / /  / / /     \\ \\  __/ /____   / /____/ /  / / /_____    \r\n");
	printf("/_/  /_/_/       \\_\\/________/  /______/_/  /_/_______/    \r\n");
	printf("*HAIZHE ROBOT Control Systemm Version:V2.0 \r\n");
	printf("*Author:cooler \r\n");
	printf("=============================================================\r\n\r\n");
	printf("[OK] System Clock Config \r\n");

}

void load_default_param(){
	param_default_init();	//使用默认值初始化系统参数
	flash_ext_init();				    //外部flash初始化
	if(sys_flag.flash_check)	//检测到外部falsh							
	{
		if(param_num_check())
		{
			flash_parameter_write();	//参数列表更改时重新写入参数
			flash_parameter_read();		//读取flash存储参数，覆盖默认参数
			return;
		}
		//有些参数枚举值没有使用 -> 没有把全部参数加入参数列表
		printf("[warnning] the size of param_id_type_value is not equal to fp \r\n");
	}
}

