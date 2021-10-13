#include "haizhe.h"

//USB_OTG_CORE_HANDLE    USB_OTG_dev;

void init_haizhe(){
	/*	*/
	delay_init(180);  //��ʼ����ʱ����
	my_mem_init(SRAMIN);		    //��ʼ���ڲ��ڴ��
	my_mem_init(SRAMCCM);		    //��ʼ��CCM�ڴ��
	//USBD_Init(&USB_OTG_dev,USB_OTG_FS_CORE_ID,&USR_desc,&USBD_CDC_cb,&USR_cb);	//USB�ӿڳ�ʼ��������mavlinkЭ�飩
	adc_init();//ADC��ʼ��
	//TODO:�ⲿ�жϳ�ʼ��
	//TODO:IIC��ʼ��
	//TODO:SPI��ʼ��
	//TODO:CAN��ʼ��
	//TODO:DAC��ʼ��
//	w5500_init();//spi���ڳ�ʼ��
	telem_init();//������̨��ʼ��
	rtc_init(); //RTC��ʼ��
	//WiFi_init();//WiFi��ʼ��
	ins_init();//AHRS��ʼ��
	gps_init();	//GPS��ʼ��
	baro_init();//��ȼƳ�ʼ��
	SD_FATFS_Init();	//sd����fat�ļ�ϵͳ��ʼ��
	//TODO:servo_init();//��̨��ʼ��
	relay_init();//�̵�����ʼ��
	//ppm_init();	//ң�ؽ���PPM�ӿڳ�ʼ��
	camera_init();	//�����ʼ��
	attitude_Controller_Init(1.0/loop_rate_hz);//��̬PID��������ʼ��[ע�⣺��Ҫʹ��1.0]
	position_Controller_Init(1.0/loop_rate_hz);//λ��PID��������ʼ�� 
	printf("[OK] PID controller init complete \r\n");
	setup_motors();//���غŵ��ģ�ͳ�ʼ��
	mission_init();	//��ȡ������
	//TODO:��openmv���͵�ǰ����
	printf("[OK] HaiZhe V2.0 init compelete! \r\n");
	printf("[OK] timer tick open \r\n");
	systick_init();//����ϵͳ��ʱ��(100Khz)		
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
	//192Mhz��Ƶ����
//  RCC_OscInitStruct.PLL.PLLM = 24;
//  RCC_OscInitStruct.PLL.PLLN = 384;
	//180Mhz���Ƶ������
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
	SCB->VTOR = FLASH_BASE | 0x10000;//����flashƫ����
	#endif
	HAL_Init(); //��ʼ��HAL��   
	SystemClock_Config();	//����ʱ��,180Mhz��ʹ��cubemx����
	CONSOLE_UART_Init(CONSOLE_UART,115200);//debug uart��ʼ��
	led_init();//led��ʼ��
	
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
	param_default_init();	//ʹ��Ĭ��ֵ��ʼ��ϵͳ����
	flash_ext_init();				    //�ⲿflash��ʼ��
	if(sys_flag.flash_check)	//��⵽�ⲿfalsh							
	{
		if(param_num_check())
		{
			flash_parameter_write();	//�����б����ʱ����д�����
			flash_parameter_read();		//��ȡflash�洢����������Ĭ�ϲ���
			return;
		}
		//��Щ����ö��ֵû��ʹ�� -> û�а�ȫ��������������б�
		printf("[warnning] the size of param_id_type_value is not equal to fp \r\n");
	}
}

