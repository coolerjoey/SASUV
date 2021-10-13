/******************************************************************************

                  版权所有 (C), 2010-2017, 北京贝虎机器人技术有限公司

 ******************************************************************************
  文 件 名   : tm7812b.c
  版 本 号   : 初稿
  作    者   : AF
  生成日期   : 2017年6月7日
  最近修改   :
  功能描述   : TM7812B底层驱动 使用的硬件有TIM5,CHANNLE_1/DMA
               通过定时器+DMA方式产生tm7812需要的波形
               800kHz发送频率，1.25us每周期。
               定时器和DMA通过cube生成，并自动初始化
  函数列表   :
              dataChange
              sign
              TM7812B_Init
              TM7812B_Test
              TM7812B_WheelTrack
              TM7812B_Wheel_1
              tm7812_flash_1
              tm7812_flash_2
              tm7812_flash_3
              TM7812_Process
              TM7812_set_flash
              TM7812_set_Wheel_1
              TM7812_show
  修改历史   :
  1.日    期   : 2017年6月7日
    作    者   : AF
    修改内容   : 创建文件

******************************************************************************/

/*----------------------------------------------*
 * 包含头文件                                   *
 *----------------------------------------------*/
#include "tm7812b.h"
#include "Adafruit_NeoPixel.h"
#include "delay.h"
//#include "BottomMotor.h"
#include <math.h>
#include "sys.h"
#include "timer.h"
/*----------------------------------------------*
 * 宏定义                                       *
 *----------------------------------------------*/
#define FLASH_WHEEL_1 1
#define PI                      3.1415692f
/*TIM+DMA输出*/
#define BIT_1                   22
#define BIT_0                   6

//#define ONCE_LEN                32                          //单次dma长度 word
/*----------------------------------------------*
 * 常量定义                                     *
 *----------------------------------------------*/

/*----------------------------------------------*
 * 外部变量说明                                 *
 *----------------------------------------------*/

/*----------------------------------------------*
 * 外部函数原型说明                             *
 *----------------------------------------------*/
void TM7812B_Init(void);
void TM7812_set_flash(uint8_t flash,uint16_t angle);
void TM7812_Process(void);
void tm7812_SetColorPalette(uint8_t g, uint8_t r, uint8_t b); /*测试用函数*/
/*多余模式暂时没有用到*/
/*----------------------------------------------*
 * 内部函数原型说明                             *
 *----------------------------------------------*/
void TM7812_show(void);
/*----------------------------------------------*
 * 全局变量                                     *
 *----------------------------------------------*/
uint8_t rBuffer[PIXEL_MAX] = {0};
uint8_t gBuffer[PIXEL_MAX] = {0};
uint8_t bBuffer[PIXEL_MAX] = {0};
/*----------------------------------------------*
 * 模块级变量                                   *
 *----------------------------------------------*/
typedef struct
{
    const uint16_t head[3];              //先发送3个0等待dma稳定
    uint16_t data[24 * PIXEL_MAX];       //真正的数据
    const uint16_t tail;                 //最后发送一个0，保证dma结束后，pwm输出低
} frame_buf_ST;

frame_buf_ST frame = { .head[0] = 0,
                       .head[1] = 0,
                       .head[2] = 0,
                       .tail    = 0,
                     };

uint8_t     gFlash_Mode     = 2;
uint8_t     f9_state        = 0;
int16_t     Flash1_speed_L, Flash1_speed_R;
/*---------------------------------内部函数---------------------------------*/



//uint16_t frame_cnt = 0;
//uint8_t  rBuffer[PIXEL_MAX]= {0};
//uint8_t  gBuffer[PIXEL_MAX]= {0};
//uint8_t  bBuffer[PIXEL_MAX]= {0};


void TM7812B_Init(void)
{
    uint32_t i = 0;
    for(;i<100;i++)
    {
        rainbow(10);
        TM7812_show();
        delay_ms(10);
    }  
}
/*****************************************************************************
 函 数 名  : TM7812_show
 功能描述  : 将数据整理到缓存数组，并通过DMA发送一帧数据
 输入参数  : void
 输出参数  : 无
 返 回 值  :
 调用函数  :
 被调函数  :

 修改历史      :
  1.日    期   : 2017年6月7日
    作    者   : AF
    修改内容   : 新生成函数

*****************************************************************************/
void TM7812_show(void)
{
    int8_t i, j;

    for(i = 0; i < PIXEL_MAX; i++)
    {
        for(j = 0; j < 8; j++)
        {
            frame.data[24 * i + j]     = (rBuffer[i] & (0x80 >> j)) ? BIT_1 : BIT_0;
            frame.data[24 * i + j + 8]   = (gBuffer[i] & (0x80 >> j)) ? BIT_1 : BIT_0;
            frame.data[24 * i + j + 16]  = (bBuffer[i] & (0x80 >> j)) ? BIT_1 : BIT_0;
        }
    }
//    for(i = 0; i < PIXEL_MAX; i++)
//    {
//        for(j = 0; j < 8; j++)
//        {
//            frame.data[24 * i + j]      = (rBuffer[PIXEL_MAX-i-1] & (0x80 >> j)) ? BIT_1 : BIT_0;
//            frame.data[24 * i + j + 8]  = (gBuffer[PIXEL_MAX-i-1] & (0x80 >> j)) ? BIT_1 : BIT_0;
//            frame.data[24 * i + j + 16] = (bBuffer[PIXEL_MAX-i-1] & (0x80 >> j)) ? BIT_1 : BIT_0;
//        }
//    }
    HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, (uint32_t *)&frame, 3 + 24 * PIXEL_MAX + 1);
}
/*****************************************************************************
 函 数 名  : HAL_TIM_PWM_PulseFinishedCallback
 功能描述  : PWM完成中断回调函数，因为hal库自身并没有关掉PWM,所以要在完成后自己关掉
 输入参数  : void
 输出参数  : 无
 返 回 值  :
 调用函数  :
 被调函数  :

 修改历史      :
  1.日    期   : 2017年9月24日
    作    者   : AF
    修改内容   : 新生成函数

*****************************************************************************/
//void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
//{
//    HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_1);
//}
/*****************************************************************************
 函 数 名  : sign
 功能描述  : 如果输入为正数则返回1，为负返回-1，否则返回0
 输入参数  : int16_t db
 输出参数  : 无
 返 回 值  : 0，-1,1
 调用函数  :
 被调函数  :
javascript:void(0);
 修改历史      :
  1.日    期   : 2017年6月7日
    作    者   : AF
    修改内容   : 新生成函数

*****************************************************************************/
int8_t sign(int16_t db)
{
    int8_t result;

    if(db == 0)
        result = 0;
    else  if( db > 0)
        result = 1;
    else
        result = -1;

    return result;
}
/*****************************************************************************
 函 数 名  : TM7812B_Wheel_1
 功能描述  : 根据外部设置左右轮速度，控制跑马灯速度
             速度设置函数为void TM7812_set_Wheel_1(int16_t speed_L,int16_t speed_R)；

 输入参数  : void
 输出参数  : 无
 返 回 值  :
 调用函数  :
 被调函数  : TM7812B_Process

 修改历史      :
  1.日    期   : 2017年6月7日
    作    者   : AF
    修改内容   : 新生成函数

*****************************************************************************/
void TM7812B_Wheel_1(void)  //
{

    static int8_t  cnt_l, cnt_r;
    static int8_t  head_l, head_r;
    //static uint8_t  tab[8]={0,0,0,80,150,170,180,255};
    static uint8_t  tab[8] = {0, 0, 0, 0, 0, 0, 0, 255};
    uint32_t color_buf;
    uint16_t Time_L;
    uint16_t Time_R;
    uint8_t i, n;

    if(Flash1_speed_L == 0)
    {
        Time_L = 0;
    }
    else
    {
        Time_L = 1000 / (16 * fabs(Flash1_speed_L));
    }

    if(Flash1_speed_R == 0)
    {
        Time_R = 0;
    }
    else
    {
        Time_R = 1000 / (16 * fabs(Flash1_speed_R));
    }

    if(cnt_l > Time_L)
    {
        cnt_l = 0;
        n = head_l;

        for(i = 0; i < 8; i++)
        {
            if(n > 7) n = 0;

            color_buf = NEO_WHI(tab[i]);
//            SetPixelColor(n,color_buf);
//            SetPixelColor(n+8,color_buf);
            SetPixelColor(n + 16, color_buf);
            SetPixelColor(n + 24, color_buf);
            n++;
        }

        //判断极性
        if(sign(Flash1_speed_L) == -1)
        {
            head_l++;

            if(head_l > 7)
                head_l = 0;
        }
        else
        {
            head_l--;

            if(head_l < 0)
                head_l = 7;
        }

        //   TM7812_show();
    }

    cnt_l++;

    if(cnt_r > Time_R)
    {
        cnt_r = 0;
        n = head_r;

        for(i = 0; i < 8; i++)
        {
            if(n > 7) n = 0;

            color_buf = NEO_WHI(tab[i]);
            SetPixelColor(n, color_buf);
            SetPixelColor(n + 8, color_buf);
//            SetPixelColor(n+16,color_buf);
//            SetPixelColor(n+24,color_buf);
            n++;
        }

        //判断极性
        if(sign(Flash1_speed_R) == 1)
        {
            head_r++;

            if(head_r > 7)
                head_r = 0;
        }
        else
        {
            head_r--;

            if(head_r < 0)
                head_r = 7;
        }
    }

    cnt_r++;
}
/*****************************************************************************
 函 数 名  : tm7812_flash_2
 功能描述  : 彩色流水灯 0.5s一颗

 输入参数  : void
 输出参数  : 无
 返 回 值  :
 调用函数  : TM7812_Process
 被调函数  :

 修改历史      :
  1.日    期   : 2017年6月7日
    作    者   : AF
    修改内容   : 新生成函数

*****************************************************************************/
void tm7812_flash_1(void)
{
    const  uint32_t FlashPeriod_ms = 500;
    static uint8_t  state          = 0;
    static uint8_t  numl           = 0;
    static uint8_t  numr           = 0;
    static uint32_t next_time      = 0;
    uint32_t scolor;
    uint32_t timestamp = HAL_GetTick();
    
    uint32_t flag = 0;  
    if(next_time < FlashPeriod_ms)
    {
        if((uint64_t)timestamp + HAL_MAX_DELAY -  next_time > 0)
            flag = 1;
    }
    else if(timestamp > next_time)
    {
        flag = 1;
    }
    
    if(flag)// && timestamp - next_time < FlashPeriod_ms*5)
    {
        next_time = timestamp + FlashPeriod_ms;
        numl++;

        if(numl > 15)
            numl = 0;
        numr = 24 - numl;
        if(numr > 15)
            numr -= 16;
        SetAllPixelColor(0);
        switch(state++)
        {
        case 1:
            scolor = FIX_RED;
            break;
        case 2:
            scolor = FIX_ORG;
            break;
        case 3:
            scolor = FIX_YLW;
            break;
        case 4:
            scolor = FIX_GRN;
            break;
        case 5:
            scolor = FIX_CYA;
            break;
        case 6:
            scolor = FIX_BLU;
            break;
        case 7:
            scolor = FIX_PUR;
            state = 1;
            break;
        default:
            state = 1;
            break;
        }
        SetPixelColor(numl, scolor);
        SetPixelColor(16 + numr, scolor);
    }
}
/*****************************************************************************
 函 数 名  : tm7812_flash_2
 功能描述  : 单色流水灯 0.5s一颗
 输入参数  : void
 输出参数  : 无
 返 回 值  :
 调用函数  : TM7812_Process
 被调函数  :

 修改历史      :
  1.日    期   : 2017年6月7日
    作    者   : AF
    修改内容   : 新生成函数

*****************************************************************************/
void tm7812_flash_2(void)
{
    const static uint32_t FlashPeriod_ms = 162;
    static uint8_t numl = 0;
    static uint8_t numr = 0;
    static uint32_t next_time = 0;
    uint32_t timestamp = HAL_GetTick();
    
		numr = numr;
		
	
    static uint8_t  loop = 0;
    if(loop == 0) next_time = timestamp; loop = 1;  //首次调用初始化

    if(timestamp > next_time)// && timestamp - next_time < FlashPeriod_ms*5)
    {
        next_time = timestamp + FlashPeriod_ms;
        numl++;

        if(numl >= PIXEL_MAX)
            numl = 0;

        SetAllPixelColor(0);
        SetPixelColor(numl, FIX_GRN);
    }
}

/*****************************************************************************
 函 数 名  : tm7812_flash_3
 功能描述  : 绿色呼吸灯
            以正弦方式产生呼吸效果
 输入参数  : void
 输出参数  : 无
 返 回 值  :
 调用函数  : TM7812_Process
 被调函数  :

 修改历史      :
  1.日    期   : 2017年6月7日
    作    者   : AF
    修改内容   : 新生成函数

*****************************************************************************/

void tm7812_flash_3(void)
{
    static uint32_t next_time      = 0;
    const  uint32_t FlashPeriod_ms = 1000;      //呼吸周期
    const  uint8_t  DPI            = 100;       //分辨率
    const  uint8_t  max            = 100;       //最大亮度
    const  uint8_t  min            = 0;         //最小亮度
    static uint8_t  cnt            = 0;
    uint8_t  brighten;
    uint32_t timestamp = HAL_GetTick();
    
    static uint8_t  loop = 0;
    if(loop == 0) next_time = timestamp; loop = 1;  //首次调用初始化

    if(timestamp > next_time)// && timestamp - next_time < FlashPeriod_ms*5)
    {
        next_time = timestamp + FlashPeriod_ms / DPI;

        if(cnt++ > DPI)cnt = 0;
        brighten = (max + min) / 2 + ((max - min) / 2) * sin(2 * PI * cnt / DPI);
        SetAllPixelColor(NEO_GRN(brighten));

    }
}
/*****************************************************************************
 函 数 名  : tm7812_flash_4
 功能描述  : 青色呼吸灯
 输入参数  : void
 输出参数  : 无
 返 回 值  :
 调用函数  : TM7812_Process
 被调函数  :

 修改历史      :
  1.日    期   : 2017年6月7日
    作    者   : AF
    修改内容   : 新生成函数

*****************************************************************************/
void tm7812_flash_4(void)
{
    static uint32_t next_time      = 0;
    const  uint32_t FlashPeriod_ms = 1000;      //呼吸周期
    const  uint8_t  DPI            = 100;       //分辨率
    const  uint8_t  max            = 100;       //最大亮度
    const  uint8_t  min            = 0;         //最小亮度
    static uint8_t  cnt            = 0;
    uint8_t  brighten;
    uint32_t timestamp = HAL_GetTick();
    
    static uint8_t  loop = 0;
    if(loop == 0) next_time = timestamp; loop = 1;  //首次调用初始化

    if(timestamp > next_time)// && timestamp - next_time < FlashPeriod_ms*5)
    {
        next_time = timestamp + FlashPeriod_ms / DPI;

        if(cnt++ > DPI)cnt = 0;
        brighten = (max + min) / 2 + ((max - min) / 2) * sin(2 * PI * cnt / DPI);
        SetAllPixelColor(NEO_CYA(brighten));

    }
}
/*****************************************************************************
 函 数 名  : tm7812_flash_5
 功能描述  : 青色常亮
 输入参数  : void
 输出参数  : 无
 返 回 值  :
 调用函数  : TM7812_Process
 被调函数  :

 修改历史      :
  1.日    期   : 2017年6月7日
    作    者   : AF
    修改内容   : 新生成函数

*****************************************************************************/
void tm7812_flash_5(void)
{
    SetAllPixelColor(FIX_CYA);
    TM7812_show();
}
/*****************************************************************************
 函 数 名  : tm7812_flash_6
 功能描述  : 黄色常亮
 输入参数  : void
 输出参数  : 无
 返 回 值  :
 调用函数  : TM7812_Process
 被调函数  :

 修改历史      :
  1.日    期   : 2017年6月7日
    作    者   : AF
    修改内容   : 新生成函数

*****************************************************************************/
void tm7812_flash_6(void)
{
    SetAllPixelColor(FIX_YLW);
    TM7812_show();
}
/*****************************************************************************
 函 数 名  : tm7812_flash_7
 功能描述  : 红色常亮
 输入参数  : void
 输出参数  : 无
 返 回 值  :
 调用函数  : TM7812_Process
 被调函数  :

 修改历史      :
  1.日    期   : 2017年6月7日
    作    者   : AF
    修改内容   : 新生成函数

*****************************************************************************/
void tm7812_flash_7(void)
{
    SetAllPixelColor(FIX_RED);
    TM7812_show();
}
/*****************************************************************************
 函 数 名  : tm7812_flash_2
 功能描述  : 红色呼吸灯
 输入参数  : void
 输出参数  : 无
 返 回 值  :
 调用函数  : TM7812_Process
 被调函数  :

 修改历史      :
  1.日    期   : 2017年6月7日
    作    者   : AF
    修改内容   : 新生成函数

*****************************************************************************/
void tm7812_flash_8(void)
{
           uint32_t timestamp      = HAL_GetTick();
    static uint32_t next_time      = 0;
    const  uint32_t FlashPeriod_ms = 1000;      //呼吸周期
    const  uint8_t  DPI            = 100;       //分辨率
    const  uint8_t  max            = 100;       //最大亮度
    const  uint8_t  min            = 0;         //最小亮度
    static uint8_t  cnt            = 0;
    uint8_t  brighten;
    
    static uint8_t  loop = 0;
    if(loop == 0) next_time = timestamp; loop = 1;  //首次调用初始化

    if((timestamp > next_time))// && (timestamp - next_time < FlashPeriod_ms*5))
    {
        next_time = timestamp + FlashPeriod_ms / DPI;

        if(cnt++ > DPI)cnt = 0;
        brighten = (max + min) / 2 + ((max - min) / 2) * sin(2 * PI * cnt / DPI);
        SetAllPixelColor(NEO_RED(brighten));

    }
}
/*****************************************************************************
 函 数 名  : tm7812_flash_9
 功能描述  : 七彩变换
 输入参数  : void
 输出参数  : 无
 返 回 值  :
 调用函数  : TM7812_Process
 被调函数  :

 修改历史      :
  1.日    期   : 2017年6月7日
    作    者   : AF
    修改内容   : 新生成函数

*****************************************************************************/
void tm7812_flash_9(void)
{
           uint32_t timestamp = HAL_GetTick();
    const  static uint32_t FlashPeriod_ms = 1000;
    static uint32_t next_time = 0;
    
    static uint8_t  loop = 0;
    if(loop == 0) next_time = timestamp; loop = 1;  //首次调用初始化
    

    if(f9_state == 0)
    {
        next_time = timestamp + FlashPeriod_ms;
        f9_state = 1;
    }
    if((timestamp > next_time))// && (timestamp - next_time < FlashPeriod_ms*5))
    {
        next_time = timestamp + FlashPeriod_ms;
        f9_state++;

        if(f9_state > 7)
            f9_state = 1;
    }

    switch(f9_state)
    {
    case 1:
        SetAllPixelColor(FIX_RED);
        break;

    case 2:
        SetAllPixelColor(FIX_ORG);
        break;

    case 3:
        SetAllPixelColor(FIX_YLW);
        break;

    case 4:
        SetAllPixelColor(FIX_GRN);
        break;

    case 5:
        SetAllPixelColor(FIX_CYA);
        break;

    case 6:
        SetAllPixelColor(FIX_BLU);
        break;

    case 7:
        SetAllPixelColor(FIX_PUR);
        break;

    case 8:
        SetAllPixelColor(FIX_WHI);
        break;

    case 9:
        SetAllPixelColor(FIX_BLK);
        break;

    default:
        f9_state = 1;
        break;
    }
}

/*****************************************************************************
 函 数 名  : tm7812_ColorPalette
 功能描述  : 调试色彩用函数
             测试函数
 输入参数  : void
 输出参数  : 无
 返 回 值  :
 调用函数  :
 被调函数  :

 修改历史      :
  1.日    期   : 2017年6月7日
    作    者   : AF
    修改内容   : 新生成函数

*****************************************************************************/
uint8_t PR, PG, PB;
void tm7812_ColorPalette(void)
{
    setAllPixelColor(PG, PR, PB);
}

/*****************************************************************************
 函 数 名  : set_micdir
 功能描述  : 设置指示方向的led
             测试函数
 输入参数  : void
 输出参数  : 无
 返 回 值  :
 调用函数  :
 被调函数  :

 修改历史      :
  1.日    期   : 2017年9月24日
    作    者   : AF
    修改内容   : 新生成函数

*****************************************************************************/
void set_micdir(uint16_t data)
{
	const static uint16_t space_angle = 360/PIXEL_MAX;
	
//    if(data == 0)
//    {
//        rainbow(10);
//    }
    int8_t dir_led;
    static uint32_t led_tab[PIXEL_MAX] = {0x00ff00};//,0x5F0000,0x3F0000,0x2F0000,0x1F0000,0x0F0000,0x080000,0x010000,0x010000};
    int8_t led_mab[20];
    uint8_t i,borrow;
		
		i=i;
		
    if(data < 360)
    {
		/*
			在一共有18颗LED的情况下（1-18），led1的角度为350-369(09)LED2=10-29,LED3=30-59 ......
			所以当余数大于等于10时，应该下个led亮
		*/
        dir_led = (359-data) /space_angle;  //哪个led应该亮
		borrow = (359-data) % space_angle;	// 求余数
		if(borrow < 10) 					//是否借位
		{
			if(--dir_led <0) 
				dir_led = PIXEL_MAX-1;
		}
		
		//硬件错了一位
		dir_led++;
		if(dir_led > PIXEL_MAX-1) 
			dir_led = 0;

		
        SetAllPixelColor(0);        
        SetPixelColor(dir_led,FIX_GRN);
		
//        led_mab[0] = dir_led;
//		//考虑过0，整理到正常的led顺序
//        for(i=1; i<8; i++)
//        {
//            led_mab[2*i-1] = dir_led + i;
//            led_mab[2*i]   = dir_led - i;
//            if(led_mab[2*i-1] >= PIXEL_MAX ) led_mab[2*i-1] -= PIXEL_MAX;
//            if(led_mab[2*i]   < 0 )  led_mab[2*i] += PIXEL_MAX;
//        }
//        SetAllPixelColor(0);        
//        SetPixelColor(led_mab[0],led_tab[0]);
//        for(i=1; i<8; i++)
//        {
//            SetPixelColor(led_mab[2*i],led_tab[i]);
//            SetPixelColor(led_mab[2*i-1],led_tab[i]);
//        }
        
    }
}
/*----------------------------------外部函数-----------------------------*/
/***********************
*函 数 名  : TM7812B_Init
*功能描述  : 配合tm7812_ColorPalette使用，设置颜色，
*说明      : 测试函数
*************************/
void tm7812_SetColorPalette(uint8_t g, uint8_t r, uint8_t b)
{
    PG = g;
    PR = r;
    PB = b;
}
/*****************************************************************************
 函 数 名  : TM7812B_Init
 功能描述  : tm7812初始化，将所有led清零
            TIM5和DMA的初始化在main函数里面
 输入参数  : void
 输出参数  : 无
 返 回 值  :
 调用函数  :
 被调函数  :

 修改历史      :
  1.日    期   : 2017年6月7日
    作    者   : AF
    修改内容   : 新生成函数

*****************************************************************************/
void TM7812B_Init0(void)
{
    setAllPixelColor(0, 0, 0);
    TM7812_show();
}

/*****************************************************************************
 函 数 名  : TM7812_Process
 功能描述  : TM7812主进程函数,应该循环调用，
 输入参数  : void
 输出参数  : 无
 返 回 值  :
 调用函数  : freertos.c StartLEDTask02()
 被调函数  :

 修改历史      :
  1.日    期   : 2017年6月7日
    作    者   : AF
    修改内容   : 新生成函数

*****************************************************************************/
void TM7812_Process(void)
{
    const uint8_t TestMode = 0xf0;
    {

        //gFlash_Mode = TestMode;
        switch(gFlash_Mode)
        {
        case 0x01:
            //rainbowCycle(10);
            //tm7812_flash_1();     //彩色跑马灯
            break;

        case 0x02:            
            tm7812_flash_2();       //绿色跑马灯
            break;

        case 0x03:
            tm7812_flash_3();       //绿色呼吸灯
            break;

        case 0x04:
            tm7812_flash_4();       //青色呼吸灯
            break;
        case 0x05:
            tm7812_flash_5();       //青色常亮
            break;
        case 0x06:
            tm7812_flash_6();       //黄色常亮
            break;
        case 0x07:
            tm7812_flash_7();       //红色常亮
            break;
        case 0x08:
            tm7812_flash_8();       //红色呼吸
            break;
        case 0x09:
            //tm7812_flash_9();     //彩色切换
            rainbow(20);
            break;
		case 0x0A:
			rainbowCycle(10);
			break;
        case TestMode:
            //tm7812_ColorPalette();
            //rainbowCycle(10);
            rainbow(20);
            break;
        default:
            gFlash_Mode = 0X02;
            break;
        }
        TM7812_show();
    }
}
/*****************************************************************************
 函 数 名  : TM7812_set_flash
 功能描述  : 设置flash模式
 输入参数  : uint8_t flash
 输出参数  : 无
 返 回 值  :
 调用函数  : uusart.c usart_cmd()
 被调函数  :

 修改历史      :
  1.日    期   : 2017年6月7日
    作    者   : AF
    修改内容   : 新生成函数

*****************************************************************************/
void TM7812_set_flash(uint8_t flash,uint16_t angle)
{
#ifdef DEBUG_ERR
    if(flash > 10)
    {
        printf("flash mode order err \r\n");
    }
#endif
    gFlash_Mode = flash;
    if(gFlash_Mode == 1)
    {
        set_micdir(angle);
    }
    if(flash == 9)      //保证flash9 每次从红开始
    {
        f9_state = 0;
    }
}

/*****************************************************************************
 函 数 名  : TM7812_set_Wheel_1
 功能描述  : 在转速模式下，设置速度
 输入参数  : int16_t speed_L
             int16_t speed_R
 输出参数  : 无
 返 回 值  :
 调用函数  :
 被调函数  :

 修改历史      :
  1.日    期   : 2017年6月7日
    作    者   : AF
    修改内容   : 新生成函数

*****************************************************************************/
void TM7812_set_Wheel_1(int16_t speed_L, int16_t speed_R)
{
    Flash1_speed_L = speed_L;
    Flash1_speed_R = speed_R;
}
void TM7812B_Test(void)
{
    TM7812_show();
    //return;
    //SetAllPixelColor(0xffffff);
    //SetAllPixelColor(0);
    uint8_t cnt = 0;
    uint16_t num = 0;
	
		cnt = cnt;
	
    while(1)
    {
        //rainbowCycle(10);
        //rainbow(10);
//        while(!cnt--)
//        {
//            cnt = 1;
            set_micdir(num++);
            if(num >= 360)
                num = 0;
//        }
        
        TM7812_show();
        delay_ms(10);
        
        
        //theaterChase(NEO_BLU(100),100);
        //theaterChaseRainbow(10);
    }
}

/*-----暂时未用到-----*/
/*init*/
///*
//  __HAL_RCC_TIM5_CLK_ENABLE();
//    __HAL_RCC_GPIOA_CLK_ENABLE();
//    __HAL_RCC_DMA1_CLK_ENABLE();
//    /**TIM5 GPIO Configuration
//    PA0-WKUP     ------> TIM5_CH1 */
//    GPIO_InitTypeDef GPIO_InitStruct;

//    GPIO_InitStruct.Pin = LED_OUT_Pin;
//    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//    GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
//    HAL_GPIO_Init(LED_OUT_GPIO_Port, &GPIO_InitStruct);

//    /* Initialize TIMx peripheral as follow:
//    + Prescaler = 1.25us
//    + Period = 104
//    + ClockDivision = 0
//    + Counter direction = Up
//    */
//    TIM_ClockConfigTypeDef sClockSourceConfig;
//    TIM_MasterConfigTypeDef sMasterConfig;
//    TIM_OC_InitTypeDef sConfigOC;

//    htim5.Instance = TIM5;
//    htim5.Init.Prescaler = 0;
//    htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
//    htim5.Init.Period = 104;
//    htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//    TIM_Base_SetConfig(TIM5, &htim5.Init);

//    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
//
//    if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
//    {
//        Error_Handler();
//    }
//    /* Configure the PWM channels */
//    sConfigOC.OCMode = TIM_OCMODE_PWM1;
//    sConfigOC.Pulse = 0;
//    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
//    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
//    if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
//    {
//        Error_Handler();
//    }
//    DMA_HandleTypeDef hdma_tim5_ch1;
//    hdma_tim5_ch1.Instance          = DMA1_Stream2;
//    hdma_tim5_ch1.Init.Channel      = DMA_CHANNEL_6;
//    hdma_tim5_ch1.Init.Direction    = DMA_MEMORY_TO_PERIPH;
//    hdma_tim5_ch1.Init.PeriphInc    = DMA_PINC_DISABLE;
//    hdma_tim5_ch1.Init.MemInc       = DMA_MINC_ENABLE;
//    hdma_tim5_ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
//    hdma_tim5_ch1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
//    hdma_tim5_ch1.Init.Mode         = DMA_NORMAL;
//    hdma_tim5_ch1.Init.Priority     = DMA_PRIORITY_LOW;
//    hdma_tim5_ch1.Init.FIFOMode     = DMA_FIFOMODE_DISABLE;
//    if (HAL_DMA_Init(&hdma_tim5_ch1) != HAL_OK)
//    {
//      Error_Handler();
//    }
//    */

//#define TM7812_OUTH     TM7812_GPIO_Port->BSRR = TM7812_Pin
//#define TM7812_OUTL     TM7812_GPIO_Port->BSRR = (uint32_t)TM7812_Pin << 16

//#define SEND_WS_0() {TM7812_OUTH;   \
//            __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP(); \
//            __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP(); \
//            __NOP();__NOP();__NOP();__NOP();__NOP(); \
//            TM7812_OUTL; \
//            __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP(); \
//            __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP(); \
//            __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP(); __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP(); \
//            }
//#define SEND_WS_1() {TM7812_OUTH;  \
//            __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP(); \
//            __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP(); \
//            __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP(); \
//            __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP(); \
//            __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP(); \
//            TM7812_OUTL;    \
//            }

///*发送帧*/
//void TM7812_show2(void)
//{
//    int16_t i;
//    frame_cnt = 0;
//    for(i=0; i<PIXEL_MAX; i++)
//    {
//        //TM7812BSend_24bit(rBuffer[i],gBuffer[i],bBuffer[i]);
//    }
//}
//#define   A_SET_DATA     GPIOA->ODR |= 0x0080;    //PA7= 1
//#define   A_CLR_DATA     GPIOA->ODR &= 0xFF7F;    //PA7= 0
//void SendOnePix(uint8_t *ptr)   //A??
//{
//	  uint8_t i,j;
//		uint8_t dat;
//	
//		for(j=0;j<3;j++)
//		{
//			dat = ptr[j];
//			for(i=0;i<8;i++)
//			{
//					if(dat&0x80)            //"1"
//					{
//							A_SET_DATA;
//							__nop();__nop();__nop();__nop();
//							__nop();__nop();__nop();__nop();
//							__nop();__nop();__nop();__nop();
//							__nop();__nop();__nop();__nop();
//							__nop();__nop();__nop();__nop();
//							__nop();__nop();__nop();__nop();
//							__nop();__nop();		
//							A_CLR_DATA;	
//							__nop();__nop();__nop();__nop();			//11个
//							__nop();__nop();__nop();__nop();
//							__nop();__nop();__nop();
//					}
//					else                             //"0"
//					{
//							A_SET_DATA;			
//							__nop();__nop();__nop();__nop();			//11个
//							__nop();__nop();__nop();__nop();
//							__nop();__nop();__nop();
//							A_CLR_DATA;
//							__nop();__nop();__nop();__nop();
//							__nop();__nop();__nop();__nop();
//							__nop();__nop();__nop();__nop();
//							__nop();__nop();__nop();__nop();
//							__nop();__nop();__nop();__nop();
//							__nop();__nop();__nop();__nop();
//							__nop();__nop();		
//					} 
//					dat<<=1;
//			}
//	}
//}
//void rst()
//{
//	  uint16_t i;
//	  for(i=0;i<400;i++)
//	  {
//		    __nop();__nop();
//		    __nop();__nop();
//			  __nop();__nop();
//	  }
//}

//void SendOneFrame(uint8_t *ptr)
//{
//	uint8_t k;

//	rst();rst();				 //???????

//	for(k=0;k<PIXEL_MAX;k++)				 //??????,SNUM???LED???
//	{
//		SendOnePix(&ptr[(3*k)]);
//	}

//	rst();rst();				 //???????
//}

//void TM7812_show(void)
//{
//    int16_t i,j,k;
//    uint32_t mask = 0x800000;
//    uint32_t byte = 0;
//    uint32_t frame[PIXEL_MAX];
//    uint8_t  frame8_buf[PIXEL_MAX*3];
//    for(i=0; i<PIXEL_MAX; i++)
//    {
//        frame[i] = rBuffer[i]<<16 | gBuffer[i]<<8 | bBuffer[i];
//        frame8_buf[i+0] = rBuffer[i*3+0];
//        frame8_buf[i+1] = rBuffer[i*3+1];
//        frame8_buf[i+2] = rBuffer[i*3+2];
//    }
//    
//    //SendOneFrame(frame8_buf);
//   // return;
//    for(i=0; i<PIXEL_MAX; i++)
//    {
//        mask = 0x800000;
//        byte = frame[i];
//        while (mask)
//        {
//            if( byte & mask )
//            {
//                SEND_WS_1()
//            }
//            else
//            {                
//                SEND_WS_0()                
//            }
//            if(mask != 0x01)
//            {
//                __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//            }
//            mask >>= 1;
//        }        
//    }
//}
