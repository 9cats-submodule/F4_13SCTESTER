/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dac.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "fsmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "base.h"
#include "lcd.h"
#include "text.h"
#include "touch.h"
#include "w25qxx.h"
#include "hmi_user_uart.h"
#include "hmi_driver.h"
#include "ADS8688.h"
#include "AD9959.h"
#include "stdio.h"
#include "output.h"
#include "arm_math.h"
#include "arm_const_structs.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TP_CHECK(x0,y0,x1,y1) tp_dev.x[0] > x0 && tp_dev.y[0] > y0 && tp_dev.x[0] < x1 && tp_dev.y[0] < y1
#define SAMPLE_BEGIN PAout(15)=0;
#define SAMPLE_END   PAout(15)=1;
#define RELAY_NO HAL_GPIO_WritePin(RELAY_IN_GPIO_Port, RELAY_IN_Pin, GPIO_PIN_SET);
#define RELAY_NC HAL_GPIO_WritePin(RELAY_IN_GPIO_Port, RELAY_IN_Pin, GPIO_PIN_RESET);
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//---------------------------------DEBUG--------------------------------
#define TFT
//#define VSOC

#define SAMPLE_POINT_MAX 2048 //采样点数
extern u8  SAMPLE_END_FLAG;
float FFT_INPUT [SAMPLE_POINT_MAX];
float FFT_OUTPUT[SAMPLE_POINT_MAX];
float FFT_OUTPUT_REAL[SAMPLE_POINT_MAX];
float WAVE[SAMPLE_POINT_MAX];

float freq = 1000.0f,L_freq = 1000.0f;
float mv   =300.0f,L_mv = 300.0f;

extern u16 RxData[];
///////////////////////////////////////////
s16 offset = 57;
float RMS_COMPENSATE = 1134.0f;
float SCAN_VPP[200];
float SCAN_MAX;
u8 DATA_SHOW[51];
//*100Hz 201个点
u16 freqScanArr[50] =
{
   1,   2,   3,   4,   5,
   6,   7,   8,   9,  10,
  12,  14,  16,  18,  20,
  26,  32,  38,  44,  50,
  60,  70,  80,  90, 100,
 120, 140, 160, 180, 200,
 240, 280, 320, 360, 400,
 480, 560, 640, 720, 800,
 960,1120,1280,1440,1600,
1760,1920,2080,2240,2400
};
//-----------------------------标志---------------------------------------
extern u8 SAMPLE_END_FLAG;    //采样结束标记
//------------------------------------------------------------------------

//-----------------------------变量---------------------------------------
extern u16 SAMPLE_POINT;       //将要采样的点数
extern s32 BUF[SAMPLE_POINT_MAX];
float CH_VPP_VALUE[5] = {0};
//------------------------------------------------------------------------

//----------------------------可储存变量------------------------------------------
const u32 SAVE_ADDR = 0x0000f000;
SVAR Svar = {
  /*u8  WIFI     ;    //WIFI状态是否开启                                                      */      0,
  /*u8  CH_SELECT;    //当前所选通道 0 1 2 3                         */      3,
  /*u8  TG_SOURCE;    //触发源 1-CH1 2-CH2                           */      0,
  /*u8  TG_MODE;      //触发模式 1-上升沿触发，2-下降沿触发，3-电平触发*/      1,
  /*u8  RUN;          //是否STOP                                     */      1,
  /*u8  AUTO;         //是否AUTO                                     */      0,
  /*u8  COUPE;        //耦合方式 0-直流 1-交流                       */      0,
  /*u16 TG_VAL;       //触发电平                                     */      0,
  /*float VREF;       //ADS参考电压                                                               */ 4.096f,
  /*float VCC;        //STM32参考电压                                                           */ 3.300f,
  /*float COMPENSATE; //频率补偿                                     */  99.0f,
  /*float CH1_COMPENSATE; //CH1补偿                                                              */ 909.09090909f
};

//数据保存操作
//mode-0 写入默认  mode-1 读出 mode-2 数据检查并更新
void DATA_OP(u8 mode)
{
  u8 *VAR_ADDR   = (u8*)&Svar; //变量地址
  u32 FLASH_ADDR = SAVE_ADDR ; //FLASH储存首地址
  u8  data;                    //暂存数据
  u16 size;                    //当前已经储存大小

  for(size=0;size<sizeof(SVAR);size++,VAR_ADDR++,FLASH_ADDR++)
  {
	switch(mode)
	{
	  case 0:W25QXX_Write(VAR_ADDR,FLASH_ADDR,1);break;
	  case 1:W25QXX_Read (VAR_ADDR,FLASH_ADDR,1);break;
	  case 2:{
	      W25QXX_Read (&data,FLASH_ADDR,1);
		  if(data != *VAR_ADDR) W25QXX_Write(VAR_ADDR,FLASH_ADDR,1);
	  }
	}
  }
}
void DATA_INIT() {u8 key = KEY_Scan(0);if(key == KEY0_PRES) DATA_OP(0);else DATA_OP(1);}
void DATA_UPDATE() {DATA_OP(2);}

void FFT(void)
{
	u16 i;
	arm_rfft_fast_instance_f32 S;

	for(i=0;i<2048;i++)
	{
		FFT_INPUT[i]  = (float)BUF[i]*20.48/0x10000;
	}
	arm_rfft_fast_init_f32(&S,2048);                    //FFT初始化
	arm_rfft_fast_f32(&S, FFT_INPUT, FFT_OUTPUT,0);     //FFT变化
	arm_cmplx_mag_f32(FFT_OUTPUT,FFT_OUTPUT_REAL,2048); //求模
}
//THD-求失真度 64k采样 1k待采
float THD(void)
{
  //基波
  float basic = FFT_OUTPUT_REAL[32];
  //谐波
  float harmon[6],harmon_power,high;

  harmon[0] = FFT_OUTPUT_REAL[32*2];
  harmon[1] = FFT_OUTPUT_REAL[32*3];
  harmon[2] = FFT_OUTPUT_REAL[32*4];
  harmon[3] = FFT_OUTPUT_REAL[32*5];
  harmon[4] = FFT_OUTPUT_REAL[32*6];
  harmon[5] = FFT_OUTPUT_REAL[32*7];

  arm_power_f32(harmon, 6, &harmon_power);
  arm_sqrt_f32(harmon_power,&high);

  return high / basic;
}


//通过FFT求某通道的1K幅度
float SCAN_FFT_1K(u8 ch)
{
  Init_ADS8688(ch);            //只开启通道1
  SAMPLE_POINT       = 2048;     //采集2048个点
  HAL_Delay(2);
  HAL_TIM_Base_Start_IT(&htim1); //开启定时器
  while(!SAMPLE_END_FLAG);       //等待采样完成
  SAMPLE_END_FLAG = 0;           //采样标记清零
  FFT();                         //FFT计算出CH1幅值
  //DEBUG:
  #ifdef VSOC
  for(i=0;i<2048;i++)
  {
  	OutData[0] = BUF[i];
  	OutData[1] = FFT_INPUT[i];
  	OutData[2] = FFT_OUTPUT[i];
  	OutData[3] = FFT_OUTPUT_REAL[i];
  	OutPut_Data();
  }
  OutPut_Data();
  #endif

  return FFT_OUTPUT_REAL[32];
}

float SCAN_RMS(u8 ch)
{
  u16 i;
  float sum=0;

  Init_ADS8688(ch);            //只开启通道1
  SAMPLE_POINT  = 64;         //采集100个点
  HAL_Delay(2);
  HAL_TIM_Base_Start_IT(&htim1); //开启定时器
  while(!SAMPLE_END_FLAG);       //等待采样完成
  SAMPLE_END_FLAG = 0;           //采样标记清零

  for(i=0;i<64;i++)
  {
    sum+= (s32)BUF[i]+offset;
  }

  return sum/64;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_FSMC_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_SPI3_Init();
  MX_DAC_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  //无时耗
  delay_init(168);
  W25QXX_Init();
  LCD_Init();
  font_init();
  tp_dev.init();
  TFT_Init();
  DATA_INIT();
  Init_AD9959();

  //时耗
  //DDS输出 -- CH3输出
  Out_freq(2, 1000);
  Out_mV(2, 300);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	u8 str[20];
	u32 i;
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	/*正常工作，测输入输出阻抗，增益，幅频特性，失真度*/
	while(1)
	{
      //开启CH1获取值
	  HAL_Delay(100);
	  CH_VPP_VALUE[0] = SCAN_FFT_1K(0x01)*Svar.CH1_COMPENSATE/511;
#ifdef TFT
	  //显示CH1_VPP
	  sprintf((char*)str,"%.2fmV",CH_VPP_VALUE[0]);
	  SetTextValue(0,41,str);
#endif


	  //开启CH2获取值
	  HAL_Delay(100);
	  CH_VPP_VALUE[1] = SCAN_FFT_1K(0x02)*Svar.CH1_COMPENSATE/511;
#ifdef TFT
	  //显示CH2_VPP
	  sprintf((char*)str,"%.2fmV",CH_VPP_VALUE[1]);
	  SetTextValue(0,42,str);
	  //显示输入阻抗
	  sprintf((char*)str,"%.3fKΩ",CH_VPP_VALUE[1]/(CH_VPP_VALUE[0]-CH_VPP_VALUE[1])*6.8);
	  SetTextValue(1,41,str);
#endif


	  //开启CH3获取值
	  HAL_Delay(110);
	  CH_VPP_VALUE[2] = SCAN_FFT_1K(0x04)*Svar.CH1_COMPENSATE/511;
#ifdef TFT
	  //显示CH3_VPP
//	  sprintf((char*)str,"%.0fmV",CH_VPP_VALUE[2]);
	  //TODO:直流!!!
//	  sprintf((char*)str,"%.0fmV",FFT_OUTPUT_REAL[0]*Svar.CH1_COMPENSATE/511/4);
//	  SetTextValue(0,43,str);
#endif


	  RELAY_NO;
	  HAL_Delay(300);
	  //开启CH4获取值
	  //CH_VPP_VALUE[3] = SCAN_FFT_1K(0x10)*Svar.CH1_COMPENSATE/511;
	  CH_VPP_VALUE[3] = SCAN_RMS(0x10)*Svar.CH1_COMPENSATE/RMS_COMPENSATE;
#ifdef TFT
	  //显示CH4_VPP

	  sprintf((char*)str,"%.0fmV",CH_VPP_VALUE[3]);
	  SetTextValue(0,44,str);
	  //显示输出阻抗
	  sprintf((char*)str,"%.2f",CH_VPP_VALUE[3]/CH_VPP_VALUE[1]);
	  SetTextValue(1,43,str);
#endif


	  RELAY_NC;
	  HAL_Delay(300);
	  //开启CH5获取值
	  CH_VPP_VALUE[4] = SCAN_FFT_1K(0x20)*Svar.CH1_COMPENSATE/511;
#ifdef TFT
	  //显示CH5_VPP
	  sprintf((char*)str,"%.0fmV",CH_VPP_VALUE[4]);
	  SetTextValue(0,45,str);
	  //显示输出阻抗
	  sprintf((char*)str,"%.3fKΩ",(CH_VPP_VALUE[3]-CH_VPP_VALUE[4])/CH_VPP_VALUE[4]);
	  SetTextValue(1,42,str);
	  //显示失真度
	  sprintf((char*)str,"%.2f%%",THD()*100);
	  SetTextValue(1,44,str);
#endif


	  RELAY_NO;
	  HAL_Delay(100);
	  //扫频
	  for(i=0;i<50;i++)
	  {
		Out_freq(2, freqScanArr[i]*100);
		HAL_Delay(100);
		SCAN_VPP[i] = SCAN_RMS(0x10)*Svar.CH1_COMPENSATE/RMS_COMPENSATE;
	  }
	  Out_freq(2, 1000);
	  //找最大值
	  arm_max_f32(SCAN_VPP, 200, &SCAN_MAX, &i);
	  DATA_SHOW[0] = 0;
	  for(i=1;i<=50;i++)
	  {
		DATA_SHOW[i] = SCAN_VPP[i-1] / SCAN_MAX * 222;
	  }
	  GraphChannelDataAdd(1,51,1,(u8*)DATA_SHOW,400);

	  //DEBUG用，调节DDS输出
      if(freq != L_freq)
      {
        Out_freq(2, freq);
        L_freq = freq;
      }
      if(mv   != L_mv)
      {
        Out_mV(2, mv);
        L_mv = mv;
      }


      LED0_T;
      DATA_UPDATE();
	}
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
