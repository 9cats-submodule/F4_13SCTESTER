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
#define CHANNEL_NUM 2   //通道数
#define SAMPLE_POINT 2048 //采样点数
extern u8  SAMPLE_END_FLAG;
extern u16 BUF[CHANNEL_NUM][SAMPLE_POINT];
float FFT_INPUT [SAMPLE_POINT];
float FFT_OUTPUT[SAMPLE_POINT];
float FFT_OUTPUT_REAL[SAMPLE_POINT];
float WAVE[SAMPLE_POINT];

extern u16 RxData[CHANNEL_NUM];

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
  /*float COMPENSATE; //频率补偿                                     */  99.0f
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

	for(i=0;i<SAMPLE_POINT;i++)
	{
		WAVE[i]       = (arm_sin_f32(3.1415926f*1*i/SAMPLE_POINT) - 0 )*1000+500;
		FFT_INPUT[i]  = (arm_sin_f32(3.1415926f*1*i/SAMPLE_POINT) - 0 )*1000+500;
	}
	arm_rfft_fast_init_f32(&S,SAMPLE_POINT);
	arm_rfft_fast_f32(&S, FFT_INPUT, FFT_OUTPUT,0);
	arm_cmplx_mag_f32(FFT_OUTPUT,FFT_OUTPUT_REAL,SAMPLE_POINT);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  u16 i=0;
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
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  //无时耗
  delay_init(168);
  W25QXX_Init();
  LCD_Init();
  font_init();
  tp_dev.init();
  TFT_Init();
  DATA_INIT();
//  Init_ADS8688(0x37);
  Init_ADS8688(0x01);
  Init_AD9959();

  //时耗
  //DDS输出 -- CH3输出
  Out_freq(2, 1000);
  Out_mV(2, 300);
  HAL_TIM_Base_Start_IT(&htim3);



// 续点器驱动
//  while(1)
//  {
//  HAL_Delay(1000);
//  HAL_GPIO_WritePin(RELAY_IN_GPIO_Port, RELAY_IN_Pin, GPIO_PIN_RESET);
//  HAL_Delay(1000);
//  HAL_GPIO_WritePin(RELAY_IN_GPIO_Port, RELAY_IN_Pin, GPIO_PIN_SET);
//  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    u8 str[20];
    /* USER CODE BEGIN 3 */
    LED0_T;
    HAL_Delay(20);
//    sprintf((char*)str,"%.3f",((s32)RxData[0]-0x8000)*20.48f/0x10000);
//    SetTextValue(0,41,str);
//    sprintf((char*)str,"%.3f",((s32)RxData[1]-0x8000)*20.48f/0x10000);
//    SetTextValue(0,42,str);
//    sprintf((char*)str,"%.3f",((s32)RxData[2]-0x8000)*20.48f/0x10000);
//    SetTextValue(0,43,str);
//    sprintf((char*)str,"%.3f",((s32)RxData[3]-0x8000)*20.48f/0x10000);
//    SetTextValue(0,44,str);
//    sprintf((char*)str,"%.3f",((s32)RxData[4]-0x8000)*20.48f/0x10000);
//    SetTextValue(0,45,str);

    if(SAMPLE_END_FLAG && i < SAMPLE_POINT)
    {
      OutData[0] = ((s32)BUF[0][i]-0x8000);
      OutData[1] = ((s32)BUF[1][i]-0x8000);
      OutData[2] = ((s32)BUF[2][i]-0x8000);
      OutData[3] = ((s32)BUF[4][i]-0x8000);
      OutPut_Data();
      i++;
    }
    if(i == SAMPLE_POINT)
    {
      OutPut_Data();
    }
//    if(i < SAMPLE_POINT)
//    {
//    	if(i==0) FFT();
//    	OutData[0] = WAVE[i];
//    	OutData[1] = FFT_INPUT[i];
//    	OutData[2] = FFT_OUTPUT[i];
//    	OutData[3] = -FFT_OUTPUT_REAL[i];
//    	OutData[0] = (float)((s32)BUF[0][i]-0x8000);
//    	OutData[1] = (float)((s32)BUF[1][i]-0x8000);
//    	OutPut_Data();
//    	i++;
//    }
//    if(i == SAMPLE_POINT)
//    {
//    	OutPut_Data();
//    	i++;
//    }
    DATA_UPDATE();
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
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
