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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TP_CHECK(x0,y0,x1,y1) tp_dev.x[0] > x0 && tp_dev.y[0] > y0 && tp_dev.x[0] < x1 && tp_dev.y[0] < y1
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

//下面为需要储存的数据
uint32_t ADDR_START = 0x0000f000;//默认储存开始地址，同时为变量起始地址
uint8_t  WIFI=0;             //WIFI状态是否开启
uint8_t  CH_SELECT=3;        //当前所选通道 0 1 2 3
uint8_t  TG_SOURCE=0;        //触发源 1-CH1 2-CH2
uint8_t  TG_MODE=1;          //触发模式 1-上升沿触发，2-下降沿触发，3-电平触发
int16_t  TG_VAL=0;           //触发电平
uint8_t  RUN=1;              //是否STOP
uint8_t  AUTO=0;             //是否AUTO
uint8_t  COUPE=0;            //耦合方式 0-直流 1-交流
float VREF = 4.096f;         //ADS参考电压
float VCC  = 3.300f;         //STM32参考电压
float COMPENSATE = 99.0f;    //频率补偿
uint32_t ADDR_END;//默认结束地址

//数据保存操作
//mode-0 写入默认  mode-1 读出 mode-2 数据检查并更新
#define DATA_SAVE(vAddr,fAddr) W25QXX_Write(vAddr,fAddr,1)
#define DATA_READ(vAddr,fAddr) W25QXX_Read (vAddr,fAddr,1)

void DATA_OP(u8 mode)
{
	u8 *VAR_ADDR   = (u8*)&ADDR_START; //变量首地址
	u32 FLASH_ADDR =       ADDR_START; //FLASH储存首地址
	u8  data;

	while(VAR_ADDR < (u8*)&ADDR_END)
	{
		switch(mode)
		{
		    case 0:DATA_SAVE(VAR_ADDR++,FLASH_ADDR++);break;
		    case 1:DATA_READ(VAR_ADDR++,FLASH_ADDR++);break;
		    case 2:
		    {
			  DATA_READ(&data,FLASH_ADDR);
			  if(data != *VAR_ADDR)
			  DATA_SAVE(VAR_ADDR++,FLASH_ADDR++);
			  else{VAR_ADDR++;FLASH_ADDR++;}
		    }break;
		}
	}
}
void DATA_INIT() {u8 key = KEY_Scan(0);if(key == KEY0_PRES) DATA_OP(0);else DATA_OP(1);}
void DATA_UPDATE() {DATA_OP(2);}
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
  MX_FSMC_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_SPI3_Init();
  MX_DAC_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */
  //无时耗
  delay_init(168);
  W25QXX_Init();
  LCD_Init();
  font_init();
  tp_dev.init();
  TFT_Init();
  ADS8688_Init(&ads8688, &hspi3, ADS8688_CS_GPIO_Port, ADS8688_CS_Pin);
  HAL_DAC_Start(&hdac,DAC1_CHANNEL_1);
  //时耗
  DATA_INIT();
  HAL_TIM_Base_Start_IT(&htim8);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	u8 temp;
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	HAL_UART_Transmit(huart, pData, Size);
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
