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
float FFT_INPUT [SAMPLE_POINT_MAX];
float FFT_OUTPUT[SAMPLE_POINT_MAX];
float FFT_OUTPUT_REAL[SAMPLE_POINT_MAX];
float WAVE[SAMPLE_POINT_MAX];

float freq = 1000.0f,L_freq = 1000.0f;
float mv   =300.0f,L_mv = 300.0f;

//*10Hz 116个点
u16 freqScanArr[116] ={     0, //   0
    0,    0,    0,    0,   10, // 100
   12,   14,   16,   18,   20, // 200
   22,   24,   26,   28,   30, // 300
   32,   34,   36,   38,   40, // 400
   42,   44,   46,   48,   50, // 500
   54,   58,   62,   66,   70, // 700
   76,   82,   88,   94,  100, //  1K
  180,  260,  340,  420,  500, //  5K
  600,  700,  800,  900, 1000, // 10K
 1100, 1200, 1300, 1400, 1500, // 15K
 1600, 1700, 1800, 1900, 2000, // 20K
 2200, 2400, 2600, 2800, 3000, // 30K
 3200, 3400, 3600, 3800, 4000, // 40K
 4200, 4400, 4600, 4800, 5000, // 50K
 5200, 5400, 5600, 5800, 6000, // 60K
 6200, 6400, 6600, 6800, 7000, // 70K
 7200, 7400, 7600, 7800, 8000, // 80K
 8400, 8800, 9200, 9600,10000, //100K
10400,10800,11200,11600,12000, //120K
12400,12800,13200,13600,14000, //140K
14400,14800,15200,15600,16000, //160K
16400,16800,17200,17600,18000, //180K
18400,18800,19200,19600,20000  //200K
};
//-----------------------------标志---------------------------------------
extern u8 SAMPLE_END_FLAG;    //采样结束标记
//------------------------------------------------------------------------

//-----------------------------变量---------------------------------------
extern u16 SAMPLE_POINT;       //将要采样的点数
extern s32 BUF[SAMPLE_POINT_MAX];

float CH_VPP_VALUE[5] = {0};
float SCAN_VPP[200];
float SCAN_MAX;
u32   SCAN_MAX_INDEX;
u8    DATA_SHOW[116];

u8    mode = 0; //当前模式 0-待机 1-测量数据 2-幅频特性 3-故障检测
//------------------------------------------------------------------------

//----------------------------可储存变量------------------------------------------
const u32 SAVE_ADDR = 0x0000f000;
SVAR Svar = {
  /*float FFT_COMPENSATE; //补偿FFT的误差*/909.09090909f,
  /*float RMS_COMPENSATE; //补偿RMS     */1134.0f,
  /*float  DC_COMPENSATE; //FFT的直流补偿*/0.254599959f,
  /*s16   ADS_OFFSET;     //ADS偏置补偿  */57
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

float SCAN_RMS(void)
{
  u16 i;
  float sum=0;

  SAMPLE_POINT  = 2048;         //采集100个点
  HAL_TIM_Base_Start_IT(&htim1); //开启定时器
  while(!SAMPLE_END_FLAG);       //等待采样完成
  SAMPLE_END_FLAG = 0;           //采样标记清零

  for(i=0;i<2048;i++)
  {
    sum+= (s32)BUF[i]+Svar.ADS_OFFSET;
  }

  return sum/2048;
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
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	/*正常工作，测输入输出阻抗，增益，幅频特性，失真度*/
	HAL_Delay(100);
	switch (mode) {
	case 0:SetTextValue(1,49,(u8*)"待机中");break;
	case 1:Measure();break;
	case 2:ScanFreq();break;
	case 3:FaultChecK();break;
	}
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
//测量参数
void Measure(void)
{
  u8 str[20];
  Out_freq(2, 1000);
  SetTextValue(1,49,(u8*)"测量电路参数中.   ");

  //开启CH1获取值
  HAL_Delay(200);
  CH_VPP_VALUE[0] = SCAN_FFT_1K(0x01)*Svar.FFT_COMPENSATE/511;
#ifdef TFT
  //显示CH1_VPP
  sprintf((char*)str,"%.2fmV",CH_VPP_VALUE[0]);
  SetTextValue(0,41,str);
#endif

  SetTextValue(1,49,(u8*)"测量电路参数中..  ");

  //开启CH2获取值
  HAL_Delay(100);
  CH_VPP_VALUE[1] = SCAN_FFT_1K(0x02)*Svar.FFT_COMPENSATE/511;
#ifdef TFT
  //显示CH2_VPP
  sprintf((char*)str,"%.2fmV",CH_VPP_VALUE[1]);
  SetTextValue(0,42,str);
  //显示输入阻抗
  sprintf((char*)str,"%.3fKΩ",CH_VPP_VALUE[1]/(CH_VPP_VALUE[0]-CH_VPP_VALUE[1])*6.8);
  SetTextValue(1,41,str);
#endif

  SetTextValue(1,49,(u8*)"测量电路参数中... ");

  //开启CH3获取值
  HAL_Delay(110);
  CH_VPP_VALUE[2] = SCAN_FFT_1K(0x04)*Svar.FFT_COMPENSATE*Svar.DC_COMPENSATE/511;
#ifdef TFT
  //显示CH3_VPP
  //	  sprintf((char*)str,"%.0fmV",CH_VPP_VALUE[2]);
  //TODO:直流!!!
  	  sprintf((char*)str,"%.0fmV",CH_VPP_VALUE[2]);
  	  SetTextValue(0,43,str);
#endif

  SetTextValue(1,49,(u8*)"测量电路参数中....");

  RELAY_NO;
  Init_ADS8688(0x10);
  HAL_Delay(300);
  //开启CH4获取值
  CH_VPP_VALUE[3] = SCAN_RMS()*Svar.FFT_COMPENSATE/Svar.RMS_COMPENSATE;
#ifdef TFT
  //显示CH4_VPP

  sprintf((char*)str,"%.2fmV",CH_VPP_VALUE[3]);
  SetTextValue(0,44,str);
  //显示输出阻抗
  sprintf((char*)str,"%.2f",CH_VPP_VALUE[3]/CH_VPP_VALUE[1]);
  SetTextValue(1,43,str);
#endif


  RELAY_NC;
  HAL_Delay(300);
  //开启CH5获取值
  CH_VPP_VALUE[4] = SCAN_FFT_1K(0x20)*Svar.FFT_COMPENSATE/511;
#ifdef TFT
  //显示CH5_VPP
  sprintf((char*)str,"%.2fmV",CH_VPP_VALUE[4]);
  SetTextValue(0,45,str);
  //显示输出阻抗
  sprintf((char*)str,"%.3fKΩ",(CH_VPP_VALUE[3]-CH_VPP_VALUE[4])/CH_VPP_VALUE[4]);
  SetTextValue(1,42,str);
  //显示失真度
  sprintf((char*)str,"%.2f%%",THD()*100);
  SetTextValue(1,44,str);
#endif
}

//幅频特性
void ScanFreq(void)
{
	u32 i;
	u8 str[20];
	{//扫频
	  RELAY_NO;
	  Init_ADS8688(0x10);
	  HAL_Delay(100);
	  //扫频
	  for(i=0;i<116;i++)
	  {
		  if(freqScanArr[i] == 0) SCAN_VPP[i] = 0;
		  else {
		  	Out_freq(2, freqScanArr[i]*10);
		  	sprintf((char*)str,"扫频:%ld%%",i*100/116);
		  	SetTextValue(1,49,str);
		  	HAL_Delay(20);
		  	SCAN_VPP[i] = SCAN_RMS()*Svar.FFT_COMPENSATE/Svar.RMS_COMPENSATE;
		  }
	  }
	  //找最大值 -- 和最大值对应的序列
	  arm_max_f32(SCAN_VPP, 200, &SCAN_MAX, &SCAN_MAX_INDEX);
	  DATA_SHOW[0] = 0;
	  for(i=0;i<116;i++)
	  {
		  if(i == 5)
		  {
		    DATA_SHOW[5] = SCAN_VPP[5] / SCAN_MAX * 255;
		    DATA_SHOW[4] = DATA_SHOW[5] * 4 / 5;
		    DATA_SHOW[3] = DATA_SHOW[5] * 3 / 5;
		    DATA_SHOW[2] = DATA_SHOW[5] * 2 / 5;
		    DATA_SHOW[1] = DATA_SHOW[5] * 1 / 5;
		  }
		  else if(i > 5)
		  {
		    DATA_SHOW[i] = SCAN_VPP[i] / SCAN_MAX * 255;
		  }
	  }
	  GraphChannelDataAdd(1,51,1,(u8*)DATA_SHOW,116);
	}


	{//查找下限频率
	  u16 freq_a,freq_b,lower_freq;
	  SetTextValue(1,49,(u8*)"测量上下限频率中.");
	  for(i=0;i<SCAN_MAX_INDEX;i++)
	  if(SCAN_VPP[i] < 0.7071067f*SCAN_MAX && SCAN_VPP[i+1] > 0.7071067f*SCAN_MAX) break;
	  freq_a = freqScanArr[i]*100;freq_b = freqScanArr[i+1]*100;
	  lower_freq = (freq_a+freq_b)/2;
	  while(freq_a != lower_freq && freq_b != lower_freq)
	  {
	    Out_freq(2, lower_freq/10.0f);
	    HAL_Delay(20);
	    SCAN_VPP[i] = SCAN_RMS()*Svar.FFT_COMPENSATE/Svar.RMS_COMPENSATE;
	    if(SCAN_VPP[i] < 0.7071067f*SCAN_MAX) freq_a=lower_freq;
	    if(SCAN_VPP[i] >=0.7071067f*SCAN_MAX) freq_b=lower_freq;
	    lower_freq = (freq_a+freq_b)/2;
	  }
	  sprintf((char*)str,"%.1fHz",lower_freq/10.0f);
	  SetTextValue(1,45,str);
	}
	{//查找上限频率
	  u16 freq_a,freq_b,lower_freq;
	  SetTextValue(1,49,(u8*)"测量上上限频率中.");
	  for(i=SCAN_MAX_INDEX;i<116;i++)
	  if(SCAN_VPP[i] > 0.7071067f*SCAN_MAX && SCAN_VPP[i+1] < 0.7071067f*SCAN_MAX) break;
	  freq_a = freqScanArr[i]*10;freq_b = freqScanArr[i+1]*10;
	  lower_freq = (freq_a+freq_b)/2;
	  while(freq_a != lower_freq && freq_b != lower_freq)
	  {
	    Out_freq(2, lower_freq);
	    HAL_Delay(20);
	    SCAN_VPP[i] = SCAN_RMS()*Svar.FFT_COMPENSATE/Svar.RMS_COMPENSATE;
	    if(SCAN_VPP[i] > 0.7071067f*SCAN_MAX) freq_a=lower_freq;
	    if(SCAN_VPP[i] <=0.7071067f*SCAN_MAX) freq_b=lower_freq;
	    lower_freq = (freq_a+freq_b)/2;
	  }
	  sprintf((char*)str,"%dHz",lower_freq);
	  SetTextValue(1,46,str);
	}
}
//检查故障
void FaultChecK(void)
{

	  u8 str[20];
	  Out_freq(2, 1000);

	  //开启CH1获取值
	  HAL_Delay(200);
	  CH_VPP_VALUE[0] = SCAN_FFT_1K(0x01)*Svar.FFT_COMPENSATE/511;

	  //开启CH2获取值
	  HAL_Delay(100);
	  CH_VPP_VALUE[1] = SCAN_FFT_1K(0x02)*Svar.FFT_COMPENSATE/511;

	  //开启CH3获取值
	  HAL_Delay(110);
	  CH_VPP_VALUE[2] = SCAN_FFT_1K(0x04)*Svar.FFT_COMPENSATE*Svar.DC_COMPENSATE/511;

	  RELAY_NO;
	  Init_ADS8688(0x10);
	  HAL_Delay(300);
	  //开启CH4获取值
	  CH_VPP_VALUE[3] = SCAN_RMS()*Svar.FFT_COMPENSATE/Svar.RMS_COMPENSATE;

	  RELAY_NC;
	  HAL_Delay(300);
	  //开启CH5获取值
	  CH_VPP_VALUE[4] = SCAN_FFT_1K(0x20)*Svar.FFT_COMPENSATE/511;
  if(CH_VPP_VALUE[2] > 2065 && CH_VPP_VALUE[2] < 2095)
	  SetTextValue(1,49,(u8*)"R2断路");
  else if(CH_VPP_VALUE[2] > 160  && CH_VPP_VALUE[2] < 210)
	  SetTextValue(1,49,(u8*)"R3断路");
  else if(CH_VPP_VALUE[2] > 5470 && CH_VPP_VALUE[2] < 5495)
	  SetTextValue(1,49,(u8*)"R1短路");
  else if(CH_VPP_VALUE[2] > 5860 && CH_VPP_VALUE[2] < 5890)
	  SetTextValue(1,49,(u8*)"R3短路");
  else if(CH_VPP_VALUE[2] > 90   && CH_VPP_VALUE[2] <  110)
	  SetTextValue(1,49,(u8*)"R4短路");
  else if(CH_VPP_VALUE[2] > 5805 && CH_VPP_VALUE[2] < 5825)
  {
	if(CH_VPP_VALUE[1] > 19.80 && CH_VPP_VALUE[1] < 20.10)
	  SetTextValue(1,49,(u8*)"R1断路");
	if(CH_VPP_VALUE[1] > 17.80 && CH_VPP_VALUE[1] < 19.20)
	  SetTextValue(1,49,(u8*)"R4断路");
	if(CH_VPP_VALUE[1] < 1)
	  SetTextValue(1,49,(u8*)"R2短路");
  }
  else if(CH_VPP_VALUE[2] > 3630 && CH_VPP_VALUE[2] < 3640)
  {
	if(CH_VPP_VALUE[4] < 1)
	  SetTextValue(1,49,(u8*)"C1断路");
	if(CH_VPP_VALUE[3] > 28 && CH_VPP_VALUE[3] < 32)
	  SetTextValue(1,49,(u8*)"C2断路");
  }
  else
  {
	  SetTextValue(1,49,(u8*)"电路正常");
  }
}
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
