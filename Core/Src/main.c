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
#include "math.h"
#include "text.h"
#include "touch.h"
#include "w25qxx.h"
#include "data.h"
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

#define SAMPLE_POINT_MAX 2048 //��������
float FFT_INPUT [SAMPLE_POINT_MAX];
float FFT_OUTPUT[SAMPLE_POINT_MAX];
float FFT_OUTPUT_REAL[SAMPLE_POINT_MAX];
float WAVE[SAMPLE_POINT_MAX];

float freq = 1000.0f,L_freq = 1000.0f;
float mv   =300.0f,L_mv = 300.0f;

//-----------------------------��־---------------------------------------
extern u8 SAMPLE_END_FLAG;    //�����������
//------------------------------------------------------------------------

//-----------------------------����---------------------------------------
extern u16 SAMPLE_POINT;       //��Ҫ�����ĵ���
extern s32 BUF[SAMPLE_POINT_MAX];

float CH_VPP_VALUE[5] = {0};
float SCAN_VPP[200];
float SCAN_MAX;
u32   SCAN_MAX_INDEX;
u8    DATA_SHOW[116];
u8    mode = 0; //��ǰģʽ 0-���� 1-�������� 2-��Ƶ���� 3-���ϼ��

float test = 500.0f;

//----------------------------�ɴ������------------------------------------------
SVAR Svar = {
  /*float FFT_COMPENSATE; //����FFT�����*/909.09090909f,
  /*float RMS_COMPENSATE; //����RMS     */1134.0f,
  /*float DC_COMPENSATE;  //FFT��ֱ������*/0.25473997f,
  /*float ADS_OFFSET;     //ADSƫ�ò���  */32.2000008f,
  /*float C3_NORMAL;      //C3*/98.0f
};

void FFT(void)
{
	u16 i;
	arm_rfft_fast_instance_f32 S;

	for(i=0;i<2048;i++)
	{
		FFT_INPUT[i]  = (float)BUF[i]*20.48/0x10000;
	}
	arm_rfft_fast_init_f32(&S,2048);                    //FFT��ʼ��
	arm_rfft_fast_f32(&S, FFT_INPUT, FFT_OUTPUT,0);     //FFT�仯
	arm_cmplx_mag_f32(FFT_OUTPUT,FFT_OUTPUT_REAL,2048); //��ģ
}
//THD-��ʧ��� 64k���� 1k����
float THD(void)
{
  //����
  float basic = FFT_OUTPUT_REAL[32];
  //г��
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


//ͨ��FFT��ĳͨ����1K����
float SCAN_FFT_1K(u8 ch)
{
  //��ʱ��Ƶ�ʸ�Ϊ64K
  __HAL_TIM_SET_PRESCALER(&htim1,25-1);
  __HAL_TIM_SET_AUTORELOAD(&htim1,105-1);

  Init_ADS8688(ch);              //ֻ����ͨ��1
  SAMPLE_POINT       = 2048;     //�ɼ�2048����
  HAL_Delay(150);
  HAL_TIM_Base_Start_IT(&htim1); //������ʱ��
  while(!SAMPLE_END_FLAG);       //�ȴ��������
  SAMPLE_END_FLAG = 0;           //�����������
  FFT();                         //FFT�����CH1��ֵ

  return FFT_OUTPUT_REAL[32];
}

float SCAN_RMS(void)
{
  u16 i;
  float sum=0;

  //��ʱ��Ƶ�ʸ�Ϊ64K
  __HAL_TIM_SET_PRESCALER(&htim1,25-1);
  __HAL_TIM_SET_AUTORELOAD(&htim1,105-1);

  SAMPLE_POINT  = 2048;         //�ɼ�100����
  HAL_TIM_Base_Start_IT(&htim1); //������ʱ��
  while(!SAMPLE_END_FLAG);       //�ȴ��������
  SAMPLE_END_FLAG = 0;           //�����������

  for(i=0;i<2048;i++)
  {
    sum+= (s32)BUF[i]+Svar.ADS_OFFSET;
  }

  return sum/2048;
}

float SCAN_FFT_10(u8 ch)
{
	u16 i;
	arm_rfft_fast_instance_f32 S;
	//��ʱ��Ƶ�ʸ�Ϊ2560Hz   __  2560Hz *  1/256 = 10 Hz
	__HAL_TIM_SET_PRESCALER(&htim1,105-1);
    __HAL_TIM_SET_AUTORELOAD(&htim1,625-1);

	Init_ADS8688(ch);            //ֻ����ͨ��1
	SAMPLE_POINT   = 256;        //�ɼ�2048����
	HAL_Delay(2);
	HAL_TIM_Base_Start_IT(&htim1); //������ʱ��
	while(!SAMPLE_END_FLAG);       //�ȴ��������
	SAMPLE_END_FLAG = 0;           //�����������

	for(i=0;i<256;i++)
	{
		FFT_INPUT[i]  = (float)BUF[i]*20.48/0x10000;
	}
	arm_rfft_fast_init_f32(&S,256);                    //FFT��ʼ��
	arm_rfft_fast_f32(&S, FFT_INPUT, FFT_OUTPUT,0);    //FFT�仯
	arm_cmplx_mag_f32(FFT_OUTPUT,FFT_OUTPUT_REAL,256); //��ģ

	return FFT_OUTPUT_REAL[5];
}
/*!
 *  \brief  ָ�� FFT �任
 *  \param ch    ָ����ͨ��
 *  \param num   ���ɳ���
 *  \param index ��Ӧ�ĵ�
 */
float SCAN_FFT(u8 ch,u16 num,u16 index)
{
	arm_rfft_fast_instance_f32 S;
	u16 i;

	Init_ADS8688(ch);            //ֻ����ĳͨ��
	SAMPLE_POINT = num;          //�ɼ� num ����
	HAL_Delay(100);
	HAL_TIM_Base_Start_IT(&htim1); //������ʱ��
	while(!SAMPLE_END_FLAG);       //�ȴ��������
	SAMPLE_END_FLAG = 0;           //�����������

	for(i=0;i<num;i++)
	{
		FFT_INPUT[i]  = (float)BUF[i]*20.48/0x10000;
	}
	arm_rfft_fast_init_f32(&S,num);                    //FFT��ʼ��
	arm_rfft_fast_f32(&S, FFT_INPUT, FFT_OUTPUT,0);    //FFT�仯
	arm_cmplx_mag_f32(FFT_OUTPUT,FFT_OUTPUT_REAL,num); //��ģ

	return FFT_OUTPUT_REAL[index];
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
  //��ʱ��
  delay_init(168);
  W25QXX_Init();
  LCD_Init();
  font_init();
  tp_dev.init();
  TFT_Init();
  DATA_INIT();
  Init_AD9959();

  //ʱ��
  //DDS��� -- CH3���
  Out_freq(2, 10);
  Out_mV(2, 300);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	/*��������������������迹�����棬��Ƶ���ԣ�ʧ���*/
	HAL_Delay(100);
	switch (mode) {
	case 0:SetTextValue(1,49,(u8*)"������");break;
	case 1:Measure();break;
	case 2:ScanFreq();break;
	case 3:FaultChecK();break;
	}


	//DEBUG�ã�����DDS���
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
//��������
void Measure(void)
{
  Out_freq(2, 1000);
  SetTextValue(1,49,(u8*)"������·������.   ");

  //����CH1��ȡֵ
  HAL_Delay(200);
  CH_VPP_VALUE[0] = SCAN_FFT_1K(0x01)*Svar.FFT_COMPENSATE/511;
  //CH1_VPP
  SetTextValue(0,41,Str("%.2fmV",CH_VPP_VALUE[0]));
  SetTextValue(1,49,(u8*)"������·������..  ");

  //����CH2��ȡֵ
  HAL_Delay(100);
  CH_VPP_VALUE[1] = SCAN_FFT_1K(0x02)*Svar.FFT_COMPENSATE/511;
  //CH2_VPP  �����迹
  SetTextValue(0,42,Str("%.2fmV",CH_VPP_VALUE[1]));
  SetTextValue(1,41,Str("%.3fK��",CH_VPP_VALUE[1]/(CH_VPP_VALUE[0]-CH_VPP_VALUE[1])*6.8));
  SetTextValue(1,49,(u8*)"������·������... ");

  //����CH3��ȡֵ
  HAL_Delay(110);
  SCAN_FFT_1K(0x04);
  CH_VPP_VALUE[2] = FFT_OUTPUT_REAL[0]*Svar.FFT_COMPENSATE*Svar.DC_COMPENSATE/511;
  SetTextValue(0,43,Str("%.0fmV",CH_VPP_VALUE[2]));
  SetTextValue(1,49,(u8*)"������·������....");

  //����CH4��ȡֵ
  RELAY_NO;
  Init_ADS8688(0x10);
  HAL_Delay(300);
  CH_VPP_VALUE[3] = SCAN_RMS()*Svar.FFT_COMPENSATE/Svar.RMS_COMPENSATE;
  //��ʾCH4_VPP ����
  SetTextValue(0,44,Str("%.2fmV",CH_VPP_VALUE[3]));
  SetTextValue(1,43,Str("%.2f",CH_VPP_VALUE[3]/CH_VPP_VALUE[1]));

  //����CH5��ȡֵ
  RELAY_NC;
  HAL_Delay(300);
  CH_VPP_VALUE[4] = SCAN_FFT_1K(0x20)*Svar.FFT_COMPENSATE/511;
  //��ʾCH5_VPP ����迹 ʧ���
  SetTextValue(0,45,Str("%.2fmV",CH_VPP_VALUE[4]));
  SetTextValue(1,42,Str("%.3fK��",(CH_VPP_VALUE[3]-CH_VPP_VALUE[4])/CH_VPP_VALUE[4]));
  SetTextValue(1,44,Str("%.2f%%",THD()*100));
}
//��Ƶ����
void ScanFreq(void)
{
  u32 i;
  u8 str[20];
  Out_mV(2, 130);
  HAL_Delay(100);
  {//ɨƵ
    RELAY_NO;
    Init_ADS8688(0x10);
    //ɨƵ
    for(i=0;i<126;i++)
    {
      Out_freq(2, freqScanArr[i]*10);
      SetTextValue(1,49,Str("ɨƵ:%ld%%",i*100/126));
      HAL_Delay(20);
      SCAN_VPP[i] = SCAN_RMS()*Svar.FFT_COMPENSATE/Svar.RMS_COMPENSATE;
    }
    //�����ֵ -- �����ֵ��Ӧ������
    arm_max_f32(SCAN_VPP, 200, &SCAN_MAX, &SCAN_MAX_INDEX);
    DATA_SHOW[0] = 0;
      for(i=0;i<126;i++)
      {
      	float temp = 20*log10f(SCAN_VPP[i] / 12.2000008f) -30;
      	DATA_SHOW[i] = temp * 202 / 16;
      }
    if(mode == 2)
    GraphChannelDataAdd(1,51,1,(u8*)DATA_SHOW,126);
  }


	{//��������Ƶ��
	  u16 freq_a,freq_b,lower_freq;
	  SetTextValue(1,49,(u8*)"����������Ƶ����.");
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
    if(mode == 2)
	  SetTextValue(1,45,str);
	}
	{//��������Ƶ��
	  u32 freq_a,freq_b,lower_freq;
	  SetTextValue(1,49,(u8*)"����������Ƶ����.");
	  for(i=SCAN_MAX_INDEX;i<126;i++)
	  if(SCAN_VPP[i] > 0.7071067f*SCAN_MAX && SCAN_VPP[i+1] < 0.7071067f*SCAN_MAX) break;
	  freq_a = (u32)freqScanArr[i]*10;freq_b = (u32)freqScanArr[i+1]*10;
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
	  sprintf((char*)str,"%ldHz",lower_freq);
    if(mode == 2)
	  SetTextValue(1,46,str);
	}
	Out_mV(2, 300);
}
//������-1
float Scan_FFT_1(u16 index)
{
  SAMPLE_POINT       = 2048;     //�ɼ�2048����
  HAL_TIM_Base_Start_IT(&htim1); //������ʱ��
  while(!SAMPLE_END_FLAG);       //�ȴ��������
  SAMPLE_END_FLAG = 0;           //�����������
  FFT();                         //FFT�����CH1��ֵ
  return FFT_OUTPUT_REAL[index];
}
float Scan_RMS(void)
{
  u16 i;
  float sum=0;

  SAMPLE_POINT  = 2048;         //�ɼ�100����
  HAL_TIM_Base_Start_IT(&htim1); //������ʱ��
  while(!SAMPLE_END_FLAG);       //�ȴ��������
  SAMPLE_END_FLAG = 0;           //�����������

  for(i=0;i<2048;i++)
  {
    sum+= (s32)BUF[i]+Svar.ADS_OFFSET;
  }

  return sum/2048;
}
void FaultChecK(void)
{
  float TCH_VALUE[5];
  __HAL_TIM_SET_PRESCALER(&htim1,25-1);
  __HAL_TIM_SET_AUTORELOAD(&htim1,105-1);
  Out_freq(2, 1000);

  //����CH1��ȡֵ
  Init_ADS8688(0x01);
  HAL_Delay(100);//
  CH_VPP_VALUE[0] = Scan_FFT_1(32)*Svar.FFT_COMPENSATE/511;
  //��ʾCH1_VPP
  SetTextValue(0,41,Str("%.2fmV",CH_VPP_VALUE[0]));

  //����CH2��ȡֵ
  Init_ADS8688(0x02);
  HAL_Delay(100);//
  CH_VPP_VALUE[1] = Scan_FFT_1(32)*Svar.FFT_COMPENSATE/511;
  //��ʾCH2_VPP
  SetTextValue(0,42,Str("%.2fmV",CH_VPP_VALUE[1]));

  //����CH3��ȡֵ FFTֱ��
  Init_ADS8688(0x04);
  HAL_Delay(100);
  CH_VPP_VALUE[2] = Scan_FFT_1(0)*Svar.FFT_COMPENSATE*Svar.DC_COMPENSATE/511;
  SetTextValue(0,43,Str("%.0fmV",CH_VPP_VALUE[2]));

  //����CH4��ȡֵ
  RELAY_NO;
  Init_ADS8688(0x10);
  HAL_Delay(100);
  CH_VPP_VALUE[3] = Scan_RMS()*Svar.FFT_COMPENSATE/Svar.RMS_COMPENSATE;
  //��ʾCH4_VPP
  SetTextValue(0,44,Str("%.2fmV",CH_VPP_VALUE[3]));

  RELAY_NC;
  Init_ADS8688(0x20);
  HAL_Delay(100);
  //����CH5��ȡֵ
  CH_VPP_VALUE[4] = Scan_FFT_1(32)*Svar.FFT_COMPENSATE/511;
  //��ʾCH5_VPP
  SetTextValue(0,45,Str("%.2fmV",CH_VPP_VALUE[4]));

  //��ȡ10HZ��
//  __HAL_TIM_SET_PRESCALER(&htim1,105-1);
//  __HAL_TIM_SET_AUTORELOAD(&htim1,625-1);
//  Out_freq(2, 40);
//  TCH_VALUE[1]=SCAN_FFT(0x02,2048,32)*Svar.FFT_COMPENSATE/511;
//
//  sprintf((char*)str,"%.2fmV",TCH_VALUE[1]);
//  SetTextValue(0,54,str);

  //100KHz�µ����

  Out_freq(2, 100000);
  __HAL_TIM_SET_PRESCALER(&htim1,25-1);
  __HAL_TIM_SET_AUTORELOAD(&htim1,105-1);
  //��ȡ100K��
  Init_ADS8688(0x10);
  HAL_Delay(150);
  TCH_VALUE[3]=SCAN_RMS()*Svar.FFT_COMPENSATE/Svar.RMS_COMPENSATE;
  SetTextValue(0,55,Str("%.2fmV",TCH_VALUE[3]));

  if(CH_VPP_VALUE[2] > 2055 && CH_VPP_VALUE[2] < 2095)
	  SetTextValue(1,49,(u8*)"R2��·");
  else if(CH_VPP_VALUE[2] > 160  && CH_VPP_VALUE[2] < 210)
	  SetTextValue(1,49,(u8*)"R3��·");
  else if(CH_VPP_VALUE[2] > 5480 && CH_VPP_VALUE[2] < 5510)
	  SetTextValue(1,49,(u8*)"R1��·");
  else if(CH_VPP_VALUE[2] > 5860 && CH_VPP_VALUE[2] < 5890)
	  SetTextValue(1,49,(u8*)"R3��·");
  else if(CH_VPP_VALUE[2] > 90   && CH_VPP_VALUE[2] <  110)
	  SetTextValue(1,49,(u8*)"R4��·");
  else if(CH_VPP_VALUE[2] > 5805 && CH_VPP_VALUE[2] < 5845)
  {
	if(CH_VPP_VALUE[1] > 19.0 && CH_VPP_VALUE[1] < 20.5)
	  SetTextValue(1,49,(u8*)"R1��·");
	if(CH_VPP_VALUE[1] > 17.60 && CH_VPP_VALUE[1] < 19.20)
	  SetTextValue(1,49,(u8*)"R4��·");
	if(CH_VPP_VALUE[1] < 1)
	  SetTextValue(1,49,(u8*)"R2��·");
  }
  else if(CH_VPP_VALUE[2] > 3580 && CH_VPP_VALUE[2] < 3650)
  {
	if(CH_VPP_VALUE[4] < 1)
	  SetTextValue(1,49,(u8*)"C1��·");
	else if(CH_VPP_VALUE[3] > 11 && CH_VPP_VALUE[3] < 400)
	  SetTextValue(1,49,(u8*)"C2��·");
	else if(CH_VPP_VALUE[3] > 1180 && CH_VPP_VALUE[3] < 1210)
	  SetTextValue(1,49,(u8*)"C2�ӱ�");
	else if(TCH_VALUE[3] > Svar.C3_NORMAL+3 && TCH_VALUE[3] < Svar.C3_NORMAL+20)
	  SetTextValue(1,49,(u8*)"C3��·");
	else if(TCH_VALUE[3] > Svar.C3_NORMAL-20 && TCH_VALUE[3] < Svar.C3_NORMAL-3)
	  SetTextValue(1,49,(u8*)"C3�ӱ�");
//	else if(TCH_VALUE[3] > 98 && TCH_VALUE[3] < 105)
//	  SetTextValue(1,49,(u8*)"C3��·");
//	else if(TCH_VALUE[3] > 84 && TCH_VALUE[3] < 91)
//	  SetTextValue(1,49,(u8*)"C3�ӱ�");
	else
	  SetTextValue(1,49,(u8*)"��·����");
  }
//  else if(TCH_VALUE[1] > 16.70 && TCH_VALUE[1] < 16.89)
//		SetTextValue(1,49,(u8*)"C1�ӱ�");

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
