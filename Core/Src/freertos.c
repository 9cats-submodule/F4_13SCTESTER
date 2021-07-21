/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TASK void (*)(void *)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for LED0_Toggle */
osThreadId_t LED0_ToggleHandle;
const osThreadAttr_t LED0_Toggle_attributes = {
  .name = "LED0_Toggle",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh4,
};
/* Definitions for LED1_Toggle */
osThreadId_t LED1_ToggleHandle;
const osThreadAttr_t LED1_Toggle_attributes = {
  .name = "LED1_Toggle",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime4,
};
/* Definitions for TFT_CMD_Process */
osThreadId_t TFT_CMD_ProcessHandle;
const osThreadAttr_t TFT_CMD_Process_attributes = {
  .name = "TFT_CMD_Process",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow3,
};
/* Definitions for FLASH_Data_Auto */
osThreadId_t FLASH_Data_AutoHandle;
const osThreadAttr_t FLASH_Data_Auto_attributes = {
  .name = "FLASH_Data_Auto",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow2,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartLED0Toggle(void *argument);
void StartLED1Toggle(void *argument);
void TFT_CMD_Process_Start(void *argument);
void FLASH_Data_AutoUpdate_Start(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of LED0_Toggle */
  LED0_ToggleHandle = osThreadNew(StartLED0Toggle, NULL, &LED0_Toggle_attributes);

  /* creation of LED1_Toggle */
  LED1_ToggleHandle = osThreadNew(StartLED1Toggle, NULL, &LED1_Toggle_attributes);

  /* creation of TFT_CMD_Process */
  TFT_CMD_ProcessHandle = osThreadNew(TFT_CMD_Process_Start, NULL, &TFT_CMD_Process_attributes);

  /* creation of FLASH_Data_Auto */
  FLASH_Data_AutoHandle = osThreadNew(FLASH_Data_AutoUpdate_Start, NULL, &FLASH_Data_Auto_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartLED0Toggle */
/**
* @brief Function implementing the LED0_T thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLED0Toggle */
__weak void StartLED0Toggle(void *argument)
{
  /* USER CODE BEGIN StartLED0Toggle */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartLED0Toggle */
}

/* USER CODE BEGIN Header_StartLED1Toggle */
/**
* @brief Function implementing the LED1_Toggle thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLED1Toggle */
__weak void StartLED1Toggle(void *argument)
{
  /* USER CODE BEGIN StartLED1Toggle */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartLED1Toggle */
}

/* USER CODE BEGIN Header_TFT_CMD_Process_Start */
/**
* @brief Function implementing the TFT_CMD_Process thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TFT_CMD_Process_Start */
__weak void TFT_CMD_Process_Start(void *argument)
{
  /* USER CODE BEGIN TFT_CMD_Process_Start */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END TFT_CMD_Process_Start */
}

/* USER CODE BEGIN Header_FLASH_Data_AutoUpdate_Start */
/**
* @brief Function implementing the FLASH_Data_Auto thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_FLASH_Data_AutoUpdate_Start */
__weak void FLASH_Data_AutoUpdate_Start(void *argument)
{
  /* USER CODE BEGIN FLASH_Data_AutoUpdate_Start */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END FLASH_Data_AutoUpdate_Start */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
