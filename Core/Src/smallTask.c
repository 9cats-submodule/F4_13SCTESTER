#include "main.h"
#include "cmsis_os.h"
#include "cmd_process.h"

/*!
 *  \brief    LED0自动翻转
 *  \details
 */
void StartLED0Toggle(void const *argument) {
	for (;;) {
		HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);
		vTaskDelay_ms(500);
		HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
		vTaskDelay_ms(500);
	}
}

/*!
 *  \brief    LED1自动翻转
 *  \details
 */
void StartLED1Toggle(void const *argument) {
	for (;;) {
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
		vTaskDelay_ms(500);
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
		vTaskDelay_ms(500);
	}
}

/*!
 *  \brief    变量自动存储
 *  \details  每隔0.1s储存一次数据，优先级最低
 */
void FLASH_Data_AutoUpdate_Start(void *arguement)	{
	for(;;)
	{
		DATA_UPDATE();
		vTaskDelay(100 / portTICK_RATE_MS);
	}
}

/*!
 *  \brief    TFT串口指令处理
 *  \details  当USART1串口接收到TFT的指令时接触阻塞
 */
void TFT_CMD_Process_Start(void *argument)	{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	CTRL_MSG   TFT_CMD_MSG = {0};

	USART1_RX = xQueueCreate(3,CMD_MAX_SIZE);

  for(;;)
  {
  	if(xQueueReceiveFromISR(USART1_RX, &TFT_CMD_MSG, &xHigherPriorityTaskWoken) == pdPASS)
  		ProcessMessage(&TFT_CMD_MSG,0);
  }
}
