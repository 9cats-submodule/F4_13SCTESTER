#ifndef _AD9959DDS_H_ 
#define _AD9959DDS_H_ 
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stdint.h" 
#include "base.h"

//AD9959寄存器地址定义 
#define CSR_ADD 0x00 //CSR 通道选择寄存器 
#define FR1_ADD 0x01 //FR1 功能寄存器1 
#define FR2_ADD 0x02 //FR2 功能寄存器2 
#define CFR_ADD 0x03 //CFR 通道功能寄存器 
#define CFTW0_ADD 0x04 //CTW0 通道频率转换字寄存器 
#define CPOW0_ADD 0x05 //CPW0 通道相位转换字寄存器 
#define ACR_ADD 0x06 //ACR 幅度控制寄存器 
#define LSRR_ADD 0x07 //LSR 通道线性扫描寄存器 
#define RDW_ADD 0x08 //RDW 通道线性向上扫描寄存器 
#define FDW_ADD 0x09 //FDW 通道线性向下扫描寄存器 

//AD9959管脚宏定义 
#define AD9959_PS0_L    HAL_GPIO_WritePin(DDS_P0_GPIO_Port   ,DDS_P0_Pin      ,GPIO_PIN_RESET)
#define AD9959_PS1_L    HAL_GPIO_WritePin(DDS_P1_GPIO_Port   ,DDS_P1_Pin      ,GPIO_PIN_RESET)
#define AD9959_PS2_L    HAL_GPIO_WritePin(DDS_P2_GPIO_Port   ,DDS_P2_Pin      ,GPIO_PIN_RESET)
#define AD9959_PS3_L    HAL_GPIO_WritePin(DDS_P3_GPIO_Port   ,DDS_P3_Pin      ,GPIO_PIN_RESET)
#define AD9959_SDIO0_H  HAL_GPIO_WritePin(DDS_SD0_GPIO_Port  ,DDS_SD0_Pin     ,GPIO_PIN_SET)
#define AD9959_SDIO0_L  HAL_GPIO_WritePin(DDS_SD0_GPIO_Port  ,DDS_SD0_Pin     ,GPIO_PIN_RESET)
#define AD9959_SDIO1_L  HAL_GPIO_WritePin(DDS_SD1_GPIO_Port  ,DDS_SD1_Pin     ,GPIO_PIN_RESET)
#define AD9959_SDIO2_L  HAL_GPIO_WritePin(DDS_SD2_GPIO_Port  ,DDS_SD2_Pin     ,GPIO_PIN_RESET)
#define AD9959_SDIO3_L  HAL_GPIO_WritePin(DDS_SD3_GPIO_Port  ,DDS_SD3_Pin     ,GPIO_PIN_RESET)
#define AD9959_PWR_L    HAL_GPIO_WritePin(DDS_PWR_GPIO_Port  ,DDS_PWR_Pin     ,GPIO_PIN_RESET)
#define AD9959_Reset_H  HAL_GPIO_WritePin(DDS_REST_GPIO_Port ,DDS_REST_Pin    ,GPIO_PIN_SET)
#define AD9959_Reset_L  HAL_GPIO_WritePin(DDS_REST_GPIO_Port ,DDS_REST_Pin    ,GPIO_PIN_RESET)
#define AD9959_UPDATE_H HAL_GPIO_WritePin(DDS_IOUP_GPIO_Port ,DDS_IOUP_Pin    ,GPIO_PIN_SET)
#define AD9959_UPDATE_L HAL_GPIO_WritePin(DDS_IOUP_GPIO_Port ,DDS_IOUP_Pin    ,GPIO_PIN_RESET)
#define AD9959_CS_H     HAL_GPIO_WritePin(DDS_CS_GPIO_Port   ,DDS_CS_Pin      ,GPIO_PIN_SET)
#define AD9959_CS_L     HAL_GPIO_WritePin(DDS_CS_GPIO_Port   ,DDS_CS_Pin      ,GPIO_PIN_RESET)
#define AD9959_SCLK_H   HAL_GPIO_WritePin(DDS_SCLK_GPIO_Port ,DDS_SCLK_Pin    ,GPIO_PIN_SET)
#define AD9959_SCLK_L   HAL_GPIO_WritePin(DDS_SCLK_GPIO_Port ,DDS_SCLK_Pin    ,GPIO_PIN_RESET)

#define FreqMax 80000000.0f 
#define FreqMin 5.0f 

typedef struct _sweep_freq //正弦波形结构体 
{ 
	u32 flag; //是否使能 
	u32 start; //起始Hz 
	u32 step; //步进Hz 
	u32 time; //单次停留时间 
	u32 end; //终点Hz 
}sweep_freq; 

typedef struct _sin_wave //正弦波参数结构体 
{ 
	u32 freq; //频率 
	u16 phase; //相位 
	float amp; //幅值 
}sinw_ave; 

extern sweep_freq sweepfreq; 
extern sinw_ave sinwave;
void Out_freq(u8 ch, u32 freq); 
void Out_mV(u8 ch, float nmV); 

void Init_AD9959(void); 
void Sweep_Freq(void); 

void Write_Phase(u8 Channel,u16 Phase);

#endif
