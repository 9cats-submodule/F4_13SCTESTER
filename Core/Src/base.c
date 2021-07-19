#include "base.h"
#include "w25qxx.h"

static u8 fac_us = 0;            //us延时倍乘数
const u32 SAVE_ADDR = 0x0000f000;//FLASH储存地址
u8 STR_BUF[40]={0};

/*!
 *  \brief 初始化延迟函数
 *  \param SYSCLK 系统时钟
 */
void delay_init(u8 SYSCLK) {
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK); //SysTick频率为HCLK
	fac_us = SYSCLK;
}

/*!
 *  \brief 延时函数
 *  \param t ns
 */
void delay_ns(u8 t) {
	do {
		;
	} while (--t);
}

/*!
 *  \brief 延时函数
 *  \param t nus
 */
void delay_us(u32 nus) {
	u32 ticks;
	u32 told, tnow, tcnt = 0;
	u32 reload = SysTick->LOAD; //LOAD的值
	ticks = nus * fac_us;       //需要的节拍数
	told = SysTick->VAL;        //刚进?时的计数器值
	while (1) {
		tnow = SysTick->VAL;
		if (tnow != told) {
			if (tnow < told)
				tcnt += told - tnow; //这?注意?下SYSTICK是?个递减的计数器就可以了.
			else
				tcnt += reload - tnow + told;
			told = tnow;
			if (tcnt >= ticks)
				break; //时间超过/等于要延迟的时间,则退出.
		}
	};
}

/*!
 *  \brief 延时函数
 *  \param t ms
 */
void delay_ms(u16 nms) {
	u32 i;
	for (i = 0; i < nms; i++)
		delay_us(1000);
}
/*!
 *  \brief 按键扫描函数
 *  \param mode 是否支持连按
 */
//按键扫描函数
//不使用时注释
u8 KEY_Scan(u8 mode) {
	static u8 key_up = 1; //按键按松开标志
	if (mode)
		key_up = 1; //持连按
	if (key_up && (KEY0 == 0 || KEY1 == 0 || KEY2 == 0)) {
		HAL_Delay(10); //去抖动
		key_up = 0;
		if (KEY0 == 0)
			return KEY0_PRES;
		else if (KEY1 == 0)
			return KEY1_PRES;
		else if (KEY2 == 0)
			return KEY2_PRES;
	} else if (KEY0 == 1 && KEY1 == 1 && KEY2 == 1)
		key_up = 1;
	return 0; // 按键按下
}

/*!
 *  \brief  数据保存操作，0-写入 1-读出 2-写入
 *  \param mode 操作模式
 */
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

/*!
 *  \brief 数据初始化，若按下KEY0则恢复默认
 */
void DATA_INIT() {
  u8 key = KEY_Scan(0);
  if(key == KEY0_PRES)  DATA_OP(0);
  else                  DATA_OP(1);
}

/*!
 *  \brief 数据更新
 */
void DATA_UPDATE() {
    DATA_OP(2);
}
