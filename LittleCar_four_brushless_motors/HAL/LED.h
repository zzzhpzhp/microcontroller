#ifndef __LED_H
#define __LED_H

#include "stm32f10x.h"
#include "sys.h"

#define LED_PORT GPIOB
#define LED_PORT_CLOCK RCC_APB2Periph_GPIOB
#define LED0_PIN GPIO_Pin_4
#define LED1_PIN GPIO_Pin_3
//LEDs 定义LED 操作宏.
//(输出低电平,灯亮;输出高电平灯灭)
#define LED0_ON()   GPIO_ResetBits(LED_PORT, LED0_PIN)
#define LED0_OFF()  GPIO_SetBits(LED_PORT, LED0_PIN)
#define LED1_ON()   GPIO_ResetBits(LED_PORT, LED1_PIN)
#define LED1_OFF()  GPIO_SetBits(LED_PORT, LED1_PIN)



void LED_Init(void);

#endif


//------------------End of File----------------------------
