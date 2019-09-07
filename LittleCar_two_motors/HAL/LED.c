
#include "LED.h"


/**************************实现函数********************************************
*函数原型:        void Initial_LED_GPIO(void)
*功　　能:        配置 LED 对应的端口为输出
*******************************************************************************/
void LED_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_APB2PeriphClockCmd(LED_PORT_CLOCK |RCC_APB2Periph_AFIO, ENABLE);     //使能PA,PD端口时钟

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4;                 //LED0-->PA.8 端口配置
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;          //推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;         //IO口速度为50MHz
    GPIO_Init(GPIOB, &GPIO_InitStructure);                     //根据设定参数初始化GPIOA.8
    GPIO_SetBits(LED_PORT,LED0_PIN | LED1_PIN);                         //PA.8 输出高

    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);

}



//------------------End of File----------------------------
