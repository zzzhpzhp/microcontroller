
#include "LED.h"


/**************************ʵ�ֺ���********************************************
*����ԭ��:        void Initial_LED_GPIO(void)
*��������:        ���� LED ��Ӧ�Ķ˿�Ϊ���
*******************************************************************************/
void LED_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_APB2PeriphClockCmd(LED_PORT_CLOCK |RCC_APB2Periph_AFIO, ENABLE);     //ʹ��PA,PD�˿�ʱ��

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4;                 //LED0-->PA.8 �˿�����
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;          //�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;         //IO���ٶ�Ϊ50MHz
    GPIO_Init(GPIOB, &GPIO_InitStructure);                     //�����趨������ʼ��GPIOA.8
    GPIO_SetBits(LED_PORT,LED0_PIN | LED1_PIN);                         //PA.8 �����

    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);

}



//------------------End of File----------------------------
