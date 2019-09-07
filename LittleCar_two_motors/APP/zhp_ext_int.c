#include "zhp_ext_int.h"

void zhp_ext_int_init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;  //����GPIO�ṹ��
  RCC_APB2PeriphClockCmd(MOTOR_INT_CLOCK_RCC, ENABLE);


  GPIO_InitStructure.GPIO_Pin = MOTOR_A_INT_IO;//�ⲿ�ж�
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  //50Mʱ���ٶ�
  GPIO_Init(MOTOR_A_GIPO_GROUP, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = MOTOR_B_INT_IO;//�ⲿ�ж�
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  //50Mʱ���ٶ�
  GPIO_Init(MOTOR_B_GIPO_GROUP, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;	
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//50Mʱ���ٶ�
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;	
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//50Mʱ���ٶ�
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  EXTI_InitTypeDef EXTI_InitStructure;

  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource11);  //PA14��Ϊ�ⲿ�ж���14����
  EXTI_ClearITPendingBit(EXTI_Line11);                          //���14�߱�־λ
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;       //�½��ش���
  EXTI_InitStructure.EXTI_Line = EXTI_Line11;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource13);
  EXTI_ClearITPendingBit(EXTI_Line13);
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_InitStructure.EXTI_Line = EXTI_Line13;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);


  NVIC_InitTypeDef NVIC_InitStructure;
  //NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  //�������2

  //�ж���10��15����һ���ж�EXTI15_10_IRQn
  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;//�����ⲿ�ж�15_10
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;//ռ��ʽ���ȼ�����
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;    //�����ȼ�����
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_Init(&NVIC_InitStructure);  
}

float circumference = 0.058 * 3.1415926;
float pulse_per_round = 100.0;
float r_speed = 0;
float l_speed = 0;
int32_t r_cnt = 0;
int32_t l_cnt = 0;

void EXTI15_10_IRQHandler(void)
{
  OSIntEnter();
  if ( EXTI_GetITStatus(EXTI_Line11) != RESET )  
  {
    EXTI_ClearITPendingBit(EXTI_Line11);
    if ((GPIOB->IDR & GPIO_Pin_12) != 0)
    {
      l_cnt++;
    }
    else if ((GPIOB->IDR & GPIO_Pin_12) == 0)
    {
      l_cnt--;
    }
   }
  else if (EXTI_GetITStatus(EXTI_Line13) != RESET)
  {
    EXTI_ClearITPendingBit(EXTI_Line13);
    if ((GPIOB->IDR & GPIO_Pin_14) != 0)
    {
      r_cnt--;
    }
    else if ((GPIOB->IDR & GPIO_Pin_14) == 0)
    {
      r_cnt++;
    }
  }
  OSIntExit();
}

void calc_speed(float cycle) //s
{
  r_speed = r_cnt;
  l_speed = l_cnt;
  r_cnt = 0;
  l_cnt = 0;
}
