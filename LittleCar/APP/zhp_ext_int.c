#include "zhp_ext_int.h"

void zhp_ext_int_init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;  //定义GPIO结构体
  RCC_APB2PeriphClockCmd(MOTOR_INT_CLOCK_RCC, ENABLE);


  GPIO_InitStructure.GPIO_Pin = MOTOR_A_INT_IO;//外部中断
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  //50M时钟速度
  GPIO_Init(MOTOR_A_GIPO_GROUP, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = MOTOR_B_INT_IO;//外部中断
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  //50M时钟速度
  GPIO_Init(MOTOR_B_GIPO_GROUP, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;	
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//50M时钟速度
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;	
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//50M时钟速度
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  EXTI_InitTypeDef EXTI_InitStructure;

  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource11);  //PA14作为外部中断线14引脚
  EXTI_ClearITPendingBit(EXTI_Line11);                          //清除14线标志位
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;       //下降沿触发
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
  //NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  //采用组别2

  //中断线10至15共用一个中断EXTI15_10_IRQn
  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;//配置外部中断15_10
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;//占先式优先级设置
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;    //副优先级设置
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
