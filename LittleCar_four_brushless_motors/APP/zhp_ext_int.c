#include "zhp_ext_int.h"

void zhp_ext_int_init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;  //定义GPIO结构体
  RCC_APB2PeriphClockCmd(MOTOR_INT_CLOCK_RCC, ENABLE);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;	
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//50M时钟速度
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  EXTI_InitTypeDef EXTI_InitStructure;

  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource12);  //PA14作为外部中断线14引脚
  EXTI_ClearITPendingBit(EXTI_Line12);                          //清除14线标志位
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;       //双边沿触发
  EXTI_InitStructure.EXTI_Line = EXTI_Line12;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource13);
  EXTI_ClearITPendingBit(EXTI_Line13);
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_InitStructure.EXTI_Line = EXTI_Line13;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
	
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource14);
  EXTI_ClearITPendingBit(EXTI_Line14);
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_InitStructure.EXTI_Line = EXTI_Line14;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
	
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource15);
  EXTI_ClearITPendingBit(EXTI_Line15);
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_InitStructure.EXTI_Line = EXTI_Line15;
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


void EXTI15_10_IRQHandler(void)
{
  OSIntEnter();
  if ( EXTI_GetITStatus(EXTI_Line12) != RESET )  
  {
    EXTI_ClearITPendingBit(EXTI_Line12);
		auto_ctrl_val.l0_cnt++;
  }
  else if (EXTI_GetITStatus(EXTI_Line13) != RESET)
  {
    EXTI_ClearITPendingBit(EXTI_Line13);
		auto_ctrl_val.l1_cnt++;
  }
  else if (EXTI_GetITStatus(EXTI_Line14) != RESET)
  {
    EXTI_ClearITPendingBit(EXTI_Line14);
		auto_ctrl_val.r0_cnt++;
  }
  else if (EXTI_GetITStatus(EXTI_Line15) != RESET)
  {
    EXTI_ClearITPendingBit(EXTI_Line15);
		auto_ctrl_val.r1_cnt++;
  }
	
  OSIntExit();
}

void calc_speed(float cycle) //s
{
//  r_speed = r_cnt;
//  l_speed = l_cnt;
//  r_cnt = 0;
//  l_cnt = 0;
}
