#include "zhp_odom.h"

void phase_detector_pin_init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC| RCC_APB2Periph_AFIO, ENABLE);                 //使能端口时钟

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_6 | GPIO_Pin_5 | GPIO_Pin_4 | GPIO_Pin_3 | GPIO_Pin_2 | GPIO_Pin_1 | GPIO_Pin_0;          //端口配置
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;                                            
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;                                           //IO口速度为50MHz
  GPIO_Init(GPIOC, &GPIO_InitStructure);                                                      //根据设定参数初始化

  GPIO_ResetBits(GPIOC, GPIO_Pin_7 | GPIO_Pin_6 | GPIO_Pin_5 | GPIO_Pin_4 | GPIO_Pin_3 | GPIO_Pin_2 | GPIO_Pin_1 | GPIO_Pin_0);
}

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

	#define RIGHT_JUDGE_INC_DEC(TRIG_PORT, TRIG_PIN, LEVEL_PORT, LEVEL_PIN, VAR) \
	do \
	{ \
		if (GPIO_ReadInputDataBit(LEVEL_PORT, LEVEL_PIN)) \
		{ \
			if (GPIO_ReadInputDataBit(TRIG_PORT, TRIG_PIN)) \
			{ \
				VAR ++; \
			} \
			else \
			{ \
				VAR --; \
			} \
		} \
		else \
		{ \
			if (GPIO_ReadInputDataBit(TRIG_PORT, TRIG_PIN)) \
			{ \
				VAR --; \
			} \
			else \
			{ \
				VAR ++; \
			} \
		} \
	} while(0)
	
	#define LEFT_JUDGE_INC_DEC(TRIG_PORT, TRIG_PIN, LEVEL_PORT, LEVEL_PIN, VAR) \
	do \
	{ \
		if (GPIO_ReadInputDataBit(LEVEL_PORT, LEVEL_PIN)) \
		{ \
			if (GPIO_ReadInputDataBit(TRIG_PORT, TRIG_PIN)) \
			{ \
				VAR --; \
			} \
			else \
			{ \
				VAR ++; \
			} \
		} \
		else \
		{ \
			if (GPIO_ReadInputDataBit(TRIG_PORT, TRIG_PIN)) \
			{ \
				VAR ++; \
			} \
			else \
			{ \
				VAR --; \
			} \
		} \
	} while(0)
	
	if ( EXTI_GetITStatus(EXTI_Line12) != RESET )  
  {
    EXTI_ClearITPendingBit(EXTI_Line12);
		
		LEFT_JUDGE_INC_DEC(GPIOB, GPIO_Pin_12, GPIOC, GPIO_Pin_4, auto_ctrl_val.lower_left_cnt);
  }
	
  if (EXTI_GetITStatus(EXTI_Line13) != RESET)
  {
    EXTI_ClearITPendingBit(EXTI_Line13);
		
		LEFT_JUDGE_INC_DEC(GPIOB, GPIO_Pin_13, GPIOC, GPIO_Pin_5, auto_ctrl_val.higher_left_cnt);
  }
	
  if (EXTI_GetITStatus(EXTI_Line15) != RESET)
  {
    EXTI_ClearITPendingBit(EXTI_Line15);
		
		RIGHT_JUDGE_INC_DEC(GPIOB, GPIO_Pin_15, GPIOC, GPIO_Pin_7, auto_ctrl_val.higher_right_cnt);
  }
	
  if (EXTI_GetITStatus(EXTI_Line14) != RESET)
  {
    EXTI_ClearITPendingBit(EXTI_Line14);
		
		RIGHT_JUDGE_INC_DEC(GPIOB, GPIO_Pin_14, GPIOC, GPIO_Pin_6, auto_ctrl_val.lower_right_cnt);
  }
	
	#undef LEFT_JUDGE_INC_DEC
	#undef RIGHT_JUDGE_INC_DEC
	
  OSIntExit();
}

void calc_speed(float cycle) //s
{
//  r_speed = r_cnt;
//  l_speed = l_cnt;
//  r_cnt = 0;
//  l_cnt = 0;
}









//if ( EXTI_GetITStatus(EXTI_Line12) != RESET )  
//  {
//    EXTI_ClearITPendingBit(EXTI_Line12);
//		if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_4) && !GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_12))
//		{
//			auto_ctrl_val.lower_left_cnt++;
//		}
//		else
//		{
//			auto_ctrl_val.lower_left_cnt--;
//		}
//  }
//  else if (EXTI_GetITStatus(EXTI_Line13) != RESET)
//  {
//    EXTI_ClearITPendingBit(EXTI_Line13);
//		if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_5) && !GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13))
//		{
//			auto_ctrl_val.higher_left_cnt++;
//		}
//		else
//		{
//			auto_ctrl_val.higher_left_cnt--;
//		}
//  }
//  else if (EXTI_GetITStatus(EXTI_Line14) != RESET)
//  {
//    EXTI_ClearITPendingBit(EXTI_Line14);
//		if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_6) && !GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_14))
//		{
//			auto_ctrl_val.lower_right_cnt++;
//		}
//		else
//		{
//			auto_ctrl_val.lower_right_cnt--;
//		}
//  }
//  else if (EXTI_GetITStatus(EXTI_Line15) != RESET)
//  {
//    EXTI_ClearITPendingBit(EXTI_Line15);
//		if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_7) && !GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_15))
//		{
//			auto_ctrl_val.higher_right_cnt++;
//		}
//		else
//		{
//			auto_ctrl_val.higher_right_cnt--;
//		}
//  }
