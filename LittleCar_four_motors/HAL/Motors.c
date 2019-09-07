#include "Motors.h"


/*
DIRECTION C8 C9 C10 Chigher_left C12 C13 C14 C15
*/
void MOTORS_Init()
{
  MotoTimerInit();

  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC| RCC_APB2Periph_AFIO, ENABLE);                 //使能端口时钟

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15 | GPIO_Pin_14 | GPIO_Pin_13 | GPIO_Pin_12 | GPIO_Pin_11 | GPIO_Pin_10 | GPIO_Pin_9 | GPIO_Pin_8;          //端口配置
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;                                            //推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;                                           //IO口速度为50MHz
  GPIO_Init(GPIOC, &GPIO_InitStructure);                                                      //根据设定参数初始化

  GPIO_ResetBits(GPIOC, GPIO_Pin_15 | GPIO_Pin_14 | GPIO_Pin_13 | GPIO_Pin_12 | GPIO_Pin_11 | GPIO_Pin_10 | GPIO_Pin_9 | GPIO_Pin_8);
	
}
/////////////////////////////////////////////////////////
void higher_left_forward(void)
{
  GPIO_SetBits(GPIOC, GPIO_Pin_14);
  GPIO_ResetBits(GPIOC, GPIO_Pin_15);
}

void higher_left_backward(void)
{
  GPIO_SetBits(GPIOC, GPIO_Pin_15);
  GPIO_ResetBits(GPIOC, GPIO_Pin_14);
}


void lower_left_forward(void)
{
  GPIO_SetBits(GPIOC, GPIO_Pin_12);
  GPIO_ResetBits(GPIOC, GPIO_Pin_13);
}

void lower_left_backward(void)
{
  GPIO_SetBits(GPIOC, GPIO_Pin_13);
  GPIO_ResetBits(GPIOC, GPIO_Pin_12);
}


void higher_right_forward(void)
{
  GPIO_SetBits(GPIOC, GPIO_Pin_10);
  GPIO_ResetBits(GPIOC, GPIO_Pin_11);
}

void higher_right_backward(void)
{
  GPIO_SetBits(GPIOC, GPIO_Pin_11);
  GPIO_ResetBits(GPIOC, GPIO_Pin_10);
}


void lower_right_forward(void)
{
  GPIO_SetBits(GPIOC, GPIO_Pin_8);
  GPIO_ResetBits(GPIOC, GPIO_Pin_9);
}

void lower_right_backward(void)
{
  GPIO_SetBits(GPIOC, GPIO_Pin_9);
  GPIO_ResetBits(GPIOC, GPIO_Pin_8);
}
////////////////////////////////////////////////////////
void higher_left_break()
{
  GPIO_ResetBits(GPIOC, GPIO_Pin_14);
  GPIO_ResetBits(GPIOC, GPIO_Pin_15);
}

void higher_left_realease()
{
  GPIO_SetBits(GPIOC, GPIO_Pin_14);
  GPIO_SetBits(GPIOC, GPIO_Pin_15);
}

void lower_left_break()
{
  GPIO_ResetBits(GPIOC, GPIO_Pin_12);
  GPIO_ResetBits(GPIOC, GPIO_Pin_13);
}

void lower_left_realease()
{
  GPIO_SetBits(GPIOC, GPIO_Pin_12);
  GPIO_SetBits(GPIOC, GPIO_Pin_13);
}

void higher_right_break()
{
  GPIO_ResetBits(GPIOC, GPIO_Pin_10);
  GPIO_ResetBits(GPIOC, GPIO_Pin_11);
}

void higher_right_realease()
{
  GPIO_SetBits(GPIOC, GPIO_Pin_10);
  GPIO_SetBits(GPIOC, GPIO_Pin_11);
}

void lower_right_break()
{
  GPIO_ResetBits(GPIOC, GPIO_Pin_8);
  GPIO_ResetBits(GPIOC, GPIO_Pin_9);
}

void lower_right_realease()
{
  GPIO_SetBits(GPIOC, GPIO_Pin_8);
  GPIO_SetBits(GPIOC, GPIO_Pin_9);
}

//////////////////////////////////////////////////////////
void higher_left_set_pwm(unsigned int pwm)
{
  if(pwm>MotoPwmMax)    pwm = MotoPwmMax;
      else if(pwm <= MotoPwmMin)    pwm = MotoPwmMin;

  HIGHER_LEFT_COUNTER = pwm;
}

void lower_left_set_pwm(unsigned int pwm)
{
  if(pwm>MotoPwmMax)    pwm = MotoPwmMax;
      else if(pwm <= MotoPwmMin)    pwm = MotoPwmMin;

  LOWER_LEFT_COUNTER = pwm;
}

void higher_right_set_pwm(unsigned int pwm)
{
  if(pwm > MotoPwmMax)    pwm = MotoPwmMax;
      else if(pwm <= MotoPwmMin)    pwm = MotoPwmMin;

  HIGHER_RIGHT_COUNTER = pwm;
}

void lower_right_set_pwm(unsigned int pwm)
{
  if(pwm>MotoPwmMax)    pwm = MotoPwmMax;
      else if(pwm <= MotoPwmMin)    pwm = MotoPwmMin;

  LOWER_RIGHT_COUNTER = pwm;
}
/////////////////////////////////////////////////////////////
void moto_control_with_val()
{
	if (auto_ctrl_val.lower_left_pwm >= 0)
	{
		auto_ctrl_val.lower_left_dir_state = TRUE;
		lower_left_forward();
		lower_left_set_pwm((uint16_t)fabs(auto_ctrl_val.lower_left_pwm));
	}
	else
	{
		auto_ctrl_val.lower_left_dir_state = FALSE;
		lower_left_backward();
		lower_left_set_pwm((uint16_t)fabs(auto_ctrl_val.lower_left_pwm));
	}
	
	if (auto_ctrl_val.higher_left_pwm >= 0)
	{
		auto_ctrl_val.higher_left_dir_state = TRUE;
	  higher_left_forward();
		higher_left_set_pwm((uint16_t)fabs(auto_ctrl_val.higher_left_pwm));
	}
	else
	{
		auto_ctrl_val.higher_left_dir_state = FALSE;
		higher_left_backward();
		higher_left_set_pwm((uint16_t)fabs(auto_ctrl_val.higher_left_pwm));
	}
	
	
	if (auto_ctrl_val.lower_right_pwm >= 0)
	{
		auto_ctrl_val.lower_right_dir_state = TRUE;
		lower_right_forward();
		lower_right_set_pwm((uint16_t)fabs(auto_ctrl_val.lower_right_pwm));
	}
	else
	{
		auto_ctrl_val.lower_right_dir_state = FALSE;
		lower_right_backward();
		lower_right_set_pwm((uint16_t)fabs(auto_ctrl_val.lower_right_pwm));
	}
	
	if (auto_ctrl_val.higher_right_pwm >= 0)
	{
		auto_ctrl_val.higher_right_dir_state = TRUE;
		higher_right_forward();
		higher_right_set_pwm((uint16_t)fabs(auto_ctrl_val.higher_right_pwm));
	}
	else
	{
		auto_ctrl_val.higher_right_dir_state = FALSE;
		higher_right_backward();
		higher_right_set_pwm((uint16_t)fabs(auto_ctrl_val.higher_right_pwm));
	}
	
}
