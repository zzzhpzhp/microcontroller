#include "Motors.h"


/*
DIRECTION C8 C9 C10 C11 C12 C13 C14 C15
*/
void MOTORS_Init()
{
  MotoTimerInit();

  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC| RCC_APB2Periph_AFIO, ENABLE);                 //使能端口时钟

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_10 | GPIO_Pin_9 | GPIO_Pin_8;          //端口配置
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;                                            //推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;                                           //IO口速度为50MHz
  GPIO_Init(GPIOC, &GPIO_InitStructure);                                                      //根据设定参数初始化

  GPIO_ResetBits(GPIOC, GPIO_Pin_11 | GPIO_Pin_10 | GPIO_Pin_9 | GPIO_Pin_8);
}
/////////////////////////////////////////////////////////
void MOTOR_A0_SetForward(void)
{
  GPIO_SetBits(GPIOC, GPIO_Pin_8);
}

void MOTOR_A1_SetForward(void)
{
  GPIO_SetBits(GPIOC, GPIO_Pin_9);
}

void MOTOR_A0_SetBackward(void)
{
  GPIO_ResetBits(GPIOC, GPIO_Pin_8);
}
void MOTOR_A1_SetBackward(void)
{
  GPIO_ResetBits(GPIOC, GPIO_Pin_9);
}

void MOTOR_B0_SetForward(void)
{
  GPIO_ResetBits(GPIOC, GPIO_Pin_10);
}

void MOTOR_B1_SetForward(void)
{
  GPIO_ResetBits(GPIOC, GPIO_Pin_11);
}

void MOTOR_B0_SetBackward(void)
{
  GPIO_SetBits(GPIOC, GPIO_Pin_10);
}

void MOTOR_B1_SetBackward(void)
{
  GPIO_SetBits(GPIOC, GPIO_Pin_11);
}
////////////////////////////////////////////////////////
void MOTOR_A0_SetBreak(unsigned int pwm)
{
  if(pwm>MotoPwmMax)    pwm = MotoPwmMax;
      else if(pwm <= MotoPwmMin)    pwm = MotoPwmMin;

  MOTOR_A0_BREAK_COUNTER = pwm;
}
void MOTOR_A1_SetBreak(unsigned int pwm)
{
  if(pwm>MotoPwmMax)    pwm = MotoPwmMax;
      else if(pwm <= MotoPwmMin)    pwm = MotoPwmMin;

  MOTOR_A1_BREAK_COUNTER = pwm;
}

void MOTOR_B0_SetBreak(unsigned int pwm)
{
  if(pwm>MotoPwmMax)    pwm = MotoPwmMax;
      else if(pwm <= MotoPwmMin)    pwm = MotoPwmMin;

  MOTOR_B0_BREAK_COUNTER = pwm;
}
void MOTOR_B1_SetBreak(unsigned int pwm)
{
  if(pwm>MotoPwmMax)    pwm = MotoPwmMax;
      else if(pwm <= MotoPwmMin)    pwm = MotoPwmMin;

  MOTOR_B1_BREAK_COUNTER = pwm;
}

//////////////////////////////////////////////////////////
void MOTOR_A0_SetPwm(unsigned int pwm)
{
  if(pwm>MotoPwmMax)    pwm = MotoPwmMax;
      else if(pwm <= MotoPwmMin)    pwm = MotoPwmMin;

  MOTOR_A0_TIMER_COUNTER = pwm;
}

void MOTOR_A1_SetPwm(unsigned int pwm)
{
  if(pwm>MotoPwmMax)    pwm = MotoPwmMax;
      else if(pwm <= MotoPwmMin)    pwm = MotoPwmMin;

  MOTOR_A1_TIMER_COUNTER = pwm;
}

void MOTOR_B0_SetPwm(unsigned int pwm)
{
  if(pwm > MotoPwmMax)    pwm = MotoPwmMax;
      else if(pwm <= MotoPwmMin)    pwm = MotoPwmMin;

  MOTOR_B0_TIMER_COUNTER = pwm;
}

void MOTOR_B1_SetPwm(unsigned int pwm)
{
  if(pwm>MotoPwmMax)    pwm = MotoPwmMax;
      else if(pwm <= MotoPwmMin)    pwm = MotoPwmMin;

  MOTOR_B1_TIMER_COUNTER = pwm;
}
/////////////////////////////////////////////////////////////
void moto_control_with_val()
{
	if (auto_ctrl_val.l0_pwm >= 0)
	{
		if ((auto_ctrl_val.spd_pid_l0.cur >= 0 && auto_ctrl_val.spd_pid_l0.target >=0) ||
			  (auto_ctrl_val.spd_pid_l0.cur < 0 && auto_ctrl_val.spd_pid_l0.target < 0) ||
		    (auto_ctrl_val.l0_speed < auto_ctrl_val.min_speed && auto_ctrl_val.l0_speed > -auto_ctrl_val.min_speed))
		{
			
			auto_ctrl_val.l0_dir_state = TRUE;
			MOTOR_A0_SetForward();
			MOTOR_A0_SetBreak(0);
			MOTOR_A0_SetPwm((uint16_t)fabs(auto_ctrl_val.l0_pwm));
		}
		else
		{
			MOTOR_A0_SetBreak(MotoPwmMax);
		}
	}
	else
	{
		if ((auto_ctrl_val.spd_pid_l0.cur >= 0 && auto_ctrl_val.spd_pid_l0.target >=0) ||
			  (auto_ctrl_val.spd_pid_l0.cur < 0 && auto_ctrl_val.spd_pid_l0.target < 0) ||
		    (auto_ctrl_val.l0_speed < auto_ctrl_val.min_speed && auto_ctrl_val.l0_speed > -auto_ctrl_val.min_speed))
		{
		  auto_ctrl_val.l0_dir_state = FALSE;
			MOTOR_A0_SetBackward();
			MOTOR_A0_SetBreak(0);
			MOTOR_A0_SetPwm((uint16_t)fabs(auto_ctrl_val.l0_pwm));
		}
		else
		{
			MOTOR_A0_SetBreak(MotoPwmMax);
		}
	}
	
	if (auto_ctrl_val.l1_pwm >= 0)
	{
		if ((auto_ctrl_val.spd_pid_l1.cur >= 0 && auto_ctrl_val.spd_pid_l1.target >=0) ||
			  (auto_ctrl_val.spd_pid_l1.cur < 0 && auto_ctrl_val.spd_pid_l1.target < 0) ||
		    (auto_ctrl_val.l1_speed < auto_ctrl_val.min_speed && auto_ctrl_val.l1_speed > -auto_ctrl_val.min_speed))
		{
			MOTOR_A1_SetBreak(0);
			auto_ctrl_val.l1_dir_state = TRUE;
			MOTOR_A1_SetForward();
			MOTOR_A1_SetPwm((uint16_t)fabs(auto_ctrl_val.l1_pwm));
		}
		else
		{
			MOTOR_A1_SetBreak(MotoPwmMax);
		}
	}
	else
	{
		if ((auto_ctrl_val.spd_pid_l1.cur >= 0 && auto_ctrl_val.spd_pid_l0.target >=0) ||
			  (auto_ctrl_val.spd_pid_l1.cur < 0 && auto_ctrl_val.spd_pid_l0.target < 0) ||
		    (auto_ctrl_val.l1_speed < auto_ctrl_val.min_speed && auto_ctrl_val.l1_speed > -auto_ctrl_val.min_speed))
		{
		  auto_ctrl_val.l1_dir_state = FALSE;
			MOTOR_A1_SetBackward();
			MOTOR_A1_SetBreak(0);
			MOTOR_A1_SetPwm((uint16_t)fabs(auto_ctrl_val.l1_pwm));
		}
		else
		{
			MOTOR_A1_SetBreak(MotoPwmMax);
		}
	}
	
	
	if (auto_ctrl_val.r0_pwm >= 0)
	{
		if ((auto_ctrl_val.spd_pid_r0.cur >= 0 && auto_ctrl_val.spd_pid_r0.target >=0) ||
			  (auto_ctrl_val.spd_pid_r0.cur < 0 && auto_ctrl_val.spd_pid_r0.target < 0) ||
		    (auto_ctrl_val.r0_speed < auto_ctrl_val.min_speed && auto_ctrl_val.r0_speed > -auto_ctrl_val.min_speed))
		{
			auto_ctrl_val.r0_dir_state = TRUE;
			MOTOR_B0_SetForward();
			MOTOR_B0_SetBreak(0);
			MOTOR_B0_SetPwm((uint16_t)fabs(auto_ctrl_val.r0_pwm));
		}
		else
		{
			MOTOR_B0_SetBreak(MotoPwmMax);
		}
	}
	else
	{
		if ((auto_ctrl_val.spd_pid_r0.cur >= 0 && auto_ctrl_val.spd_pid_r0.target >=0) ||
			  (auto_ctrl_val.spd_pid_r0.cur < 0 && auto_ctrl_val.spd_pid_r0.target < 0) ||
		    (auto_ctrl_val.r0_speed < auto_ctrl_val.min_speed && auto_ctrl_val.r0_speed > -auto_ctrl_val.min_speed))
		{
		  auto_ctrl_val.r0_dir_state = FALSE;
			MOTOR_B0_SetBackward();
		  MOTOR_B0_SetBreak(0);
			MOTOR_B0_SetPwm((uint16_t)fabs(auto_ctrl_val.r0_pwm));
		}
		else
		{
			MOTOR_B0_SetBreak(MotoPwmMax);
		}
	}
	
	if (auto_ctrl_val.r1_pwm >= 0)
	{
		if ((auto_ctrl_val.spd_pid_r1.cur >= 0 && auto_ctrl_val.spd_pid_r1.target >=0) ||
			  (auto_ctrl_val.spd_pid_r1.cur < 0 && auto_ctrl_val.spd_pid_r1.target < 0) ||
		    (auto_ctrl_val.r1_speed < auto_ctrl_val.min_speed && auto_ctrl_val.r1_speed > -auto_ctrl_val.min_speed))
		{
			auto_ctrl_val.r1_dir_state = TRUE;
			MOTOR_B1_SetForward();
			MOTOR_B1_SetBreak(0);
			MOTOR_B1_SetPwm((uint16_t)fabs(auto_ctrl_val.r1_pwm));
		}
		else
		{
			MOTOR_B1_SetBreak(MotoPwmMax);
		}
	}
	else
	{
		if ((auto_ctrl_val.spd_pid_r1.cur >= 0 && auto_ctrl_val.spd_pid_r1.target >=0) ||
			  (auto_ctrl_val.spd_pid_r1.cur < 0 && auto_ctrl_val.spd_pid_r1.target < 0) ||
		    (auto_ctrl_val.r1_speed < auto_ctrl_val.min_speed && auto_ctrl_val.r1_speed > -auto_ctrl_val.min_speed))
		{
		  auto_ctrl_val.r1_dir_state = FALSE;
			MOTOR_B1_SetBackward();
			MOTOR_B1_SetBreak(0);
			MOTOR_B1_SetPwm((uint16_t)fabs(auto_ctrl_val.r1_pwm));
		}
		else
		{
			MOTOR_B1_SetBreak(MotoPwmMax);
		}
	}
	
	
//		MOTOR_A0_SetBreak(4500);
//		MOTOR_A1_SetBreak(4500);
//		MOTOR_B0_SetBreak(4500);
//		MOTOR_B1_SetBreak(4500);
}
