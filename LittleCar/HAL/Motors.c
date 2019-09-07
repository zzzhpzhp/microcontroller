#include "Motors.h"

two_motors_pwm_typedef car_pwm;

void MOTORS_Init()
{
  car_pwm.motor_a_pwm = 0;
  car_pwm.motor_b_pwm = 0;

  MotoTimerInit();

  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_APB2PeriphClockCmd(MOTORS_CLOCK_RCC | RCC_APB2Periph_AFIO, ENABLE);                     //ʹ�ܶ˿�ʱ��

  GPIO_InitStructure.GPIO_Pin = MOTOR_A_IN1 | MOTOR_A_IN2 | MOTOR_B_IN1 | MOTOR_B_IN2;        //�˿�����
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;                                            //�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;                                           //IO���ٶ�Ϊ50MHz
  GPIO_Init(MOTORS_GIPO_GROUP, &GPIO_InitStructure);                                          //�����趨������ʼ��

  GPIO_ResetBits(MOTORS_GIPO_GROUP, MOTOR_A_IN1 | MOTOR_A_IN2 | MOTOR_B_IN1 | MOTOR_B_IN2);
}

void MOTOR_A_SetForward(void)
{
  GPIO_SetBits(MOTORS_GIPO_GROUP, MOTOR_A_IN1);
  GPIO_ResetBits(MOTORS_GIPO_GROUP, MOTOR_A_IN2);
}

void MOTOR_A_SetBackward(void)
{
  GPIO_SetBits(MOTORS_GIPO_GROUP, MOTOR_A_IN2);
  GPIO_ResetBits(MOTORS_GIPO_GROUP, MOTOR_A_IN1);
}

void MOTOR_A_Brake(void)
{
  GPIO_SetBits(MOTORS_GIPO_GROUP, MOTOR_A_IN2 | MOTOR_A_IN1);
}

void MOTOR_B_SetForward(void)
{
  GPIO_SetBits(MOTORS_GIPO_GROUP, MOTOR_B_IN2);
  GPIO_ResetBits(MOTORS_GIPO_GROUP, MOTOR_B_IN1);
}

void MOTOR_B_SetBackward(void)
{
  GPIO_SetBits(MOTORS_GIPO_GROUP, MOTOR_B_IN1);
  GPIO_ResetBits(MOTORS_GIPO_GROUP, MOTOR_B_IN2);
}

void MOTOR_B_Brake(void)
{
  GPIO_SetBits(MOTORS_GIPO_GROUP, MOTOR_B_IN1 | MOTOR_B_IN2);
}

//CONNECTED PWM1 FOR KCOPTER BOARD
void MOTOR_A_SetPwm(unsigned int pwm)
{
  if(pwm>MotoPwmMax)    pwm = MotoPwmMax;
      else if(pwm <= MotoPwmMin)    pwm = MotoPwmMin;

  MOTOR_A_TIMER_COUNTER = pwm;
}

//CONNECTED PWM2 FOR KCOPTER BOARD
void MOTOR_B_SetPwm(unsigned int pwm)
{
  if(pwm>MotoPwmMax)    pwm = MotoPwmMax;
      else if(pwm <= MotoPwmMin)    pwm = MotoPwmMin;

  MOTOR_B_TIMER_COUNTER = pwm;
}
