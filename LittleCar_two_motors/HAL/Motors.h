#ifndef __Motors_H
#define __Motors_H

#include "stm32f10x.h"
#include "sys.h"
#include "timer.h"

#define MOTORS_CLOCK_RCC RCC_APB2Periph_GPIOB
#define MOTORS_GIPO_GROUP GPIOB

#define MOTOR_A_IN1 GPIO_Pin_6 //NSS
#define MOTOR_A_IN2 GPIO_Pin_7 //SCK
#define MOTOR_B_IN1 GPIO_Pin_8 //MISO
#define MOTOR_B_IN2 GPIO_Pin_9 //MOSI

#define MOTOR_A_TIMER_COUNTER (TIM1->CCR1)
#define MOTOR_B_TIMER_COUNTER (TIM1->CCR4)

typedef struct two_motors_pwm_str
{
  int16_t motor_a_pwm;
  int16_t motor_b_pwm;
} two_motors_pwm_typedef;

extern two_motors_pwm_typedef car_pwm;

void MOTORS_Init(void);

void MOTOR_A_SetForward(void);
void MOTOR_A_SetBackward(void);
void MOTOR_B_SetForward(void);
void MOTOR_B_SetBackward(void);

void MOTOR_A_Brake(void);
void MOTOR_B_Brake(void);

void MOTOR_A_SetPwm(unsigned int pwm);
void MOTOR_B_SetPwm(unsigned int pwm);


#endif
