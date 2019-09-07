#ifndef __Motors_H
#define __Motors_H

#include "stm32f10x.h"
#include "sys.h"
#include "timer.h"

#include "zhp_control_app.h"



#define MOTOR_A0_TIMER_COUNTER (TIM4->CCR1)
#define MOTOR_A1_TIMER_COUNTER (TIM4->CCR2)
#define MOTOR_B0_TIMER_COUNTER (TIM4->CCR3)
#define MOTOR_B1_TIMER_COUNTER (TIM4->CCR4)

#define MOTOR_A0_BREAK_COUNTER (TIM2->CCR1)
#define MOTOR_A1_BREAK_COUNTER (TIM2->CCR2)
#define MOTOR_B0_BREAK_COUNTER (TIM2->CCR3)
#define MOTOR_B1_BREAK_COUNTER (TIM2->CCR4)

typedef struct two_motors_pwm_str
{
  int16_t motor_a_pwm;
  int16_t motor_b_pwm;
} two_motors_pwm_typedef;

extern two_motors_pwm_typedef car_pwm;

void MOTORS_Init(void);

void MOTOR_A0_SetForward(void);
void MOTOR_A1_SetForward(void);
void MOTOR_A0_SetBackward(void);
void MOTOR_A1_SetBackward(void);
void MOTOR_B0_SetForward(void);
void MOTOR_B1_SetForward(void);
void MOTOR_B0_SetBackward(void);
void MOTOR_B1_SetBackward(void);

void MOTOR_A0_SetBreak(unsigned int pwm);
void MOTOR_A1_SetBreak(unsigned int pwm);
void MOTOR_B0_SetBreak(unsigned int pwm);
void MOTOR_B1_SetBreak(unsigned int pwm);

void MOTOR_A0_SetPwm(unsigned int pwm);
void MOTOR_A1_SetPwm(unsigned int pwm);
void MOTOR_B0_SetPwm(unsigned int pwm);
void MOTOR_B1_SetPwm(unsigned int pwm);

void moto_control_with_val(void);
#endif
