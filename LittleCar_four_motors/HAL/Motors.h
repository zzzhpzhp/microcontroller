#ifndef __Motors_H
#define __Motors_H

#include "stm32f10x.h"
#include "sys.h"
#include "timer.h"

#include "zhp_control_app.h"



#define HIGHER_LEFT_COUNTER    (TIM4->CCR4)
#define LOWER_LEFT_COUNTER     (TIM4->CCR3)
#define HIGHER_RIGHT_COUNTER   (TIM4->CCR2)
#define LOWER_RIGHT_COUNTER    (TIM4->CCR1)

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

void higher_left_forward(void);
void higher_left_backward(void);
void lower_left_forward(void);
void lower_left_backward(void);
void higher_right_forward(void);
void higher_right_backward(void);
void lower_right_forward(void);
void lower_right_backward(void);

void higher_left_break(void);
void higher_left_realease(void);
void lower_left_break(void);
void lower_left_realease(void);
void higher_right_break(void);
void higher_right_realease(void);
void lower_right_break(void);
void lower_right_realease(void);

void higher_left_set_pwm(unsigned int pwm);
void lower_left_set_pwm(unsigned int pwm);
void higher_right_set_pwm(unsigned int pwm);
void lower_right_set_pwm(unsigned int pwm);

void moto_control_with_val(void);
#endif
