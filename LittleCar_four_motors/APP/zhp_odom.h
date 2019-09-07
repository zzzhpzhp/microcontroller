#ifndef __ZHP_EXT_INT_H
#define	__ZHP_EXT_INT_H
#include "stm32f10x.h"
#include "ucos_ii.h"
#include "os_cpu.h"
#include "os_cfg.h"

#include "zhp_control_app.h"



#define MOTOR_INT_CLOCK_RCC (RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO)
#define MOTOR_A_GIPO_GROUP GPIOB
#define MOTOR_B_GIPO_GROUP GPIOB

#define MOTOR_A0_INT_IO GPIO_Pin_12
#define MOTOR_A1_INT_IO GPIO_Pin_13
#define MOTOR_B0_INT_IO GPIO_Pin_14
#define MOTOR_B1_INT_IO GPIO_Pin_15

#define MOTOR_A0_INT_PINSOURCE GPIO_PinSource12
#define MOTOR_A1_INT_PINSOURCE GPIO_PinSource13
#define MOTOR_B0_INT_PINSOURCE GPIO_PinSource14
#define MOTOR_B2_INT_PINSOURCE GPIO_PinSource15


extern float r_speed;
extern float l_speed;

void zhp_ext_int_init(void);
void phase_detector_pin_init(void);
void calc_speed(float cycle);
  
#endif
