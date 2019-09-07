#ifndef __MAIN_H__
#define	__MAIN_H__

#include "stm32f10x.h" 
#include "ucos_ii.h"
#include "os_cpu.h"
#include "os_cfg.h"

/*
Kcopter

SWC 黄色
SWD 橙色

CH1 PA0 TIM2_CH1            右摇杆 左右方向   左 1000 右 2000          关机 1500   CH1 --------- A0 encoder
CH2 PA1 TIM2_CH2            左摇杆 上下方向   上 1000 右 2000          关机 1500   CH2 --------- A1 encoder
CH3 PA2 TIM2_CH3                                                                       --------- B0 encoder
CH4 PA3 TIM2_CH4                                                                       --------- B1 encoder
CH5 PA6 TIM3_CH1 (SDMISO)   右摇杆 上下方向   上 2000 下 1000          关机 1000   CH3
CH6 PA7 TIM3_CH2 (SDMOSI)   左摇杆 左右方向   左 2000 右 1000          关机 1500   CH4
CH7 PB0 TIM3_CH3            左边两段开关      上 1000 下 2000          关机 1500   CH5
CH8 PB1 TIM3_CH4            右边三段开关      上 1000 中 1500 下2000   关机 1500   CH6

PWM1 PA8  TIM1_CH1     A_IN
PWM2 PA11 TIM1_CH4     B_IN
PWM3 PB6  TIM4_CH1     PWM_A0
PWM4 PB7  TIM4_CH2     PWM_A1
PWM5 PB8  TIM4_CH3     PWM_B0
PWM6 PB9  TIM4_CH4     PWM_B1

PB10 SCL
PB11 SDA

PB12 NSS
PB13 SCK
PB14 MISO
PB15 MOSI

PA4 PADC

PB4 LED0
PB3 LED1

PA9  TX  橙色
PA10 RX  黄色

PA5  SDSCK
PA13 SWDIO
PA14 SWCLK

PA12 BEEP

PD0 OSD_IN
PD1 OSD_OUT

PC12 INT1
PC13 INT2
PC14 INT3
PC15 INT4

GPS
SDA 白色 PA11
SCL 橙色 PA10
RX 黄色 PA2  USART2
TX 绿色 PA3
VCC 红色
GND 黑色

*/

extern uint16_t recv_frame_cnt;
extern uint16_t recv_frame_freq;
extern OS_EVENT * feedback_sem;

extern float two_wheel_dist;

extern float cart_wheel_diameter;
extern float pulse_num_per_round;
extern float perimeter;

#endif
