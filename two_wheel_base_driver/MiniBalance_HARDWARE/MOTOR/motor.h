#ifndef __MOTOR_H
#define __MOTOR_H
#include <sys.h>	 
  /**************************************************************************
���ߣ�ƽ��С��֮��
�ҵ��Ա�С�꣺http://shop114407458.taobao.com/
**************************************************************************/
#define PWMA1   TIM8->CCR3  
#define PWMA2   TIM8->CCR1 
#define SERVO   TIM1->CCR1  //�������
#define PWMB1   TIM8->CCR4  
#define PWMB2   TIM8->CCR2
void MiniBalance_PWM_Init(u16 arr,u16 psc);
void Servo_PWM_Init(u16 arr,u16 psc);
#endif
