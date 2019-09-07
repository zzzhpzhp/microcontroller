#ifndef _TIMER_H
#define _TIMER_H

#include "stm32f10x.h"


typedef struct RC_Typedef
{   //�����ܸߵ�ƽ��ʱ��
    u16 Tim2Ch1CapVal;
    u16 Tim2Ch2CapVal;
    u16 Tim2Ch3CapVal;
    u16 Tim2Ch4CapVal;
    u16 Tim3Ch1CapVal;
    u16 Tim3Ch2CapVal;
    u16 Tim3Ch3CapVal;
    u16 Tim3Ch4CapVal;
    u16 Tim4Ch1CapVal;
    u16 Tim4Ch2CapVal;
    u16 Tim4Ch3CapVal;
    u16 Tim4Ch4CapVal;
    u16 Tim5Ch1CapVal;
    u16 Tim5Ch2CapVal;
    u16 Tim5Ch3CapVal;
    u16 Tim5Ch4CapVal;
} rc_st;

extern rc_st rc;

#define MotoPwmMax 2999
#define MotoPwmMin 0

#define TIMER_CYCLE (3000 - 1)

#define CH1 (rc.Tim2Ch1CapVal)
#define CH2 (rc.Tim2Ch2CapVal)
#define CH3 (rc.Tim2Ch3CapVal)
#define CH4 (rc.Tim2Ch4CapVal)
#define CH5 (rc.Tim3Ch1CapVal)
#define CH6 (rc.Tim3Ch2CapVal)
#define CH7 (rc.Tim3Ch3CapVal)
#define CH8 (rc.Tim3Ch4CapVal)

#define RC_ROL CH1
#define RC_THR CH2
#define RC_PIT CH3
#define RC_YAW CH4
#define RC_SWITCH CH5

#define SWITCH_THRESHOLD 1800

#define RC_H 2000.0
#define RC_L 1000.0
#define RC_CONTROL_RANGE 500.0
#define RC_MID 1500.0
#define RC_DEAD_BAND 20.0
#define RC_MID_H (RC_MID + RC_DEAD_BAND)
#define RC_MID_L (RC_MID - RC_DEAD_BAND)

void MotoPwmRefresh(u16 Motor1Pwm,u16 Motor2Pwm,u16 Motor3Pwm,u16 Motor4Pwm,u16 Motor5Pwm,u16 Motor6Pwm);
void MotoTimerInit(void);
void RemoteControllerCaptureInit(void);//ң�����źŲ���ʱ����ʼ��

void TIM2_Cap_Init(u16 arr, u16 psc); //��STM32F103CBT6��Ƭ����ʹ����������ܹ���ȷ��ʼ��TIM2,���������ͺŵ�Ƭ������Ҫ�鿴DATASHEET�����ܻ���TIM5��ͻ
void TIM3_Cap_Init(u16 arr, u16 psc);
void TIM4_Cap_Init(u16 arr, u16 psc); //��STM32F103ZET6��Ƭ����ʹ����������ܹ���ȷ��ʼ��TIM5,���������ͺŵ�Ƭ������Ҫ�鿴DATASHEET�����ܻ���TIM2��ͻ
void TIM5_Cap_Init(u16 arr, u16 psc);

#endif

