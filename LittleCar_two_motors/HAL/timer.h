#ifndef _TIMER_H
#define _TIMER_H

#include "stm32f10x.h"


typedef struct RC_Typedef
{   //捕获总高电平的时间
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
void RemoteControllerCaptureInit(void);//遥控器信号捕获定时器初始化

void TIM2_Cap_Init(u16 arr, u16 psc); //在STM32F103CBT6单片机上使用这个函数能够正确初始化TIM2,若在其他型号单片机上需要查看DATASHEET，可能会与TIM5冲突
void TIM3_Cap_Init(u16 arr, u16 psc);
void TIM4_Cap_Init(u16 arr, u16 psc); //在STM32F103ZET6单片机上使用这个函数能够正确初始化TIM5,若在其他型号单片机上需要查看DATASHEET，可能会与TIM2冲突
void TIM5_Cap_Init(u16 arr, u16 psc);

#endif

