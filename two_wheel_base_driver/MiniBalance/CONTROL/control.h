#ifndef __CONTROL_H
#define __CONTROL_H
#include "sys.h"
  /**************************************************************************
作者：平衡小车之家
我的淘宝小店：http://shop114407458.taobao.com/
**************************************************************************/
#define PI 3.14159265
#define ZHONGZHI 0 
#define DIFFERENCE 100

#define USE_ZHP_FRAME 1

#pragma pack(1)
typedef struct
{
	int16_t encoder_left;
  int8_t direction_left;
	int16_t encoder_right;
	int8_t direction_right;
	double yaw;
	double roll;
	double pitch;
	uint16_t yaw_accel;
	uint16_t roll_accel;
	uint16_t pitch_accel;
	uint16_t yaw_angular;
	uint16_t roll_angular;
	uint16_t pitch_angular;
	uint16_t voltage;
} upload_data_type;
#pragma pack()

#pragma pack(1)
typedef struct
{
	uint16_t left_target_pulse;
	int8_t left_target_dir;
	uint16_t right_target_pulse;
	int8_t right_target_dir;
} control_data_type;
#pragma pack()


#if USE_ZHP_FRAME
extern upload_data_type ud;
extern control_data_type* cd;
#endif

extern long overtime_ticks;
extern long overtime_tick_cnt;
extern	int Balance_Pwm,Velocity_Pwm,Turn_Pwm;
void Kinematic_Analysis(float velocity,float turn);
int EXTI15_10_IRQHandler(void);
void Set_Pwm(int motor_a,int motor_b,int servo);
void Key(void);
void Xianfu_Pwm(int amplitude);
void Xianfu_Velocity(int amplitude_A,int amplitude_B);
u8 Turn_Off( int voltage);
int myabs(int a);
int Incremental_PI_Left (int Encoder,int Target);
int Incremental_PI_Right (int Encoder,int Target);
void Get_RC(void);
void  Find_CCD_Zhongzhi(void);
#endif
