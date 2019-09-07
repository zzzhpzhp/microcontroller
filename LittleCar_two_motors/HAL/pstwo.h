#ifndef __PSTWO_H
#define __PSTWO_H
#include "delay.h"
#include "stm32f10x.h"
#include "sys.h"
/*********************************************************
Copyright (C), 2015-2025, YFRobot.
www.yfrobot.com
File��PS2��������
Author��pinggai    Version:1.0     Data:2015/05/16
Description: PS2��������
**********************************************************/	 
#define DI   PBin(12)           //PB12  ����

#define DO_H PBout(13)=1        //����λ��
#define DO_L PBout(13)=0        //����λ��

#define CS_H PBout(14)=1       //CS����
#define CS_L PBout(14)=0       //CS����

#define CLK_H PBout(15)=1      //ʱ������
#define CLK_L PBout(15)=0      //ʱ������


//These are our button constants
#define PSB_SELECT      1
#define PSB_L3          2
#define PSB_R3          3
#define PSB_START       4
#define PSB_PAD_UP      5
#define PSB_PAD_RIGHT   6
#define PSB_PAD_DOWN    7
#define PSB_PAD_LEFT    8
#define PSB_L2          9
#define PSB_R2          10
#define PSB_L1          11
#define PSB_R1          12
#define PSB_GREEN       13
#define PSB_RED         14
#define PSB_BLUE        15
#define PSB_PINK        16
#define PSB_TRIANGLE    13
#define PSB_CIRCLE      14
#define PSB_CROSS       15
#define PSB_SQUARE      16

//#define WHAMMY_BAR		8

//These are stick values
#define PSS_RX 5                //��ҡ��X������
#define PSS_RY 6
#define PSS_LX 7
#define PSS_LY 8

typedef struct ps2_val_str 
{
  uint32_t ps2_light_state;
	uint32_t ps2_key_val;

	uint32_t ps2_ly;
	uint32_t ps2_lx;
	uint32_t ps2_ry;
	uint32_t ps2_rx;
} ps2_val_typedef;

extern uint8_t Data[9];
extern uint8_t MASK[16];
extern uint8_t Handkey; 
extern ps2_val_typedef ps2_val;

void PS2_Init(void);
uint8_t PS2_RedLight(void);//�ж��Ƿ�Ϊ���ģʽ
void PS2_ReadData(void);
void PS2_Cmd(uint8_t CMD);		  //
uint8_t PS2_DataKey(void);		  //��ֵ��ȡ
uint8_t PS2_AnologData(uint8_t button); //�õ�һ��ҡ�˵�ģ����
void PS2_ClearData(void);	  //������ݻ�����

#endif





