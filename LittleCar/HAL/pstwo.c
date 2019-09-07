#include "pstwo.h"
#include "usart.h"
/*********************************************************
Copyright (C), 2015-2025, YFRobot.
www.yfrobot.com
File��PS2��������
Author��pinggai    Version:1.0     Data:2015/05/16
Description: PS2��������
**********************************************************/   
uint8_t Comd[2]={0x01,0x42};  //��ʼ�����������
uint8_t Data[9]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; //���ݴ洢����
uint8_t MASK[]=
{
  PSB_SELECT,
  PSB_L3,
  PSB_R3,
  PSB_START,
  PSB_PAD_UP,
  PSB_PAD_RIGHT,
  PSB_PAD_DOWN,
  PSB_PAD_LEFT,
  PSB_L2,
  PSB_R2,
  PSB_L1,
  PSB_R1 ,
  PSB_GREEN,
  PSB_RED,
  PSB_BLUE,
  PSB_PINK
};  //����ֵ�밴����
        
ps2_val_typedef ps2_val;


//�ֱ��ӿڳ�ʼ��    ����  DI->PB12 
//                  ���  DO->PB13    CLC->PB14  CS->PB15
void PS2_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
}

#define DELAY_TIME 10

//���ֱ���������
void PS2_Cmd(uint8_t CMD)
{
  volatile uint16_t ref=0x01;
  Data[1] = 0;
  for(ref=0x01;ref<0x0100;ref<<=1)
  {
    if(ref&CMD)
    {
      DO_H;                   //�����Ϊ����λ
    }
    else DO_L;

    CLK_H;                        //ʱ������
    delay_us(DELAY_TIME);
    CLK_L;
    delay_us(DELAY_TIME);
    CLK_H;
    if(DI)
      Data[1] = ref|Data[1];
  }
}
//�ж��Ƿ�Ϊ���ģʽ
//����ֵ��TRUE�����ģʽ
//����������ģʽ
u8 PS2_RedLight(void)
{
  CS_L;
  PS2_Cmd(Comd[0]);  //��ʼ����
  PS2_Cmd(Comd[1]);  //��������
  CS_H;
  if( Data[1] == 0X73)
    return TRUE ;
  else
    return FALSE;

}
//��ȡ�ֱ�����
void PS2_ReadData(void)
{
  volatile uint8_t byte=0;
  volatile uint16_t ref=0x01;

  CS_L;

  PS2_Cmd(Comd[0]);  //��ʼ����
  PS2_Cmd(Comd[1]);  //��������

  for(byte=2;byte<9;byte++)          //��ʼ��������
  {
    for(ref=0x01;ref<0x100;ref<<=1)
    {
      CLK_H;
      CLK_L;
      delay_us(DELAY_TIME);
      CLK_H;
          if(DI)
          Data[byte] = ref|Data[byte];
    }
        delay_us(DELAY_TIME);
  }
  CS_H;  
}

//�Զ�������PS2�����ݽ��д���      ֻ�����˰�������         Ĭ�������Ǻ��ģʽ  ֻ��һ����������ʱ
//����Ϊ0�� δ����Ϊ1
uint8_t PS2_DataKey()
{
  uint8_t index;
  uint16_t Handkey;

  PS2_ClearData();
  PS2_ReadData();

  Handkey=(Data[4]<<8)|Data[3];     //����16������  ����Ϊ0�� δ����Ϊ1
  for(index=0;index<16;index++)
  {      
    if((Handkey&(1<<(MASK[index]-1)))==0)
    return index+1;
  }
  return 0;          //û���κΰ�������
}

//�õ�һ��ҡ�˵�ģ����   ��Χ0~256
uint8_t PS2_AnologData(uint8_t button)
{
  return Data[button];
}

//������ݻ�����
void PS2_ClearData()
{
  uint8_t a;
  for(a=0;a<9;a++)
    Data[a]=0x00;
}



