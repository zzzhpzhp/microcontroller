#ifndef __TIMER_H
#define __TIMER_H
#include <sys.h>	 
  /**************************************************************************
���ߣ�ƽ��С��֮��
�ҵ��Ա�С�꣺http://shop114407458.taobao.com/
**************************************************************************/
#define TRIPA PAout(11)//A·������ PA11�Ӵ������� PA0�ӻز�����
#define TRIPB PAout(4)//B·������ PA4�Ӵ������� PA1�ӻز�����
#define TRIPC PCout(5)//C·������ PC5�Ӵ������� PA2�ӻز�����
#define TRIPD PBout(12)//D·������ PB12�Ӵ������� PA3�ӻز�����

/*-------------������������ʹ��-----------
��������ʼ��֮����control.c���жϷ������������ֱ��ͨ��Read_Distane������ȡ
�������еı�������
Distance_A,Distance_B,Distance_C,Distance_D;//��������ر��� 
-----------������������ʹ��-----------*/

/*-------------��ģң�س���ʹ�úͽ���-----------
TIM3�� 4��ͨ�����ԽӺ�ģң��
PA6 PA7 PB0 PB1��Ӧ4��ͨ��������Ķ�Ӧ��ϵȡ���ڿ��ƴ���
��������ǲɼ���������Ҫ��ʹ�����ָ����Ա���趨
��ʼ����ģң�أ�����ͨ���ж϶�ȡ��ģң�ص������������еı�������
Remoter_Ch1,Remoter_Ch2,Remoter_Ch3,Remoter_Ch4;//��ģң�زɼ���ر���
-----------��ģң�ؽ���-----------*/
void TIM5_Cap_Init(u16 arr,u16 psc)	;  
void Read_Distane(void);
void TIM5_IRQHandler(void);
void TIM3_Cap_Init(u16 arr, u16 psc);
void TIM3_IRQHandler(void);
#endif
