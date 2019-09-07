
/* UARTs.C file
STM32-SDK �������������
��д�ߣ�lisn3188
��ַ��www.chiplab7.com
����E-mail��lisn3188@163.com
���뻷����MDK-Lite  Version: 4.23
����ʱ��: 2012-02-28
���ԣ� ���������ڵ���ʵ���ҵ�STM32-SDK����ɲ���
���ܣ�ʵ��  STM32-SDK �������ϵ� UART1-RS232 �ӿڲ���

---------Ӳ���ϵ���������:----------
RS232�ӿڣ�
RS232TXD  -->  PA9  (UART1-TXD)
RS232RXD  -->  PA10 (UART1-RXD)
------------------------------------
 */

#include "UART2.h"
#include "UARTs.h"
#include "stdlib.h"
#include "mavlink_types.h"
#include "mavlink.h"
#include "DMA.h"

u8 TxBuffer[258];
u8 TxCounter=0;
u8 count=0; 

void UART_NVIC_Configuration(void)
{
        NVIC_InitTypeDef NVIC_InitStructure; 
          /* Enable the USART1 Interrupt */
        NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 7;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:    void Initial_UART1(u32 baudrate)
*��������:    ��ʼ��STM32-SDK�������ϵ�RS232�ӿ�
���������
    u32 baudrate   ����RS232���ڵĲ�����
���������û��  
*******************************************************************************/
void Initial_UART1(u32 bound)
{
  //GPIO�˿�����
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);  //ʹ��USART1��GPIOAʱ��
  USART_DeInit(USART1);  //��λ����1
  //USART1_TX   PA.9
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //�����������
  GPIO_Init(GPIOA, &GPIO_InitStructure); //��ʼ��PA9

  //USART1_RX    PA.10
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
  GPIO_Init(GPIOA, &GPIO_InitStructure);  //��ʼ��PA10

  //Usart1 NVIC ����

  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//��ռ���ȼ�3
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;    //�����ȼ�3
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;      //IRQͨ��ʹ��
  NVIC_Init(&NVIC_InitStructure);  //����ָ���Ĳ�����ʼ��VIC�Ĵ���

  //USART ��ʼ������

  USART_InitStructure.USART_BaudRate = bound;//һ������Ϊ9600;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
  USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
  USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;  //�շ�ģʽ

  USART_Init(USART1, &USART_InitStructure); //��ʼ������
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//�����ж�|USART_IT_TXE
  USART_Cmd(USART1, ENABLE);      
  USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE); //ʹ�ܴ���1��DMA����
}
/**************************ʵ�ֺ���********************************************
*����ԭ��:    void UART1_Put_Char(unsigned char DataToSend)
*��������:    RS232����һ���ֽ�
���������
    unsigned char DataToSend   Ҫ���͵��ֽ�����
���������û��  
*******************************************************************************/
void UART1_Put_Char(unsigned char DataToSend)
{
  //��Ҫ���͵��ֽ�д��UART1�ķ��ͻ�����
  //USART_SendData(USART1, (unsigned char) DataToSend);
  //�ȴ��������
    //while (!(USART1->SR & USART_FLAG_TXE));

  TxBuffer[count++] = DataToSend;  
    USART_ITConfig(USART1, USART_IT_TXE, ENABLE);  
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:    u8 UART1_Get_Char(void)
*��������:    RS232����һ���ֽ�  һֱ�ȴ���ֱ��UART1���յ�һ���ֽڵ����ݡ�
���������     û��
���������       UART1���յ�������  
*******************************************************************************/
u8 UART1_Get_Char(void)
{
  while (!(USART1->SR & USART_FLAG_RXNE));
  return(USART_ReceiveData(USART1));
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:    void UART1_Put_String(unsigned char *Str)
*��������:    RS232�����ַ���
���������
    unsigned char *Str   Ҫ���͵��ַ���
���������û��  
*******************************************************************************/
void UART1_Put_String(unsigned char *Str)
{
  //�ж�Strָ��������Ƿ���Ч.
  while(*Str){
  //�Ƿ��ǻس��ַ� �����,������Ӧ�Ļس� 0x0d 0x0a
  if(*Str=='\r')UART1_Put_Char(0x0d);
    else if(*Str=='\n')UART1_Put_Char(0x0a);
      else UART1_Put_Char(*Str);
  //�ȴ��������.
    //while (!(USART1->SR & USART_FLAG_TXE));
  //ָ��++ ָ����һ���ֽ�.
  Str++;
  }
/*
  //�ж�Strָ��������Ƿ���Ч.
  while(*Str){
  //�Ƿ��ǻس��ַ� �����,������Ӧ�Ļس� 0x0d 0x0a
  if(*Str=='\r')USART_SendData(USART1, 0x0d);
    else if(*Str=='\n')USART_SendData(USART1, 0x0a);
      else USART_SendData(USART1, *Str);
  //�ȴ��������.
    while (!(USART1->SR & USART_FLAG_TXE));
  //ָ��++ ָ����һ���ֽ�.
  Str++;
  }     */
}


