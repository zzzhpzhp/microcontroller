
/* UARTs.C file
STM32-SDK 开发板相关例程
编写者：lisn3188
网址：www.chiplab7.com
作者E-mail：lisn3188@163.com
编译环境：MDK-Lite  Version: 4.23
初版时间: 2012-02-28
测试： 本程序已在第七实验室的STM32-SDK上完成测试
功能：实现  STM32-SDK 开发板上的 UART1-RS232 接口操作

---------硬件上的引脚连接:----------
RS232接口：
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

/**************************实现函数********************************************
*函数原型:    void Initial_UART1(u32 baudrate)
*功　　能:    初始化STM32-SDK开发板上的RS232接口
输入参数：
    u32 baudrate   设置RS232串口的波特率
输出参数：没有  
*******************************************************************************/
void Initial_UART1(u32 bound)
{
  //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);  //使能USART1，GPIOA时钟
  USART_DeInit(USART1);  //复位串口1
  //USART1_TX   PA.9
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
  GPIO_Init(GPIOA, &GPIO_InitStructure); //初始化PA9

  //USART1_RX    PA.10
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
  GPIO_Init(GPIOA, &GPIO_InitStructure);  //初始化PA10

  //Usart1 NVIC 配置

  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//抢占优先级3
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;    //子优先级3
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;      //IRQ通道使能
  NVIC_Init(&NVIC_InitStructure);  //根据指定的参数初始化VIC寄存器

  //USART 初始化设置

  USART_InitStructure.USART_BaudRate = bound;//一般设置为9600;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
  USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
  USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;  //收发模式

  USART_Init(USART1, &USART_InitStructure); //初始化串口
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启中断|USART_IT_TXE
  USART_Cmd(USART1, ENABLE);      
  USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE); //使能串口1的DMA发送
}
/**************************实现函数********************************************
*函数原型:    void UART1_Put_Char(unsigned char DataToSend)
*功　　能:    RS232发送一个字节
输入参数：
    unsigned char DataToSend   要发送的字节数据
输出参数：没有  
*******************************************************************************/
void UART1_Put_Char(unsigned char DataToSend)
{
  //将要发送的字节写到UART1的发送缓冲区
  //USART_SendData(USART1, (unsigned char) DataToSend);
  //等待发送完成
    //while (!(USART1->SR & USART_FLAG_TXE));

  TxBuffer[count++] = DataToSend;  
    USART_ITConfig(USART1, USART_IT_TXE, ENABLE);  
}

/**************************实现函数********************************************
*函数原型:    u8 UART1_Get_Char(void)
*功　　能:    RS232接收一个字节  一直等待，直到UART1接收到一个字节的数据。
输入参数：     没有
输出参数：       UART1接收到的数据  
*******************************************************************************/
u8 UART1_Get_Char(void)
{
  while (!(USART1->SR & USART_FLAG_RXNE));
  return(USART_ReceiveData(USART1));
}

/**************************实现函数********************************************
*函数原型:    void UART1_Put_String(unsigned char *Str)
*功　　能:    RS232发送字符串
输入参数：
    unsigned char *Str   要发送的字符串
输出参数：没有  
*******************************************************************************/
void UART1_Put_String(unsigned char *Str)
{
  //判断Str指向的数据是否有效.
  while(*Str){
  //是否是回车字符 如果是,则发送相应的回车 0x0d 0x0a
  if(*Str=='\r')UART1_Put_Char(0x0d);
    else if(*Str=='\n')UART1_Put_Char(0x0a);
      else UART1_Put_Char(*Str);
  //等待发送完成.
    //while (!(USART1->SR & USART_FLAG_TXE));
  //指针++ 指向下一个字节.
  Str++;
  }
/*
  //判断Str指向的数据是否有效.
  while(*Str){
  //是否是回车字符 如果是,则发送相应的回车 0x0d 0x0a
  if(*Str=='\r')USART_SendData(USART1, 0x0d);
    else if(*Str=='\n')USART_SendData(USART1, 0x0a);
      else USART_SendData(USART1, *Str);
  //等待发送完成.
    while (!(USART1->SR & USART_FLAG_TXE));
  //指针++ 指向下一个字节.
  Str++;
  }     */
}


