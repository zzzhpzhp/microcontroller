#ifndef __USRATX_H
#define __USRATX_H 
#include "sys.h"	    	
#include "zhp_frame.h"
#include "control.h"
  /**************************************************************************
���ߣ�ƽ��С��֮��
�ҵ��Ա�С�꣺http://shop114407458.taobao.com/
**************************************************************************/
void uart5_send(u8 data);
void uart5_init(u32 bound);
void UART5_IRQHandler(void);
void UART_TX(void);
#endif

