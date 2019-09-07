#ifndef __MAIN_H__
#define	__MAIN_H__

#include "stm32f10x.h" 
#include "ucos_ii.h"
#include "os_cpu.h"
#include "os_cfg.h"

extern uint16_t recv_frame_cnt;
extern uint16_t recv_frame_freq;

extern OS_EVENT * feedback_sem;

#endif
