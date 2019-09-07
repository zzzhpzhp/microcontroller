#ifndef __ZHP_CONTROL_APP_H__
#define	__ZHP_CONTROL_APP_H__

#include "stm32f10x.h"
#include "LED.h"
#include "UARTs.h"
#include "UART2.h"
#include "IOI2C.h"
#include "delay.h"
#include "MPU6050.h"
#include "HMC5883L.h"
#include "IMU.h"
#include "eeprom.h"
#include "MS5611.h"
#include "stdlib.h"
#include "DMA.h"

#include "zhp_app.h"
#include "cpal_i2c.h"
#include "cpal_conf.h"
#include "timer.h"

#include "stdint.h"
#include "zhp_frame.h"
#include "pstwo.h"
#include "Motors.h"
#include "timer.h"


//定义电机PWM分辨率
#define MIDPOINT (1500)
#define ERR_TOLERANCE (50)
#define PWM_RESOLUTION ((TIMER_CYCLE) / (MIDPOINT - ERR_TOLERANCE * 2.0 - 1000.0)) //乘以2是因为在中点和极限点都需要容忍误差
#define VERIFYED_MIDPOINT_H (MIDPOINT + ERR_TOLERANCE)
#define VERIFYED_MIDPOINT_L (MIDPOINT - ERR_TOLERANCE)

#define CTRL_DISABLE 0         //两段开关
#define CTRL_ENABLE  1
#define CTRL_OFF     2

#define CTRL_VAL_LIMIT_M 1500
#define CTRL_VAL_LIMIT_H 1950  //摇杆值限制
#define CTRL_VAL_LIMIT_L 1050
#define CTRL_TRI_SWI_JUDGE_H   (2000 - ERR_TOLERANCE * 5)  //三段开关
#define CTRL_TRI_SWI_JUDGE_L   (1000 + ERR_TOLERANCE * 5)
#define CTRL_TRI_SWI_H 0
#define CTRL_TRI_SWI_M 1
#define CTRL_TRI_SWI_L 2

#define TURN_SENSITIVE 1       //转弯敏感度，范围：0 - 2

typedef struct PID_Typedef
{
    float target;
    float pout;
    float iout;
    float dout;
    float out;
    float ek;
    float ek1;
    float ek2;
    float p;
    float i;
    float d;
} pid_t;

typedef struct ctrl_val_str 
{
  uint32_t ctrl_en_state;
  uint32_t ctrl_dat_state;

  uint32_t ctrl_ly;
  uint32_t ctrl_lx;
  uint32_t ctrl_ry;
  uint32_t ctrl_rx;

  uint32_t ctrl_comp_thr;
  uint32_t ctrl_comp_dir;
} ctrl_val_typedef;

extern float pwm_resolution;   
extern ctrl_val_typedef ctrl_val;
void zhp_remote_control(void);    
void zhp_computer_control(void);

#endif
