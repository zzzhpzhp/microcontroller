
#include "stm32f10x.h"
#include "LED.h"
#include "usart.h"
#include "usart2.h"
#include "IOI2C.h"
#include "delay.h"
#include "MPU6050.h"
#include "HMC5883L.h"
#include "IMU.h"
#include "eeprom.h"
#include "MS5611.h"
#include "stdlib.h"
#include "DMA.h"
                       
#include "zhp_frame.h"
#include "zhp_control_app.h"   
#include "zhp_app.h"
#include "zhp_ext_int.h"
#include "cpal_i2c.h"
#include "cpal_conf.h"
#include "timer.h"
#include "Motors.h"
#include "pstwo.h"
#include "wdg.h"


#define BUFFER_NUMBER 30
/*
I2C B10 B11
MOTOR DIRECTION C8 C9 C10 C11 C12 C13 C14 C15
MOTOR PWM 
EXIT (WHEEL SPEED) B12 B13 B14 B15
*/

long frame_cnt = 0;

void zhp_app_init(void)
{
  SystemInit();
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  delay_init();     //延时初始化
  LED_Init();       //初始化STM32-SDK板子上的LED接口
  delay_ms(30);     //等待器件上电
//  IMU_init();     //初始化IMU和传感器
  MOTORS_Init();
  PS2_Init();
  CRC_Config();
  RemoteControllerCaptureInit();
  IWDG_Init(4,625);    //与分频数为64,重载值为625,溢出时间为1s
  pwm_resolution = PWM_RESOLUTION;
  zhp_ext_int_init();
	zhp_control_init();

  uart_init(115200);        //串口初始化
  //USART2_Init(115200);      //初始化串口2

  printf("Initialize over!\r\n");
}

void zhp_get_imu_data(void)
{
  IMU_DataGetAndFilter();
  imu.Pressure_Diff = Get_Pressure_Diff(MS5611_OSR4096);
  IMUupdate(
               imu.Gyro_x_AfterFilter * Gyro_Gr , \
               imu.Gyro_y_AfterFilter * Gyro_Gr ,\
               imu.Gyro_z_AfterFilter * Gyro_Gr ,\
               imu.Accel_x_AfterFilter , \
               imu.Accel_y_AfterFilter , \
               imu.Accel_z_AfterFilter , \
              NULL , &imu.Roll , &imu.Pitch
            );
  imu.Yaw = 0.9 * (imu.Yaw - ( imu.Gyro_z_AfterFilter + imu.Gyro_z_Offset  ) * Gyro_Gr*2*halfT) + 0.1 * imu.yaw_mag;
}
