
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
#include "zhp_odom.h"
#include "cpal_i2c.h"
#include "cpal_conf.h"
#include "timer.h"
#include "Motors.h"
#include "pstwo.h"
#include "wdg.h"


#define BUFFER_NUMBER 30
/*
IN1    IN2    ENA1    OUT1 OUT2输出
0       0      X          刹车
1       1      X          悬空
1       0     PWM       正转调速
0       1     PWM       反转调速
1       0      1        全速正转       
0       1      1        全速反转

IN3    IN4    ENA2    OUT3 OUT4输出
0       0      X          刹车
1       1      X          悬空
1       0     PWM       正转调速
0       1     PWM       反转调速
1       0      1        全速正转       
0       1      1        全速反转

Interface:

I2C B10 B11
MOTOR DIRECTION C8 C9 C10 C11 C12 C13 C14 C15
MOTOR PWM 
EXIT (WHEEL SPEED) B12 B13 B14 B15

Connection:

higher_left    = A0
lower_left     = A1
higher_right   = B0
lower_right    = B1

motor driver board wire connection
higher_left:   encoder B13-encoder_c1 C5-encoder_c2   en-B9 L1
lower_left:    encoder B12-encoder_c1 C4-encoder_c2   en-B8 L0 
higher_right:  encoder B15-encoder_c1 C7-encoder_c2   en-B7 R1
lower_right:   encoder B14-encoder_c1 C6-encoder_c2   en-B6 R0

right_board:    
               R1
							 ENA2 B7
							 N3-C10 N4-C11
               OUT3 - higher_right_motor_M1 
							 OUT4 - higher_right_motor_M2
							 
							 R0
							 ENA1 B6
							 N1-C8 N2-C9 
							 OUT1 - lower_right_motor_M1 
							 OUT2 - lower_right_motor_M2
							 
left_board:   
               L1
							 ENA2 B9
							 N3-C14 N4-C15 
               OUT3 - higher_left_motor_M2 
							 OUT4 - higher_left_motor_M1 
							 
							 L0
							 ENA1 B8
							 N1-C12 N2-C13 
							 OUT1 - lower_left_motor_M2 
							 OUT2 - lower_left_motor_M1
*/

long frame_cnt = 0;

void zhp_app_init(void)
{
  SystemInit();
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  delay_init();     //延时初始化
//  LED_Init();       //初始化STM32-SDK板子上的LED接口
  delay_ms(30);     //等待器件上电
//  IMU_init();     //初始化IMU和传感器
  MOTORS_Init();
//  PS2_Init();
//  CRC_Config();
//  RemoteControllerCaptureInit();
  IWDG_Init(4,625);    //与分频数为64,重载值为625,溢出时间为1s
  pwm_resolution = PWM_RESOLUTION;
	phase_detector_pin_init();
  zhp_ext_int_init();
	zhp_control_init();
	auto_ctrl_val.cmd_recv_gap = 0;

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
