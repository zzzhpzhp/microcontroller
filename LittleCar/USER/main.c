#include "led.h"
#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "includes.h"
#include "timer.h"
#include "IMU.h"
#include "Motors.h"
#include "pstwo.h"
#include "wdg.h"
#include "zhp_frame.h"
#include "zhp_control_app.h"
#include "zhp_app.h"
#include "zhp_ext_int.h"

#include "ucos_ii.h"
#include "os_cpu.h"
#include "os_cfg.h"
/*
Kcopter

SWC ��ɫ
SWD ��ɫ

CH1 PA0 TIM2_CH1            ��ҡ�� ���ҷ���   �� 1000 �� 2000          �ػ� 1500   CH1
CH2 PA1 TIM2_CH2            ��ҡ�� ���·���   �� 1000 �� 2000          �ػ� 1500   CH2
CH3 PA2 TIM2_CH3
CH4 PA3 TIM2_CH4
CH5 PA6 TIM3_CH1 (SDMISO)   ��ҡ�� ���·���   �� 2000 �� 1000          �ػ� 1000   CH3
CH6 PA7 TIM3_CH2 (SDMOSI)   ��ҡ�� ���ҷ���   �� 2000 �� 1000          �ػ� 1500   CH4
CH7 PB0 TIM3_CH3            ������ο���      �� 1000 �� 2000          �ػ� 1500   CH5
CH8 PB1 TIM3_CH4            �ұ����ο���      �� 1000 �� 1500 ��2000   �ػ� 1500   CH6

PWM1 PA8  TIM1_CH1     ENA
PWM2 PA11 TIM1_CH4     ENB
PWM3 PB6  TIM4_CH1     IN1
PWM4 PB7  TIM4_CH2     IN2
PWM5 PB8  TIM4_CH3     IN3
PWM6 PB9  TIM4_CH4     IN4

PB10 SCL
PB11 SDA

PB12 NSS
PB13 SCK
PB14 MISO
PB15 MOSI

PA4 PADC

PB4 LED0
PB3 LED1

PA9  TX  ��ɫ
PA10 RX  ��ɫ

PA5  SDSCK
PA13 SWDIO
PA14 SWCLK

PA12 BEEP

PD0 OSD_IN
PD1 OSD_OUT

PC13 INT1
PC14 INT2
PC15 INT3

GPS
SDA ��ɫ PA11
SCL ��ɫ PA10
RX ��ɫ PA2  USART2
TX ��ɫ PA3
VCC ��ɫ
GND ��ɫ

*/

/////////////////////////////////UCOSII�����ջ����///////////////////////////////////

//������ģ��
#define XXX_TASK_PRIO                                  6
#define XXX_STK_SIZE                                   64
OS_STK XXX_TASK_STK[XXX_STK_SIZE];
void xxx_task(void *pdata);

#define START_TASK_PRIO                                10 //��ʼ��������ȼ�����Ϊ���
#define START_STK_SIZE                                 128
OS_STK START_TASK_STK[START_STK_SIZE];
void start_task(void *pdata);

#define IMU_TASK_PRIO                                  6
#define IMU_STK_SIZE                                   256
OS_STK IMU_TASK_STK[IMU_STK_SIZE];
void imu_task(void *pdata);

#define CONTROL_TASK_PRIO                              5
#define CONTROL_STK_SIZE                               256
OS_STK CONTROL_TASK_STK[CONTROL_STK_SIZE];
void control_task(void *pdata);

//������ģ��
#define TEST_TASK_PRIO                                 7
#define TEST_STK_SIZE                                  256
OS_STK TEST_TASK_STK[TEST_STK_SIZE];
void test_task(void *pdata);

//������ģ��
#define FEEDBACK_TASK_PRIO                             9
#define FEEDBACK_STK_SIZE                              256
OS_STK FEEDBACK_TASK_STK[FEEDBACK_STK_SIZE];
void feedback_task(void *pdata);

OS_EVENT * feedback_sem;

void feedback_task(void *pdata)
{
  while (1)
  {
    OSTimeDlyHMSM(0,0,0,120); //���ݱ���Ƶ��

    IWDG_Feed();
    if (ctrl_val.ctrl_dat_state == CTRL_TRI_SWI_H)
    {
      uint16_t buf_size = sprintf((char *)tx_frame_buf, "%u %u %u", ctrl_val.ctrl_ly, ctrl_val.ctrl_rx, ctrl_val.ctrl_dat_state);
      if (sender_encoder_frame((char *)tx_frame_buf, buf_size) == TRUE)
      {
        printf((const char *)final_tx_frame);
      }
    }
    else
    {
      uint16_t buf_size = sprintf((char *)tx_frame_buf, "%u %u %u", ctrl_val.ctrl_ly, ctrl_val.ctrl_rx, ctrl_val.ctrl_dat_state);
      if (sender_encoder_frame((char *)tx_frame_buf, buf_size) == TRUE)
      {
        printf((const char *)final_tx_frame);
      }

      OSTimeDlyHMSM(0,0,0,200); //Ԥ������ͷ����һ�����ݵ��Զ˻��һ������ͷͼ�񣩣�����տ���ʱһƬ�ڣ��մ�����ͷʱǰ����ͼƬ����ȫ�ڵģ�
    }
  }
}

uint16_t recv_frame_cnt = 0;
uint16_t recv_frame_freq = 0;
void test_task(void *pdata)
{
  while(1)
  {
    recv_frame_freq = recv_frame_cnt;
    recv_frame_cnt = 0;  
    OSTimeDlyHMSM(0,0,1,0);
  }
}

//IMU����
void imu_task(void *pdata)
{
    while(1)
    {
//        zhp_get_imu_data();
        OSTimeDlyHMSM(0,0,0,50);

        IWDG_Feed();
    };
}
//CONTROL����
void control_task(void *pdata)
{
  while(1)
  {
    //app_control(imu.Yaw, imu.Roll, imu.Pitch, imu.Gyro_x_AfterFilter, imu.Gyro_y_AfterFilter, imu.Gyro_z_AfterFilter);
    //MotoPwmRefresh(1500,300,500,700,800,1000);
      
//    MOTOR_A_SetPwm(1000);
//    MOTOR_A_SetForward();
//    MOTOR_B_SetPwm(2000);
//    MOTOR_B_SetForward();
//    OSTimeDlyHMSM(0,0,5,0);

//    MOTOR_A_SetPwm(2000);
//    MOTOR_A_SetBackward();
//    MOTOR_B_SetPwm(1000);
//    MOTOR_B_SetBackward();
//    OSTimeDlyHMSM(0,0,5,0);

//    MOTOR_A_SetPwm(1000);
//    MOTOR_A_SetForward();
//    MOTOR_B_SetPwm(2000);
//    MOTOR_B_SetForward();
//    OSTimeDlyHMSM(0,0,5,0);

//    MOTOR_A_SetPwm(2000);
//    MOTOR_A_SetBackward();
//    MOTOR_B_SetPwm(1000);
//    MOTOR_B_SetBackward();
//    OSTimeDlyHMSM(0,0,5,0);

//    MOTOR_A_Brake();
//    MOTOR_B_Brake();
//    OSTimeDlyHMSM(0,0,2,0);

    OSTimeDlyHMSM(0,0,0,20);

    calc_speed(0.02);
    //���������Կ��ƣ�����δ����ҡ�ˣ�����Կ��Ƽ�ʻ
    if (ctrl_val.ctrl_dat_state == CTRL_TRI_SWI_L \
        && ctrl_val.ctrl_ly < VERIFYED_MIDPOINT_H \
        && ctrl_val.ctrl_ly > VERIFYED_MIDPOINT_L \
        && ctrl_val.ctrl_rx < VERIFYED_MIDPOINT_H \
        && ctrl_val.ctrl_rx > VERIFYED_MIDPOINT_L \
       )
    {
      zhp_computer_control();
    }
    else
    {
      zhp_remote_control();
    }

    IWDG_Feed();


  };
}




//��ʼ����
void start_task(void *pdata)
{
  OS_CPU_SR cpu_sr=0;
  pdata = pdata;
  OSStatInit();                    //��ʼ��ͳ������.�������ʱ1��������
  OS_ENTER_CRITICAL();            //�����ٽ���(�޷����жϴ��)

//  OSTaskCreate(XXX_task,
//              (void *)0,
//              (OS_STK*)&XXX_TASK_STK[XXX_STK_SIZE-1],
//              XXX_TASK_PRIO
//              );
  OSTaskCreate(
              imu_task,(void *)0,
              (OS_STK*)&IMU_TASK_STK[IMU_STK_SIZE-1],
              IMU_TASK_PRIO
              );
  OSTaskCreate(
              control_task,
              (void *)0,
              (OS_STK*)&CONTROL_TASK_STK[CONTROL_STK_SIZE-1],
              CONTROL_TASK_PRIO
              );
  OSTaskCreate(
              test_task,
              (void *)0,
              (OS_STK*)&TEST_TASK_STK[TEST_STK_SIZE-1],
              TEST_TASK_PRIO
              );
  OSTaskCreate(
              feedback_task,
              (void *)0,
              (OS_STK*)&FEEDBACK_TASK_STK[FEEDBACK_STK_SIZE-1],
              FEEDBACK_TASK_PRIO
              );

   feedback_sem = OSSemCreate(0);

   zhp_app_init();


  OSTaskSuspend(START_TASK_PRIO);    //������ʼ����.
  OS_EXIT_CRITICAL();                //�˳��ٽ���(���Ա��жϴ��)
}

int main(void)
{
  delay_init();              //��ʱ��ʼ��
  NVIC_Configuration();      //����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�

  OSInit();
  OSTaskCreate(start_task,(void *)0,(OS_STK *)&START_TASK_STK[START_STK_SIZE-1],START_TASK_PRIO );//������ʼ����
  OSStart();
}
