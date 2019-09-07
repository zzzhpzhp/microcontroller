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


/////////////////////////////////UCOSII任务堆栈设置///////////////////////////////////

//任务定义模板
#define XXX_TASK_PRIO                                  6
#define XXX_STK_SIZE                                   64
OS_STK XXX_TASK_STK[XXX_STK_SIZE];
void xxx_task(void *pdata);

#define START_TASK_PRIO                                10 //开始任务的优先级设置为最低
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

//任务定义模板
#define TEST_TASK_PRIO                                 7
#define TEST_STK_SIZE                                  256
OS_STK TEST_TASK_STK[TEST_STK_SIZE];
void test_task(void *pdata);

//任务定义模板
#define FEEDBACK_TASK_PRIO                             9
#define FEEDBACK_STK_SIZE                              256
OS_STK FEEDBACK_TASK_STK[FEEDBACK_STK_SIZE];
void feedback_task(void *pdata);

OS_EVENT * feedback_sem;

void feedback_task(void *pdata)
{
  while (1)
  {
    OSTimeDlyHMSM(0,0,0,120); //数据保存频率

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

      OSTimeDlyHMSM(0,0,0,200); //预热摄像头（发一次数据电脑端会读一次摄像头图像），避免刚开启时一片黑（刚打开摄像头时前几张图片会是全黑的）
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

//IMU任务
void imu_task(void *pdata)
{
    while(1)
    {
//        zhp_get_imu_data();
        OSTimeDlyHMSM(0,0,0,50);

        IWDG_Feed();
    };
}
//CONTROL任务
void control_task(void *pdata)
{
  while(1)
  {
    //app_control(imu.Yaw, imu.Roll, imu.Pitch, imu.Gyro_x_AfterFilter, imu.Gyro_y_AfterFilter, imu.Gyro_z_AfterFilter);
    //MotoPwmRefresh(1500,300,500,700,800,1000);
      
    MOTOR_A_SetPwm(1000);
    MOTOR_A_SetForward();
    MOTOR_B_SetPwm(1000);
    MOTOR_B_SetForward();
    OSTimeDlyHMSM(0,0,5,0);

    MOTOR_A_SetPwm(500);
    MOTOR_A_SetBackward();
    MOTOR_B_SetPwm(500);
    MOTOR_B_SetBackward();
    OSTimeDlyHMSM(0,0,5,0);

    MOTOR_A_SetPwm(1000);
    MOTOR_A_SetForward();
    MOTOR_B_SetPwm(1000);
    MOTOR_B_SetForward();
    OSTimeDlyHMSM(0,0,5,0);

    MOTOR_A_SetPwm(500);
    MOTOR_A_SetBackward();
    MOTOR_B_SetPwm(500);
    MOTOR_B_SetBackward();
    OSTimeDlyHMSM(0,0,5,0);

//    MOTOR_A_Brake();
//    MOTOR_B_Brake();
//    OSTimeDlyHMSM(0,0,2,0);

    OSTimeDlyHMSM(0,0,0,20);

    calc_speed(0.02);
    //如果允许电脑控制，并且未操作摇杆，则电脑控制驾驶
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




//开始任务
void start_task(void *pdata)
{
  OS_CPU_SR cpu_sr=0;
  pdata = pdata;
  OSStatInit();                    //初始化统计任务.这里会延时1秒钟左右
  OS_ENTER_CRITICAL();            //进入临界区(无法被中断打断)

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


  OSTaskSuspend(START_TASK_PRIO);    //挂起起始任务.
  OS_EXIT_CRITICAL();                //退出临界区(可以被中断打断)
}

int main(void)
{
  delay_init();              //延时初始化
  NVIC_Configuration();      //设置NVIC中断分组2:2位抢占优先级，2位响应优先级

  OSInit();
  OSTaskCreate(start_task,(void *)0,(OS_STK *)&START_TASK_STK[START_STK_SIZE-1],START_TASK_PRIO );//创建起始任务
  OSStart();
}
