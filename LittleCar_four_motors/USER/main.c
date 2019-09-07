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
#include "zhp_odom.h"

#include "ucos_ii.h"
#include "os_cpu.h"
#include "os_cfg.h"


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

//#define IMU_TASK_PRIO                                  6
//#define IMU_STK_SIZE                                   256
//OS_STK IMU_TASK_STK[IMU_STK_SIZE];
//void imu_task(void *pdata);

#define CONTROL_TASK_PRIO                              5
#define CONTROL_STK_SIZE                               256
OS_STK CONTROL_TASK_STK[CONTROL_STK_SIZE];
void control_task(void *pdata);

//������ģ��
#define TEST_TASK_PRIO                                 7
#define TEST_STK_SIZE                                  256
OS_STK TEST_TASK_STK[TEST_STK_SIZE];
void supervising_task(void *pdata);

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
    OSTimeDlyHMSM(0,0,0,20); //���ݱ���Ƶ��

    IWDG_Feed();
//		float aa = 123.345;
//		uint16_t buf_size = sprintf((char *)tx_frame_buf, " %f  ", 123.432);
//		if (sender_encoder_frame((char *)tx_frame_buf, buf_size) == TRUE)
//		{
//			printf((const char *)final_tx_frame);
//		}
		
		memcpy(tx_frame_buf, &frame_data, sizeof(frame_data));
		tx_frame_buf[sizeof(frame_data) + 1] = '\0';
		if (sender_encoder_frame((char *)tx_frame_buf, sizeof(frame_data)) == TRUE)
		{
			printf((const char *)final_tx_frame);
		}
		
//		uint16_t buf_size = sprintf ((char *)tx_frame_buf, "%d %d", auto_ctrl_val.velocity_feedback * 10000, auto_ctrl_val.angular_feedback * 1000);
//		if (sender_encoder_frame((char *)tx_frame_buf, buf_size) == TRUE)
//		{
//			printf((const char *)final_tx_frame);
//		}

  }
}

void supervising_task(void *pdata)
{
  while(1)
  {
		// supervise velocity command
		auto_ctrl_val.cmd_recv_gap++;
		if (auto_ctrl_val.cmd_recv_gap >= 10)
		{
			memset(final_rx_frame, 0, sizeof(final_rx_frame));
			auto_ctrl_val.velocity_cmd = 0.0;
			auto_ctrl_val.angular_cmd = 0.0;
		}
		
    OSTimeDlyHMSM(0,0,0,100);
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
long test_cnt = 0;

void test_fun()
{
		auto_ctrl_val.higher_left_pwm = 3000;
    higher_left_forward();
    higher_left_set_pwm(auto_ctrl_val.higher_left_pwm);
    OSTimeDlyHMSM(0,0,3,0);
		higher_left_break();
    OSTimeDlyHMSM(0,0,0,300);
    higher_left_backward();
    higher_left_set_pwm(auto_ctrl_val.higher_left_pwm);
    OSTimeDlyHMSM(0,0,5,0);
		higher_left_break();
    OSTimeDlyHMSM(0,0,0,300);
		
		auto_ctrl_val.lower_left_pwm = 3000;
    lower_left_forward();
    lower_left_set_pwm(auto_ctrl_val.lower_left_pwm);
    OSTimeDlyHMSM(0,0,3,0);
		lower_right_break();
    OSTimeDlyHMSM(0,0,0,300);
    lower_left_backward();
    lower_left_set_pwm(auto_ctrl_val.lower_left_pwm);
    OSTimeDlyHMSM(0,0,3,0);
		lower_right_break();
    OSTimeDlyHMSM(0,0,0,300);
		
		
		auto_ctrl_val.higher_right_pwm = 3000;
    higher_right_forward();
    higher_right_set_pwm(auto_ctrl_val.higher_right_pwm);
    OSTimeDlyHMSM(0,0,3,0);
		higher_right_break();
    OSTimeDlyHMSM(0,0,0,300);
    higher_right_backward();
    higher_right_set_pwm(auto_ctrl_val.higher_right_pwm);
    OSTimeDlyHMSM(0,0,3,0);
		higher_right_break();
    OSTimeDlyHMSM(0,0,0,300);


		auto_ctrl_val.lower_right_pwm = 3000;
    lower_right_forward();
    lower_right_set_pwm(auto_ctrl_val.lower_right_pwm);
    OSTimeDlyHMSM(0,0,3,0);
		lower_right_break();
    OSTimeDlyHMSM(0,0,0,300);
    lower_right_backward();
    lower_right_set_pwm(auto_ctrl_val.lower_right_pwm);
    OSTimeDlyHMSM(0,0,3,0);
		lower_right_break();
    OSTimeDlyHMSM(0,0,0,300);

    test_cnt++;
}


//CONTROL����
void control_task(void *pdata)
{
	auto_ctrl_val.lower_left_pwm = 0;
	auto_ctrl_val.higher_left_pwm = 0;
	auto_ctrl_val.lower_right_pwm = 0;
	auto_ctrl_val.higher_right_pwm = 0;

	auto_ctrl_val.lower_left_dir_state = TRUE;
	auto_ctrl_val.higher_left_dir_state = TRUE;
	auto_ctrl_val.lower_right_dir_state = TRUE;
	auto_ctrl_val.higher_right_dir_state = TRUE;
	
	higher_left_break();
	lower_left_break();
	higher_right_break();
	lower_right_break();

	OSTimeDlyHMSM(0,0,0,100);
	
  while(1)
  {
		
//		moto_control_with_val();
//		test_fun();
		
    OSTimeDlyHMSM(0,0,0,5);
		auto_ctrl_val.period = 0.005;

    zhp_computer_control();

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
//  OSTaskCreate(
//              imu_task,(void *)0,
//              (OS_STK*)&IMU_TASK_STK[IMU_STK_SIZE-1],
//              IMU_TASK_PRIO
//              );
  OSTaskCreate(
              control_task,
              (void *)0,
              (OS_STK*)&CONTROL_TASK_STK[CONTROL_STK_SIZE-1],
              CONTROL_TASK_PRIO
              );
  OSTaskCreate(
              supervising_task,
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
  OSTaskCreate(start_task,(void *)0, (OS_STK *)&START_TASK_STK[START_STK_SIZE-1], START_TASK_PRIO );//������ʼ����
  OSStart();
}
