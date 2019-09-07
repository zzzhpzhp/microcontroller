#include "stm32f10x.h"
#include "sys.h"
#include "stdint.h"
  /**************************************************************************
���ߣ�ƽ��С��֮��
�ҵ��Ա�С�꣺http://shop114407458.taobao.com/
**************************************************************************/
u8 Flag_Left,Flag_Right,Flag_Direction=0,Flag_Way,Flag_Next; //����ң����صı���
u8 Flag_Stop=1,Flag_Show; //ֹͣ��־λ�� ��ʾ��־λ Ĭ��ֹͣ ��ʾ��
int16_t Encoder_Left,Encoder_Right;  //���������������                 
long int Motor_Left,Motor_Right; //���PWM����
long int Target_Left,Target_Right; //���Ŀ��ֵ
float Velocity,Turn,Servo; //�ٶȺͽǶȱ���
int Voltage;//��ص�ѹ������صı���                       
u8 delay_cnt,delay_flag; //��ʱ��ر���
u8 rxbuf[8],Urxbuf[8],CAN_ON_Flag=0,Usart_ON_Flag=0,Usart_Flag,PID_Send;  //CAN�ʹ��ڿ�����ر���
u8 txbuf[8],txbuf2[8];  //CAN������ر���
float Pitch,Roll,Yaw,Gryo_Z;   //����Ƕ� Z�������Ǻ�XYZ��Ŀ���ٶ�
float Velocity_KP=3.0,Velocity_KI=1.0;	          //�ٶȿ���PID����
int RC_Velocity=30;         //����ң�ص��ٶȺ�λ��ֵ
int PS2_LX,PS2_LY,PS2_RX,PS2_RY,PS2_KEY; //PS2��ر��� 
u16 CCD_Zhongzhi,CCD_Yuzhi,ADV[128]={0};//CCD��ر���
int Sensor_Left,Sensor_Middle,Sensor_Right,Sensor;//���Ѳ�����
int Remoter_Ch1,Remoter_Ch2,Remoter_Ch3,Remoter_Ch4;//��ģң�زɼ���ر���
int Distance_A,Distance_B,Distance_C,Distance_D;//��������ر��� 
long overtime_ticks = 200;
long overtime_tick_cnt = 0;

int main(void)
{ 
	delay_init();	    	            //=====��ʱ������ʼ��	
	JTAG_Set(SWD_ENABLE);           //=====��SWD�ӿ� �������������SWD�ӿڵ���
	LED_Init();                     //=====��ʼ���� LED ���ӵ�Ӳ���ӿ�
	KEY_Init();                     //=====������ʼ��
	MY_NVIC_PriorityGroupConfig(2);	//=====�����жϷ���
	MiniBalance_PWM_Init(7199,0);   //=====��ʼ��PWM 10KHZ������������� �����ʼ������ӿ�
	Servo_PWM_Init(9999,71);   		  //=====��ʼ��PWM50HZ���� ���	
	uart5_init(115200);             //=====����5��ʼ��
	usart3_init(9600);              //=====����3��ʼ�� 
	usart1_init(115200);						//=====����1��ʼ�� 
	OLED_Init();                    //=====OLED��ʼ��	    
	Encoder_Init_TIM2();            //=====�������ӿ�
	Encoder_Init_TIM4();            //=====�������ӿ�
	while(select())	{	}	            //=====ѡ������ģʽ 		
	delay_ms(500);                  //=====��ʱ�ȴ��ȶ�
	IIC_Init();                     //=====IIC��ʼ��
	MPU6050_initialize();           //=====MPU6050��ʼ��	
	DMP_Init();                     //=====��ʼ��DMP   
	CAN1_Mode_Init(1,2,3,6,0);			//=====CAN��ʼ��,������1Mbps
	Adc_Init();                     //=====adc��ʼ��		
	if(Flag_Way==1)
	{
		PS2_Init();											//=====PS2�ֱ���ʼ��
		PS2_SetInit();									//=====ps2���ó�ʼ��,���á����̵�ģʽ������ѡ���Ƿ�����޸�
	}
	else if(Flag_Way==2)
		ccd_Init();  //=====CCD��ʼ��
	else if(Flag_Way==3)
		ele_Init();  //=====��Ŵ�������ʼ��	
	//TIM5_Cap_Init(0XFFFF,72-1);	  //=====������ʼ�� Ĭ��ע�� ���������� �ο�timer.h�ļ�
	//TIM3_Cap_Init(0XFFFF,72-1);	  //=====��ģң�س�ʼ��  Ĭ��ע�ͺ�ģң�������� �ο�timer.h�ļ�
	MiniBalance_EXTI_Init();        //=====MPU6050 5ms��ʱ�жϳ�ʼ��
	while(1)
	{
		#if 0
		if(Flag_Way==1) //PS2ģʽ��ִ��
		{
			PS2_LX=PS2_AnologData(PSS_LX);    //PS2���ݲɼ�    
			PS2_LY=PS2_AnologData(PSS_LY);
			PS2_RX=PS2_AnologData(PSS_RX);
			PS2_RY=PS2_AnologData(PSS_RY);
			PS2_KEY=PS2_DataKey();
		}
		#endif
		
		if(Flag_Way==0)           //APPģʽ��ͨ�����ں�CAN���ⷢ��ָ��
		{
			#if 0
			CAN1_SEND();             //CAN����
			#endif		
			UART_TX();               //���ڷ���
		}
		
		#if 0
		if(Flag_Show==0)         //ʹ��MiniBalance APP��OLED��ʾ��
		{
			APP_Show();	
			oled_show();          //===��ʾ����
		}
		else                      //ʹ��MiniBalance��λ�� ��λ��ʹ�õ�ʱ����Ҫ�ϸ��ʱ�򣬹ʴ�ʱ�ر�app��ز��ֺ�OLED��ʾ��
		{
			DataScope();          //����MiniBalance��λ��
		}
		#endif
		
		#if 0
		delay_flag=1;	
		delay_cnt=0;
		while(delay_flag);	     //ͨ����ʱ�ж�ʵ�ֵ�50ms��׼��ʱ	
	  #endif
	} 
}

