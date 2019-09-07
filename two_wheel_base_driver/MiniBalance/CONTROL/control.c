#include "control.h"	
#include "filter.h"	
  /**************************************************************************
���ߣ�ƽ��С��֮��
�ҵ��Ա�С�꣺http://shop114407458.taobao.com/
**************************************************************************/
#if USE_ZHP_FRAME
upload_data_type ud;
control_data_type *cd;
#endif

u8 Flag_Target,Flag_Change;  //��ر�־λ
float Voltage_Count,Voltage_All;  //��ѹ������ر���
int j,sum;
#define T 0.156f
#define L 0.1445f
#define K 622.8f
/**************************************************************************
�������ܣ�С���˶���ѧģ��
��ڲ������ٶȺ�ת��
����  ֵ����
**************************************************************************/
void Kinematic_Analysis(float velocity,float turn)
{
		Target_Left=(velocity-turn); 
		Target_Right=(velocity+turn);      //���ֲ���
}
/**************************************************************************
�������ܣ����еĿ��ƴ��붼��������
         5ms��ʱ�ж���MPU6050��INT���Ŵ���
         �ϸ�֤���������ݴ����ʱ��ͬ��				 
**************************************************************************/
int EXTI15_10_IRQHandler(void)
{
	if (INT==0)
	{
		EXTI->PR=1<<15;           //���LINE15�ϵ��жϱ�־λ  		
		Flag_Target=!Flag_Target; //��Ƶ��־λ
		if(delay_flag==1)
		{
			if(++delay_cnt==2)	 
				delay_cnt=0, delay_flag=0; //���������ṩ��ʱ
		}
		if(Flag_Target==1) //5ms��ȡһ�������Ǻͼ��ٶȼƵ�ֵ����ִ��һ������������
		{
			if(Usart_Flag==0&&Usart_ON_Flag==1)  memcpy(rxbuf,Urxbuf,8*sizeof(u8));	//��������˴��ڿ��Ʊ�־λ����ȡ��ص�ָ��
			Read_DMP();   //===������̬								 
			Key();//ɨ�谴���仯
			
			Voltage_All+=Get_battery_volt();     //��β����ۻ�
			//Read_Distane();//��·����������ȡ��Ĭ��ע�ͣ���������Ҫȥ���������ĳ�ʼ�������ע�͡���ȡ�ı�����Distance_A,Distance_B,Distance_C,Distance_D
			if(++Voltage_Count==100) Voltage=Voltage_All/100,Voltage_All=0,Voltage_Count=0;//��ƽ��ֵ ��ȡ��ص�ѹ	   				
			return 0;	        //===10ms����һ�Σ�                                        
		}
		static long cnt__=0;
		cnt__++;
		if (cnt__ >=  100)
		{			
      Led_Reverse();
			cnt__=0;
		}
		Read_DMP();  //===������̬	 	
		
		Encoder_Right=-Read_Encoder(4);  //===��ȡ��������ֵ	 //Ϊ�˱�֤M�����ٵ�ʱ���׼�����ȶ�ȡ����������
		Encoder_Left=Read_Encoder(2);    //===��ȡ��������ֵ
		Get_RC();   //===���տ���ָ��
		if(Turn_Off(Voltage)==0)   //===�����ص�ѹ�������쳣
		{ 	
			if (overtime_tick_cnt++ > overtime_ticks)
			{
			  Motor_Left=Incremental_PI_Left(Encoder_Left, 0);  //===�ٶȱջ����Ƽ�����������PWM
			  Motor_Right=Incremental_PI_Right(Encoder_Right, 0);  //===�ٶȱջ����Ƽ����ҵ������PWM
			}	
			else
			{
				Motor_Left=Incremental_PI_Left(Encoder_Left,Target_Left);  //===�ٶȱջ����Ƽ�����������PWM
				Motor_Right=Incremental_PI_Right(Encoder_Right,Target_Right);  //===�ٶȱջ����Ƽ����ҵ������PWM
			}	
			 Xianfu_Pwm(6900);                          //===PWM�޷�
			 Set_Pwm(Motor_Left,Motor_Right,Servo);     //===��ֵ��PWM�Ĵ���  
		}
		else	
		{
			Set_Pwm(0,0,SERVO_INIT);    //===��ֵ��PWM�Ĵ���  	
		}
	}
	return 0;	 //����ֵ
} 
/**************************************************************************
�������ܣ���ֵ��PWM�Ĵ���
��ڲ�����PWM
����  ֵ����
**************************************************************************/
void Set_Pwm(int motor_a,int motor_b,int servo)
{
	if(motor_a>0)			
		PWMA1=7200,PWMA2=7200-motor_a;
	else 	            
		PWMA2=7200,PWMA1=7200+motor_a;
	
	if(motor_b>0)			
		PWMB1=7200,PWMB2=7200-motor_b;
	else 	            
		PWMB2=7200,PWMB1=7200+motor_b;
	
	SERVO=servo;	
}
/**************************************************************************
�������ܣ�����PWM��ֵ 
��ڲ�������ֵ
����  ֵ����
**************************************************************************/
void Xianfu_Pwm(int amplitude)
{	
    if(Motor_Left<-amplitude) Motor_Left=-amplitude;	//������Сֵ
		if(Motor_Left>amplitude)  Motor_Left=amplitude;	  //�������ֵ
	  if(Motor_Right<-amplitude) Motor_Right=-amplitude;//������Сֵ	
		if(Motor_Right>amplitude)  Motor_Right=amplitude;	//�������ֵ		
}
/************************************************************************
�������ܣ������޸�С������״̬ 
��ڲ�������
����  ֵ����
**************************************************************************/
void Key(void)
{	
	u8 tmp,tmp2;
	tmp=click_N_Double(50); //˫����˫���ȴ�ʱ��500ms
	if(tmp==1)Flag_Stop=!Flag_Stop;//��������С������ͣ
	if(tmp==2)Flag_Show=!Flag_Show;//˫������С������ʾ״̬
	tmp2=Long_Press();  //����        
  if(tmp2==1)Flag_Show=!Flag_Show;//����С������ʾ״̬                 
}
/**************************************************************************
�������ܣ��쳣�رյ��
��ڲ�������ѹ
����  ֵ��1���쳣  0������
**************************************************************************/
u8 Turn_Off( int voltage)
{
	u8 temp;
	if(voltage<1110||Flag_Stop==1)//��ص�ѹ����11.1V�رյ��
	{	                                                
		temp=1;      
		PWMA1=0; //�������λ����                                           
		PWMB1=0; //�������λ����
		PWMA2=0; //�������λ����
		PWMB2=0; //�������λ����					
	}
	else
		temp=0;
	return temp;			
}

/**************************************************************************
�������ܣ�����ֵ����
��ڲ�����int
����  ֵ��unsigned int
**************************************************************************/
int myabs(int a)
{ 		   
	  int temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}

/**************************************************************************
�������ܣ�����PI������
��ڲ���������������ֵ��Ŀ���ٶ�
����  ֵ�����PWM
��������ʽ��ɢPID��ʽ 
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)������ƫ�� 
e(k-1)������һ�ε�ƫ��  �Դ����� 
pwm�����������
�����ǵ��ٶȿ��Ʊջ�ϵͳ���棬ֻʹ��PI����
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)
**************************************************************************/
int Incremental_PI_Left (int Encoder,int Target)
{ 	
	 static int Bias, Pwm, Last_bias;
	 Bias = Encoder - Target;                //����ƫ��
	 Pwm += Velocity_KP * (Bias-Last_bias) + Velocity_KI * Bias;   //����ʽPI������
	 if(Pwm > 7200)
		 Pwm = 7200;
	 if (Pwm < -7200) 
		 Pwm = -7200;
	 Last_bias = Bias;	                   //������һ��ƫ�� 
	 return Pwm;                         //�������
}
int Incremental_PI_Right (int Encoder,int Target)
{ 	
	 static int Bias, Pwm, Last_bias;
	 Bias = Encoder-Target;                //����ƫ��
	 Pwm += Velocity_KP * (Bias - Last_bias) + Velocity_KI * Bias;   //����ʽPI������
	 if (Pwm > 7200) 
		 Pwm = 7200;
	 if (Pwm < -7200) 
		 Pwm = -7200;
	 Last_bias = Bias;	                   //������һ��ƫ�� 
	 return Pwm;                         //�������
}
/**************************************************************************
�������ܣ�ͨ��ָ���С������ң��
��ڲ���������ָ��
����  ֵ����
**************************************************************************/
void Get_RC(void)
{
	int Yuzhi=2;  		//PS2���Ʒ�����ֵ
	float LY,RX,RY;  //PS2�ֱ����Ʊ���
	static float Bias,Last_Bias;  //ƫ�����ʷֵ
	if(CAN_ON_Flag == 0 && Usart_ON_Flag == 0) 
	{
		if(Flag_Way==0)//���ڵȿ��Ʒ�ʽ
		{
			if (Flag_Direction==0||Flag_Direction==10) 
				Velocity=0,Turn=0;   //ֹͣ
			else if(Flag_Direction==1) Velocity=RC_Velocity,Turn=0;        //ǰ��
			else if(Flag_Direction==2) Velocity=RC_Velocity,Turn=-RC_Velocity/2;     //��ǰ
			else if(Flag_Direction==3) Velocity=0,Turn=-RC_Velocity/2;        //��ת
			else if(Flag_Direction==4) Velocity=-RC_Velocity,Turn=RC_Velocity/2;   // �Һ�
			else if(Flag_Direction==5) Velocity=-RC_Velocity,Turn=0;      //����
			else if(Flag_Direction==6) Velocity=-RC_Velocity,Turn=-RC_Velocity/2;  //���
			else if(Flag_Direction==7) Velocity=0,Turn=RC_Velocity/2;       //��ת
			else if(Flag_Direction==8) Velocity=RC_Velocity,Turn=RC_Velocity/2;   //��ǰ
			else Velocity=0,Turn=0;   //ֹͣ
			if (Turn==0&&Roll>-45&&Roll<45)
				Turn=-gyro[2]*0.002;  
		}		
		else	if(Flag_Way==1) //PS2����
		{
			LY=PS2_LY-128; //����ƫ��
			RX=PS2_RX-128;
			RY=PS2_RY-128;     
			if(LY>-Yuzhi&&LY<Yuzhi)
				LY=0; //С�Ƕ���Ϊ���� ��ֹ���������쳣
			if(RX>-Yuzhi&&RX<Yuzhi)
				RX=0;
			Velocity=-LY/4-RY/4; //�ٶȺ�ҡ�˵�������ء�
			Turn=-RX/6; 	
			if (Turn==0&&Roll>-45&&Roll<45)
				Turn=-gyro[2]*0.001;  
		}
		else	if(Flag_Way==2) //CCDѲ��
		{
			Velocity=45; //CCDѲ��ģʽ���ٶ�
			Bias=CCD_Zhongzhi-64;   //��ȡƫ��
			Turn=-Bias*0.4-(Bias-Last_Bias)*2; //PD����
			Last_Bias=Bias; //������һ�ε�ƫ��
		}
		else	if(Flag_Way==3)//���Ѳ��
		{
			Velocity=45; //���Ѳ��ģʽ�µ��ٶ�
			Bias=100-Sensor; //��ȡƫ��
			Turn=abs(Bias)*Bias*0.008+Bias*0.08+(Bias-Last_Bias)*3; //
			Last_Bias=Bias; //��һ�ε�ƫ��
		}
		Kinematic_Analysis(Velocity,Turn); 	//С���˶�ѧ����   	
	}
	else //can���ߴ��ڿ���ģʽ
	{
#if USE_ZHP_FRAME
		const int8_t FORWARD_CMD = 1;
		//const int8_t BACKWARD_CMD = -1;
		if (cd->left_target_dir == FORWARD_CMD)
		{
			Target_Left = cd->left_target_pulse;
		}
		else
		{
			Target_Left = -cd->left_target_pulse;
		}			
		
		if (cd->right_target_dir == FORWARD_CMD)
		{
			Target_Right = cd->right_target_pulse;
		}
		else
		{
			Target_Right = -cd->right_target_pulse;
		}
#else
		if(rxbuf[1]==0)
			Target_Left=rxbuf[0]; //ʶ���˶����� 
		else           
			Target_Left=-rxbuf[0]; //�ٶ���
		if(rxbuf[3]==0)
			Target_Right=rxbuf[2]; //ʶ���˶����� 
		else
			Target_Right=-rxbuf[2]; //�ٶ���
#endif
	}
}

/**************************************************************************
�������ܣ�����CCDȡ��ֵ
��ڲ�������
����  ֵ����
**************************************************************************/
void  Find_CCD_Zhongzhi(void)
{ 
	static u16 i,j,Left,Right,Last_CCD_Zhongzhi;
	static u16 value1_max,value1_min;

	value1_max=ADV[0]; //��̬��ֵ�㷨����ȡ������Сֵ
	for(i=5;i<123;i++) //���߸�ȥ��5����
	{
		if(value1_max<=ADV[i])
			value1_max=ADV[i];
	}
	value1_min=ADV[0]; //��Сֵ
	for(i=5;i<123;i++) 
	{
		if(value1_min>=ADV[i])
			value1_min=ADV[i];
	}
	CCD_Yuzhi=(value1_max+value1_min)/2; //���������������ȡ����ֵ
	for(i = 5;i<118; i++) //Ѱ�����������
	{
		if(ADV[i]>CCD_Yuzhi&&ADV[i+1]>CCD_Yuzhi&&ADV[i+2]>CCD_Yuzhi&&ADV[i+3]<CCD_Yuzhi&&ADV[i+4]<CCD_Yuzhi&&ADV[i+5]<CCD_Yuzhi)
		{	
			Left=i;
			break;	
		}
	}
	
	for(j = 118;j>5; j--) //Ѱ���ұ�������
	{
		if(ADV[j]<CCD_Yuzhi&&ADV[j+1]<CCD_Yuzhi&&ADV[j+2]<CCD_Yuzhi&&ADV[j+3]>CCD_Yuzhi&&ADV[j+4]>CCD_Yuzhi&&ADV[j+5]>CCD_Yuzhi)
		{	
			Right=j;
			break;	
		}
	}
	
	CCD_Zhongzhi=(Right+Left)/2;//��������λ��
	if (myabs(CCD_Zhongzhi-Last_CCD_Zhongzhi)>90)   //�������ߵ�ƫ����̫��
		CCD_Zhongzhi=Last_CCD_Zhongzhi;    //��ȡ��һ�ε�ֵ
	Last_CCD_Zhongzhi=CCD_Zhongzhi;  //������һ�ε�ƫ��
}
