#include "show.h"
  /**************************************************************************
���ߣ�ƽ��С��֮��
�ҵ��Ա�С�꣺http://shop114407458.taobao.com/
**************************************************************************/
unsigned char i;          //��������
unsigned char Send_Count; //������Ҫ���͵����ݸ���
/**************************************************************************
�������ܣ�OLED��ʾ
��ڲ�������
����  ֵ����
**************************************************************************/
void oled_show(void)
{
						if(Flag_Way==0)	//ң��ģʽ
					{
					 //=============��1����ʾ3��Ƕ�===============//	
								OLED_ShowString(0,0,"X:");
								if(Pitch<0)		OLED_ShowNumber(15,0,Pitch+360,3,12);
								else					OLED_ShowNumber(15,0,Pitch,3,12);	
									 
								OLED_ShowString(40,0,"Y:");
								if(Roll<0)		OLED_ShowNumber(55,0,Roll+360,3,12);
								else					OLED_ShowNumber(55,0,Roll,3,12);	
							
								 OLED_ShowString(80,0,"Z:");
								if(Yaw<0)		  OLED_ShowNumber(95,0,Yaw+360,3,12);
								else					OLED_ShowNumber(95,0,Yaw,3,12);		
							//=============�ڶ���Z�������Ǻ�Ŀ���ٶ�===============//	
																		OLED_ShowString(00,10,"GZ");
							if( gyro[2]<0)      	OLED_ShowString(20,10,"-"),
																		OLED_ShowNumber(30,10,-gyro[2],5,12);
							else                 	OLED_ShowString(20,10,"+"),
																		OLED_ShowNumber(30,10, gyro[2],5,12);		//z������������	
																		OLED_ShowString(70,10,"V");
																		OLED_ShowNumber(80,10, RC_Velocity,5,12);	//�˶��ٶ�
					}
					else if(Flag_Way==1)//PS2ģʽ
					{
						OLED_ShowString(00,0,"LX");
						OLED_ShowNumber(15,0, PS2_LX,3,12);  //PS2������			
						OLED_ShowString(40,0,"LY");
						OLED_ShowNumber(55,0, PS2_LY,3,12);  //PS2������
						OLED_ShowString(80,0,"RX");
						OLED_ShowNumber(95,0, PS2_RX,3,12);
						OLED_ShowString(0,10,"KEY");
						OLED_ShowNumber(25,10,PS2_KEY,2,12);
					}
					else if(Flag_Way==2)//ccdģʽ
					{
						OLED_Show_CCD(); 
						OLED_ShowString(00,10,"Mid");
						OLED_ShowNumber(25,10, CCD_Zhongzhi,3,12);
						OLED_ShowString(65,10,"Light");
						OLED_ShowNumber(105,10, CCD_Yuzhi,3,12);
					}
					else if(Flag_Way==3)//���Ѳ��ģʽ
					{
						OLED_ShowString(00,0,"L");
						OLED_ShowNumber(10,0,Sensor_Left,4,12);	
						OLED_ShowString(40,0,"M");
						OLED_ShowNumber(50,0,Sensor_Middle,4,12);
						OLED_ShowString(80,0,"R");
						OLED_ShowNumber(90,0,Sensor_Right,4,12);
						OLED_ShowString(0,10,"Mid");
						OLED_ShowNumber(25,10,Sensor,4,12);		  
					}									
					//=============��3����ʾ������״̬=======================//	
						if( Target_Left<0)		  OLED_ShowString(00,20,"-"),
																OLED_ShowNumber(15,20,-Target_Left,5,12);
					else                 	OLED_ShowString(0,20,"+"),
																OLED_ShowNumber(15,20, Target_Left,5,12); 			
					if( Encoder_Left<0)		  OLED_ShowString(80,20,"-"),
																OLED_ShowNumber(95,20,-Encoder_Left,4,12);
					else                 	OLED_ShowString(80,20,"+"),
																OLED_ShowNumber(95,20, Encoder_Left,4,12);
					//=============��4����ʾ�ҵ����״̬=======================//	
						if( Target_Right<0)		  OLED_ShowString(00,30,"-"),
																OLED_ShowNumber(15,30,-Target_Right,5,12);
					else                 	OLED_ShowString(0,30,"+"),
																OLED_ShowNumber(15,30, Target_Right,5,12); 					
					if( Encoder_Right<0)		  OLED_ShowString(80,30,"-"),
																OLED_ShowNumber(95,30,-Encoder_Right,4,12);
					else                 	OLED_ShowString(80,30,"+"),
																OLED_ShowNumber(95,30, Encoder_Right,4,12);	

//					//=============��5����ʾ�����״̬=======================//	
//				 	OLED_ShowString(00,40,"Servo:"),	//���״̬
//					OLED_ShowNumber(60,40, Servo,4,12);	
//		OLED_ShowNumber(0,40, Remoter_Ch1,4,12);	OLED_ShowNumber(30,40, Remoter_Ch2,4,12);//��ģң������	
//		OLED_ShowNumber(60,40, Remoter_Ch3,4,12);	OLED_ShowNumber(90,40, Remoter_Ch4,4,12);	//Ĭ��ע�͵���
//		OLED_ShowNumber(0,40, Distance_A,4,12);	  OLED_ShowNumber(30,40, Distance_B,4,12);	//����������
//	  OLED_ShowNumber(60,40, Distance_C,4,12);	OLED_ShowNumber(90,40, Distance_D,4,12);	//Ĭ��ע�͵���
					//=============��6����ʾ��ѹģʽ��=======================//	
											OLED_ShowString(48,50,".");
											OLED_ShowString(70,50,"V");
											OLED_ShowNumber(35,50,Voltage/100,2,12);//��ѹ
											OLED_ShowNumber(58,50,Voltage%100,2,12);
 if(Voltage%100<10) 	OLED_ShowNumber(52,50,0,2,12);
                       //  ���ʹ��/ʹ����ʾ
											if(Flag_Stop==0)//����Flag_Stop��־λ��ʾ�����״̬
											OLED_ShowString(103,50,"O-N");
											if(Flag_Stop==1)
											OLED_ShowString(103,50,"OFF");
											if(Flag_Way==0)               OLED_ShowString(0,50,"APP");//��ͬ��ģʽ
											else if(Flag_Way==1)          OLED_ShowString(0,50,"PS2");
											else if(Flag_Way==2)				  OLED_ShowString(0,50,"CCD");
											else if(Flag_Way==3)				  OLED_ShowString(0,50,"ELE");
											OLED_Refresh_Gram();	//ˢ��
}
/**************************************************************************
�������ܣ���APP��������
��ڲ�������
����  ֵ����
��    �ߣ�ƽ��С��֮��
**************************************************************************/
void APP_Show(void)
{    
	static u8 flag;
	int app_2,app_3,app_4;
	app_4=(Voltage-1110)*2/3;	
	if(app_4>100)app_4=100;   //�Ե�ѹ���ݽ��д���
	app_2=Velocity*0.7;  if(app_2<0)app_2=-app_2;	  //�Ա��������ݾ������ݴ������ͼ�λ�
	app_3=Velocity*0.7;  if(app_3<0)app_3=-app_3;
	flag=!flag;//��Ƶ������ʱ���Ͳ�ͬ�Ĳ���
	if(PID_Send==1)//����PID����
	{
		printf("{C%d:%d:%d:%d:%d:%d:%d:%d:%d}$",(int)RC_Velocity,(int)Velocity_KP,(int)Velocity_KI,0,0,0,0,0,0);//��ӡ��APP����	
		PID_Send=0;	
	}	
	else	if(flag==0)
		printf("{A%d:%d:%d:%d}$",(u8)app_2,(u8)app_3,app_4,(int)((Servo-SERVO_INIT)*0.16)); //��ӡ��APP����
	else
		printf("{B%d:%d:%d:%d}$",(int)Pitch,(int)Roll,(int)Yaw,app_4);//��ӡ��APP���� ��ʾ����
}
/**************************************************************************
�������ܣ�����ʾ��������λ���������� �ر���ʾ��
��ڲ�������
����  ֵ����
��    �ߣ�ƽ��С��֮��
**************************************************************************/
void DataScope(void)
{   
		DataScope_Get_Channel_Data( Roll, 1 );  //��ʾ�Ƕ���Ϣ
		DataScope_Get_Channel_Data( Encoder_Left, 2 );   // ��ʾ�����ٶ�
		DataScope_Get_Channel_Data( Encoder_Right, 3 );  //��ʾ�ҵ���ٶ�
//		DataScope_Get_Channel_Data( 0 , 4 );   
//		DataScope_Get_Channel_Data(0, 5 ); //����Ҫ��ʾ�������滻0������
//		DataScope_Get_Channel_Data(0 , 6 );//����Ҫ��ʾ�������滻0������
//		DataScope_Get_Channel_Data(0, 7 );
//		DataScope_Get_Channel_Data( 0, 8 ); 
//		DataScope_Get_Channel_Data(0, 9 );  
//		DataScope_Get_Channel_Data( 0 , 10);
		Send_Count = DataScope_Data_Generate(3);
		for( i = 0 ; i < Send_Count; i++) 
		{
		while((USART1->SR&0X40)==0);  
		USART1->DR = DataScope_OutPut_Buffer[i]; 
		}
}
//OLED���㺬˰
void OLED_DrawPoint_Shu(u8 x,u8 y,u8 t)
{ 
	 u8 i=0;
  OLED_DrawPoint(x,y,t);
	OLED_DrawPoint(x,y,t);
	  for(i = 0;i<8; i++)
  {
      OLED_DrawPoint(x,y+i,t);
  }
}
//OLED��ʾLCD����
void OLED_Show_CCD(void)  
{ 
	 u8 i,t;
	 for(i = 0;i<128; i++)
  {
		if(ADV[i]<CCD_Yuzhi) t=1; else t=0;
		OLED_DrawPoint_Shu(i,0,t);
  }
}

//������ʾһ�ε�����
void oled_show_once(void)
{
   OLED_ShowString(0,00,"Turn Right Wheel");  //ת������
   OLED_ShowString(0,10,"TO Select Mode"); //ѡ��ģʽ
	 OLED_ShowString(0,20,"Current Mode Is");//��ǰ��ģʽ�ǣ�
	if(Flag_Way==0)         OLED_ShowString(50,30,"APP");//APPģʽ
	if(Flag_Way==1)         OLED_ShowString(50,30,"PS2");//PS2ģʽ
	if(Flag_Way==2)				  OLED_ShowString(50,30,"CCD");//CCDģʽ
	if(Flag_Way==3)				  OLED_ShowString(50,30,"ELE");//���Ѳ��ģʽ
	OLED_ShowString(0,40,"Press User Key");// ��һ���û�����
  OLED_ShowString(0,50,"TO End Selection");//����ѡ��
	OLED_Refresh_Gram();	//OLEDˢ��
	}

