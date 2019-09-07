#include "timer.h"
  /**************************************************************************
���ߣ�ƽ��С��֮��
�ҵ��Ա�С�꣺http://shop114407458.taobao.com/
**************************************************************************/
/**************************************************************************
�������ܣ���ʱ��5ͨ�����벶���ʼ��
��ڲ�������ڲ�����arr���Զ���װֵ  psc��ʱ��Ԥ��Ƶ�� 
����  ֵ����
**************************************************************************/
void TIM5_Cap_Init(u16 arr,u16 psc)	
{	 
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef  TIM_ICInitStructure;
 	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);	//ʹ��TIM5ʱ��
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC,ENABLE);  //ʹ��GPIOʱ��
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; //PA  ����  
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_4|GPIO_Pin_11;     
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;     //��� 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;     //2M
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_12;     
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;     //��� 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;     //2M
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_5;     
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;     //��� 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;     //2M
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	//��ʼ����ʱ��5	 
	TIM_TimeBaseStructure.TIM_Period = arr; //�趨�������Զ���װֵ 
	TIM_TimeBaseStructure.TIM_Prescaler =psc; 	//Ԥ��Ƶ��   
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
  
	//��ʼ��TIM5���벶�����
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1; //ѡ������� 
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//�����ز���
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //���������Ƶ,����Ƶ 
  TIM_ICInitStructure.TIM_ICFilter = 0x00;//���������˲��� ���˲�
  TIM_ICInit(TIM5, &TIM_ICInitStructure);
	
		//��ʼ��TIM5���벶�����
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2; //ѡ������� 
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//�����ز���
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //���������Ƶ,����Ƶ 
  TIM_ICInitStructure.TIM_ICFilter = 0x00;//���������˲��� ���˲�
  TIM_ICInit(TIM5, &TIM_ICInitStructure);
	
		//��ʼ��TIM5���벶�����
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_3; //ѡ������� 
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//�����ز���
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //���������Ƶ,����Ƶ 
  TIM_ICInitStructure.TIM_ICFilter = 0x00;//���������˲��� ���˲�
  TIM_ICInit(TIM5, &TIM_ICInitStructure);
	
		//��ʼ��TIM5���벶�����
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_4; //ѡ������� 
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//�����ز���
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //���������Ƶ,����Ƶ 
  TIM_ICInitStructure.TIM_ICFilter = 0x00;//���������˲��� ���˲�
  TIM_ICInit(TIM5, &TIM_ICInitStructure);
	
	//�жϷ����ʼ��
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;  //TIM�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //��ռ���ȼ�2��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //�����ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ��� 	
	TIM_ITConfig(TIM5,TIM_IT_Update|TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4,ENABLE);//��������ж� ,�������ж�	
  TIM_Cmd(TIM5,ENABLE ); 	//ʹ�ܶ�ʱ��
}
u16 TIM5CH1_CAPTURE_STA,TIM5CH1_CAPTURE_VAL,TIM5CH2_CAPTURE_STA,TIM5CH2_CAPTURE_VAL;
u16 TIM5CH3_CAPTURE_STA,TIM5CH3_CAPTURE_VAL,TIM5CH4_CAPTURE_STA,TIM5CH4_CAPTURE_VAL;
/**************************************************************************
�������ܣ����������ջز�����
��ڲ�������
����  ֵ����
**************************************************************************/
void Read_Distane(void)
{   
	 TRIPA=1;//�ߵ�ƽ����
	 delay_us(15);  //��ʱ
	 TRIPA=0;	//�͵�ƽ
		 if(TIM5CH1_CAPTURE_STA&0X80)//�ɹ�������һ�θߵ�ƽ
		{
			Distance_A=TIM5CH1_CAPTURE_STA&0X3F;
			Distance_A*=65536;	 //���ʱ���ܺ�
			Distance_A+=TIM5CH1_CAPTURE_VAL;//�õ��ܵĸߵ�ƽʱ��
			Distance_A=Distance_A*170/1000;//ת����mmΪ��λ�����������ڿ����д����ٶ�340m/s
			TIM5CH1_CAPTURE_STA=0;			//������һ�β���
		}	
		
	  TRIPB=1;//�ߵ�ƽ����
	  delay_us(15);  //��ʱ
	  TRIPB=0;	//�͵�ƽ
		 if(TIM5CH2_CAPTURE_STA&0X80)//�ɹ�������һ�θߵ�ƽ
		{
			Distance_B=TIM5CH2_CAPTURE_STA&0X3F;
			Distance_B*=65536;	 //���ʱ���ܺ�
			Distance_B+=TIM5CH2_CAPTURE_VAL;		//�õ��ܵĸߵ�ƽʱ��
			Distance_B=Distance_B*170/1000;//ת����mmΪ��λ�����������ڿ����д����ٶ�340m/s
			TIM5CH2_CAPTURE_STA=0;			//������һ�β���
		}		

	  TRIPC=1;//�ߵ�ƽ����
	  delay_us(15);  //��ʱ
	  TRIPC=0;	//�͵�ƽ
		 if(TIM5CH3_CAPTURE_STA&0X80)//�ɹ�������һ�θߵ�ƽ
		{
			Distance_C=TIM5CH3_CAPTURE_STA&0X3F;
			Distance_C*=65536;	 //���ʱ���ܺ�
			Distance_C+=TIM5CH3_CAPTURE_VAL;		//�õ��ܵĸߵ�ƽʱ��
			Distance_C=Distance_C*170/1000;//ת����mmΪ��λ�����������ڿ����д����ٶ�340m/s
			TIM5CH3_CAPTURE_STA=0;			//������һ�β���
		}		

	  TRIPD=1;//�ߵ�ƽ����
	  delay_us(15);  //��ʱ
	  TRIPD=0;	//�͵�ƽ
		 if(TIM5CH4_CAPTURE_STA&0X80)//�ɹ�������һ�θߵ�ƽ
		{
			Distance_D=TIM5CH4_CAPTURE_STA&0X3F;
			Distance_D*=65536;	 //���ʱ���ܺ�
			Distance_D+=TIM5CH4_CAPTURE_VAL;		//�õ��ܵĸߵ�ƽʱ��
			Distance_D=Distance_D*170/1000;//ת����mmΪ��λ�����������ڿ����д����ٶ�340m/s
			TIM5CH4_CAPTURE_STA=0;			//������һ�β���
		}				
}
/**************************************************************************
�������ܣ��������ز������ȡ�ж�
��ڲ�������
����  ֵ����
��    �ߣ�ƽ��С��֮��
**************************************************************************/
void TIM5_IRQHandler(void)
{ 		    		  			    
	u16 tsr;
	tsr=TIM5->SR;
				/////////////////////ͨ��һ///////////////////////////////
	if((TIM5CH1_CAPTURE_STA&0X80)==0)//��δ�ɹ�����	
				{
					if(tsr&0X01)//���
						{	     
								if(TIM5CH1_CAPTURE_STA&0X40)//�Ѿ����񵽸ߵ�ƽ��
								{
									if((TIM5CH1_CAPTURE_STA&0X3F)==0X3F)//�ߵ�ƽ̫����
									{
										TIM5CH1_CAPTURE_STA|=0X80;//��ǳɹ�������һ��
										TIM5CH1_CAPTURE_VAL=0XFFFF;
									}else TIM5CH1_CAPTURE_STA++;
								}	 
						}
						if(tsr&0x02)//����1���������¼�
					{	 	
									if(TIM5CH1_CAPTURE_STA&0X40)		//����һ���½��� 		
									{	  	
									TIM5CH1_CAPTURE_STA|=0X80;		//��ǳɹ�����һ�θߵ�ƽ����
									TIM5CH1_CAPTURE_VAL=TIM5->CCR1;	//��ȡ��ǰ�Ĳ���ֵ.
									TIM5->CCER&=~(1<<1);			//CC1P=0 ����Ϊ�����ز���
									}else  								//��δ��ʼ,��һ�β���������
									{	
									TIM5CH1_CAPTURE_STA=0;			//���
									TIM5CH1_CAPTURE_VAL=0;
									TIM5CH1_CAPTURE_STA|=0X40;		//��ǲ�����������
									TIM5->CNT=0;					//���������
									TIM5->CCER|=1<<1; 				//CC1P=1 ����Ϊ�½��ز���
									}		    
					 }			     	    					   
		   }
		 /////////////////////ͨ����///////////////////////////////
			if((TIM5CH2_CAPTURE_STA&0X80)==0)//��δ�ɹ�����	
				{
					if(tsr&0X01)//���
						{	     
								if(TIM5CH2_CAPTURE_STA&0X40)//�Ѿ����񵽸ߵ�ƽ��
								{
									if((TIM5CH2_CAPTURE_STA&0X3F)==0X3F)//�ߵ�ƽ̫����
									{
										TIM5CH2_CAPTURE_STA|=0X80;//��ǳɹ�������һ��
										TIM5CH2_CAPTURE_VAL=0XFFFF;
									}else TIM5CH2_CAPTURE_STA++;
								}	 
						}
						if(tsr&0x04)//����2���������¼�
					{	 	
									if(TIM5CH2_CAPTURE_STA&0X40)		//����һ���½��� 		
									{	  	
									TIM5CH2_CAPTURE_STA|=0X80;		//��ǳɹ�����һ�θߵ�ƽ����
									TIM5CH2_CAPTURE_VAL=TIM5->CCR2;	//��ȡ��ǰ�Ĳ���ֵ.
									TIM5->CCER&=~(1<<5);   //CC1P=0 ����Ϊ�����ز���
									}else  								//��δ��ʼ,��һ�β���������
									{	
									TIM5CH2_CAPTURE_STA=0;//���
									TIM5CH2_CAPTURE_VAL=0;
									TIM5CH2_CAPTURE_STA|=0X40;//��ǲ�����������
									TIM5->CNT=0;					//���������
									TIM5->CCER|=1<<5; 				//CC1P=1 ����Ϊ�½��ز���
										}		    
					 }			     	    					   
		   } 
			  /////////////////////ͨ����///////////////////////////////
			if((TIM5CH3_CAPTURE_STA&0X80)==0)//��δ�ɹ�����	
				{
					if(tsr&0X01)//���
						{	     
								if(TIM5CH3_CAPTURE_STA&0X40)//�Ѿ����񵽸ߵ�ƽ��
								{
									if((TIM5CH3_CAPTURE_STA&0X3F)==0X3F)//�ߵ�ƽ̫����
									{
										TIM5CH3_CAPTURE_STA|=0X80;//��ǳɹ�������һ��
										TIM5CH3_CAPTURE_VAL=0XFFFF;
									}else TIM5CH3_CAPTURE_STA++;
								}	 
						}
						if(tsr&0x08)//����3���������¼�
					{	 	
									if(TIM5CH3_CAPTURE_STA&0X40)		//����һ���½��� 		
									{	  	
									TIM5CH3_CAPTURE_STA|=0X80;		//��ǳɹ�����һ�θߵ�ƽ����
									TIM5CH3_CAPTURE_VAL=TIM5->CCR3;	//��ȡ��ǰ�Ĳ���ֵ.
									TIM5->CCER&=~(1<<9);   //CC1P=0 ����Ϊ�����ز���
									}else  								//��δ��ʼ,��һ�β���������
									{	
									TIM5CH3_CAPTURE_STA=0;//���
									TIM5CH3_CAPTURE_VAL=0;
									TIM5CH3_CAPTURE_STA|=0X40;//��ǲ�����������
									TIM5->CNT=0;					//���������
									TIM5->CCER|=1<<9; 				//CC1P=1 ����Ϊ�½��ز���
									}		    
					 }			     	    					   
		   } 
	    /////////////////////ͨ����///////////////////////////////
			if((TIM5CH4_CAPTURE_STA&0X80)==0)//��δ�ɹ�����	
				{
					if(tsr&0X01)//���
						{	     
								if(TIM5CH4_CAPTURE_STA&0X40)//�Ѿ����񵽸ߵ�ƽ��
								{
									if((TIM5CH4_CAPTURE_STA&0X3F)==0X3F)//�ߵ�ƽ̫����
									{
										TIM5CH4_CAPTURE_STA|=0X80;//��ǳɹ�������һ��
										TIM5CH4_CAPTURE_VAL=0XFFFF;
									}else TIM5CH4_CAPTURE_STA++;
								}	 
						}
						if(tsr&0x10)//����4���������¼�
					{	 	
									if(TIM5CH4_CAPTURE_STA&0X40)		//����һ���½��� 		
									{	  	
									TIM5CH4_CAPTURE_STA|=0X80;		//��ǳɹ�����һ�θߵ�ƽ����
									TIM5CH4_CAPTURE_VAL=TIM5->CCR4;	//��ȡ��ǰ�Ĳ���ֵ.
									TIM5->CCER&=~(1<<13);   //CC1P=0 ����Ϊ�����ز���
									}else  								//��δ��ʼ,��һ�β���������
									{	
									TIM5CH4_CAPTURE_STA=0;//���
									TIM5CH4_CAPTURE_VAL=0;
									TIM5CH4_CAPTURE_STA|=0X40;//��ǲ�����������
									TIM5->CNT=0;					//���������
									TIM5->CCER|=1<<13; 				//CC1P=1 ����Ϊ�½��ز���
									}		    
					 }			     	    					   
		   } 		 
			 TIM5->SR=0;//����жϱ�־λ 	     
}



u8 TIM3CH1_CAPTURE_STA = 0;	//ͨ��1���벶���־������λ�������־����6λ�������־		
u16 TIM3CH1_CAPTURE_UPVAL;
u16 TIM3CH1_CAPTURE_DOWNVAL;

u8 TIM3CH2_CAPTURE_STA = 0;	//ͨ��2���벶���־������λ�������־����6λ�������־		
u16 TIM3CH2_CAPTURE_UPVAL;
u16 TIM3CH2_CAPTURE_DOWNVAL;

u8 TIM3CH3_CAPTURE_STA = 0;	//ͨ��3���벶���־������λ�������־����6λ�������־		
u16 TIM3CH3_CAPTURE_UPVAL;
u16 TIM3CH3_CAPTURE_DOWNVAL;

u8 TIM3CH4_CAPTURE_STA = 0;	//ͨ��4���벶���־������λ�������־����6λ�������־		
u16 TIM3CH4_CAPTURE_UPVAL;
u16 TIM3CH4_CAPTURE_DOWNVAL;

u32 TIM3_T1;
u32 TIM3_T2;
u32 TIM3_T3;
u32 TIM3_T4;
//��ʱ��2���벶������
int pwmout1, pwmout2, pwmout3, pwmout4; 				//���ռ�ձ�
/**************************************************************************
�������ܣ���ģң�س�ʼ������
��ڲ�����arr���Զ���װֵ  psc��ʱ��Ԥ��Ƶ�� 
����  ֵ����
**************************************************************************/
void TIM3_Cap_Init(u16 arr, u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
  TIM_ICInitTypeDef TIM_ICInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);	//ʹ��TIM3ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB, ENABLE);  //ʹ��GPIOA Bʱ��

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 ; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; //���� 
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOA, GPIO_Pin_6 | GPIO_Pin_7);//PA6 7  ����   ����

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0| GPIO_Pin_1 ; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; //���� 
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOB, GPIO_Pin_0 | GPIO_Pin_1);//PA0 1  ����   ����
	//��ʼ����ʱ�� TIM3	 
	TIM_TimeBaseStructure.TIM_Period = arr; //�趨�������Զ���װֵ 
	TIM_TimeBaseStructure.TIM_Prescaler = psc; 	//Ԥ��Ƶ�� 
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ

	//��ʼ��TIM3���벶����� ͨ��1
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1; //CC1S=01 	ѡ������� 
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//�����ز���
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	  //���������Ƶ,����Ƶ 
	TIM_ICInitStructure.TIM_ICFilter = 0x0F;	  //IC1F=0000 ���������˲��� ���˲�
	TIM_ICInit(TIM3, &TIM_ICInitStructure);

	//��ʼ��TIM3���벶����� ͨ��2
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2; //CC1S=01 	ѡ�������  
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//�����ز���
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	  //���������Ƶ,����Ƶ 
	TIM_ICInitStructure.TIM_ICFilter = 0x00;	  //IC1F=0000 ���������˲��� ���˲�
	TIM_ICInit(TIM3, &TIM_ICInitStructure);

	//��ʼ��TIM3���벶����� ͨ��3
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_3; //CC1S=01 	ѡ�������  
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//�����ز���
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	  //���������Ƶ,����Ƶ 
	TIM_ICInitStructure.TIM_ICFilter = 0x00;	  //IC1F=0000 ���������˲��� ���˲�
	TIM_ICInit(TIM3, &TIM_ICInitStructure);

	//��ʼ��TIM3���벶����� ͨ��4
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_4; //CC1S=01 	ѡ������� 
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//�����ز���
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	  //���������Ƶ,����Ƶ 
	TIM_ICInitStructure.TIM_ICFilter = 0x00;	  //IC1F=0000 ���������˲��� ���˲�
	TIM_ICInit(TIM3, &TIM_ICInitStructure);

	//�жϷ����ʼ��
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //��ռ���ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //�����ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);   //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ��� 
	TIM_ITConfig(TIM3, TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4,	ENABLE);   //����������жϣ�����CC1IE,CC2IE,CC3IE,CC4IE�����ж�	
	TIM_Cmd(TIM3, ENABLE); 		//ʹ�ܶ�ʱ��

}

/**************************************************************************
�������ܣ���ģң�ؽ����ж�
��ڲ�������
����  ֵ����
**************************************************************************/
void TIM3_IRQHandler(void)
{
	  /////////////////////ͨ��һ///////////////////////////////
	if ((TIM3CH1_CAPTURE_STA & 0X80) == 0) 		//��δ�ɹ�����	
	{
		if (TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET) 		//����1���������¼�
		{
			TIM_ClearITPendingBit(TIM3, TIM_IT_CC1); 		//����жϱ�־λ
			if (TIM3CH1_CAPTURE_STA & 0X40)		//����һ���½���
			{
				TIM3CH1_CAPTURE_DOWNVAL = TIM_GetCapture1(TIM3);//��¼�´�ʱ�Ķ�ʱ������ֵ
				if (TIM3CH1_CAPTURE_DOWNVAL < TIM3CH1_CAPTURE_UPVAL)
				{
					TIM3_T1 = 65535;
				}
				else
					TIM3_T1 = 0;
				Remoter_Ch1 = TIM3CH1_CAPTURE_DOWNVAL - TIM3CH1_CAPTURE_UPVAL
						+ TIM3_T1;		//�õ��ܵĸߵ�ƽ��ʱ��
				pwmout1 = Remoter_Ch1;		//�ܵĸߵ�ƽ��ʱ��
				TIM3CH1_CAPTURE_STA = 0;		//�����־λ����
				TIM_OC1PolarityConfig(TIM3, TIM_ICPolarity_Rising); //����Ϊ�����ز���		  
			}
			else //��������ʱ�䵫�����½��أ���һ�β��������أ���¼��ʱ�Ķ�ʱ������ֵ
			{
				TIM3CH1_CAPTURE_UPVAL = TIM_GetCapture1(TIM3);		//��ȡ����������
				TIM3CH1_CAPTURE_STA |= 0X40;		//����Ѳ���������
				TIM_OC1PolarityConfig(TIM3, TIM_ICPolarity_Falling);//����Ϊ�½��ز���
			}
		}
	}
  /////////////////////ͨ����///////////////////////////////
	if ((TIM3CH2_CAPTURE_STA & 0X80) == 0)		//��δ�ɹ�����	
	{
		if (TIM_GetITStatus(TIM3, TIM_IT_CC2) != RESET)		//����2���������¼�
		{
			TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);		//����жϱ�־λ
			if (TIM3CH2_CAPTURE_STA & 0X40)		//����һ���½���
			{
				TIM3CH2_CAPTURE_DOWNVAL = TIM_GetCapture2(TIM3);//��¼�´�ʱ�Ķ�ʱ������ֵ
				if (TIM3CH2_CAPTURE_DOWNVAL < TIM3CH2_CAPTURE_UPVAL)
				{
					TIM3_T2 = 65535;
				}
				else
					TIM3_T2 = 0;
				Remoter_Ch2 = TIM3CH2_CAPTURE_DOWNVAL - TIM3CH2_CAPTURE_UPVAL
						+ TIM3_T2;		//�õ��ܵĸߵ�ƽ��ʱ��
				pwmout2 = Remoter_Ch2;		//�ܵĸߵ�ƽ��ʱ��
				TIM3CH2_CAPTURE_STA = 0;		//�����־λ����
				TIM_OC2PolarityConfig(TIM3, TIM_ICPolarity_Rising); //����Ϊ�����ز���		  
			}
			else //��������ʱ�䵫�����½��أ���һ�β��������أ���¼��ʱ�Ķ�ʱ������ֵ
			{
				TIM3CH2_CAPTURE_UPVAL = TIM_GetCapture2(TIM3);		//��ȡ����������
				TIM3CH2_CAPTURE_STA |= 0X40;		//����Ѳ���������
				TIM_OC2PolarityConfig(TIM3, TIM_ICPolarity_Falling);//����Ϊ�½��ز���
			}
		}
	}
  /////////////////////ͨ����///////////////////////////////
	if ((TIM3CH3_CAPTURE_STA & 0X80) == 0)		//��δ�ɹ�����	
	{
		if (TIM_GetITStatus(TIM3, TIM_IT_CC3) != RESET)		//����3���������¼�
		{
			TIM_ClearITPendingBit(TIM3, TIM_IT_CC3);		//����жϱ�־λ
			if (TIM3CH3_CAPTURE_STA & 0X40)		//����һ���½���
			{
				TIM3CH3_CAPTURE_DOWNVAL = TIM_GetCapture3(TIM3);//��¼�´�ʱ�Ķ�ʱ������ֵ
				if (TIM3CH3_CAPTURE_DOWNVAL < TIM3CH3_CAPTURE_UPVAL)
				{
					TIM3_T3 = 65535;
				}
				else
					TIM3_T3 = 0;
				Remoter_Ch3 = TIM3CH3_CAPTURE_DOWNVAL - TIM3CH3_CAPTURE_UPVAL
						+ TIM3_T3;		//�õ��ܵĸߵ�ƽ��ʱ��
				pwmout3 = Remoter_Ch3;		//�ܵĸߵ�ƽ��ʱ��
				TIM3CH3_CAPTURE_STA = 0;		//�����־λ����
				TIM_OC3PolarityConfig(TIM3, TIM_ICPolarity_Rising); //����Ϊ�����ز���		  
			}
			else //��������ʱ�䵫�����½��أ���һ�β��������أ���¼��ʱ�Ķ�ʱ������ֵ
			{
				TIM3CH3_CAPTURE_UPVAL = TIM_GetCapture3(TIM3);		//��ȡ����������
				TIM3CH3_CAPTURE_STA |= 0X40;		//����Ѳ���������
				TIM_OC3PolarityConfig(TIM3, TIM_ICPolarity_Falling);//����Ϊ�½��ز���
			}
		}
	}
  /////////////////////ͨ����///////////////////////////////
	if ((TIM3CH4_CAPTURE_STA & 0X80) == 0)		//��δ�ɹ�����	
	{
		if (TIM_GetITStatus(TIM3, TIM_IT_CC4) != RESET)		//����4���������¼�
		{
			TIM_ClearITPendingBit(TIM3, TIM_IT_CC4);		//����жϱ�־λ
			if (TIM3CH4_CAPTURE_STA & 0X40)		//����һ���½���
			{
				TIM3CH4_CAPTURE_DOWNVAL = TIM_GetCapture4(TIM3);//��¼�´�ʱ�Ķ�ʱ������ֵ
				if (TIM3CH4_CAPTURE_DOWNVAL < TIM3CH4_CAPTURE_UPVAL)
				{
					TIM3_T4 = 65535;
				}
				else
					TIM3_T4 = 0;
				Remoter_Ch4 = TIM3CH4_CAPTURE_DOWNVAL - TIM3CH4_CAPTURE_UPVAL
						+ TIM3_T4;		//�õ��ܵĸߵ�ƽ��ʱ��
				pwmout4 = Remoter_Ch4;		//�ܵĸߵ�ƽ��ʱ��
				TIM3CH4_CAPTURE_STA = 0;		//�����־λ����
				TIM_OC4PolarityConfig(TIM3, TIM_ICPolarity_Rising); //����Ϊ�����ز���		  
			}
			else //��������ʱ�䵫�����½��أ���һ�β��������أ���¼��ʱ�Ķ�ʱ������ֵ
			{
				TIM3CH4_CAPTURE_UPVAL = TIM_GetCapture4(TIM3);		//��ȡ����������
				TIM3CH4_CAPTURE_STA |= 0X40;		//����Ѳ���������
				TIM_OC4PolarityConfig(TIM3, TIM_ICPolarity_Falling);//����Ϊ�½��ز���
			}
		}
	}
}


