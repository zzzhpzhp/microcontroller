#include "timer.h"
#include "ucos_ii.h"
/************************************************���PWM����************************************************/


rc_st rc;

void Tim1_init(void);
void Tim4_init(void);  
void Tim2_init(void);
void MotoTimerInit(void)
{
	Tim4_init();// B6 B7 B8 B9
//	Tim1_init();// A8 A11
//	Tim2_init();// A0 A1 A2 A3
}
void MotoPwmRefresh(u16 Motor1Pwm, u16 Motor2Pwm, u16 Motor3Pwm, u16 Motor4Pwm, u16 Motor5Pwm, u16 Motor6Pwm)
{        
    if (Motor1Pwm >= MotoPwmMax)    Motor1Pwm = MotoPwmMax; 
        else if(Motor1Pwm <= MotoPwmMin)    Motor1Pwm = MotoPwmMin;
    
    if (Motor2Pwm >= MotoPwmMax)    Motor2Pwm = MotoPwmMax; 
        else if(Motor2Pwm <= MotoPwmMin)    Motor2Pwm = MotoPwmMin;
    
    if (Motor3Pwm >= MotoPwmMax)    Motor3Pwm = MotoPwmMax; 
        else if(Motor3Pwm <= MotoPwmMin)    Motor3Pwm = MotoPwmMin;
    
    if (Motor4Pwm >= MotoPwmMax)    Motor4Pwm = MotoPwmMax; 
        else if(Motor4Pwm <= MotoPwmMin)    Motor4Pwm = MotoPwmMin;
    
    if (Motor5Pwm >= MotoPwmMax)    Motor5Pwm = MotoPwmMax; 
        else if(Motor5Pwm <= MotoPwmMin)    Motor5Pwm = MotoPwmMin;
    
    if (Motor6Pwm >= MotoPwmMax)    Motor6Pwm = MotoPwmMax; 
        else if(Motor6Pwm <= MotoPwmMin)    Motor6Pwm = MotoPwmMin;
    
    TIM1->CCR1 = Motor1Pwm;
    TIM1->CCR4 = Motor2Pwm;
    TIM4->CCR1 = Motor3Pwm;
    TIM4->CCR2 = Motor4Pwm;
    TIM4->CCR3 = Motor5Pwm;
    TIM4->CCR4 = Motor6Pwm;
}

/*************************************************��ʱ��1*************************************************/
void Tim1_init(void)
{
    TIM_TimeBaseInitTypeDef        TIM_TimeBaseStructure;
    TIM_OCInitTypeDef                  TIM_OCInitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    uint16_t PrescalerValue = 0;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE); //GPIOAʱ��ʹ��
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_11 ; 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //IO����50MHz
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //���ù������
    GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);//ʹ�ܶ�ʱ��2ʱ��
    PrescalerValue = (uint16_t) 7;//�����Ƶϵ��
    TIM_TimeBaseStructure.TIM_Period = TIMER_CYCLE;    //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ
    TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;    
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;//����ʱ�ӷָ�    
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;    //���������ϼ���
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);//��ʱ��2��ʼ��

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;//TIM_CNT<TIMX_CCRXʱ�����Ϊ��Ч��ƽ
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_OCInitStructure.TIM_Pulse = MotoPwmMin;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

    TIM_OC1Init(TIM1, &TIM_OCInitStructure);//��ʼ��TIM_OC1
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);//ʹ��Ԥװ��

    TIM_OC4Init(TIM1, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM1, ENABLE);
    TIM_Cmd(TIM1, ENABLE);//ʹ�ܶ�ʱ��1
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
}
/*************************************************��ʱ��4*************************************************/
void Tim4_init(void)
{
    TIM_TimeBaseInitTypeDef        TIM_TimeBaseStructure;
    TIM_OCInitTypeDef                  TIM_OCInitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    uint16_t PrescalerValue = 0;
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE); //GPIOAʱ��ʹ��
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 ; 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //IO����50MHz
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //���ù������
    GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);//ʹ�ܶ�ʱ��2ʱ��
    PrescalerValue = (uint16_t) 7;//�����Ƶϵ��
    TIM_TimeBaseStructure.TIM_Period = TIMER_CYCLE;    //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ
    TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;    
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;//����ʱ�ӷָ�    
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;    //���������ϼ���
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);//��ʱ��2��ʼ��
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;//TIM_CNT<TIMX_CCRXʱ�����Ϊ��Ч��ƽ
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;//�Ƚ����ʹ��
    TIM_OCInitStructure.TIM_Pulse = MotoPwmMin;//��ʼռ�ձ�Ϊ
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;//������Ч��ƽΪ�ߵ�ƽ�����PWMģʽ�������õ���PWM1
    
    TIM_OC1Init(TIM4, &TIM_OCInitStructure);//��ʼ��TIM_OC1
    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);//ʹ��Ԥװ��
    
    TIM_OC2Init(TIM4, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
    
    TIM_OC3Init(TIM4, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
    
    TIM_OC4Init(TIM4, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
    
    TIM_ARRPreloadConfig(TIM4, ENABLE);
    TIM_Cmd(TIM4, ENABLE);//ʹ�ܶ�ʱ��2
}

void Tim2_init(void)
{
    TIM_TimeBaseInitTypeDef        TIM_TimeBaseStructure;
    TIM_OCInitTypeDef                  TIM_OCInitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    uint16_t PrescalerValue = 0;
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);    //ʹ��TIM2ʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE); //GPIOAʱ��ʹ��
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 ; 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //IO����50MHz
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //���ù������
    GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);//ʹ�ܶ�ʱ��2ʱ��
    PrescalerValue = (uint16_t) 7;//�����Ƶϵ��
    TIM_TimeBaseStructure.TIM_Period = TIMER_CYCLE;    //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ
    TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;    
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;//����ʱ�ӷָ�    
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;    //���������ϼ���
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);//��ʱ��2��ʼ��
	
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;//TIM_CNT<TIMX_CCRXʱ�����Ϊ��Ч��ƽ
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;//�Ƚ����ʹ��
    TIM_OCInitStructure.TIM_Pulse = MotoPwmMin;//��ʼռ�ձ�Ϊ
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;//������Ч��ƽΪ�ߵ�ƽ�����PWMģʽ�������õ���PWM1
    
    TIM_OC1Init(TIM2, &TIM_OCInitStructure);//��ʼ��TIM_OC1
    TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);//ʹ��Ԥװ��
    
    TIM_OC2Init(TIM2, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
    
    TIM_OC3Init(TIM2, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
    
    TIM_OC4Init(TIM2, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);
    
    TIM_ARRPreloadConfig(TIM2, ENABLE);
    TIM_Cmd(TIM2, ENABLE);//ʹ�ܶ�ʱ��2
}




/*************************************************��ʱ��2*************************************************/
//��ʱ��2���벶������

u32 tim2_T1;
u32 tim2_T2;
u32 tim2_T3;
u32 tim2_T4;

u8 TIM2CH1_CAPTURE_STA = 0;    //ͨ��1���벶���־������λ�������־����6λ�������־        
u16 TIM2CH1_CAPTURE_UPVAL;
u16 TIM2CH1_CAPTURE_DOWNVAL;

u8 TIM2CH2_CAPTURE_STA = 0;    //ͨ��2���벶���־������λ�������־����6λ�������־        
u16 TIM2CH2_CAPTURE_UPVAL;
u16 TIM2CH2_CAPTURE_DOWNVAL;

u8 TIM2CH3_CAPTURE_STA = 0;    //ͨ��3���벶���־������λ�������־����6λ�������־        
u16 TIM2CH3_CAPTURE_UPVAL;
u16 TIM2CH3_CAPTURE_DOWNVAL;

u8 TIM2CH4_CAPTURE_STA = 0;    //ͨ��1���벶���־������λ�������־����6λ�������־        
u16 TIM2CH4_CAPTURE_UPVAL;
u16 TIM2CH4_CAPTURE_DOWNVAL;

TIM_ICInitTypeDef TIM2_ICInitStructure;

void TIM2_Cap_Init(u16 arr, u16 psc) //��STM32F103CBT6��Ƭ����ʹ����������ܹ���ȷ��ʼ��TIM2,���������ͺŵ�Ƭ������Ҫ�鿴DATASHEET�����ܻ���TIM5��ͻ
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);    //ʹ��TIM2ʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  //ʹ��GPIOBʱ��

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2
            | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOA, GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3);//PB6,7,8,9  ����

    //��ʼ����ʱ��4 TIM2     
    TIM_TimeBaseStructure.TIM_Period = arr; //�趨�������Զ���װֵ 
    TIM_TimeBaseStructure.TIM_Prescaler = psc;     //Ԥ��Ƶ�� 
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�:TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ

    //��ʼ��TIM2���벶����� ͨ��1
    TIM2_ICInitStructure.TIM_Channel = TIM_Channel_1; //CC1S=01     ѡ������� IC1ӳ�䵽TI1��
    TIM2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;    //�����ز���
    TIM2_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
    TIM2_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;      //���������Ƶ,����Ƶ 
    TIM2_ICInitStructure.TIM_ICFilter = 0x00;      //IC1F=0000 ���������˲��� ���˲�
    TIM_ICInit(TIM2, &TIM2_ICInitStructure);

    //��ʼ��TIM2���벶����� ͨ��2
    TIM2_ICInitStructure.TIM_Channel = TIM_Channel_2; //CC1S=01     ѡ������� IC1ӳ�䵽TI1��
    TIM2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;    //�����ز���
    TIM2_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
    TIM2_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;      //���������Ƶ,����Ƶ 
    TIM2_ICInitStructure.TIM_ICFilter = 0x00;      //IC1F=0000 ���������˲��� ���˲�
    TIM_ICInit(TIM2, &TIM2_ICInitStructure);

    //��ʼ��TIM2���벶����� ͨ��3
    TIM2_ICInitStructure.TIM_Channel = TIM_Channel_3; //CC1S=01     ѡ������� IC1ӳ�䵽TI1��
    TIM2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;    //�����ز���
    TIM2_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
    TIM2_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;      //���������Ƶ,����Ƶ 
    TIM2_ICInitStructure.TIM_ICFilter = 0x00;      //IC1F=0000 ���������˲��� ���˲�
    TIM_ICInit(TIM2, &TIM2_ICInitStructure);

    //��ʼ��TIM2���벶����� ͨ��4
    TIM2_ICInitStructure.TIM_Channel = TIM_Channel_4; //CC1S=01     ѡ������� IC1ӳ�䵽TI1��
    TIM2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;    //�����ز���
    TIM2_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
    TIM2_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;      //���������Ƶ,����Ƶ 
    TIM2_ICInitStructure.TIM_ICFilter = 0x00;      //IC1F=0000 ���������˲��� ���˲�
    TIM_ICInit(TIM2, &TIM2_ICInitStructure);

    //�жϷ����ʼ��
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;  //TIM2�ж�
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //��ռ���ȼ�1��
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //�����ȼ�0��
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
    NVIC_Init(&NVIC_InitStructure);   //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ��� 

    TIM_ITConfig(TIM2,TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4,
            ENABLE);   //����������жϣ�����CC1IE,CC2IE,CC3IE,CC4IE�����ж�    

    TIM_Cmd(TIM2, ENABLE);         //ʹ�ܶ�ʱ��4

}

//��ʱ��5�жϷ������
void TIM2_IRQHandler(void)
{
    OSIntEnter();
    if ((TIM2CH1_CAPTURE_STA & 0X80) == 0)         //��δ�ɹ�����    
    {
        if (TIM_GetITStatus(TIM2, TIM_IT_CC1) != RESET)         //����1���������¼�
        {
            TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);         //����жϱ�־λ
            if (TIM2CH1_CAPTURE_STA & 0X40)        //����һ���½���
            {
                TIM2CH1_CAPTURE_DOWNVAL = TIM_GetCapture1(TIM2);//��¼�´�ʱ�Ķ�ʱ������ֵ
                if (TIM2CH1_CAPTURE_DOWNVAL < TIM2CH1_CAPTURE_UPVAL)
                {
                    tim2_T1 = 65535;
                }
                else
                    tim2_T1 = 0;
                rc.Tim2Ch1CapVal = TIM2CH1_CAPTURE_DOWNVAL - TIM2CH1_CAPTURE_UPVAL
                        + tim2_T1;        //�õ��ܵĸߵ�ƽ��ʱ��
                TIM2CH1_CAPTURE_STA = 0;        //�����־λ����
                TIM_OC1PolarityConfig(TIM2, TIM_ICPolarity_Rising); //����Ϊ�����ز���          
            }
            else //��������ʱ�䵫�����½��أ���һ�β��������أ���¼��ʱ�Ķ�ʱ������ֵ
            {
                TIM2CH1_CAPTURE_UPVAL = TIM_GetCapture1(TIM2);        //��ȡ����������
                TIM2CH1_CAPTURE_STA |= 0X40;        //����Ѳ���������
                TIM_OC1PolarityConfig(TIM2, TIM_ICPolarity_Falling);//����Ϊ�½��ز���
            }
        }
    }

    if ((TIM2CH2_CAPTURE_STA & 0X80) == 0)        //��δ�ɹ�����    
    {
        if (TIM_GetITStatus(TIM2, TIM_IT_CC2) != RESET)        //����2���������¼�
        {
            TIM_ClearITPendingBit(TIM2, TIM_IT_CC2);        //����жϱ�־λ
            if (TIM2CH2_CAPTURE_STA & 0X40)        //����һ���½���
            {
                TIM2CH2_CAPTURE_DOWNVAL = TIM_GetCapture2(TIM2);//��¼�´�ʱ�Ķ�ʱ������ֵ
                if (TIM2CH2_CAPTURE_DOWNVAL < TIM2CH2_CAPTURE_UPVAL)
                {
                    tim2_T2 = 65535;
                }
                else
                    tim2_T2 = 0;
                rc.Tim2Ch2CapVal = TIM2CH2_CAPTURE_DOWNVAL - TIM2CH2_CAPTURE_UPVAL
                        + tim2_T2;        //�õ��ܵĸߵ�ƽ��ʱ��
                TIM2CH2_CAPTURE_STA = 0;        //�����־λ����
                TIM_OC2PolarityConfig(TIM2, TIM_ICPolarity_Rising); //����Ϊ�����ز���          
            }
            else //��������ʱ�䵫�����½��أ���һ�β��������أ���¼��ʱ�Ķ�ʱ������ֵ
            {
                TIM2CH2_CAPTURE_UPVAL = TIM_GetCapture2(TIM2);        //��ȡ����������
                TIM2CH2_CAPTURE_STA |= 0X40;        //����Ѳ���������
                TIM_OC2PolarityConfig(TIM2, TIM_ICPolarity_Falling);//����Ϊ�½��ز���
            }
        }
    }

    if ((TIM2CH3_CAPTURE_STA & 0X80) == 0)        //��δ�ɹ�����    
    {
        if (TIM_GetITStatus(TIM2, TIM_IT_CC3) != RESET)        //����3���������¼�
        {
            TIM_ClearITPendingBit(TIM2, TIM_IT_CC3);        //����жϱ�־λ
            if (TIM2CH3_CAPTURE_STA & 0X40)        //����һ���½���
            {
                TIM2CH3_CAPTURE_DOWNVAL = TIM_GetCapture3(TIM2);//��¼�´�ʱ�Ķ�ʱ������ֵ
                if (TIM2CH3_CAPTURE_DOWNVAL < TIM2CH3_CAPTURE_UPVAL)
                {
                    tim2_T3 = 65535;
                }
                else
                    tim2_T3 = 0;
                rc.Tim2Ch3CapVal = TIM2CH3_CAPTURE_DOWNVAL - TIM2CH3_CAPTURE_UPVAL
                        + tim2_T3;        //�õ��ܵĸߵ�ƽ��ʱ��
                TIM2CH3_CAPTURE_STA = 0;        //�����־λ����
                TIM_OC3PolarityConfig(TIM2, TIM_ICPolarity_Rising); //����Ϊ�����ز���          
            }
            else //��������ʱ�䵫�����½��أ���һ�β��������أ���¼��ʱ�Ķ�ʱ������ֵ
            {
                TIM2CH3_CAPTURE_UPVAL = TIM_GetCapture3(TIM2);        //��ȡ����������
                TIM2CH3_CAPTURE_STA |= 0X40;        //����Ѳ���������
                TIM_OC3PolarityConfig(TIM2, TIM_ICPolarity_Falling);//����Ϊ�½��ز���
            }
        }
    }

    if ((TIM2CH4_CAPTURE_STA & 0X80) == 0)        //��δ�ɹ�����    
    {
        if (TIM_GetITStatus(TIM2, TIM_IT_CC4) != RESET)        //����4���������¼�
        {
            TIM_ClearITPendingBit(TIM2, TIM_IT_CC4);        //����жϱ�־λ
            if (TIM2CH4_CAPTURE_STA & 0X40)        //����һ���½���
            {
                TIM2CH4_CAPTURE_DOWNVAL = TIM_GetCapture4(TIM2);//��¼�´�ʱ�Ķ�ʱ������ֵ
                if (TIM2CH4_CAPTURE_DOWNVAL < TIM2CH4_CAPTURE_UPVAL)
                {
                    tim2_T4 = 65535;
                }
                else
                    tim2_T4 = 0;
                rc.Tim2Ch4CapVal = TIM2CH4_CAPTURE_DOWNVAL - TIM2CH4_CAPTURE_UPVAL
                        + tim2_T4;        //�õ��ܵĸߵ�ƽ��ʱ��
                TIM2CH4_CAPTURE_STA = 0;        //�����־λ����
                TIM_OC4PolarityConfig(TIM2, TIM_ICPolarity_Rising); //����Ϊ�����ز���          
            }
            else //��������ʱ�䵫�����½��أ���һ�β��������أ���¼��ʱ�Ķ�ʱ������ֵ
            {
                TIM2CH4_CAPTURE_UPVAL = TIM_GetCapture4(TIM2);        //��ȡ����������
                TIM2CH4_CAPTURE_STA |= 0X40;        //����Ѳ���������
                TIM_OC4PolarityConfig(TIM2, TIM_ICPolarity_Falling);//����Ϊ�½��ز���
            }
        }
    }
    OSIntExit();
}

/*************************************************��ʱ��3*************************************************/

//��ʱ��3���벶������



u32 tim3_T1;
u32 tim3_T2;
u32 tim3_T3;
u32 tim3_T4;

u8 TIM3CH1_CAPTURE_STA = 0;    //ͨ��1���벶���־������λ�������־����6λ�������־        
u16 TIM3CH1_CAPTURE_UPVAL;
u16 TIM3CH1_CAPTURE_DOWNVAL;

u8 TIM3CH2_CAPTURE_STA = 0;    //ͨ��2���벶���־������λ�������־����6λ�������־        
u16 TIM3CH2_CAPTURE_UPVAL;
u16 TIM3CH2_CAPTURE_DOWNVAL;

u8 TIM3CH3_CAPTURE_STA = 0;    //ͨ��3���벶���־������λ�������־����6λ�������־        
u16 TIM3CH3_CAPTURE_UPVAL;
u16 TIM3CH3_CAPTURE_DOWNVAL;

u8 TIM3CH4_CAPTURE_STA = 0;    //ͨ��1���벶���־������λ�������־����6λ�������־        
u16 TIM3CH4_CAPTURE_UPVAL;
u16 TIM3CH4_CAPTURE_DOWNVAL;

TIM_ICInitTypeDef TIM3_ICInitStructure;

void TIM3_Cap_Init(u16 arr, u16 psc) //��STM32F103CBT6��Ƭ����ʹ����������ܹ���ȷ��ʼ��TIM3,���������ͺŵ�Ƭ������Ҫ�鿴DATASHEET�����ܻ���TIM5��ͻ
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);    //ʹ��TIM3ʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);  //ʹ��GPIOBʱ��

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 ;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOA, GPIO_Pin_6 | GPIO_Pin_7);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOB, GPIO_Pin_0 | GPIO_Pin_1);
    
    //��ʼ����ʱ��4 TIM3     
    TIM_TimeBaseStructure.TIM_Period = arr; //�趨�������Զ���װֵ 
    TIM_TimeBaseStructure.TIM_Prescaler = psc;     //Ԥ��Ƶ�� 
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�:TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ

    //��ʼ��TIM3���벶����� ͨ��1
    TIM3_ICInitStructure.TIM_Channel = TIM_Channel_1; //CC1S=01     ѡ������� IC1ӳ�䵽TI1��
    TIM3_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;    //�����ز���
    TIM3_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
    TIM3_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;      //���������Ƶ,����Ƶ 
    TIM3_ICInitStructure.TIM_ICFilter = 0x00;      //IC1F=0000 ���������˲��� ���˲�
    TIM_ICInit(TIM3, &TIM3_ICInitStructure);

    //��ʼ��TIM3���벶����� ͨ��2
    TIM3_ICInitStructure.TIM_Channel = TIM_Channel_2; //CC1S=01     ѡ������� IC1ӳ�䵽TI1��
    TIM3_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;    //�����ز���
    TIM3_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
    TIM3_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;      //���������Ƶ,����Ƶ 
    TIM3_ICInitStructure.TIM_ICFilter = 0x00;      //IC1F=0000 ���������˲��� ���˲�
    TIM_ICInit(TIM3, &TIM3_ICInitStructure);

    //��ʼ��TIM3���벶����� ͨ��3
    TIM3_ICInitStructure.TIM_Channel = TIM_Channel_3; //CC1S=01     ѡ������� IC1ӳ�䵽TI1��
    TIM3_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;    //�����ز���
    TIM3_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
    TIM3_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;      //���������Ƶ,����Ƶ 
    TIM3_ICInitStructure.TIM_ICFilter = 0x00;      //IC1F=0000 ���������˲��� ���˲�
    TIM_ICInit(TIM3, &TIM3_ICInitStructure);

    //��ʼ��TIM3���벶����� ͨ��4
    TIM3_ICInitStructure.TIM_Channel = TIM_Channel_4; //CC1S=01     ѡ������� IC1ӳ�䵽TI1��
    TIM3_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;    //�����ز���
    TIM3_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
    TIM3_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;      //���������Ƶ,����Ƶ 
    TIM3_ICInitStructure.TIM_ICFilter = 0x00;      //IC1F=0000 ���������˲��� ���˲�
    TIM_ICInit(TIM3, &TIM3_ICInitStructure);

    //�жϷ����ʼ��
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3�ж�
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //��ռ���ȼ�1��
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //�����ȼ�0��
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
    NVIC_Init(&NVIC_InitStructure);   //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ��� 

    TIM_ITConfig(TIM3,TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4,
            ENABLE);   //����������жϣ�����CC1IE,CC2IE,CC3IE,CC4IE�����ж�    

    TIM_Cmd(TIM3, ENABLE);         //ʹ�ܶ�ʱ��4

}

/*************************************************��ʱ��5*************************************************/
//��ʱ��5�жϷ������
void TIM3_IRQHandler(void)
{
    OSIntEnter();
    if ((TIM3CH1_CAPTURE_STA & 0X80) == 0)         //��δ�ɹ�����    
    {
        if (TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET)         //����1���������¼�
        {
            TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);         //����жϱ�־λ
            if (TIM3CH1_CAPTURE_STA & 0X40)        //����һ���½���
            {
                TIM3CH1_CAPTURE_DOWNVAL = TIM_GetCapture1(TIM3);//��¼�´�ʱ�Ķ�ʱ������ֵ
                if (TIM3CH1_CAPTURE_DOWNVAL < TIM3CH1_CAPTURE_UPVAL)
                {
                    tim3_T1 = 65535;
                }
                else
                    tim3_T1 = 0;
                rc.Tim3Ch1CapVal = TIM3CH1_CAPTURE_DOWNVAL - TIM3CH1_CAPTURE_UPVAL
                        + tim3_T1;        //�õ��ܵĸߵ�ƽ��ʱ��
                TIM3CH1_CAPTURE_STA = 0;        //�����־λ����
                TIM_OC1PolarityConfig(TIM3, TIM_ICPolarity_Rising); //����Ϊ�����ز���          
            }
            else //��������ʱ�䵫�����½��أ���һ�β��������أ���¼��ʱ�Ķ�ʱ������ֵ
            {
                TIM3CH1_CAPTURE_UPVAL = TIM_GetCapture1(TIM3);        //��ȡ����������
                TIM3CH1_CAPTURE_STA |= 0X40;        //����Ѳ���������
                TIM_OC1PolarityConfig(TIM3, TIM_ICPolarity_Falling);//����Ϊ�½��ز���
            }
        }
    }

    if ((TIM3CH2_CAPTURE_STA & 0X80) == 0)        //��δ�ɹ�����    
    {
        if (TIM_GetITStatus(TIM3, TIM_IT_CC2) != RESET)        //����2���������¼�
        {
            TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);        //����жϱ�־λ
            if (TIM3CH2_CAPTURE_STA & 0X40)        //����һ���½���
            {
                TIM3CH2_CAPTURE_DOWNVAL = TIM_GetCapture2(TIM3);//��¼�´�ʱ�Ķ�ʱ������ֵ
                if (TIM3CH2_CAPTURE_DOWNVAL < TIM3CH2_CAPTURE_UPVAL)
                {
                    tim3_T2 = 65535;
                }
                else
                    tim3_T2 = 0;
                rc.Tim3Ch2CapVal = TIM3CH2_CAPTURE_DOWNVAL - TIM3CH2_CAPTURE_UPVAL
                        + tim3_T2;        //�õ��ܵĸߵ�ƽ��ʱ��
                TIM3CH2_CAPTURE_STA = 0;        //�����־λ����
                TIM_OC2PolarityConfig(TIM3, TIM_ICPolarity_Rising); //����Ϊ�����ز���          
            }
            else //��������ʱ�䵫�����½��أ���һ�β��������أ���¼��ʱ�Ķ�ʱ������ֵ
            {
                TIM3CH2_CAPTURE_UPVAL = TIM_GetCapture2(TIM3);        //��ȡ����������
                TIM3CH2_CAPTURE_STA |= 0X40;        //����Ѳ���������
                TIM_OC2PolarityConfig(TIM3, TIM_ICPolarity_Falling);//����Ϊ�½��ز���
            }
        }
    }

    if ((TIM3CH3_CAPTURE_STA & 0X80) == 0)        //��δ�ɹ�����    
    {
        if (TIM_GetITStatus(TIM3, TIM_IT_CC3) != RESET)        //����3���������¼�
        {
            TIM_ClearITPendingBit(TIM3, TIM_IT_CC3);        //����жϱ�־λ
            if (TIM3CH3_CAPTURE_STA & 0X40)        //����һ���½�O��
            {
                TIM3CH3_CAPTURE_DOWNVAL = TIM_GetCapture3(TIM3);//��¼�´�ʱ�Ķ�ʱ������ֵ
                if (TIM3CH3_CAPTURE_DOWNVAL < TIM3CH3_CAPTURE_UPVAL)
                {
                    tim3_T3 = 65535;
                }
                else
                    tim3_T3 = 0;
                rc.Tim3Ch3CapVal = TIM3CH3_CAPTURE_DOWNVAL - TIM3CH3_CAPTURE_UPVAL
                        + tim3_T3;        //�õ��ܵĸߵ�ƽ��ʱ��
                TIM3CH3_CAPTURE_STA = 0;        //�����־λ����
                TIM_OC3PolarityConfig(TIM3, TIM_ICPolarity_Rising); //����Ϊ�����ز���          
            }
            else //��������ʱ�䵫�����½��أ���һ�β��������أ���¼��ʱ�Ķ�ʱ������ֵ
            {
                TIM3CH3_CAPTURE_UPVAL = TIM_GetCapture3(TIM3);        //��ȡ����������
                TIM3CH3_CAPTURE_STA |= 0X40;        //����Ѳ���������
                TIM_OC3PolarityConfig(TIM3, TIM_ICPolarity_Falling);//����Ϊ�½��ز���
            }
        }
    }

    if ((TIM3CH4_CAPTURE_STA & 0X80) == 0)        //��δ�ɹ�����    
    {
        if (TIM_GetITStatus(TIM3, TIM_IT_CC4) != RESET)        //����4���������¼�
        {
            TIM_ClearITPendingBit(TIM3, TIM_IT_CC4);        //����жϱ�־λ
            if (TIM3CH4_CAPTURE_STA & 0X40)        //����һ���½���
            {
                TIM3CH4_CAPTURE_DOWNVAL = TIM_GetCapture4(TIM3);//��¼�´�ʱ�Ķ�ʱ������ֵ
                if (TIM3CH4_CAPTURE_DOWNVAL < TIM3CH4_CAPTURE_UPVAL)
                {
                    tim3_T4 = 65535;
                }
                else
                    tim3_T4 = 0;
                rc.Tim3Ch4CapVal = TIM3CH4_CAPTURE_DOWNVAL - TIM3CH4_CAPTURE_UPVAL
                        + tim3_T4;        //�õ��ܵĸߵ�ƽ��ʱ��
                TIM3CH4_CAPTURE_STA = 0;        //�����־λ����
                TIM_OC4PolarityConfig(TIM3, TIM_ICPolarity_Rising); //����Ϊ�����ز���          
            }
            else //��������ʱ�䵫�����½��أ���һ�β��������أ���¼��ʱ�Ķ�ʱ������ֵ
            {
                TIM3CH4_CAPTURE_UPVAL = TIM_GetCapture4(TIM3);        //��ȡ����������
                TIM3CH4_CAPTURE_STA |= 0X40;        //����Ѳ���������
                TIM_OC4PolarityConfig(TIM3, TIM_ICPolarity_Falling);//����Ϊ�½��ز���
            }
        }
    }
    OSIntExit();
}

/*************************************************��ʱ��4*************************************************/

//��ʱ��4���벶������

u32 tim4_T1;
u32 tim4_T2;
u32 tim4_T3;
u32 tim4_T4;

u8 TIM4CH1_CAPTURE_STA = 0;    //ͨ��1���벶���־������λ�������־����6λ�������־        
u16 TIM4CH1_CAPTURE_UPVAL;
u16 TIM4CH1_CAPTURE_DOWNVAL;

u8 TIM4CH2_CAPTURE_STA = 0;    //ͨ��2���벶���־������λ�������־����6λ�������־        
u16 TIM4CH2_CAPTURE_UPVAL;
u16 TIM4CH2_CAPTURE_DOWNVAL;

u8 TIM4CH3_CAPTURE_STA = 0;    //ͨ��3���벶���־������λ�������־����6λ�������־        
u16 TIM4CH3_CAPTURE_UPVAL;
u16 TIM4CH3_CAPTURE_DOWNVAL;

u8 TIM4CH4_CAPTURE_STA = 0;    //ͨ��1���벶���־������λ�������־����6λ�������־        
u16 TIM4CH4_CAPTURE_UPVAL;
u16 TIM4CH4_CAPTURE_DOWNVAL;

TIM_ICInitTypeDef TIM4_ICInitStructure;

void TIM4_Cap_Init(u16 arr, u16 psc)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);    //ʹ��TIM4ʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);  //ʹ��GPIOBʱ��

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;  //PB6,7,8,9 ���֮ǰ����  
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; //PB6,7,8,9 ���� 
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOB, GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9);//PB6,7,8,9  ����

    //��ʼ����ʱ��4 TIM4     
    TIM_TimeBaseStructure.TIM_Period = arr; //�趨�������Զ���װֵ 
    TIM_TimeBaseStructure.TIM_Prescaler = psc;     //Ԥ��Ƶ�� 
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�:TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ

    //��ʼ��TIM4���벶����� ͨ��1
    TIM4_ICInitStructure.TIM_Channel = TIM_Channel_1; //CC1S=01     ѡ������� IC1ӳ�䵽TI1��
    TIM4_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;    //�����ز���
    TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
    TIM4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;      //���������Ƶ,����Ƶ 
    TIM4_ICInitStructure.TIM_ICFilter = 0x00;      //IC1F=0000 ���������˲��� ���˲�
    TIM_ICInit(TIM4, &TIM4_ICInitStructure);

    //��ʼ��TIM4���벶����� ͨ��2
    TIM4_ICInitStructure.TIM_Channel = TIM_Channel_2; //CC1S=01     ѡ������� IC1ӳ�䵽TI1��
    TIM4_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;    //�����ز���
    TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
    TIM4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;      //���������Ƶ,����Ƶ 
    TIM4_ICInitStructure.TIM_ICFilter = 0x00;      //IC1F=0000 ���������˲��� ���˲�
    TIM_ICInit(TIM4, &TIM4_ICInitStructure);

    //��ʼ��TIM4���벶����� ͨ��3
    TIM4_ICInitStructure.TIM_Channel = TIM_Channel_3; //CC1S=01     ѡ������� IC1ӳ�䵽TI1��
    TIM4_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;    //�����ز���
    TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
    TIM4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;      //���������Ƶ,����Ƶ 
    TIM4_ICInitStructure.TIM_ICFilter = 0x00;      //IC1F=0000 ���������˲��� ���˲�
    TIM_ICInit(TIM4, &TIM4_ICInitStructure);

    //��ʼ��TIM4���벶����� ͨ��4
    TIM4_ICInitStructure.TIM_Channel = TIM_Channel_4; //CC1S=01     ѡ������� IC1ӳ�䵽TI1��
    TIM4_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;    //�����ز���
    TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
    TIM4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;      //���������Ƶ,����Ƶ 
    TIM4_ICInitStructure.TIM_ICFilter = 0x00;      //IC1F=0000 ���������˲��� ���˲�
    TIM_ICInit(TIM4, &TIM4_ICInitStructure);

    //�жϷ����ʼ��
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;  //TIM4�ж�
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //��ռ���ȼ�1��
    //NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //�����ȼ�0��
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
    NVIC_Init(&NVIC_InitStructure);   //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ��� 

    TIM_ITConfig(TIM4, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4,
            ENABLE);   //����������жϣ�����CC1IE,CC2IE,CC3IE,CC4IE�����ж�    

    TIM_Cmd(TIM4, ENABLE);         //ʹ�ܶ�ʱ��4

}

//��ʱ��4�жϷ������
void TIM4_IRQHandler(void)
{
    OSIntEnter();
    if ((TIM4CH1_CAPTURE_STA & 0X80) == 0)         //��δ�ɹ�����    
    {
        if (TIM_GetITStatus(TIM4, TIM_IT_CC1) != RESET)         //����1���������¼�
        {
            TIM_ClearITPendingBit(TIM4, TIM_IT_CC1);         //����жϱ�־λ
            if (TIM4CH1_CAPTURE_STA & 0X40)        //����һ���½���
            {
                TIM4CH1_CAPTURE_DOWNVAL = TIM_GetCapture1(TIM4);//��¼�´�ʱ�Ķ�ʱ������ֵ
                if (TIM4CH1_CAPTURE_DOWNVAL < TIM4CH1_CAPTURE_UPVAL)
                {
                    tim4_T1 = 65535;
                }
                else
                    tim4_T1 = 0;
                rc.Tim4Ch1CapVal = TIM4CH1_CAPTURE_DOWNVAL - TIM4CH1_CAPTURE_UPVAL
                        + tim4_T1;        //�õ��ܵĸߵ�ƽ��ʱ��
                TIM4CH1_CAPTURE_STA = 0;        //�����־λ����
                TIM_OC1PolarityConfig(TIM4, TIM_ICPolarity_Rising); //����Ϊ�����ز���          
            }
            else //��������ʱ�䵫�����½��أ���һ�β��������أ���¼��ʱ�Ķ�ʱ������ֵ
            {
                TIM4CH1_CAPTURE_UPVAL = TIM_GetCapture1(TIM4);        //��ȡ����������
                TIM4CH1_CAPTURE_STA |= 0X40;        //����Ѳ���������
                TIM_OC1PolarityConfig(TIM4, TIM_ICPolarity_Falling);//����Ϊ�½��ز���
            }
        }
    }

    if ((TIM4CH2_CAPTURE_STA & 0X80) == 0)        //��δ�ɹ�����    
    {
        if (TIM_GetITStatus(TIM4, TIM_IT_CC2) != RESET)        //����2���������¼�
        {
            TIM_ClearITPendingBit(TIM4, TIM_IT_CC2);        //����жϱ�־λ
            if (TIM4CH2_CAPTURE_STA & 0X40)        //����һ���½���
            {
                TIM4CH2_CAPTURE_DOWNVAL = TIM_GetCapture2(TIM4);//��¼�´�ʱ�Ķ�ʱ������ֵ
                if (TIM4CH2_CAPTURE_DOWNVAL < TIM4CH2_CAPTURE_UPVAL)
                {
                    tim4_T2 = 65535;
                }
                else
                    tim4_T2 = 0;
                rc.Tim4Ch2CapVal = TIM4CH2_CAPTURE_DOWNVAL - TIM4CH2_CAPTURE_UPVAL
                        + tim4_T2;        //�õ��ܵĸߵ�ƽ��ʱ��
                TIM4CH2_CAPTURE_STA = 0;        //�����־λ����
                TIM_OC2PolarityConfig(TIM4, TIM_ICPolarity_Rising); //����Ϊ�����ز���          
            }
            else //��������ʱ�䵫�����½��أ���һ�β��������أ���¼��ʱ�Ķ�ʱ������ֵ
            {
                TIM4CH2_CAPTURE_UPVAL = TIM_GetCapture2(TIM4);        //��ȡ����������
                TIM4CH2_CAPTURE_STA |= 0X40;        //����Ѳ���������
                TIM_OC2PolarityConfig(TIM4, TIM_ICPolarity_Falling);//����Ϊ�½��ز���
            }
        }
    }

    if ((TIM4CH3_CAPTURE_STA & 0X80) == 0)        //��δ�ɹ�����    
    {
        if (TIM_GetITStatus(TIM4, TIM_IT_CC3) != RESET)        //����3���������¼�
        {
            TIM_ClearITPendingBit(TIM4, TIM_IT_CC3);        //����жϱ�־λ
            if (TIM4CH3_CAPTURE_STA & 0X40)        //����һ���½���
            {
                TIM4CH3_CAPTURE_DOWNVAL = TIM_GetCapture3(TIM4);//��¼�´�ʱ�Ķ�ʱ������ֵ
                if (TIM4CH3_CAPTURE_DOWNVAL < TIM4CH3_CAPTURE_UPVAL)
                {
                    tim4_T3 = 65535;
                }
                else
                    tim4_T3 = 0;
                rc.Tim4Ch3CapVal = TIM4CH3_CAPTURE_DOWNVAL - TIM4CH3_CAPTURE_UPVAL
                        + tim4_T3;        //�õ��ܵĸߵ�ƽ��ʱ��
                TIM4CH3_CAPTURE_STA = 0;        //�����־λ����
                TIM_OC3PolarityConfig(TIM4, TIM_ICPolarity_Rising); //����Ϊ�����ز���          
            }
            else //��������ʱ�䵫�����½��أ���һ�β��������أ���¼��ʱ�Ķ�ʱ������ֵ
            {
                TIM4CH3_CAPTURE_UPVAL = TIM_GetCapture3(TIM4);        //��ȡ����������
                TIM4CH3_CAPTURE_STA |= 0X40;        //����Ѳ���������
                TIM_OC3PolarityConfig(TIM4, TIM_ICPolarity_Falling);//����Ϊ�½��ز���
            }
        }
    }

    if ((TIM4CH4_CAPTURE_STA & 0X80) == 0)        //��δ�ɹ�����    
    {
        if (TIM_GetITStatus(TIM4, TIM_IT_CC4) != RESET)        //����4���������¼�
        {
            TIM_ClearITPendingBit(TIM4, TIM_IT_CC4);        //����жϱ�־λ
            if (TIM4CH4_CAPTURE_STA & 0X40)        //����һ���½���
            {
                TIM4CH4_CAPTURE_DOWNVAL = TIM_GetCapture4(TIM4);//��¼�´�ʱ�Ķ�ʱ������ֵ
                if (TIM4CH4_CAPTURE_DOWNVAL < TIM4CH4_CAPTURE_UPVAL)
                {
                    tim4_T4 = 65535;
                }
                else
                    tim4_T4 = 0;
                rc.Tim4Ch4CapVal = TIM4CH4_CAPTURE_DOWNVAL - TIM4CH4_CAPTURE_UPVAL
                        + tim4_T4;        //�õ��ܵĸߵ�ƽ��ʱ��
                TIM4CH4_CAPTURE_STA = 0;        //�����־λ����
                TIM_OC4PolarityConfig(TIM4, TIM_ICPolarity_Rising); //����Ϊ�����ز���          
            }
            else //��������ʱ�䵫�����½��أ���һ�β��������أ���¼��ʱ�Ķ�ʱ������ֵ
            {
                TIM4CH4_CAPTURE_UPVAL = TIM_GetCapture4(TIM4);        //��ȡ����������
                TIM4CH4_CAPTURE_STA |= 0X40;        //����Ѳ���������
                TIM_OC4PolarityConfig(TIM4, TIM_ICPolarity_Falling);//����Ϊ�½��ز���
            }
        }
    }
    OSIntExit();
}

/*************************************************��ʱ��5*************************************************/

//��ʱ��5���벶������


u32 tim5_T1;
u32 tim5_T2;
u32 tim5_T3;
u32 tim5_T4;

u8 TIM5CH1_CAPTURE_STA = 0;    //ͨ��1���벶���־������λ�������־����6λ�������־        
u16 TIM5CH1_CAPTURE_UPVAL;
u16 TIM5CH1_CAPTURE_DOWNVAL;

u8 TIM5CH2_CAPTURE_STA = 0;    //ͨ��2���벶���־������λ�������־����6λ�������־        
u16 TIM5CH2_CAPTURE_UPVAL;
u16 TIM5CH2_CAPTURE_DOWNVAL;

u8 TIM5CH3_CAPTURE_STA = 0;    //ͨ��3���벶���־������λ�������־����6λ�������־        
u16 TIM5CH3_CAPTURE_UPVAL;
u16 TIM5CH3_CAPTURE_DOWNVAL;

u8 TIM5CH4_CAPTURE_STA = 0;    //ͨ��1���벶���־������λ�������־����6λ�������־        
u16 TIM5CH4_CAPTURE_UPVAL;
u16 TIM5CH4_CAPTURE_DOWNVAL;

TIM_ICInitTypeDef TIM5_ICInitStructure;

void TIM5_Cap_Init(u16 arr, u16 psc) //��STM32F103CBT6��Ƭ����ʹ����������ܹ���ȷ��ʼ��TIM2,���������ͺŵ�Ƭ������Ҫ�鿴DATASHEET�����ܻ���TIM5��ͻ
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);    //ʹ��TIM5ʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  //ʹ��GPIOBʱ��

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2
            | GPIO_Pin_3;  //PB6,7,8,9 ���֮ǰ����  
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; //PB6,7,8,9 ���� 
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOA, GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3);//PB6,7,8,9  ����

    //��ʼ����ʱ��4 TIM5     
    TIM_TimeBaseStructure.TIM_Period = arr; //�趨�������Զ���װֵ 
    TIM_TimeBaseStructure.TIM_Prescaler = psc;     //Ԥ��Ƶ�� 
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�:TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ

    //��ʼ��TIM5���벶����� ͨ��1
    TIM5_ICInitStructure.TIM_Channel = TIM_Channel_1; //CC1S=01     ѡ������� IC1ӳ�䵽TI1��
    TIM5_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;    //�����ز���
    TIM5_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
    TIM5_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;      //���������Ƶ,����Ƶ 
    TIM5_ICInitStructure.TIM_ICFilter = 0x00;      //IC1F=0000 ���������˲��� ���˲�
    TIM_ICInit(TIM5, &TIM5_ICInitStructure);

    //��ʼ��TIM5���벶����� ͨ��2
    TIM5_ICInitStructure.TIM_Channel = TIM_Channel_2; //CC1S=01     ѡ������� IC1ӳ�䵽TI1��
    TIM5_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;    //�����ز���
    TIM5_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
    TIM5_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;      //���������Ƶ,����Ƶ 
    TIM5_ICInitStructure.TIM_ICFilter = 0x00;      //IC1F=0000 ���������˲��� ���˲�
    TIM_ICInit(TIM5, &TIM5_ICInitStructure);

    //��ʼ��TIM5���벶����� ͨ��3
    TIM5_ICInitStructure.TIM_Channel = TIM_Channel_3; //CC1S=01     ѡ������� IC1ӳ�䵽TI1��
    TIM5_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;    //�����ز���
    TIM5_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
    TIM5_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;      //���������Ƶ,����Ƶ 
    TIM5_ICInitStructure.TIM_ICFilter = 0x00;      //IC1F=0000 ���������˲��� ���˲�
    TIM_ICInit(TIM5, &TIM5_ICInitStructure);

    //��ʼ��TIM5���벶����� ͨ��4
    TIM5_ICInitStructure.TIM_Channel = TIM_Channel_4; //CC1S=01     ѡ������� IC1ӳ�䵽TI1��
    TIM5_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;    //�����ز���
    TIM5_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
    TIM5_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;      //���������Ƶ,����Ƶ 
    TIM5_ICInitStructure.TIM_ICFilter = 0x00;      //IC1F=0000 ���������˲��� ���˲�
    TIM_ICInit(TIM5, &TIM5_ICInitStructure);

    //�жϷ����ʼ��
    NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;  //TIM5�ж�
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //��ռ���ȼ�1��
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //�����ȼ�0��
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
    NVIC_Init(&NVIC_InitStructure);   //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ��� 

    TIM_ITConfig(TIM5,TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4,
            ENABLE);   //����������жϣ�����CC1IE,CC2IE,CC3IE,CC4IE�����ж�    

    TIM_Cmd(TIM5, ENABLE);         //ʹ�ܶ�ʱ��4

}

//��ʱ��5�жϷ������
void TIM5_IRQHandler(void)
{
    OSIntEnter();
    if ((TIM5CH1_CAPTURE_STA & 0X80) == 0)         //��δ�ɹ�����    
    {
        if (TIM_GetITStatus(TIM5, TIM_IT_CC1) != RESET)         //����1���������¼�
        {
            TIM_ClearITPendingBit(TIM5, TIM_IT_CC1);         //����жϱ�־λ
            if (TIM5CH1_CAPTURE_STA & 0X40)        //����һ���½���
            {
                TIM5CH1_CAPTURE_DOWNVAL = TIM_GetCapture1(TIM5);//��¼�´�ʱ�Ķ�ʱ������ֵ
                if (TIM5CH1_CAPTURE_DOWNVAL < TIM5CH1_CAPTURE_UPVAL)
                {
                    tim5_T1 = 65535;
                }
                else
                    tim5_T1 = 0;
                rc.Tim5Ch1CapVal = TIM5CH1_CAPTURE_DOWNVAL - TIM5CH1_CAPTURE_UPVAL
                        + tim5_T1;        //�õ��ܵĸߵ�ƽ��ʱ��
                TIM5CH1_CAPTURE_STA = 0;        //�����־λ����
                TIM_OC1PolarityConfig(TIM5, TIM_ICPolarity_Rising); //����Ϊ�����ز���          
            }
            else //��������ʱ�䵫�����½��أ���һ�β��������أ���¼��ʱ�Ķ�ʱ������ֵ
            {
                TIM5CH1_CAPTURE_UPVAL = TIM_GetCapture1(TIM5);        //��ȡ����������
                TIM5CH1_CAPTURE_STA |= 0X40;        //����Ѳ���������
                TIM_OC1PolarityConfig(TIM5, TIM_ICPolarity_Falling);//����Ϊ�½��ز���
            }
        }
    }

    if ((TIM5CH2_CAPTURE_STA & 0X80) == 0)        //��δ�ɹ�����    
    {
        if (TIM_GetITStatus(TIM5, TIM_IT_CC2) != RESET)        //����2���������¼�
        {
            TIM_ClearITPendingBit(TIM5, TIM_IT_CC2);        //����жϱ�־λ
            if (TIM5CH2_CAPTURE_STA & 0X40)        //����һ���½���
            {
                TIM5CH2_CAPTURE_DOWNVAL = TIM_GetCapture2(TIM5);//��¼�´�ʱ�Ķ�ʱ������ֵ
                if (TIM5CH2_CAPTURE_DOWNVAL < TIM5CH2_CAPTURE_UPVAL)
                {
                    tim5_T2 = 65535;
                }
                else
                    tim5_T2 = 0;
                rc.Tim5Ch2CapVal = TIM5CH2_CAPTURE_DOWNVAL - TIM5CH2_CAPTURE_UPVAL
                        + tim5_T2;        //�õ��ܵĸߵ�ƽ��ʱ��
                TIM5CH2_CAPTURE_STA = 0;        //�����־λ����
                TIM_OC2PolarityConfig(TIM5, TIM_ICPolarity_Rising); //����Ϊ�����ز���          
            }
            else //��������ʱ�䵫�����½��أ���һ�β��������أ���¼��ʱ�Ķ�ʱ������ֵ
            {
                TIM5CH2_CAPTURE_UPVAL = TIM_GetCapture2(TIM5);        //��ȡ����������
                TIM5CH2_CAPTURE_STA |= 0X40;        //����Ѳ���������
                TIM_OC2PolarityConfig(TIM5, TIM_ICPolarity_Falling);//����Ϊ�½��ز���
            }
        }
    }

    if ((TIM5CH3_CAPTURE_STA & 0X80) == 0)        //��δ�ɹ�����    
    {
        if (TIM_GetITStatus(TIM5, TIM_IT_CC3) != RESET)        //����3���������¼�
        {
            TIM_ClearITPendingBit(TIM5, TIM_IT_CC3);        //����жϱ�־λ
            if (TIM5CH3_CAPTURE_STA & 0X40)        //����һ���½���
            {
                TIM5CH3_CAPTURE_DOWNVAL = TIM_GetCapture3(TIM5);//��¼�´�ʱ�Ķ�ʱ������ֵ
                if (TIM5CH3_CAPTURE_DOWNVAL < TIM5CH3_CAPTURE_UPVAL)
                {
                    tim5_T3 = 65535;
                }
                else
                    tim5_T3 = 0;
                rc.Tim5Ch3CapVal = TIM5CH3_CAPTURE_DOWNVAL - TIM5CH3_CAPTURE_UPVAL
                        + tim5_T3;        //�õ��ܵĸߵ�ƽ��ʱ��
                TIM5CH3_CAPTURE_STA = 0;        //�����־λ����
                TIM_OC3PolarityConfig(TIM5, TIM_ICPolarity_Rising); //����Ϊ�����ز���          
            }
            else //��������ʱ�䵫�����½��أ���һ�β��������أ���¼��ʱ�Ķ�ʱ������ֵ
            {
                TIM5CH3_CAPTURE_UPVAL = TIM_GetCapture3(TIM5);        //��ȡ����������
                TIM5CH3_CAPTURE_STA |= 0X40;        //����Ѳ���������
                TIM_OC3PolarityConfig(TIM5, TIM_ICPolarity_Falling);//����Ϊ�½��ز���
            }
        }
    }

    if ((TIM5CH4_CAPTURE_STA & 0X80) == 0)        //��δ�ɹ�����    
    {
        if (TIM_GetITStatus(TIM5, TIM_IT_CC4) != RESET)        //����4���������¼�
        {
            TIM_ClearITPendingBit(TIM5, TIM_IT_CC4);        //����жϱ�־λ
            if (TIM5CH4_CAPTURE_STA & 0X40)        //����һ���½���
            {
                TIM5CH4_CAPTURE_DOWNVAL = TIM_GetCapture4(TIM5);//��¼�´�ʱ�Ķ�ʱ������ֵ
                if (TIM5CH4_CAPTURE_DOWNVAL < TIM5CH4_CAPTURE_UPVAL)
                {
                    tim5_T4 = 65535;
                }
                else
                    tim5_T4 = 0;
                rc.Tim5Ch4CapVal = TIM5CH4_CAPTURE_DOWNVAL - TIM5CH4_CAPTURE_UPVAL
                        + tim5_T4;        //�õ��ܵĸߵ�ƽ��ʱ��
                TIM5CH4_CAPTURE_STA = 0;        //�����־λ����
                TIM_OC4PolarityConfig(TIM5, TIM_ICPolarity_Rising); //����Ϊ�����ز���          
            }
            else //��������ʱ�䵫�����½��أ���һ�β��������أ���¼��ʱ�Ķ�ʱ������ֵ
            {
                TIM5CH4_CAPTURE_UPVAL = TIM_GetCapture4(TIM5);        //��ȡ����������
                TIM5CH4_CAPTURE_STA |= 0X40;        //����Ѳ���������
                TIM_OC4PolarityConfig(TIM5, TIM_ICPolarity_Falling);//����Ϊ�½��ز���
            }
        }
    }
    OSIntExit();
}
/********************************************************************************************************/


void RemoteControllerCaptureInit()//ң�����źŲ���ʱ����ʼ��
{
    //TIM2_Cap_Init(0xffff,72-1); //PWM�����ʼ��,��1Mhz��Ƶ�ʼ��� CH1-CH4
    TIM3_Cap_Init(0xffff,72-1); //PWM�����ʼ��,��1Mhz��Ƶ�ʼ��� CH1-CH4
}
/********************************************************************************************************/

