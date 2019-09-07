#include "timer.h"
#include "ucos_ii.h"
/************************************************电机PWM更新************************************************/


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

/*************************************************定时器1*************************************************/
void Tim1_init(void)
{
    TIM_TimeBaseInitTypeDef        TIM_TimeBaseStructure;
    TIM_OCInitTypeDef                  TIM_OCInitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    uint16_t PrescalerValue = 0;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE); //GPIOA时钟使能
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_11 ; 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //IO速率50MHz
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //复用功能输出
    GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);//使能定时器2时钟
    PrescalerValue = (uint16_t) 7;//计算分频系数
    TIM_TimeBaseStructure.TIM_Period = TIMER_CYCLE;    //设置在下一个更新事件装入活动的自动重装载寄存器周期的值
    TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;    
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;//设置时钟分割    
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;    //计数器向上记数
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);//定时器2初始化

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;//TIM_CNT<TIMX_CCRX时，输出为有效电平
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_OCInitStructure.TIM_Pulse = MotoPwmMin;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

    TIM_OC1Init(TIM1, &TIM_OCInitStructure);//初始化TIM_OC1
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);//使能预装载

    TIM_OC4Init(TIM1, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM1, ENABLE);
    TIM_Cmd(TIM1, ENABLE);//使能定时器1
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
}
/*************************************************定时器4*************************************************/
void Tim4_init(void)
{
    TIM_TimeBaseInitTypeDef        TIM_TimeBaseStructure;
    TIM_OCInitTypeDef                  TIM_OCInitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    uint16_t PrescalerValue = 0;
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE); //GPIOA时钟使能
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 ; 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //IO速率50MHz
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //复用功能输出
    GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);//使能定时器2时钟
    PrescalerValue = (uint16_t) 7;//计算分频系数
    TIM_TimeBaseStructure.TIM_Period = TIMER_CYCLE;    //设置在下一个更新事件装入活动的自动重装载寄存器周期的值
    TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;    
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;//设置时钟分割    
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;    //计数器向上记数
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);//定时器2初始化
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;//TIM_CNT<TIMX_CCRX时，输出为有效电平
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;//比较输出使能
    TIM_OCInitStructure.TIM_Pulse = MotoPwmMin;//初始占空比为
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;//设置有效电平为高电平，结合PWM模式，上面用的是PWM1
    
    TIM_OC1Init(TIM4, &TIM_OCInitStructure);//初始化TIM_OC1
    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);//使能预装载
    
    TIM_OC2Init(TIM4, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
    
    TIM_OC3Init(TIM4, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
    
    TIM_OC4Init(TIM4, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
    
    TIM_ARRPreloadConfig(TIM4, ENABLE);
    TIM_Cmd(TIM4, ENABLE);//使能定时器2
}

void Tim2_init(void)
{
    TIM_TimeBaseInitTypeDef        TIM_TimeBaseStructure;
    TIM_OCInitTypeDef                  TIM_OCInitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    uint16_t PrescalerValue = 0;
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);    //使能TIM2时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE); //GPIOA时钟使能
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 ; 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //IO速率50MHz
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //复用功能输出
    GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);//使能定时器2时钟
    PrescalerValue = (uint16_t) 7;//计算分频系数
    TIM_TimeBaseStructure.TIM_Period = TIMER_CYCLE;    //设置在下一个更新事件装入活动的自动重装载寄存器周期的值
    TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;    
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;//设置时钟分割    
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;    //计数器向上记数
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);//定时器2初始化
	
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;//TIM_CNT<TIMX_CCRX时，输出为有效电平
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;//比较输出使能
    TIM_OCInitStructure.TIM_Pulse = MotoPwmMin;//初始占空比为
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;//设置有效电平为高电平，结合PWM模式，上面用的是PWM1
    
    TIM_OC1Init(TIM2, &TIM_OCInitStructure);//初始化TIM_OC1
    TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);//使能预装载
    
    TIM_OC2Init(TIM2, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
    
    TIM_OC3Init(TIM2, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
    
    TIM_OC4Init(TIM2, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);
    
    TIM_ARRPreloadConfig(TIM2, ENABLE);
    TIM_Cmd(TIM2, ENABLE);//使能定时器2
}




/*************************************************定时器2*************************************************/
//定时器2输入捕获配置

u32 tim2_T1;
u32 tim2_T2;
u32 tim2_T3;
u32 tim2_T4;

u8 TIM2CH1_CAPTURE_STA = 0;    //通道1输入捕获标志，高两位做捕获标志，低6位做溢出标志        
u16 TIM2CH1_CAPTURE_UPVAL;
u16 TIM2CH1_CAPTURE_DOWNVAL;

u8 TIM2CH2_CAPTURE_STA = 0;    //通道2输入捕获标志，高两位做捕获标志，低6位做溢出标志        
u16 TIM2CH2_CAPTURE_UPVAL;
u16 TIM2CH2_CAPTURE_DOWNVAL;

u8 TIM2CH3_CAPTURE_STA = 0;    //通道3输入捕获标志，高两位做捕获标志，低6位做溢出标志        
u16 TIM2CH3_CAPTURE_UPVAL;
u16 TIM2CH3_CAPTURE_DOWNVAL;

u8 TIM2CH4_CAPTURE_STA = 0;    //通道1输入捕获标志，高两位做捕获标志，低6位做溢出标志        
u16 TIM2CH4_CAPTURE_UPVAL;
u16 TIM2CH4_CAPTURE_DOWNVAL;

TIM_ICInitTypeDef TIM2_ICInitStructure;

void TIM2_Cap_Init(u16 arr, u16 psc) //在STM32F103CBT6单片机上使用这个函数能够正确初始化TIM2,若在其他型号单片机上需要查看DATASHEET，可能会与TIM5冲突
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);    //使能TIM2时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  //使能GPIOB时钟

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2
            | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOA, GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3);//PB6,7,8,9  下拉

    //初始化定时器4 TIM2     
    TIM_TimeBaseStructure.TIM_Period = arr; //设定计数器自动重装值 
    TIM_TimeBaseStructure.TIM_Prescaler = psc;     //预分频器 
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位

    //初始化TIM2输入捕获参数 通道1
    TIM2_ICInitStructure.TIM_Channel = TIM_Channel_1; //CC1S=01     选择输入端 IC1映射到TI1上
    TIM2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;    //上升沿捕获
    TIM2_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
    TIM2_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;      //配置输入分频,不分频 
    TIM2_ICInitStructure.TIM_ICFilter = 0x00;      //IC1F=0000 配置输入滤波器 不滤波
    TIM_ICInit(TIM2, &TIM2_ICInitStructure);

    //初始化TIM2输入捕获参数 通道2
    TIM2_ICInitStructure.TIM_Channel = TIM_Channel_2; //CC1S=01     选择输入端 IC1映射到TI1上
    TIM2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;    //上升沿捕获
    TIM2_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
    TIM2_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;      //配置输入分频,不分频 
    TIM2_ICInitStructure.TIM_ICFilter = 0x00;      //IC1F=0000 配置输入滤波器 不滤波
    TIM_ICInit(TIM2, &TIM2_ICInitStructure);

    //初始化TIM2输入捕获参数 通道3
    TIM2_ICInitStructure.TIM_Channel = TIM_Channel_3; //CC1S=01     选择输入端 IC1映射到TI1上
    TIM2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;    //上升沿捕获
    TIM2_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
    TIM2_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;      //配置输入分频,不分频 
    TIM2_ICInitStructure.TIM_ICFilter = 0x00;      //IC1F=0000 配置输入滤波器 不滤波
    TIM_ICInit(TIM2, &TIM2_ICInitStructure);

    //初始化TIM2输入捕获参数 通道4
    TIM2_ICInitStructure.TIM_Channel = TIM_Channel_4; //CC1S=01     选择输入端 IC1映射到TI1上
    TIM2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;    //上升沿捕获
    TIM2_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
    TIM2_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;      //配置输入分频,不分频 
    TIM2_ICInitStructure.TIM_ICFilter = 0x00;      //IC1F=0000 配置输入滤波器 不滤波
    TIM_ICInit(TIM2, &TIM2_ICInitStructure);

    //中断分组初始化
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;  //TIM2中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //先占优先级1级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //从优先级0级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
    NVIC_Init(&NVIC_InitStructure);   //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器 

    TIM_ITConfig(TIM2,TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4,
            ENABLE);   //不允许更新中断，允许CC1IE,CC2IE,CC3IE,CC4IE捕获中断    

    TIM_Cmd(TIM2, ENABLE);         //使能定时器4

}

//定时器5中断服务程序
void TIM2_IRQHandler(void)
{
    OSIntEnter();
    if ((TIM2CH1_CAPTURE_STA & 0X80) == 0)         //还未成功捕获    
    {
        if (TIM_GetITStatus(TIM2, TIM_IT_CC1) != RESET)         //捕获1发生捕获事件
        {
            TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);         //清除中断标志位
            if (TIM2CH1_CAPTURE_STA & 0X40)        //捕获到一个下降沿
            {
                TIM2CH1_CAPTURE_DOWNVAL = TIM_GetCapture1(TIM2);//记录下此时的定时器计数值
                if (TIM2CH1_CAPTURE_DOWNVAL < TIM2CH1_CAPTURE_UPVAL)
                {
                    tim2_T1 = 65535;
                }
                else
                    tim2_T1 = 0;
                rc.Tim2Ch1CapVal = TIM2CH1_CAPTURE_DOWNVAL - TIM2CH1_CAPTURE_UPVAL
                        + tim2_T1;        //得到总的高电平的时间
                TIM2CH1_CAPTURE_STA = 0;        //捕获标志位清零
                TIM_OC1PolarityConfig(TIM2, TIM_ICPolarity_Rising); //设置为上升沿捕获          
            }
            else //发生捕获时间但不是下降沿，第一次捕获到上升沿，记录此时的定时器计数值
            {
                TIM2CH1_CAPTURE_UPVAL = TIM_GetCapture1(TIM2);        //获取上升沿数据
                TIM2CH1_CAPTURE_STA |= 0X40;        //标记已捕获到上升沿
                TIM_OC1PolarityConfig(TIM2, TIM_ICPolarity_Falling);//设置为下降沿捕获
            }
        }
    }

    if ((TIM2CH2_CAPTURE_STA & 0X80) == 0)        //还未成功捕获    
    {
        if (TIM_GetITStatus(TIM2, TIM_IT_CC2) != RESET)        //捕获2发生捕获事件
        {
            TIM_ClearITPendingBit(TIM2, TIM_IT_CC2);        //清除中断标志位
            if (TIM2CH2_CAPTURE_STA & 0X40)        //捕获到一个下降沿
            {
                TIM2CH2_CAPTURE_DOWNVAL = TIM_GetCapture2(TIM2);//记录下此时的定时器计数值
                if (TIM2CH2_CAPTURE_DOWNVAL < TIM2CH2_CAPTURE_UPVAL)
                {
                    tim2_T2 = 65535;
                }
                else
                    tim2_T2 = 0;
                rc.Tim2Ch2CapVal = TIM2CH2_CAPTURE_DOWNVAL - TIM2CH2_CAPTURE_UPVAL
                        + tim2_T2;        //得到总的高电平的时间
                TIM2CH2_CAPTURE_STA = 0;        //捕获标志位清零
                TIM_OC2PolarityConfig(TIM2, TIM_ICPolarity_Rising); //设置为上升沿捕获          
            }
            else //发生捕获时间但不是下降沿，第一次捕获到上升沿，记录此时的定时器计数值
            {
                TIM2CH2_CAPTURE_UPVAL = TIM_GetCapture2(TIM2);        //获取上升沿数据
                TIM2CH2_CAPTURE_STA |= 0X40;        //标记已捕获到上升沿
                TIM_OC2PolarityConfig(TIM2, TIM_ICPolarity_Falling);//设置为下降沿捕获
            }
        }
    }

    if ((TIM2CH3_CAPTURE_STA & 0X80) == 0)        //还未成功捕获    
    {
        if (TIM_GetITStatus(TIM2, TIM_IT_CC3) != RESET)        //捕获3发生捕获事件
        {
            TIM_ClearITPendingBit(TIM2, TIM_IT_CC3);        //清除中断标志位
            if (TIM2CH3_CAPTURE_STA & 0X40)        //捕获到一个下降沿
            {
                TIM2CH3_CAPTURE_DOWNVAL = TIM_GetCapture3(TIM2);//记录下此时的定时器计数值
                if (TIM2CH3_CAPTURE_DOWNVAL < TIM2CH3_CAPTURE_UPVAL)
                {
                    tim2_T3 = 65535;
                }
                else
                    tim2_T3 = 0;
                rc.Tim2Ch3CapVal = TIM2CH3_CAPTURE_DOWNVAL - TIM2CH3_CAPTURE_UPVAL
                        + tim2_T3;        //得到总的高电平的时间
                TIM2CH3_CAPTURE_STA = 0;        //捕获标志位清零
                TIM_OC3PolarityConfig(TIM2, TIM_ICPolarity_Rising); //设置为上升沿捕获          
            }
            else //发生捕获时间但不是下降沿，第一次捕获到上升沿，记录此时的定时器计数值
            {
                TIM2CH3_CAPTURE_UPVAL = TIM_GetCapture3(TIM2);        //获取上升沿数据
                TIM2CH3_CAPTURE_STA |= 0X40;        //标记已捕获到上升沿
                TIM_OC3PolarityConfig(TIM2, TIM_ICPolarity_Falling);//设置为下降沿捕获
            }
        }
    }

    if ((TIM2CH4_CAPTURE_STA & 0X80) == 0)        //还未成功捕获    
    {
        if (TIM_GetITStatus(TIM2, TIM_IT_CC4) != RESET)        //捕获4发生捕获事件
        {
            TIM_ClearITPendingBit(TIM2, TIM_IT_CC4);        //清除中断标志位
            if (TIM2CH4_CAPTURE_STA & 0X40)        //捕获到一个下降沿
            {
                TIM2CH4_CAPTURE_DOWNVAL = TIM_GetCapture4(TIM2);//记录下此时的定时器计数值
                if (TIM2CH4_CAPTURE_DOWNVAL < TIM2CH4_CAPTURE_UPVAL)
                {
                    tim2_T4 = 65535;
                }
                else
                    tim2_T4 = 0;
                rc.Tim2Ch4CapVal = TIM2CH4_CAPTURE_DOWNVAL - TIM2CH4_CAPTURE_UPVAL
                        + tim2_T4;        //得到总的高电平的时间
                TIM2CH4_CAPTURE_STA = 0;        //捕获标志位清零
                TIM_OC4PolarityConfig(TIM2, TIM_ICPolarity_Rising); //设置为上升沿捕获          
            }
            else //发生捕获时间但不是下降沿，第一次捕获到上升沿，记录此时的定时器计数值
            {
                TIM2CH4_CAPTURE_UPVAL = TIM_GetCapture4(TIM2);        //获取上升沿数据
                TIM2CH4_CAPTURE_STA |= 0X40;        //标记已捕获到上升沿
                TIM_OC4PolarityConfig(TIM2, TIM_ICPolarity_Falling);//设置为下降沿捕获
            }
        }
    }
    OSIntExit();
}

/*************************************************定时器3*************************************************/

//定时器3输入捕获配置



u32 tim3_T1;
u32 tim3_T2;
u32 tim3_T3;
u32 tim3_T4;

u8 TIM3CH1_CAPTURE_STA = 0;    //通道1输入捕获标志，高两位做捕获标志，低6位做溢出标志        
u16 TIM3CH1_CAPTURE_UPVAL;
u16 TIM3CH1_CAPTURE_DOWNVAL;

u8 TIM3CH2_CAPTURE_STA = 0;    //通道2输入捕获标志，高两位做捕获标志，低6位做溢出标志        
u16 TIM3CH2_CAPTURE_UPVAL;
u16 TIM3CH2_CAPTURE_DOWNVAL;

u8 TIM3CH3_CAPTURE_STA = 0;    //通道3输入捕获标志，高两位做捕获标志，低6位做溢出标志        
u16 TIM3CH3_CAPTURE_UPVAL;
u16 TIM3CH3_CAPTURE_DOWNVAL;

u8 TIM3CH4_CAPTURE_STA = 0;    //通道1输入捕获标志，高两位做捕获标志，低6位做溢出标志        
u16 TIM3CH4_CAPTURE_UPVAL;
u16 TIM3CH4_CAPTURE_DOWNVAL;

TIM_ICInitTypeDef TIM3_ICInitStructure;

void TIM3_Cap_Init(u16 arr, u16 psc) //在STM32F103CBT6单片机上使用这个函数能够正确初始化TIM3,若在其他型号单片机上需要查看DATASHEET，可能会与TIM5冲突
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);    //使能TIM3时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);  //使能GPIOB时钟

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 ;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOA, GPIO_Pin_6 | GPIO_Pin_7);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOB, GPIO_Pin_0 | GPIO_Pin_1);
    
    //初始化定时器4 TIM3     
    TIM_TimeBaseStructure.TIM_Period = arr; //设定计数器自动重装值 
    TIM_TimeBaseStructure.TIM_Prescaler = psc;     //预分频器 
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位

    //初始化TIM3输入捕获参数 通道1
    TIM3_ICInitStructure.TIM_Channel = TIM_Channel_1; //CC1S=01     选择输入端 IC1映射到TI1上
    TIM3_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;    //上升沿捕获
    TIM3_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
    TIM3_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;      //配置输入分频,不分频 
    TIM3_ICInitStructure.TIM_ICFilter = 0x00;      //IC1F=0000 配置输入滤波器 不滤波
    TIM_ICInit(TIM3, &TIM3_ICInitStructure);

    //初始化TIM3输入捕获参数 通道2
    TIM3_ICInitStructure.TIM_Channel = TIM_Channel_2; //CC1S=01     选择输入端 IC1映射到TI1上
    TIM3_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;    //上升沿捕获
    TIM3_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
    TIM3_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;      //配置输入分频,不分频 
    TIM3_ICInitStructure.TIM_ICFilter = 0x00;      //IC1F=0000 配置输入滤波器 不滤波
    TIM_ICInit(TIM3, &TIM3_ICInitStructure);

    //初始化TIM3输入捕获参数 通道3
    TIM3_ICInitStructure.TIM_Channel = TIM_Channel_3; //CC1S=01     选择输入端 IC1映射到TI1上
    TIM3_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;    //上升沿捕获
    TIM3_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
    TIM3_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;      //配置输入分频,不分频 
    TIM3_ICInitStructure.TIM_ICFilter = 0x00;      //IC1F=0000 配置输入滤波器 不滤波
    TIM_ICInit(TIM3, &TIM3_ICInitStructure);

    //初始化TIM3输入捕获参数 通道4
    TIM3_ICInitStructure.TIM_Channel = TIM_Channel_4; //CC1S=01     选择输入端 IC1映射到TI1上
    TIM3_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;    //上升沿捕获
    TIM3_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
    TIM3_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;      //配置输入分频,不分频 
    TIM3_ICInitStructure.TIM_ICFilter = 0x00;      //IC1F=0000 配置输入滤波器 不滤波
    TIM_ICInit(TIM3, &TIM3_ICInitStructure);

    //中断分组初始化
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //先占优先级1级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //从优先级0级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
    NVIC_Init(&NVIC_InitStructure);   //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器 

    TIM_ITConfig(TIM3,TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4,
            ENABLE);   //不允许更新中断，允许CC1IE,CC2IE,CC3IE,CC4IE捕获中断    

    TIM_Cmd(TIM3, ENABLE);         //使能定时器4

}

/*************************************************定时器5*************************************************/
//定时器5中断服务程序
void TIM3_IRQHandler(void)
{
    OSIntEnter();
    if ((TIM3CH1_CAPTURE_STA & 0X80) == 0)         //还未成功捕获    
    {
        if (TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET)         //捕获1发生捕获事件
        {
            TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);         //清除中断标志位
            if (TIM3CH1_CAPTURE_STA & 0X40)        //捕获到一个下降沿
            {
                TIM3CH1_CAPTURE_DOWNVAL = TIM_GetCapture1(TIM3);//记录下此时的定时器计数值
                if (TIM3CH1_CAPTURE_DOWNVAL < TIM3CH1_CAPTURE_UPVAL)
                {
                    tim3_T1 = 65535;
                }
                else
                    tim3_T1 = 0;
                rc.Tim3Ch1CapVal = TIM3CH1_CAPTURE_DOWNVAL - TIM3CH1_CAPTURE_UPVAL
                        + tim3_T1;        //得到总的高电平的时间
                TIM3CH1_CAPTURE_STA = 0;        //捕获标志位清零
                TIM_OC1PolarityConfig(TIM3, TIM_ICPolarity_Rising); //设置为上升沿捕获          
            }
            else //发生捕获时间但不是下降沿，第一次捕获到上升沿，记录此时的定时器计数值
            {
                TIM3CH1_CAPTURE_UPVAL = TIM_GetCapture1(TIM3);        //获取上升沿数据
                TIM3CH1_CAPTURE_STA |= 0X40;        //标记已捕获到上升沿
                TIM_OC1PolarityConfig(TIM3, TIM_ICPolarity_Falling);//设置为下降沿捕获
            }
        }
    }

    if ((TIM3CH2_CAPTURE_STA & 0X80) == 0)        //还未成功捕获    
    {
        if (TIM_GetITStatus(TIM3, TIM_IT_CC2) != RESET)        //捕获2发生捕获事件
        {
            TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);        //清除中断标志位
            if (TIM3CH2_CAPTURE_STA & 0X40)        //捕获到一个下降沿
            {
                TIM3CH2_CAPTURE_DOWNVAL = TIM_GetCapture2(TIM3);//记录下此时的定时器计数值
                if (TIM3CH2_CAPTURE_DOWNVAL < TIM3CH2_CAPTURE_UPVAL)
                {
                    tim3_T2 = 65535;
                }
                else
                    tim3_T2 = 0;
                rc.Tim3Ch2CapVal = TIM3CH2_CAPTURE_DOWNVAL - TIM3CH2_CAPTURE_UPVAL
                        + tim3_T2;        //得到总的高电平的时间
                TIM3CH2_CAPTURE_STA = 0;        //捕获标志位清零
                TIM_OC2PolarityConfig(TIM3, TIM_ICPolarity_Rising); //设置为上升沿捕获          
            }
            else //发生捕获时间但不是下降沿，第一次捕获到上升沿，记录此时的定时器计数值
            {
                TIM3CH2_CAPTURE_UPVAL = TIM_GetCapture2(TIM3);        //获取上升沿数据
                TIM3CH2_CAPTURE_STA |= 0X40;        //标记已捕获到上升沿
                TIM_OC2PolarityConfig(TIM3, TIM_ICPolarity_Falling);//设置为下降沿捕获
            }
        }
    }

    if ((TIM3CH3_CAPTURE_STA & 0X80) == 0)        //还未成功捕获    
    {
        if (TIM_GetITStatus(TIM3, TIM_IT_CC3) != RESET)        //捕获3发生捕获事件
        {
            TIM_ClearITPendingBit(TIM3, TIM_IT_CC3);        //清除中断标志位
            if (TIM3CH3_CAPTURE_STA & 0X40)        //捕获到一个下降O沿
            {
                TIM3CH3_CAPTURE_DOWNVAL = TIM_GetCapture3(TIM3);//记录下此时的定时器计数值
                if (TIM3CH3_CAPTURE_DOWNVAL < TIM3CH3_CAPTURE_UPVAL)
                {
                    tim3_T3 = 65535;
                }
                else
                    tim3_T3 = 0;
                rc.Tim3Ch3CapVal = TIM3CH3_CAPTURE_DOWNVAL - TIM3CH3_CAPTURE_UPVAL
                        + tim3_T3;        //得到总的高电平的时间
                TIM3CH3_CAPTURE_STA = 0;        //捕获标志位清零
                TIM_OC3PolarityConfig(TIM3, TIM_ICPolarity_Rising); //设置为上升沿捕获          
            }
            else //发生捕获时间但不是下降沿，第一次捕获到上升沿，记录此时的定时器计数值
            {
                TIM3CH3_CAPTURE_UPVAL = TIM_GetCapture3(TIM3);        //获取上升沿数据
                TIM3CH3_CAPTURE_STA |= 0X40;        //标记已捕获到上升沿
                TIM_OC3PolarityConfig(TIM3, TIM_ICPolarity_Falling);//设置为下降沿捕获
            }
        }
    }

    if ((TIM3CH4_CAPTURE_STA & 0X80) == 0)        //还未成功捕获    
    {
        if (TIM_GetITStatus(TIM3, TIM_IT_CC4) != RESET)        //捕获4发生捕获事件
        {
            TIM_ClearITPendingBit(TIM3, TIM_IT_CC4);        //清除中断标志位
            if (TIM3CH4_CAPTURE_STA & 0X40)        //捕获到一个下降沿
            {
                TIM3CH4_CAPTURE_DOWNVAL = TIM_GetCapture4(TIM3);//记录下此时的定时器计数值
                if (TIM3CH4_CAPTURE_DOWNVAL < TIM3CH4_CAPTURE_UPVAL)
                {
                    tim3_T4 = 65535;
                }
                else
                    tim3_T4 = 0;
                rc.Tim3Ch4CapVal = TIM3CH4_CAPTURE_DOWNVAL - TIM3CH4_CAPTURE_UPVAL
                        + tim3_T4;        //得到总的高电平的时间
                TIM3CH4_CAPTURE_STA = 0;        //捕获标志位清零
                TIM_OC4PolarityConfig(TIM3, TIM_ICPolarity_Rising); //设置为上升沿捕获          
            }
            else //发生捕获时间但不是下降沿，第一次捕获到上升沿，记录此时的定时器计数值
            {
                TIM3CH4_CAPTURE_UPVAL = TIM_GetCapture4(TIM3);        //获取上升沿数据
                TIM3CH4_CAPTURE_STA |= 0X40;        //标记已捕获到上升沿
                TIM_OC4PolarityConfig(TIM3, TIM_ICPolarity_Falling);//设置为下降沿捕获
            }
        }
    }
    OSIntExit();
}

/*************************************************定时器4*************************************************/

//定时器4输入捕获配置

u32 tim4_T1;
u32 tim4_T2;
u32 tim4_T3;
u32 tim4_T4;

u8 TIM4CH1_CAPTURE_STA = 0;    //通道1输入捕获标志，高两位做捕获标志，低6位做溢出标志        
u16 TIM4CH1_CAPTURE_UPVAL;
u16 TIM4CH1_CAPTURE_DOWNVAL;

u8 TIM4CH2_CAPTURE_STA = 0;    //通道2输入捕获标志，高两位做捕获标志，低6位做溢出标志        
u16 TIM4CH2_CAPTURE_UPVAL;
u16 TIM4CH2_CAPTURE_DOWNVAL;

u8 TIM4CH3_CAPTURE_STA = 0;    //通道3输入捕获标志，高两位做捕获标志，低6位做溢出标志        
u16 TIM4CH3_CAPTURE_UPVAL;
u16 TIM4CH3_CAPTURE_DOWNVAL;

u8 TIM4CH4_CAPTURE_STA = 0;    //通道1输入捕获标志，高两位做捕获标志，低6位做溢出标志        
u16 TIM4CH4_CAPTURE_UPVAL;
u16 TIM4CH4_CAPTURE_DOWNVAL;

TIM_ICInitTypeDef TIM4_ICInitStructure;

void TIM4_Cap_Init(u16 arr, u16 psc)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);    //使能TIM4时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);  //使能GPIOB时钟

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;  //PB6,7,8,9 清除之前设置  
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; //PB6,7,8,9 输入 
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOB, GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9);//PB6,7,8,9  下拉

    //初始化定时器4 TIM4     
    TIM_TimeBaseStructure.TIM_Period = arr; //设定计数器自动重装值 
    TIM_TimeBaseStructure.TIM_Prescaler = psc;     //预分频器 
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位

    //初始化TIM4输入捕获参数 通道1
    TIM4_ICInitStructure.TIM_Channel = TIM_Channel_1; //CC1S=01     选择输入端 IC1映射到TI1上
    TIM4_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;    //上升沿捕获
    TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
    TIM4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;      //配置输入分频,不分频 
    TIM4_ICInitStructure.TIM_ICFilter = 0x00;      //IC1F=0000 配置输入滤波器 不滤波
    TIM_ICInit(TIM4, &TIM4_ICInitStructure);

    //初始化TIM4输入捕获参数 通道2
    TIM4_ICInitStructure.TIM_Channel = TIM_Channel_2; //CC1S=01     选择输入端 IC1映射到TI1上
    TIM4_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;    //上升沿捕获
    TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
    TIM4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;      //配置输入分频,不分频 
    TIM4_ICInitStructure.TIM_ICFilter = 0x00;      //IC1F=0000 配置输入滤波器 不滤波
    TIM_ICInit(TIM4, &TIM4_ICInitStructure);

    //初始化TIM4输入捕获参数 通道3
    TIM4_ICInitStructure.TIM_Channel = TIM_Channel_3; //CC1S=01     选择输入端 IC1映射到TI1上
    TIM4_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;    //上升沿捕获
    TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
    TIM4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;      //配置输入分频,不分频 
    TIM4_ICInitStructure.TIM_ICFilter = 0x00;      //IC1F=0000 配置输入滤波器 不滤波
    TIM_ICInit(TIM4, &TIM4_ICInitStructure);

    //初始化TIM4输入捕获参数 通道4
    TIM4_ICInitStructure.TIM_Channel = TIM_Channel_4; //CC1S=01     选择输入端 IC1映射到TI1上
    TIM4_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;    //上升沿捕获
    TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
    TIM4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;      //配置输入分频,不分频 
    TIM4_ICInitStructure.TIM_ICFilter = 0x00;      //IC1F=0000 配置输入滤波器 不滤波
    TIM_ICInit(TIM4, &TIM4_ICInitStructure);

    //中断分组初始化
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;  //TIM4中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //先占优先级1级
    //NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //从优先级0级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
    NVIC_Init(&NVIC_InitStructure);   //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器 

    TIM_ITConfig(TIM4, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4,
            ENABLE);   //不允许更新中断，允许CC1IE,CC2IE,CC3IE,CC4IE捕获中断    

    TIM_Cmd(TIM4, ENABLE);         //使能定时器4

}

//定时器4中断服务程序
void TIM4_IRQHandler(void)
{
    OSIntEnter();
    if ((TIM4CH1_CAPTURE_STA & 0X80) == 0)         //还未成功捕获    
    {
        if (TIM_GetITStatus(TIM4, TIM_IT_CC1) != RESET)         //捕获1发生捕获事件
        {
            TIM_ClearITPendingBit(TIM4, TIM_IT_CC1);         //清除中断标志位
            if (TIM4CH1_CAPTURE_STA & 0X40)        //捕获到一个下降沿
            {
                TIM4CH1_CAPTURE_DOWNVAL = TIM_GetCapture1(TIM4);//记录下此时的定时器计数值
                if (TIM4CH1_CAPTURE_DOWNVAL < TIM4CH1_CAPTURE_UPVAL)
                {
                    tim4_T1 = 65535;
                }
                else
                    tim4_T1 = 0;
                rc.Tim4Ch1CapVal = TIM4CH1_CAPTURE_DOWNVAL - TIM4CH1_CAPTURE_UPVAL
                        + tim4_T1;        //得到总的高电平的时间
                TIM4CH1_CAPTURE_STA = 0;        //捕获标志位清零
                TIM_OC1PolarityConfig(TIM4, TIM_ICPolarity_Rising); //设置为上升沿捕获          
            }
            else //发生捕获时间但不是下降沿，第一次捕获到上升沿，记录此时的定时器计数值
            {
                TIM4CH1_CAPTURE_UPVAL = TIM_GetCapture1(TIM4);        //获取上升沿数据
                TIM4CH1_CAPTURE_STA |= 0X40;        //标记已捕获到上升沿
                TIM_OC1PolarityConfig(TIM4, TIM_ICPolarity_Falling);//设置为下降沿捕获
            }
        }
    }

    if ((TIM4CH2_CAPTURE_STA & 0X80) == 0)        //还未成功捕获    
    {
        if (TIM_GetITStatus(TIM4, TIM_IT_CC2) != RESET)        //捕获2发生捕获事件
        {
            TIM_ClearITPendingBit(TIM4, TIM_IT_CC2);        //清除中断标志位
            if (TIM4CH2_CAPTURE_STA & 0X40)        //捕获到一个下降沿
            {
                TIM4CH2_CAPTURE_DOWNVAL = TIM_GetCapture2(TIM4);//记录下此时的定时器计数值
                if (TIM4CH2_CAPTURE_DOWNVAL < TIM4CH2_CAPTURE_UPVAL)
                {
                    tim4_T2 = 65535;
                }
                else
                    tim4_T2 = 0;
                rc.Tim4Ch2CapVal = TIM4CH2_CAPTURE_DOWNVAL - TIM4CH2_CAPTURE_UPVAL
                        + tim4_T2;        //得到总的高电平的时间
                TIM4CH2_CAPTURE_STA = 0;        //捕获标志位清零
                TIM_OC2PolarityConfig(TIM4, TIM_ICPolarity_Rising); //设置为上升沿捕获          
            }
            else //发生捕获时间但不是下降沿，第一次捕获到上升沿，记录此时的定时器计数值
            {
                TIM4CH2_CAPTURE_UPVAL = TIM_GetCapture2(TIM4);        //获取上升沿数据
                TIM4CH2_CAPTURE_STA |= 0X40;        //标记已捕获到上升沿
                TIM_OC2PolarityConfig(TIM4, TIM_ICPolarity_Falling);//设置为下降沿捕获
            }
        }
    }

    if ((TIM4CH3_CAPTURE_STA & 0X80) == 0)        //还未成功捕获    
    {
        if (TIM_GetITStatus(TIM4, TIM_IT_CC3) != RESET)        //捕获3发生捕获事件
        {
            TIM_ClearITPendingBit(TIM4, TIM_IT_CC3);        //清除中断标志位
            if (TIM4CH3_CAPTURE_STA & 0X40)        //捕获到一个下降沿
            {
                TIM4CH3_CAPTURE_DOWNVAL = TIM_GetCapture3(TIM4);//记录下此时的定时器计数值
                if (TIM4CH3_CAPTURE_DOWNVAL < TIM4CH3_CAPTURE_UPVAL)
                {
                    tim4_T3 = 65535;
                }
                else
                    tim4_T3 = 0;
                rc.Tim4Ch3CapVal = TIM4CH3_CAPTURE_DOWNVAL - TIM4CH3_CAPTURE_UPVAL
                        + tim4_T3;        //得到总的高电平的时间
                TIM4CH3_CAPTURE_STA = 0;        //捕获标志位清零
                TIM_OC3PolarityConfig(TIM4, TIM_ICPolarity_Rising); //设置为上升沿捕获          
            }
            else //发生捕获时间但不是下降沿，第一次捕获到上升沿，记录此时的定时器计数值
            {
                TIM4CH3_CAPTURE_UPVAL = TIM_GetCapture3(TIM4);        //获取上升沿数据
                TIM4CH3_CAPTURE_STA |= 0X40;        //标记已捕获到上升沿
                TIM_OC3PolarityConfig(TIM4, TIM_ICPolarity_Falling);//设置为下降沿捕获
            }
        }
    }

    if ((TIM4CH4_CAPTURE_STA & 0X80) == 0)        //还未成功捕获    
    {
        if (TIM_GetITStatus(TIM4, TIM_IT_CC4) != RESET)        //捕获4发生捕获事件
        {
            TIM_ClearITPendingBit(TIM4, TIM_IT_CC4);        //清除中断标志位
            if (TIM4CH4_CAPTURE_STA & 0X40)        //捕获到一个下降沿
            {
                TIM4CH4_CAPTURE_DOWNVAL = TIM_GetCapture4(TIM4);//记录下此时的定时器计数值
                if (TIM4CH4_CAPTURE_DOWNVAL < TIM4CH4_CAPTURE_UPVAL)
                {
                    tim4_T4 = 65535;
                }
                else
                    tim4_T4 = 0;
                rc.Tim4Ch4CapVal = TIM4CH4_CAPTURE_DOWNVAL - TIM4CH4_CAPTURE_UPVAL
                        + tim4_T4;        //得到总的高电平的时间
                TIM4CH4_CAPTURE_STA = 0;        //捕获标志位清零
                TIM_OC4PolarityConfig(TIM4, TIM_ICPolarity_Rising); //设置为上升沿捕获          
            }
            else //发生捕获时间但不是下降沿，第一次捕获到上升沿，记录此时的定时器计数值
            {
                TIM4CH4_CAPTURE_UPVAL = TIM_GetCapture4(TIM4);        //获取上升沿数据
                TIM4CH4_CAPTURE_STA |= 0X40;        //标记已捕获到上升沿
                TIM_OC4PolarityConfig(TIM4, TIM_ICPolarity_Falling);//设置为下降沿捕获
            }
        }
    }
    OSIntExit();
}

/*************************************************定时器5*************************************************/

//定时器5输入捕获配置


u32 tim5_T1;
u32 tim5_T2;
u32 tim5_T3;
u32 tim5_T4;

u8 TIM5CH1_CAPTURE_STA = 0;    //通道1输入捕获标志，高两位做捕获标志，低6位做溢出标志        
u16 TIM5CH1_CAPTURE_UPVAL;
u16 TIM5CH1_CAPTURE_DOWNVAL;

u8 TIM5CH2_CAPTURE_STA = 0;    //通道2输入捕获标志，高两位做捕获标志，低6位做溢出标志        
u16 TIM5CH2_CAPTURE_UPVAL;
u16 TIM5CH2_CAPTURE_DOWNVAL;

u8 TIM5CH3_CAPTURE_STA = 0;    //通道3输入捕获标志，高两位做捕获标志，低6位做溢出标志        
u16 TIM5CH3_CAPTURE_UPVAL;
u16 TIM5CH3_CAPTURE_DOWNVAL;

u8 TIM5CH4_CAPTURE_STA = 0;    //通道1输入捕获标志，高两位做捕获标志，低6位做溢出标志        
u16 TIM5CH4_CAPTURE_UPVAL;
u16 TIM5CH4_CAPTURE_DOWNVAL;

TIM_ICInitTypeDef TIM5_ICInitStructure;

void TIM5_Cap_Init(u16 arr, u16 psc) //在STM32F103CBT6单片机上使用这个函数能够正确初始化TIM2,若在其他型号单片机上需要查看DATASHEET，可能会与TIM5冲突
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);    //使能TIM5时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  //使能GPIOB时钟

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2
            | GPIO_Pin_3;  //PB6,7,8,9 清除之前设置  
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; //PB6,7,8,9 输入 
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOA, GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3);//PB6,7,8,9  下拉

    //初始化定时器4 TIM5     
    TIM_TimeBaseStructure.TIM_Period = arr; //设定计数器自动重装值 
    TIM_TimeBaseStructure.TIM_Prescaler = psc;     //预分频器 
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位

    //初始化TIM5输入捕获参数 通道1
    TIM5_ICInitStructure.TIM_Channel = TIM_Channel_1; //CC1S=01     选择输入端 IC1映射到TI1上
    TIM5_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;    //上升沿捕获
    TIM5_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
    TIM5_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;      //配置输入分频,不分频 
    TIM5_ICInitStructure.TIM_ICFilter = 0x00;      //IC1F=0000 配置输入滤波器 不滤波
    TIM_ICInit(TIM5, &TIM5_ICInitStructure);

    //初始化TIM5输入捕获参数 通道2
    TIM5_ICInitStructure.TIM_Channel = TIM_Channel_2; //CC1S=01     选择输入端 IC1映射到TI1上
    TIM5_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;    //上升沿捕获
    TIM5_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
    TIM5_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;      //配置输入分频,不分频 
    TIM5_ICInitStructure.TIM_ICFilter = 0x00;      //IC1F=0000 配置输入滤波器 不滤波
    TIM_ICInit(TIM5, &TIM5_ICInitStructure);

    //初始化TIM5输入捕获参数 通道3
    TIM5_ICInitStructure.TIM_Channel = TIM_Channel_3; //CC1S=01     选择输入端 IC1映射到TI1上
    TIM5_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;    //上升沿捕获
    TIM5_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
    TIM5_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;      //配置输入分频,不分频 
    TIM5_ICInitStructure.TIM_ICFilter = 0x00;      //IC1F=0000 配置输入滤波器 不滤波
    TIM_ICInit(TIM5, &TIM5_ICInitStructure);

    //初始化TIM5输入捕获参数 通道4
    TIM5_ICInitStructure.TIM_Channel = TIM_Channel_4; //CC1S=01     选择输入端 IC1映射到TI1上
    TIM5_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;    //上升沿捕获
    TIM5_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
    TIM5_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;      //配置输入分频,不分频 
    TIM5_ICInitStructure.TIM_ICFilter = 0x00;      //IC1F=0000 配置输入滤波器 不滤波
    TIM_ICInit(TIM5, &TIM5_ICInitStructure);

    //中断分组初始化
    NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;  //TIM5中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //先占优先级1级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //从优先级0级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
    NVIC_Init(&NVIC_InitStructure);   //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器 

    TIM_ITConfig(TIM5,TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4,
            ENABLE);   //不允许更新中断，允许CC1IE,CC2IE,CC3IE,CC4IE捕获中断    

    TIM_Cmd(TIM5, ENABLE);         //使能定时器4

}

//定时器5中断服务程序
void TIM5_IRQHandler(void)
{
    OSIntEnter();
    if ((TIM5CH1_CAPTURE_STA & 0X80) == 0)         //还未成功捕获    
    {
        if (TIM_GetITStatus(TIM5, TIM_IT_CC1) != RESET)         //捕获1发生捕获事件
        {
            TIM_ClearITPendingBit(TIM5, TIM_IT_CC1);         //清除中断标志位
            if (TIM5CH1_CAPTURE_STA & 0X40)        //捕获到一个下降沿
            {
                TIM5CH1_CAPTURE_DOWNVAL = TIM_GetCapture1(TIM5);//记录下此时的定时器计数值
                if (TIM5CH1_CAPTURE_DOWNVAL < TIM5CH1_CAPTURE_UPVAL)
                {
                    tim5_T1 = 65535;
                }
                else
                    tim5_T1 = 0;
                rc.Tim5Ch1CapVal = TIM5CH1_CAPTURE_DOWNVAL - TIM5CH1_CAPTURE_UPVAL
                        + tim5_T1;        //得到总的高电平的时间
                TIM5CH1_CAPTURE_STA = 0;        //捕获标志位清零
                TIM_OC1PolarityConfig(TIM5, TIM_ICPolarity_Rising); //设置为上升沿捕获          
            }
            else //发生捕获时间但不是下降沿，第一次捕获到上升沿，记录此时的定时器计数值
            {
                TIM5CH1_CAPTURE_UPVAL = TIM_GetCapture1(TIM5);        //获取上升沿数据
                TIM5CH1_CAPTURE_STA |= 0X40;        //标记已捕获到上升沿
                TIM_OC1PolarityConfig(TIM5, TIM_ICPolarity_Falling);//设置为下降沿捕获
            }
        }
    }

    if ((TIM5CH2_CAPTURE_STA & 0X80) == 0)        //还未成功捕获    
    {
        if (TIM_GetITStatus(TIM5, TIM_IT_CC2) != RESET)        //捕获2发生捕获事件
        {
            TIM_ClearITPendingBit(TIM5, TIM_IT_CC2);        //清除中断标志位
            if (TIM5CH2_CAPTURE_STA & 0X40)        //捕获到一个下降沿
            {
                TIM5CH2_CAPTURE_DOWNVAL = TIM_GetCapture2(TIM5);//记录下此时的定时器计数值
                if (TIM5CH2_CAPTURE_DOWNVAL < TIM5CH2_CAPTURE_UPVAL)
                {
                    tim5_T2 = 65535;
                }
                else
                    tim5_T2 = 0;
                rc.Tim5Ch2CapVal = TIM5CH2_CAPTURE_DOWNVAL - TIM5CH2_CAPTURE_UPVAL
                        + tim5_T2;        //得到总的高电平的时间
                TIM5CH2_CAPTURE_STA = 0;        //捕获标志位清零
                TIM_OC2PolarityConfig(TIM5, TIM_ICPolarity_Rising); //设置为上升沿捕获          
            }
            else //发生捕获时间但不是下降沿，第一次捕获到上升沿，记录此时的定时器计数值
            {
                TIM5CH2_CAPTURE_UPVAL = TIM_GetCapture2(TIM5);        //获取上升沿数据
                TIM5CH2_CAPTURE_STA |= 0X40;        //标记已捕获到上升沿
                TIM_OC2PolarityConfig(TIM5, TIM_ICPolarity_Falling);//设置为下降沿捕获
            }
        }
    }

    if ((TIM5CH3_CAPTURE_STA & 0X80) == 0)        //还未成功捕获    
    {
        if (TIM_GetITStatus(TIM5, TIM_IT_CC3) != RESET)        //捕获3发生捕获事件
        {
            TIM_ClearITPendingBit(TIM5, TIM_IT_CC3);        //清除中断标志位
            if (TIM5CH3_CAPTURE_STA & 0X40)        //捕获到一个下降沿
            {
                TIM5CH3_CAPTURE_DOWNVAL = TIM_GetCapture3(TIM5);//记录下此时的定时器计数值
                if (TIM5CH3_CAPTURE_DOWNVAL < TIM5CH3_CAPTURE_UPVAL)
                {
                    tim5_T3 = 65535;
                }
                else
                    tim5_T3 = 0;
                rc.Tim5Ch3CapVal = TIM5CH3_CAPTURE_DOWNVAL - TIM5CH3_CAPTURE_UPVAL
                        + tim5_T3;        //得到总的高电平的时间
                TIM5CH3_CAPTURE_STA = 0;        //捕获标志位清零
                TIM_OC3PolarityConfig(TIM5, TIM_ICPolarity_Rising); //设置为上升沿捕获          
            }
            else //发生捕获时间但不是下降沿，第一次捕获到上升沿，记录此时的定时器计数值
            {
                TIM5CH3_CAPTURE_UPVAL = TIM_GetCapture3(TIM5);        //获取上升沿数据
                TIM5CH3_CAPTURE_STA |= 0X40;        //标记已捕获到上升沿
                TIM_OC3PolarityConfig(TIM5, TIM_ICPolarity_Falling);//设置为下降沿捕获
            }
        }
    }

    if ((TIM5CH4_CAPTURE_STA & 0X80) == 0)        //还未成功捕获    
    {
        if (TIM_GetITStatus(TIM5, TIM_IT_CC4) != RESET)        //捕获4发生捕获事件
        {
            TIM_ClearITPendingBit(TIM5, TIM_IT_CC4);        //清除中断标志位
            if (TIM5CH4_CAPTURE_STA & 0X40)        //捕获到一个下降沿
            {
                TIM5CH4_CAPTURE_DOWNVAL = TIM_GetCapture4(TIM5);//记录下此时的定时器计数值
                if (TIM5CH4_CAPTURE_DOWNVAL < TIM5CH4_CAPTURE_UPVAL)
                {
                    tim5_T4 = 65535;
                }
                else
                    tim5_T4 = 0;
                rc.Tim5Ch4CapVal = TIM5CH4_CAPTURE_DOWNVAL - TIM5CH4_CAPTURE_UPVAL
                        + tim5_T4;        //得到总的高电平的时间
                TIM5CH4_CAPTURE_STA = 0;        //捕获标志位清零
                TIM_OC4PolarityConfig(TIM5, TIM_ICPolarity_Rising); //设置为上升沿捕获          
            }
            else //发生捕获时间但不是下降沿，第一次捕获到上升沿，记录此时的定时器计数值
            {
                TIM5CH4_CAPTURE_UPVAL = TIM_GetCapture4(TIM5);        //获取上升沿数据
                TIM5CH4_CAPTURE_STA |= 0X40;        //标记已捕获到上升沿
                TIM_OC4PolarityConfig(TIM5, TIM_ICPolarity_Falling);//设置为下降沿捕获
            }
        }
    }
    OSIntExit();
}
/********************************************************************************************************/


void RemoteControllerCaptureInit()//遥控器信号捕获定时器初始化
{
    //TIM2_Cap_Init(0xffff,72-1); //PWM捕获初始化,以1Mhz的频率计数 CH1-CH4
    TIM3_Cap_Init(0xffff,72-1); //PWM捕获初始化,以1Mhz的频率计数 CH1-CH4
}
/********************************************************************************************************/

