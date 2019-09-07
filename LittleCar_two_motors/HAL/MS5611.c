#include "MS5611.h"
#include "delay.h"
#include "math.h"
#include "IIC.h"

volatile float P1=0,P2=0;

uint32_t  Cal_C[7];          //用于存放PROM中的8组数据
uint32_t D1_Pres,D2_Temp;  // 存放压力和温度

uint64_t dT,TEMP;
uint64_t OFF_,SENS;
uint32_t Pressure;        //大气压
uint32_t TEMP2,Aux,OFF2,SENS2;  //温度校验值

/////////////////////////////////////////////////////////////////////////////////////////
#include "IOI2C.h"
#include "delay.h"
//MS5611 专用IIC读写程序
/*在指定地址写命令*/
void I2C_Write_Me_MS5611(u8 addr,u8 commands)
{
  IIC_Start();

  IIC_Send_Byte(addr);
  IIC_Wait_Ack();

  IIC_Send_Byte(commands);
   IIC_Wait_Ack();

  IIC_Stop();
}

/*在指定地址指定寄存器读值*/
u8 I2C_Read_Me(u8 addr1,u8 addr2,u8 reg)
{
  u8 p;
  IIC_Start();

  IIC_Send_Byte(addr1);
   IIC_Wait_Ack();

  IIC_Send_Byte(reg);
   IIC_Wait_Ack();

  IIC_Stop();
  IIC_Start();

  IIC_Send_Byte(addr2);
   IIC_Wait_Ack();

  p=IIC_Read_Byte(0);
  IIC_Stop();
  return p;
}

/*在指定地址读16位数*/
u16 I2C_Read_Me_MS5611_16BIT(u8 addr)
{
  u16 temp[2];
  IIC_Start();

  IIC_Send_Byte(addr);
  IIC_Wait_Ack();

  temp[0]=IIC_Read_Byte(1);

  temp[1]=IIC_Read_Byte(0);

  IIC_Stop();
  return (temp[0]<<8)+temp[1];
}

/*在指定地址读24位数*/
u32 I2C_Read_Me_MS5611_24BIT(u8 addr)
{
  u32 temp[3];
  IIC_Start();

  IIC_Send_Byte(addr);
  IIC_Wait_Ack();

  temp[0]=IIC_Read_Byte(1);

  temp[1]=IIC_Read_Byte(1);

  temp[2]=IIC_Read_Byte(0);
  IIC_Stop();
  return (temp[0]<<16)|(temp[1]<<8)|temp[2];
}
/////////////////////////////////////////////////////////////////////////////////////////

/*******************************************************************************
  * @函数名称  MS5611_Init
  * @函数说明   初始化5611
  * @输入参数    无
  * @输出参数   无
  * @返回参数   无
*******************************************************************************/
void MS5611_Init(void)
{
  uint8_t i;
  I2C_Write_Me_MS5611(MS561101BA_SlaveAddress,MS561101BA_RST);
  delay_ms(20);
  for(i=1;i<=6;i++)
  {
    I2C_Write_Me_MS5611(MS561101BA_SlaveAddress,MS561101BA_PROM_RD+i*2);
    delay_ms(10);
    Cal_C[i]=I2C_Read_Me_MS5611_16BIT(MS561101BA_SlaveAddress+1);
    delay_ms(10);
  }
  I2C_Write_Me_MS5611(MS561101BA_SlaveAddress,MS5611_OSR4096+0x10);
  delay_ms(10);
}

/*******************************************************************************
  * @函数名称     MS5611_Read24bits
  * @函数说明
  * @输入参数
  * @输出参数   24位温度值或者压强值
  * @返回参数   无
*******************************************************************************/
uint32_t MS5611_Read24bits(uint8_t command)
{

  uint32_t Read_Temp=0;

  I2C_Write_Me_MS5611(MS561101BA_SlaveAddress,0x00);

  Read_Temp=I2C_Read_Me_MS5611_24BIT(MS561101BA_SlaveAddress+1);

  I2C_Write_Me_MS5611(MS561101BA_SlaveAddress,command);//触发ADC开始转换数据；

  return Read_Temp;
}


/*******************************************************************************
  * @函数名称  MS5611_GetTemperature
  * @函数说明   获取温度值
  * @输入参数    转换频率
  * @输出参数   无
  * @返回参数   温度
*******************************************************************************/
float MS5611_GetTemperature(uint8_t OSR_Temp)
{

  float T_float;
  D2_Temp= MS5611_Read24bits(OSR_Temp);
  delay_us(10);
  dT=D2_Temp - (Cal_C[5]<<8);
  TEMP=2000+((dT*Cal_C[6])>>23);
  T_float=TEMP/100.0;
  return T_float;
}



/*******************************************************************************
  * @函数名称  MS5611_GetPressure
  * @函数说明   获得压强值
  * @输入参数     转换频率
  * @输出参数   无
  * @返回参数   压强
*******************************************************************************/
float MS5611_GetPressure(uint8_t OSR_Pres)
{
  float P_float;
  D1_Pres= MS5611_Read24bits(OSR_Pres);
  delay_us(10);
  OFF_=(Cal_C[2]<<16)+((Cal_C[4]*dT)>>7);
  SENS=(Cal_C[1]<<15)+((Cal_C[3]*dT)>>8);

  if(TEMP<2000)//温度补偿压强
  {
    Aux = TEMP*TEMP;
    OFF2 = 2.5*Aux;
    SENS2 = 1.25*Aux;

    TEMP = TEMP - TEMP2;
    OFF_ = OFF_ - OFF2;
    SENS = SENS - SENS2;
  }

  Pressure=((((D1_Pres*SENS)>>21)-OFF_)>>15);
  P_float=Pressure;
  return P_float;
}


/*******************************************************************************
  * @函数名称  Get_High
  * @函数说明   得到高度
  * @输入参数    转换频率
  * @输出参数   高度
  * @返回参数   高度
*******************************************************************************/
float Get_Pressure_Diff(uint8_t OSR)//OSR为压强转换速率
{
  static u16 filter_cnt_t=0,filter_cnt_p=0;
  static float t=0;
  static float P_BUF_Weighted[FILTER_NUM_Weighted];
  static float T_BUF_Weighted[FILTER_NUM_Weighted];
  u16 i;
  float P_TEMP_Weighted=0;
  float T_TEMP_Weighted=0;
  static u16 sample_me=0;
  sample_me++;

  if(sample_me==1)
  {

    t=MS5611_GetTemperature(OSR);//触发压强读数，进行ADC转换

    T_BUF_Weighted[filter_cnt_t]=t;
    for(i=0;i<FILTER_NUM_Weighted;i++)
    {
      T_TEMP_Weighted+=T_BUF_Weighted[i]*(i+1);
    }
    t=T_TEMP_Weighted/((1+FILTER_NUM_Weighted)*FILTER_NUM_Weighted/2.0); //加权滤波


    filter_cnt_t++;
    if(filter_cnt_t==FILTER_NUM_Weighted)  filter_cnt_t=0;
  }

  if(sample_me==2)
  {
    sample_me=0;
    P2=MS5611_GetPressure(OSR+0x10);//触发温度读数，进行ADC转换

    P_BUF_Weighted[filter_cnt_p]=P2;
    for(i=0;i<FILTER_NUM_Weighted;i++)
    {
      P_TEMP_Weighted+=P_BUF_Weighted[i]*(i+1);
    }
    P2=P_TEMP_Weighted/((1+FILTER_NUM_Weighted)*FILTER_NUM_Weighted/2.0);//加权滤波

    filter_cnt_p++;
    if(filter_cnt_p==FILTER_NUM_Weighted)  filter_cnt_p=0;
  }

  return P1-P2;

}










