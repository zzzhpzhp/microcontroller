/* IOI2C.c file
编写者：lisn3188
网址：www.chiplab7.com
作者E-mail：lisn3188@163.com
编译环境：MDK-Lite  Version: 4.23
初版时间: 2012-04-25
测试： 本程序已在第七实验室的mini IMU上完成测试
功能：
提供I2C接口操作API 。
使用IO模拟方式
------------------------------------
 */
#include "IOI2C.h"
#include "delay.h"
static void IICDelay(u16 del)
{
  u16 i,j;
  for(i=0;i<del;i++)
    for(j=0;j<20;j++);
  
}
 void SDA_IN()
 {
    GPIO_InitTypeDef GPIO_InitStructure;  
    
    GPIO_InitStructure.GPIO_Pin=I2C_SDA;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;
    GPIO_Init(GPIO_I2C,&GPIO_InitStructure);
 }
 void SDA_OUT()
 {
    GPIO_InitTypeDef GPIO_InitStructure;  
    
    GPIO_InitStructure.GPIO_Pin=I2C_SDA;
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_OD;
    GPIO_Init(GPIO_I2C,&GPIO_InitStructure);
 }
/**************************实现函数********************************************
*函数原型:    void IIC_Init(void)
*功　　能:    初始化I2C对应的接口引脚。
*******************************************************************************/
void IIC_Init(void)
{      
  GPIO_InitTypeDef GPIO_InitStructure;  
  
   RCC_APB2PeriphClockCmd(I2C_PORT_CLOCK, ENABLE);  
  GPIO_InitStructure.GPIO_Pin=I2C_SCL|I2C_SDA;
  GPIO_InitStructure.GPIO_Speed=GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_OD;
  GPIO_Init(GPIO_I2C,&GPIO_InitStructure);
}

/**************************实现函数********************************************
*函数原型:    void IIC_Start(void)
*功　　能:    产生IIC起始信号
*******************************************************************************/
void IIC_Start(void)
{
  SDA_OUT();     //sda线输出
  I2C_SDA_H;        
  I2C_SCL_H;
  IICDelay(4);
   I2C_SDA_L;//START:when CLK is high,DATA change form high to low 
  IICDelay(4);
  I2C_SCL_L;//钳住I2C总线，准备发送或接收数据 
}

/**************************实现函数********************************************
*函数原型:    void IIC_Stop(void)
*功　　能:      //产生IIC停止信号
*******************************************************************************/    
void IIC_Stop(void)
{
  SDA_OUT();//sda线输出
  I2C_SCL_L;
  I2C_SDA_L;//STOP:when CLK is high DATA change form low to high
   IICDelay(4);
  I2C_SCL_H; 
  I2C_SDA_H;//发送I2C总线结束信号
  IICDelay(4);                   
}

/**************************实现函数********************************************
*函数原型:    u8 IIC_Wait_Ack(void)
*功　　能:      等待应答信号到来 
//返回值：1，接收应答失败
//        0，接收应答成功
*******************************************************************************/
u8 IIC_Wait_Ack(void)
{
  u8 ucErrTime=0;
  SDA_IN();      //SDA设置为输入  
  I2C_SDA_H;IICDelay(1);     
  I2C_SCL_H;IICDelay(1);   
  while(READ_SDA)
  {
    ucErrTime++;
    if(ucErrTime>50)
    {
      IIC_Stop();
      return 1;
    }
    IICDelay(1);
  }
  I2C_SCL_L;//时钟输出0      
  return 0;  
} 

/**************************实现函数********************************************
*函数原型:    void IIC_Ack(void)
*功　　能:      产生ACK应答
*******************************************************************************/
void IIC_Ack(void)
{
  I2C_SCL_L;
  SDA_OUT();
  I2C_SDA_L;
  IICDelay(2);
  I2C_SCL_H;
  IICDelay(2);
  I2C_SCL_L;
}
  
/**************************实现函数********************************************
*函数原型:    void IIC_NAck(void)
*功　　能:      产生NACK应答
*******************************************************************************/      
void IIC_NAck(void)
{
  I2C_SCL_L;
  SDA_OUT();
  I2C_SDA_H;
  IICDelay(2);
  I2C_SCL_H;
  IICDelay(2);
  I2C_SCL_L;
}                        

/**************************实现函数********************************************
*函数原型:    void IIC_Send_Byte(u8 txd)
*功　　能:      IIC发送一个字节
*******************************************************************************/      
void IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
  SDA_OUT();       
    I2C_SCL_L;//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
    if((txd&0x80)>0) //0x80  1000 0000
      I2C_SDA_H;
    else
      I2C_SDA_L;
        txd<<=1;     
    IICDelay(2);   
    I2C_SCL_H;
    IICDelay(2); 
    I2C_SCL_L;  
    IICDelay(2);
    }   
}    
   
/**************************实现函数********************************************
*函数原型:    u8 IIC_Read_Byte(unsigned char ack)
*功　　能:      //读1个字节，ack=1时，发送ACK，ack=0，发送nACK 
*******************************************************************************/  
u8 IIC_Read_Byte(unsigned char ack)
{
  unsigned char i,receive=0;
  SDA_IN();//SDA设置为输入
    for(i=0;i<8;i++ )
  {
        I2C_SCL_L; 
        IICDelay(2);
    I2C_SCL_H;
        receive<<=1;
        if(READ_SDA)receive++;   
    IICDelay(2); 
    }           
    if (ack)
        IIC_Ack(); //发送ACK 
    else
        IIC_NAck();//发送nACK  
    return receive;
}

/**************************实现函数********************************************
*函数原型:    unsigned char I2C_ReadOneByte(unsigned char I2C_Addr,unsigned char addr)
*功　　能:      读取指定设备 指定寄存器的一个值
输入  I2C_Addr  目标设备地址
    addr     寄存器地址
返回   读出来的值
*******************************************************************************/ 
unsigned char I2C_ReadOneByte(unsigned char I2C_Addr,unsigned char addr)
{
  unsigned char res=0;
  
  IIC_Start();  
  IIC_Send_Byte(I2C_Addr);     //发送写命令
  res++;
  IIC_Wait_Ack();
  IIC_Send_Byte(addr); res++;  //发送地址
  IIC_Wait_Ack();    
  //IIC_Stop();//产生一个停止条件  
  IIC_Start();
  IIC_Send_Byte(I2C_Addr+1); res++;          //进入接收模式         
  IIC_Wait_Ack();
  res=IIC_Read_Byte(0);     
    IIC_Stop();//产生一个停止条件

  return res;
}


/**************************实现函数********************************************
*函数原型:    u8 IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data)
*功　　能:      读取指定设备 指定寄存器的 length个值
输入  dev  目标设备地址
    reg    寄存器地址
    length 要读的字节数
    *data  读出的数据将要存放的指针
返回   读出来的字节数量
*******************************************************************************/ 
u8 IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data){
    u8 count = 0;
  
  IIC_Start();
  IIC_Send_Byte(dev);     //发送写命令
  IIC_Wait_Ack();
  IIC_Send_Byte(reg);   //发送地址
    IIC_Wait_Ack();    
  IIC_Start();
  IIC_Send_Byte(dev+1);  //进入接收模式  
  IIC_Wait_Ack();
  
    for(count=0;count<length;count++){
     
     if(count!=length-1)data[count]=IIC_Read_Byte(1);  //带ACK的读数据
       else  data[count]=IIC_Read_Byte(0);   //最后一个字节NACK
  }
    IIC_Stop();//产生一个停止条件
    return count;
}

/**************************实现函数********************************************
*函数原型:    u8 IICwriteBytes(u8 dev, u8 reg, u8 length, u8* data)
*功　　能:      将多个字节写入指定设备 指定寄存器
输入  dev  目标设备地址
    reg    寄存器地址
    length 要写的字节数
    *data  将要写的数据的首地址
返回   返回是否成功
*******************************************************************************/ 
u8 IICwriteBytes(u8 dev, u8 reg, u8 length, u8* data){
  
   u8 count = 0;
  IIC_Start();
  IIC_Send_Byte(dev);     //发送写命令
  IIC_Wait_Ack();
  IIC_Send_Byte(reg);   //发送地址
    IIC_Wait_Ack();    
  for(count=0;count<length;count++){
    IIC_Send_Byte(data[count]); 
    IIC_Wait_Ack(); 
   }
  IIC_Stop();//产生一个停止条件

    return 1; //status == 0;
}

/**************************实现函数********************************************
*函数原型:    u8 IICreadByte(u8 dev, u8 reg, u8 *data)
*功　　能:      读取指定设备 指定寄存器的一个值
输入  dev  目标设备地址
    reg     寄存器地址
    *data  读出的数据将要存放的地址
返回   1
*******************************************************************************/ 
u8 IICreadByte(u8 dev, u8 reg, u8 *data){
  *data=I2C_ReadOneByte(dev, reg);
    return 1;
}

/**************************实现函数********************************************
*函数原型:    unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data)
*功　　能:      写入指定设备 指定寄存器一个字节
输入  dev  目标设备地址
    reg     寄存器地址
    data  将要写入的字节
返回   1
*******************************************************************************/ 
unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data){
    return IICwriteBytes(dev, reg, 1, &data);
}

/**************************实现函数********************************************
*函数原型:    u8 IICwriteBits(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data)
*功　　能:      读 修改 写 指定设备 指定寄存器一个字节 中的多个位
输入  dev  目标设备地址
    reg     寄存器地址
    bitStart  目标字节的起始位
    length   位长度
    data    存放改变目标字节位的值
返回   成功 为1 
     失败为0
*******************************************************************************/ 
u8 IICwriteBits(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data)
{

    u8 b;
    if (IICreadByte(dev, reg, &b) != 0) {
        u8 mask = (0xFF << (bitStart + 1)) | 0xFF >> ((8 - bitStart) + length - 1);
        data <<= (8 - length);
        data >>= (7 - bitStart);
        b &= mask;
        b |= data;
        return IICwriteByte(dev, reg, b);
    } else {
        return 0;
    }
}

/**************************实现函数********************************************
*函数原型:    u8 IICwriteBit(u8 dev, u8 reg, u8 bitNum, u8 data)
*功　　能:      读 修改 写 指定设备 指定寄存器一个字节 中的1个位
输入  dev  目标设备地址
    reg     寄存器地址
    bitNum  要修改目标字节的bitNum位
    data  为0 时，目标位将被清0 否则将被置位
返回   成功 为1 
     失败为0
*******************************************************************************/ 
u8 IICwriteBit(u8 dev, u8 reg, u8 bitNum, u8 data){
    u8 b;
    IICreadByte(dev, reg, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return IICwriteByte(dev, reg, b);
}

//------------------End of File----------------------------
