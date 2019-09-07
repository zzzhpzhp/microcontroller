/* IOI2C.c file
��д�ߣ�lisn3188
��ַ��www.chiplab7.com
����E-mail��lisn3188@163.com
���뻷����MDK-Lite  Version: 4.23
����ʱ��: 2012-04-25
���ԣ� ���������ڵ���ʵ���ҵ�mini IMU����ɲ���
���ܣ�
�ṩI2C�ӿڲ���API ��
ʹ��IOģ�ⷽʽ
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
/**************************ʵ�ֺ���********************************************
*����ԭ��:    void IIC_Init(void)
*��������:    ��ʼ��I2C��Ӧ�Ľӿ����š�
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

/**************************ʵ�ֺ���********************************************
*����ԭ��:    void IIC_Start(void)
*��������:    ����IIC��ʼ�ź�
*******************************************************************************/
void IIC_Start(void)
{
  SDA_OUT();     //sda�����
  I2C_SDA_H;        
  I2C_SCL_H;
  IICDelay(4);
   I2C_SDA_L;//START:when CLK is high,DATA change form high to low 
  IICDelay(4);
  I2C_SCL_L;//ǯסI2C���ߣ�׼�����ͻ�������� 
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:    void IIC_Stop(void)
*��������:      //����IICֹͣ�ź�
*******************************************************************************/    
void IIC_Stop(void)
{
  SDA_OUT();//sda�����
  I2C_SCL_L;
  I2C_SDA_L;//STOP:when CLK is high DATA change form low to high
   IICDelay(4);
  I2C_SCL_H; 
  I2C_SDA_H;//����I2C���߽����ź�
  IICDelay(4);                   
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:    u8 IIC_Wait_Ack(void)
*��������:      �ȴ�Ӧ���źŵ��� 
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
*******************************************************************************/
u8 IIC_Wait_Ack(void)
{
  u8 ucErrTime=0;
  SDA_IN();      //SDA����Ϊ����  
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
  I2C_SCL_L;//ʱ�����0      
  return 0;  
} 

/**************************ʵ�ֺ���********************************************
*����ԭ��:    void IIC_Ack(void)
*��������:      ����ACKӦ��
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
  
/**************************ʵ�ֺ���********************************************
*����ԭ��:    void IIC_NAck(void)
*��������:      ����NACKӦ��
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

/**************************ʵ�ֺ���********************************************
*����ԭ��:    void IIC_Send_Byte(u8 txd)
*��������:      IIC����һ���ֽ�
*******************************************************************************/      
void IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
  SDA_OUT();       
    I2C_SCL_L;//����ʱ�ӿ�ʼ���ݴ���
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
   
/**************************ʵ�ֺ���********************************************
*����ԭ��:    u8 IIC_Read_Byte(unsigned char ack)
*��������:      //��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK 
*******************************************************************************/  
u8 IIC_Read_Byte(unsigned char ack)
{
  unsigned char i,receive=0;
  SDA_IN();//SDA����Ϊ����
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
        IIC_Ack(); //����ACK 
    else
        IIC_NAck();//����nACK  
    return receive;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:    unsigned char I2C_ReadOneByte(unsigned char I2C_Addr,unsigned char addr)
*��������:      ��ȡָ���豸 ָ���Ĵ�����һ��ֵ
����  I2C_Addr  Ŀ���豸��ַ
    addr     �Ĵ�����ַ
����   ��������ֵ
*******************************************************************************/ 
unsigned char I2C_ReadOneByte(unsigned char I2C_Addr,unsigned char addr)
{
  unsigned char res=0;
  
  IIC_Start();  
  IIC_Send_Byte(I2C_Addr);     //����д����
  res++;
  IIC_Wait_Ack();
  IIC_Send_Byte(addr); res++;  //���͵�ַ
  IIC_Wait_Ack();    
  //IIC_Stop();//����һ��ֹͣ����  
  IIC_Start();
  IIC_Send_Byte(I2C_Addr+1); res++;          //�������ģʽ         
  IIC_Wait_Ack();
  res=IIC_Read_Byte(0);     
    IIC_Stop();//����һ��ֹͣ����

  return res;
}


/**************************ʵ�ֺ���********************************************
*����ԭ��:    u8 IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data)
*��������:      ��ȡָ���豸 ָ���Ĵ����� length��ֵ
����  dev  Ŀ���豸��ַ
    reg    �Ĵ�����ַ
    length Ҫ�����ֽ���
    *data  ���������ݽ�Ҫ��ŵ�ָ��
����   ���������ֽ�����
*******************************************************************************/ 
u8 IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data){
    u8 count = 0;
  
  IIC_Start();
  IIC_Send_Byte(dev);     //����д����
  IIC_Wait_Ack();
  IIC_Send_Byte(reg);   //���͵�ַ
    IIC_Wait_Ack();    
  IIC_Start();
  IIC_Send_Byte(dev+1);  //�������ģʽ  
  IIC_Wait_Ack();
  
    for(count=0;count<length;count++){
     
     if(count!=length-1)data[count]=IIC_Read_Byte(1);  //��ACK�Ķ�����
       else  data[count]=IIC_Read_Byte(0);   //���һ���ֽ�NACK
  }
    IIC_Stop();//����һ��ֹͣ����
    return count;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:    u8 IICwriteBytes(u8 dev, u8 reg, u8 length, u8* data)
*��������:      ������ֽ�д��ָ���豸 ָ���Ĵ���
����  dev  Ŀ���豸��ַ
    reg    �Ĵ�����ַ
    length Ҫд���ֽ���
    *data  ��Ҫд�����ݵ��׵�ַ
����   �����Ƿ�ɹ�
*******************************************************************************/ 
u8 IICwriteBytes(u8 dev, u8 reg, u8 length, u8* data){
  
   u8 count = 0;
  IIC_Start();
  IIC_Send_Byte(dev);     //����д����
  IIC_Wait_Ack();
  IIC_Send_Byte(reg);   //���͵�ַ
    IIC_Wait_Ack();    
  for(count=0;count<length;count++){
    IIC_Send_Byte(data[count]); 
    IIC_Wait_Ack(); 
   }
  IIC_Stop();//����һ��ֹͣ����

    return 1; //status == 0;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:    u8 IICreadByte(u8 dev, u8 reg, u8 *data)
*��������:      ��ȡָ���豸 ָ���Ĵ�����һ��ֵ
����  dev  Ŀ���豸��ַ
    reg     �Ĵ�����ַ
    *data  ���������ݽ�Ҫ��ŵĵ�ַ
����   1
*******************************************************************************/ 
u8 IICreadByte(u8 dev, u8 reg, u8 *data){
  *data=I2C_ReadOneByte(dev, reg);
    return 1;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:    unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data)
*��������:      д��ָ���豸 ָ���Ĵ���һ���ֽ�
����  dev  Ŀ���豸��ַ
    reg     �Ĵ�����ַ
    data  ��Ҫд����ֽ�
����   1
*******************************************************************************/ 
unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data){
    return IICwriteBytes(dev, reg, 1, &data);
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:    u8 IICwriteBits(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data)
*��������:      �� �޸� д ָ���豸 ָ���Ĵ���һ���ֽ� �еĶ��λ
����  dev  Ŀ���豸��ַ
    reg     �Ĵ�����ַ
    bitStart  Ŀ���ֽڵ���ʼλ
    length   λ����
    data    ��Ÿı�Ŀ���ֽ�λ��ֵ
����   �ɹ� Ϊ1 
     ʧ��Ϊ0
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

/**************************ʵ�ֺ���********************************************
*����ԭ��:    u8 IICwriteBit(u8 dev, u8 reg, u8 bitNum, u8 data)
*��������:      �� �޸� д ָ���豸 ָ���Ĵ���һ���ֽ� �е�1��λ
����  dev  Ŀ���豸��ַ
    reg     �Ĵ�����ַ
    bitNum  Ҫ�޸�Ŀ���ֽڵ�bitNumλ
    data  Ϊ0 ʱ��Ŀ��λ������0 ���򽫱���λ
����   �ɹ� Ϊ1 
     ʧ��Ϊ0
*******************************************************************************/ 
u8 IICwriteBit(u8 dev, u8 reg, u8 bitNum, u8 data){
    u8 b;
    IICreadByte(dev, reg, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return IICwriteByte(dev, reg, b);
}

//------------------End of File----------------------------
