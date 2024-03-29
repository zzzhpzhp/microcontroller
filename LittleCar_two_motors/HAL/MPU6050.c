/* MPU6050.c file
编写者：lisn3188
网址：www.chiplab7.com
作者E-mail：lisn3188@163.com
编译环境：MDK-Lite  Version: 4.23
初版时间: 2012-04-25
测试： 本程序已在第七实验室的mini IMU上完成测试
功能：
提供MPU6050 初始化 读取当前测量值的API
------------------------------------
 */

#include "MPU6050.h"
#include "IOI2C.h"
#include "LED.h"
#include "eeprom.h"

uint8_t buffer[14];

int16_t  MPU6050_FIFO[6][11];
int16_t Gx_offset=0,Gy_offset=0,Gz_offset=0;
float Acc1G_Values;



/**************************实现函数********************************************
*函数原型:    void MPU6050_setClockSource(uint8_t source)
*功　　能:      设置  MPU6050 的时钟源
 * CLK_SEL | Clock Source
 * --------+--------------------------------------
 * 0       | Internal oscillator
 * 1       | PLL with X Gyro reference
 * 2       | PLL with Y Gyro reference
 * 3       | PLL with Z Gyro reference
 * 4       | PLL with external 32.768kHz reference
 * 5       | PLL with external 19.2MHz reference
 * 6       | Reserved
 * 7       | Stops the clock and keeps the timing generator in reset
*******************************************************************************/
void MPU6050_setClockSource(uint8_t source){
    IICwriteBits(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);

}

/** Set full-scale gyroscope range.
 * @param range New full-scale gyroscope range value
 * @see getFullScaleRange()
 * @see MPU6050_GYRO_FS_250
 * @see MPU6050_RA_GYRO_CONFIG
 * @see MPU6050_GCONFIG_FS_SEL_BIT
 * @see MPU6050_GCONFIG_FS_SEL_LENGTH
 */
void MPU6050_setFullScaleGyroRange(uint8_t range) {
    IICwriteBits(devAddr, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
}

/**************************实现函数********************************************
*函数原型:    void MPU6050_setFullScaleAccelRange(uint8_t range)
*功　　能:      设置  MPU6050 加速度计的最大量程
*******************************************************************************/
void MPU6050_setFullScaleAccelRange(uint8_t range) {
    IICwriteBits(devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
}

/**************************实现函数********************************************
*函数原型:    void MPU6050_setSleepEnabled(uint8_t enabled)
*功　　能:      设置  MPU6050 是否进入睡眠模式
        enabled =1   睡觉
          enabled =0   工作
*******************************************************************************/
void MPU6050_setSleepEnabled(uint8_t enabled) {
    IICwriteBit(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
}

/**************************实现函数********************************************
*函数原型:    uint8_t MPU6050_getDeviceID(void)
*功　　能:      读取  MPU6050 WHO_AM_I 标识   将返回 0x68
*******************************************************************************/
uint8_t MPU6050_getDeviceID(void) {

    IICreadBytes(devAddr, MPU6050_RA_WHO_AM_I, 1, buffer);
    return buffer[0];
}

/**************************实现函数********************************************
*函数原型:    uint8_t MPU6050_testConnection(void)
*功　　能:      检测MPU6050 是否已经连接
*******************************************************************************/
uint8_t MPU6050_testConnection(void) {
   if(MPU6050_getDeviceID() == 0x68)  //0b01101000;
   return 1;
     else return 0;
}

/**************************实现函数********************************************
*函数原型:    void MPU6050_setI2CMasterModeEnabled(uint8_t enabled)
*功　　能:      设置 MPU6050 是否为AUX I2C线的主机
*******************************************************************************/
void MPU6050_setI2CMasterModeEnabled(uint8_t enabled) {
    IICwriteBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT, enabled);
}

/**************************实现函数********************************************
*函数原型:    void MPU6050_setI2CBypassEnabled(uint8_t enabled)
*功　　能:      设置 MPU6050 是否为AUX I2C线的主机
*******************************************************************************/
void MPU6050_setI2CBypassEnabled(uint8_t enabled) {
    IICwriteBit(devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_I2C_BYPASS_EN_BIT, enabled);
}

/**************************实现函数********************************************
*函数原型:    void MPU6050_initialize(void)
*功　　能:      初始化   MPU6050 以进入可用状态。
*******************************************************************************/
void MPU6050_initialize(void)
{
    MPU6050_setClockSource(MPU6050_CLOCK_PLL_XGYRO); //设置时钟
    MPU6050_setFullScaleGyroRange(MPU6050_GYRO_FS_2000);//陀螺仪最大量程 +-2000度每秒
    MPU6050_setFullScaleAccelRange(MPU6050_ACCEL_FS_2);  //加速度度最大量程 +-2G
    MPU6050_setSleepEnabled(0); //进入工作状态
    MPU6050_setI2CMasterModeEnabled(0);   //不让MPU6050 控制AUXI2C
    MPU6050_setI2CBypassEnabled(1);   //主控制器的I2C与  MPU6050的AUXI2C  直通。控制器可以直接访问HMC5883L


    //配置MPU6050 的中断模式 和中断电平模式
    IICwriteBit(devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_INT_LEVEL_BIT, 0);
    IICwriteBit(devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_INT_OPEN_BIT, 0);
    IICwriteBit(devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_LATCH_INT_EN_BIT, 1);
    IICwriteBit(devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_INT_RD_CLEAR_BIT, 1);
    //开数据转换完成中断
    IICwriteBit(devAddr, MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_DATA_RDY_BIT, 1);

}

/**************************实现函数********************************************
*函数原型:    unsigned char MPU6050_is_DRY(void)
*功　　能:      检查 MPU6050的中断引脚，测试是否完成转换
返回 1  转换完成
0 数据寄存器还没有更新
*******************************************************************************/
unsigned char MPU6050_is_DRY(void)
{
    if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6)==Bit_SET){
    return 1;
   }
   else return 0;
}



void MPU6050_GetRawData(int16_t *gx , int16_t *gy , int16_t *gz , int16_t *ax , int16_t *ay , int16_t *az) //得到最原始的、未经过滤波的数据
{
  uint8_t buffer[14];
  IICreadBytes(devAddr, MPU6050_RA_ACCEL_XOUT_H, 14, buffer);
  *ax=(buffer[0] << 8) | buffer[1];
  *ay=(buffer[2] << 8) | buffer[3];
  *az=(buffer[4] << 8) | buffer[5];
  //跳过温度ADC
  *gx=(buffer[8] << 8) | buffer[9];
  *gy=(buffer[10] << 8) | buffer[11];
  *gz=(buffer[12] << 8) | buffer[13];
}
void MPU6050GetAccel(int16_t *ax , int16_t *ay , int16_t *az)
{
  uint8_t buffer[6];
  IICreadBytes(devAddr, MPU6050_RA_ACCEL_XOUT_H, 5, buffer);
  *ax=(buffer[0] << 8) | buffer[1];
  *ay=(buffer[2] << 8) | buffer[3];
  *az=(buffer[4] << 8) | buffer[5];
}
void MPU6050GetGyro(int16_t *gx , int16_t *gy , int16_t *gz)
{
  uint8_t buffer[6];
  IICreadBytes(devAddr, MPU6050_RA_GYRO_XOUT_H, 5, buffer);
  *gx=(buffer[0] << 8) | buffer[1];
  *gy=(buffer[2] << 8) | buffer[3];
  *gz=(buffer[4] << 8) | buffer[5];
}
//------------------End of File----------------------------
