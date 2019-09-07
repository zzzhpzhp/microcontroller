/* main.c file
编写者：lisn3188
网址：www.chiplab7.com
作者E-mail：lisn3188@163.com
编译环境：MDK-Lite  Version: 4.23
初版时间: 2012-04-25
测试： 本程序已在第七实验室的mini IMU上完成测试
功能：
姿态解算 IMU
将传感器的输出值进行姿态解算。得到目标载体的俯仰角和横滚角 和航向角
------------------------------------
 */

#include "IMU.h"
#include "MS5611.h"
#include "MPU6050.h"


imu_st imu;


void IMU_init(void)
{
  IIC_Init();   //初始化I2C接口
  MPU6050_initialize();
  HMC5883L_SetUp();
  MS5611_Init();
  delay_ms(50);
}

/*************************************************************************************************/
float Kp=10.0f;                                  // 比例增益支配率收敛到加速度计/磁强计
float Ki=0.008f;                                  // 积分增益支配率的陀螺仪偏见的衔接

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;          // 四元数的元素，代表估计方向
float exInt = 0, eyInt = 0, ezInt = 0;         // 按比例缩小积分误差
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az , float *OutPut_Yaw , float *OutPut_Roll , float *OutPut_Pitch)
{
  float norm;
  float vx, vy, vz;// wx, wy, wz;
  float ex, ey, ez;

  float q0q0 = q0*q0;
  float q0q1 = q0*q1;
  float q0q2 = q0*q2;
  float q1q1 = q1*q1;
  float q1q3 = q1*q3;
  float q2q2 = q2*q2;
  float q2q3 = q2*q3;
  float q3q3 = q3*q3;

  if(ax*ay*az==0)
     return;

  norm = sqrt(ax*ax + ay*ay + az*az);
  ax = ax / norm;
  ay = ay / norm;
  az = az / norm;

  // estimated direction of gravity and flux (v and w)
  vx = 2*(q1q3 - q0q2);
  vy = 2*(q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3 ;

  // error is sum of cross product between reference direction of fields and direction measured by sensors
  ex = (ay*vz - az*vy) ;
  ey = (az*vx - ax*vz) ;
  ez = (ax*vy - ay*vx) ;

  exInt = exInt + ex * Ki;
  eyInt = eyInt + ey * Ki;
  ezInt = ezInt + ez * Ki;

  // adjusted gyroscope measurements
  gx = gx + Kp*ex + exInt;
  gy = gy + Kp*ey + eyInt;
  gz = gz + Kp*ez + ezInt;

  // integrate quaternion rate and normalise
  q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
  q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
  q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
  q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;

  // normalise quaternion
  norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);

  q0 = q0 / norm;
  q1 = q1 / norm;
  q2 = q2 / norm;
  q3 = q3 / norm;

//T21/T11
//  *OutPut_Yaw = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3* q3 + 1)* 57.3; // Yaw仅由地磁提供 57.3为Rad  1为四元数四个系数的平方的和
  *OutPut_Roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; // roll  T32/T33
  *OutPut_Pitch = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3; // pitch   T31
}
/*************************************************************************************************/


void IMU_Gyro_Offset()
{
#define FILTER_NUM 100
  unsigned int i;
  long sum_x=0;
  long sum_y=0;
  long sum_z=0;
  for(i=0;i<FILTER_NUM;i++)
  {
    GET_GYRO
    DELAY
    sum_x += imu.Gyro_x;
    sum_y += imu.Gyro_y;
    sum_z += imu.Gyro_z;
  }
  imu.Gyro_x_Offset = -sum_x / FILTER_NUM;
  imu.Gyro_y_Offset = -sum_y / FILTER_NUM;
  imu.Gyro_z_Offset = -sum_z / FILTER_NUM;
#undef FILTER_NUM
}
void IMU_Accel_Offset()
{
#define FILTER_NUM 100
  unsigned int i;
  long sum_x=0;
  long sum_y=0;
  long sum_z=0;
  for(i=0;i<FILTER_NUM;i++)
  {
    GET_ACCEL
    DELAY
    sum_x += imu.Accel_x;
    sum_y += imu.Accel_y;
    sum_z += imu.Accel_z;
  }
  imu.Accel_x_Offset = -sum_x / FILTER_NUM;
  imu.Accel_y_Offset = -sum_y / FILTER_NUM;
  imu.Accel_z_Offset = -sum_z / FILTER_NUM;
#undef FILTER_NUM
}

void IMU_AccelDataFilter()
{
#define FILTER_NUM 10

  unsigned int i,DatCnt=0;
  int16_t AccelBuf_x[FILTER_NUM]={0};
  int16_t AccelBuf_y[FILTER_NUM]={0};
  int16_t AccelBuf_z[FILTER_NUM]={0};
  long sum_accel_x=0;
  long sum_accel_y=0;
  long sum_accel_z=0;
  AccelBuf_x[DatCnt] = imu.Accel_x;
  AccelBuf_y[DatCnt] = imu.Accel_y;
  AccelBuf_z[DatCnt] = imu.Accel_z;
  DatCnt++;
  if(DatCnt == FILTER_NUM)
    DatCnt = 0;
  for(i=0;i<FILTER_NUM;i++)
  {
    sum_accel_x += AccelBuf_x[i];
    sum_accel_y += AccelBuf_y[i];
    sum_accel_z += AccelBuf_z[i];
  }
  imu.Accel_x_AfterFilter = sum_accel_x / FILTER_NUM;
  imu.Accel_y_AfterFilter = sum_accel_y / FILTER_NUM;
  imu.Accel_z_AfterFilter = sum_accel_z / FILTER_NUM;
#undef FILTER_NUM
}


#define Get_IMU_Data { \
                        MPU6050_GetRawData(&imu.Gyro_x, &imu.Gyro_y, &imu.Gyro_z ,\
                        &imu.Accel_x, &imu.Accel_y, &imu.Accel_z); \
                        HMC5883_GetRawData(&imu.Mag_x, &imu.Mag_y, &imu.Mag_z); \
                      }

void IMU_DataGetAndFilter(void)
{
  #define FILTER_NUM 10

  unsigned int i;
  static unsigned int DatCnt=0;
  static int16_t AccelBuf_x[FILTER_NUM]={0};
  static int16_t AccelBuf_y[FILTER_NUM]={0};
  static int16_t AccelBuf_z[FILTER_NUM]={0};

  static int16_t GyroBuf_x[FILTER_NUM]={0};
  static int16_t GyroBuf_y[FILTER_NUM]={0};
  static int16_t GyroBuf_z[FILTER_NUM]={0};

  static int16_t MagBuf_x[FILTER_NUM]={0};
  static int16_t MagBuf_y[FILTER_NUM]={0};
  static int16_t MagBuf_z[FILTER_NUM]={0};

  long sum_accel_x=0;
  long sum_accel_y=0;
  long sum_accel_z=0;

  long sum_gyro_x=0;
  long sum_gyro_y=0;
  long sum_gyro_z=0;

  long sum_mag_x=0;
  long sum_mag_y=0;
  long sum_mag_z=0;

  Get_IMU_Data

  AccelBuf_x[DatCnt] = imu.Accel_x;
  AccelBuf_y[DatCnt] = imu.Accel_y;
  AccelBuf_z[DatCnt] = imu.Accel_z;

  GyroBuf_x[DatCnt] = imu.Gyro_x;
  GyroBuf_y[DatCnt] = imu.Gyro_y;
  GyroBuf_z[DatCnt] = imu.Gyro_z;

  MagBuf_x[DatCnt] = imu.Mag_x;
  MagBuf_y[DatCnt] = imu.Mag_y;
  MagBuf_z[DatCnt] = imu.Mag_z;
  DatCnt++;
  if(DatCnt == FILTER_NUM)
  {
    DatCnt = 0;

    sum_accel_x=0;
    sum_accel_y=0;
    sum_accel_z=0;

    sum_gyro_x=0;
    sum_gyro_y=0;
    sum_gyro_z=0;

    sum_mag_x=0;
    sum_mag_y=0;
    sum_mag_z=0;
  }
  for(i=0;i<FILTER_NUM;i++)
  {
    sum_accel_x += AccelBuf_x[i];
    sum_accel_y += AccelBuf_y[i];
    sum_accel_z += AccelBuf_z[i];

    sum_gyro_x += GyroBuf_x[i];
    sum_gyro_y += GyroBuf_y[i];
    sum_gyro_z += GyroBuf_z[i];

    sum_mag_x += MagBuf_x[i];
    sum_mag_y += MagBuf_y[i];
    sum_mag_z += MagBuf_z[i];
  }
  imu.Accel_x_AfterFilter = sum_accel_x / FILTER_NUM + imu.Accel_x_Offset;
  imu.Accel_y_AfterFilter = sum_accel_y / FILTER_NUM + imu.Accel_y_Offset;
  imu.Accel_z_AfterFilter = sum_accel_z / FILTER_NUM + imu.Accel_z_Offset;

  imu.Gyro_x_AfterFilter = sum_gyro_x / FILTER_NUM + imu.Gyro_x_Offset;
  imu.Gyro_y_AfterFilter = sum_gyro_y / FILTER_NUM + imu.Gyro_y_Offset;
  imu.Gyro_z_AfterFilter = sum_gyro_z / FILTER_NUM + imu.Gyro_z_Offset;

  imu.Mag_x_AfterFilter = sum_mag_x / FILTER_NUM;
  imu.Mag_y_AfterFilter = sum_mag_y / FILTER_NUM;
  imu.Mag_z_AfterFilter = sum_mag_z / FILTER_NUM;

  imu.yaw_mag = atan2((double)imu.Mag_y_AfterFilter,(double)imu.Mag_x_AfterFilter) * (180 / 3.14159265) + 180;
  #undef FILTER_NUM
}





//------------------End of File----------------------------
