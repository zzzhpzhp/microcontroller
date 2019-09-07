#ifndef __IMU_H
#define __IMU_H

#include "MPU6050.h"
#include "HMC5883L.h"
#include "UARTs.h"
#include "delay.h"

#include <math.h>
#define Gyro_Gr	0.0010653		//3.14/(180*16.4)=0.0010642251   AD采集值/精度 - 零点中值 = 角速度   当前精度为16.4 LSB/度/秒
#define DEG_TO_RAD	0.0174444		//Degree/360*2*PI 将角度转化为弧度
#define GET_GYRO {MPU6050GetGyro(&imu.Gyro_x,&imu.Gyro_y,&imu.Gyro_z);}
#define GET_ACCEL  {MPU6050GetAccel(&imu.Accel_x,&imu.Accel_y,&imu.Accel_z);}
#define Get_Mag  {HMC5883_GetRawData(&imu.Mag_x, &imu.Mag_y, &imu.Mag_z); Yaw = atan2((double)imu.Mag_y_AfterFilter,(double)imu.Mag_x_AfterFilter) * (180 / 3.14159265) + 180;}
#define DELAY {delay_us(200);}
#define halfT 0.025f                              // 采样周期的一半 若更改了周期，要记得修改这个参数，单位 S

typedef struct IMU_Typedef
{
    float Yaw;
    float Roll;
    float Pitch;
    float yaw_mag;
    float Pressure_Diff;

    int16_t Accel_x;   //X轴加速度值
    int16_t Accel_y;   //Y轴加速度值
    int16_t Accel_z;   //Z轴加速度值

    int16_t Gyro_x;     //X轴陀螺仪数据
    int16_t Gyro_y;    //Y轴陀螺仪数据
    int16_t Gyro_z;     //Z轴陀螺仪数据

    int16_t Mag_x;     //X轴陀螺仪数据
    int16_t Mag_y;    //Y轴陀螺仪数据
    int16_t Mag_z;     //Z轴陀螺仪数据

    float Accel_x_Offset;   //X轴加速度值水平校准
    float Accel_y_Offset;   //Y轴加速度值水平校准
    float Accel_z_Offset;  //Z轴加速度值水平校准

    float Gyro_x_Offset;   //X轴陀螺仪数据水平校准
    float Gyro_y_Offset;   //Y轴陀螺仪数据水平校准
    float Gyro_z_Offset;   //Z轴陀螺仪数据水平校准

    float Accel_x_AfterFilter;
    float Accel_y_AfterFilter;
    float Accel_z_AfterFilter;

    float Gyro_x_AfterFilter;
    float Gyro_y_AfterFilter;
    float Gyro_z_AfterFilter;

    float Mag_x_AfterFilter;
    float Mag_y_AfterFilter;
    float Mag_z_AfterFilter;
} imu_st;

extern imu_st imu;

void IMU_Gyro_Offset(void);
void IMU_Accel_Offset(void);
void IMU_AccelDataFilter(void);

//Mini IMU AHRS 解算的API
void IMU_init(void); //初始化
void IMU_getYawPitchRoll(float * ypr); //更新姿态
uint32_t micros(void); //读取系统上电后的时间  单位 us 
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az , float *OutPut_Yaw , float *OutPut_Roll , float *OutPut_Pitch);
void IMU_DataGetAndFilter(void);
#endif

//------------------End of File----------------------------
