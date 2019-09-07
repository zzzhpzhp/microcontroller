#ifndef __MS5611_H
#define  __MS5611_H
#include "stm32f10x.h"
/* MPU6050 Register Address ------------------------------------------------------------*/
#define MS561101BA_ADC_RD          0x00
#define  MS561101BA_PROM_RD          0xA0
#define MS561101BA_PROM_CRC        0xAE

#define MS561101BA_SlaveAddress    0xEE  //MS5611�ĵ�ַ
#define MS561101BA_RST             0x1E  //cmd ��λ


#define MS5611_OSR256                0x40
#define MS5611_OSR512                0x42
#define MS5611_OSR1024             0x44
#define MS5611_OSR2048             0x46
#define MS5611_OSR4096             0x48
#define FILTER_NUM_Weighted 40//��Ȩ�˲��Ĵ���
#define FILTER_NUM_Average 40//ƽ���˲��������������˲�����������ͬ���ұ���Ϊż��

extern volatile float Pressure_Diff,P1,P2;



void MS5611_Init(void);
uint32_t MS5611_Read24bits(uint8_t command);
float MS5611_GetTemperature(uint8_t OSR_Temp);
float MS5611_GetPressure(uint8_t OSR_Pres);
float Get_Pressure_Diff(uint8_t OSR);//OSRΪѹǿת������


#endif
