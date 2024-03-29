/* eeprom.c file
编写者：lisn3188
网址：www.chiplab7.com
作者E-mail：lisn3188@163.com
编译环境：MDK-Lite  Version: 4.23
初版时间: 2012-05-05
测试： 本程序已在第七实验室的mini IMU上完成测试
功能：
将Flash用作EEPROM 用于保存偏置和标定数据
------------------------------------
 */
#include "eeprom.h"

#define  PAGE_Gyro   (0x08000000 + 60 * 1024) //将陀螺仪的偏置存放在第60页Flash
#define  PAGE_Magic  (0x08000000 + 61 * 1024) //将磁力计的标定值存放在第61页Flash



/**************************实现函数********************************************
*函数原型:	   void Read_Gyro_Offset(int16_t *offset_gx,int16_t *offset_gy,int16_t *offset_gz)
*功　　能:	   读取存放在Flash中的陀螺仪偏置数据。
输入参数：  三个轴的偏置，存放指针
输出参数：  无
*******************************************************************************/
void Read_Gyro_Offset(int16_t *offset_gx,int16_t *offset_gy,int16_t *offset_gz)                                
{
    int16_t *temp_addr = (int16_t *)PAGE_Gyro;
	*offset_gx=*temp_addr;
	temp_addr++;
    *offset_gy=*temp_addr;
	temp_addr++;
    *offset_gz=*temp_addr;                                                                                             
}

/**************************实现函数********************************************
*函数原型:	   void Write_Gyro_Offset(int16_t offset_gx,int16_t offset_gy,int16_t offset_gz)
*功　　能:	   将陀螺仪偏置存放到Flash
输入参数：  三个轴的偏置
输出参数：  无
*******************************************************************************/
void Write_Gyro_Offset(int16_t offset_gx,int16_t offset_gy,int16_t offset_gz)
{
 FLASH_Status temp_stat;
 uint32_t temp_addr = PAGE_Gyro;
 FLASH_Unlock();
 temp_stat = FLASH_ErasePage(PAGE_Gyro); 
 if(temp_stat != FLASH_COMPLETE)
        {
          FLASH_Lock();
          return ;
        }

temp_stat = FLASH_ProgramHalfWord(temp_addr,offset_gx);
if(temp_stat != FLASH_COMPLETE)
{
    FLASH_Lock();
    return ;
}
temp_addr+=2;

temp_stat = FLASH_ProgramHalfWord(temp_addr,offset_gy);
if(temp_stat != FLASH_COMPLETE)
{
    FLASH_Lock();
    return ;
}
temp_addr+=2;

temp_stat = FLASH_ProgramHalfWord(temp_addr,offset_gz);
if(temp_stat != FLASH_COMPLETE)
{
    FLASH_Lock();
    return ;
}
temp_addr+=2;

FLASH_Lock();    
}

/**************************实现函数********************************************
*函数原型:	   void Read_Magic_Offset(int16_t *min_mx,int16_t *min_my,int16_t *min_mz,
										int16_t *max_mx,int16_t *max_my,int16_t *max_mz)
*功　　能:	   从Flash读取磁力计标定
输入参数：  磁力计三个轴的对应的最大值和最小值
输出参数：  无
*******************************************************************************/
void Read_Magic_Offset(int16_t *min_mx,int16_t *min_my,int16_t *min_mz,
int16_t *max_mx,int16_t *max_my,int16_t *max_mz)                                
{
    int16_t *temp_addr = (int16_t *)PAGE_Magic;
	*min_mx=*temp_addr;
	temp_addr++;
    *min_my=*temp_addr;
	temp_addr++;
    *min_mz=*temp_addr;   
	temp_addr++;

	*max_mx=*temp_addr;   
	temp_addr++;
	*max_my=*temp_addr;   
	temp_addr++;
	*max_mz=*temp_addr;                                                                  
}

/**************************实现函数********************************************
*函数原型:	   void Write_Magic_Offset(int16_t min_mx,int16_t min_my,int16_t min_mz,
							int16_t max_mx,int16_t max_my,int16_t max_mz)
*功　　能:	   将磁力计标定写到Flash
输入参数：  磁力计三个轴的对应的最大值和最小值
输出参数：  无
*******************************************************************************/
void Write_Magic_Offset(int16_t min_mx,int16_t min_my,int16_t min_mz,
int16_t max_mx,int16_t max_my,int16_t max_mz)
{
 FLASH_Status temp_stat;
 uint32_t temp_addr = PAGE_Magic;
 FLASH_Unlock();
 temp_stat = FLASH_ErasePage(PAGE_Gyro); 
 if(temp_stat != FLASH_COMPLETE)
        {
          FLASH_Lock();
          return ;
        }

temp_stat = FLASH_ProgramHalfWord(temp_addr,min_mx);
if(temp_stat != FLASH_COMPLETE)
{
    FLASH_Lock();
    return ;
}
temp_addr+=2;

temp_stat = FLASH_ProgramHalfWord(temp_addr,min_my);
if(temp_stat != FLASH_COMPLETE)
{
    FLASH_Lock();
    return ;
}
temp_addr+=2;

temp_stat = FLASH_ProgramHalfWord(temp_addr,min_mz);
if(temp_stat != FLASH_COMPLETE)
{
    FLASH_Lock();
    return ;
}
temp_addr+=2;

temp_stat = FLASH_ProgramHalfWord(temp_addr,max_mx);
if(temp_stat != FLASH_COMPLETE)
{
    FLASH_Lock();
    return ;
}
temp_addr+=2;

temp_stat = FLASH_ProgramHalfWord(temp_addr,max_my);
if(temp_stat != FLASH_COMPLETE)
{
    FLASH_Lock();
    return ;
}
temp_addr+=2;

temp_stat = FLASH_ProgramHalfWord(temp_addr,max_mz);
if(temp_stat != FLASH_COMPLETE)
{
    FLASH_Lock();
    return ;
}

FLASH_Lock();    
}

//------------------End of File----------------------------
