#include "zhp_crc.h"

__IO uint32_t CRCValue = 0;		 // 用于存放产生的CRC校验值
/*
 * 函数名：CRC_Config
 * 描述  ：使能CRC时钟
 * 输入  ：无
 * 输出  ：无
 * 调用  : 外部调用
 */
void CRC_Config(void)
{
	/* Enable CRC clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_CRC, ENABLE);
}


uint32_t POLYNOMIAL = 0xEDB88320 ;
int16_t have_table = 0 ;
uint32_t table[256] ;


static void make_table()
{
    int16_t i, j;
    have_table = 1 ;
    for (i = 0 ; i < 256 ; i++)
        for (j = 0, table[i] = i ; j < 8 ; j++)
            table[i] = (table[i]>>1)^((table[i]&1)?POLYNOMIAL:0) ;
}


uint32_t zhp_crc32(uint32_t crc, char *buff, int16_t len)
{
    if (!have_table) make_table() ;
    crc = ~crc;
    for (int16_t i = 0; i < len; i++)
        crc = (crc >> 8) ^ table[(crc ^ buff[i]) & 0xff];
    return ~crc;
}
