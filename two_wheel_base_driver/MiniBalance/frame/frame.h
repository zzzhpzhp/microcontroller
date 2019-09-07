#ifndef __ZHP_FRAME_H
#define	__ZHP_FRAME_H

#include "string.h"
#include "stdint.h"

namespace zhp_frame
{

class zhp_frame_cls
{
    public:
    zhp_frame_cls();
    ~zhp_frame_cls();
    
    

    volatile uint32_t CRCValue = 0;		 // ���ڴ�Ų����CRCУ��ֵ
    uint32_t POLYNOMIAL = 0xEDB88320 ;
    int16_t have_table = 0 ;
    uint32_t table[256] ;


    #define START_TRANS 1
    #define STOP_TRANS  0

    #define FRAME_BUR_NUM     100                      //���ԭʼ֡����
    #define MAX_FRAME_BUF_NUM (FRAME_BUR_NUM * 2 - 2)  //������շ���/����֡���ȣ����������(�����غɶ���ת���ַ�)�����buf�ֽ����2����Ϊ֡ͷ֡β���ᱻת��

    #define START_OF_FRAME    0X7D //֡ͷ
    #define END_OF_FRAME      0X7E //֡β
    #define ESCAPED_CHARACTER 0X7F //ת���ַ� ����֡ͷתΪ0X7F+0X00 ����֡βת��ΪOX7F+0X01 �����Լ�ת��Ϊ0X7F+0X02
    #define ESCAPED_SOF       0X00
    #define ESCAPED_EOF       0X01
    #define ESCAPED_ESC       0X02

    char tx_frame_buf[FRAME_BUR_NUM];
    char final_tx_frame[MAX_FRAME_BUF_NUM];

    char rx_frame_buf[MAX_FRAME_BUF_NUM];
    char final_rx_frame[FRAME_BUR_NUM];

    uint8_t uart_trans_flag;

    void make_table();
    uint32_t zhp_crc32(uint32_t crc, char *buff, int16_t len);
    int8_t sender_encode_frame(char *frame_buf, int16_t *final_buf_size);
    int8_t receiver_decode_frame(uint8_t rx_char);
};
}
#endif
