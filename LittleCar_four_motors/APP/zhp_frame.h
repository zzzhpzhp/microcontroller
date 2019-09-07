#ifndef __ZHP_FRAME_H
#define	__ZHP_FRAME_H

#include "stm32f10x.h"    
#include "string.h"
#include "stdio.h"
#include "zhp_crc.h"
#include "stm32f10x_conf.h"
#include "pstwo.h"

#define START_TRANS 1
#define STOP_TRANS  0

#define FRAME_BUR_NUM     100                      //���ԭʼ֡����
#define MAX_FRAME_BUF_NUM (FRAME_BUR_NUM * 2 - 2)  //������շ���/����֡���ȣ����������(�����غɶ���ת���ַ�)�����buf�ֽ�������2����Ϊ֡ͷ֡β���ᱻת��

#define START_OF_FRAME    0X7D //֡ͷ
#define END_OF_FRAME      0X7E //֡β
#define ESCAPED_CHARACTER 0X7F //ת���ַ� ����֡ͷתΪ0X7F+0X00 ����֡βת��ΪOX7F+0X01 �����Լ�ת��Ϊ0X7F+0X02
#define ESCAPED_SOF       0X00
#define ESCAPED_EOF       0X01
#define ESCAPED_ESC       0X02

extern int8_t tx_frame_buf[FRAME_BUR_NUM];
extern int8_t final_tx_frame[MAX_FRAME_BUF_NUM];

extern int8_t rx_frame_buf[MAX_FRAME_BUF_NUM];
extern int8_t final_rx_frame[FRAME_BUR_NUM];

extern uint8_t uart_trans_flag;

#pragma pack(push)
#pragma pack(1)
struct frame_str
{
	float velocity_feedback;
	float angular_feedback;
} ;
#pragma pack(pop)

extern struct frame_str frame_data;

int8_t sender_encoder_frame(char *frame_buf, uint16_t buf_size);
int8_t receiver_decoder_frame(uint8_t rx_char);
#endif