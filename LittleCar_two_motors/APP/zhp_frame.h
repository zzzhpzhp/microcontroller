#ifndef __ZHP_FRAME_H
#define	__ZHP_FRAME_H

#include "stm32f10x.h"    
#include "string.h"
#include "zhp_crc.h"
#include "stm32f10x_conf.h"
#include "pstwo.h"

#define START_TRANS 1
#define STOP_TRANS  0

#define FRAME_BUR_NUM     100                      //最大原始帧长度
#define MAX_FRAME_BUF_NUM (FRAME_BUR_NUM * 2 - 2)  //最大最终发送/接收帧长度，极端情况下(所有载荷都是转义字符)所需的buf字节数，减2是因为帧头帧尾不会被转义

#define START_OF_FRAME    0X7D //帧头
#define END_OF_FRAME      0X7E //帧尾
#define ESCAPED_CHARACTER 0X7F //转义字符 遇见帧头转为0X7F+0X00 遇见帧尾转义为OX7F+0X01 遇见自己转义为0X7F+0X02
#define ESCAPED_SOF       0X00
#define ESCAPED_EOF       0X01
#define ESCAPED_ESC       0X02

extern int8_t tx_frame_buf[FRAME_BUR_NUM];
extern int8_t final_tx_frame[MAX_FRAME_BUF_NUM];

extern int8_t rx_frame_buf[MAX_FRAME_BUF_NUM];
extern int8_t final_rx_frame[FRAME_BUR_NUM];

extern uint8_t uart_trans_flag;

int8_t sender_encoder_frame(char *frame_buf, uint16_t buf_size);
int8_t receiver_decoder_frame(uint8_t rx_char);
#endif
