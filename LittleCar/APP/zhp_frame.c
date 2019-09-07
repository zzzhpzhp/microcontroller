#include "zhp_frame.h"


uint8_t uart_trans_flag = STOP_TRANS;

int8_t tx_frame_buf[FRAME_BUR_NUM];
int8_t final_tx_frame[MAX_FRAME_BUF_NUM];

int8_t rx_frame_buf[MAX_FRAME_BUF_NUM];
int8_t final_rx_frame[FRAME_BUR_NUM];

int8_t sender_encoder_frame(char *frame_buf, uint16_t buf_size)
{
  uint16_t tx_buf_size = 0;
  uint32_t crc_val;

  if (buf_size + 6 > FRAME_BUR_NUM)  //6����4�ֽ�crcУ��ֵ����2�ֽ�֡ͷ֡β
  {
    return FALSE;   //������ȳ�����buf�ֽ�������ֱ�ӷ��ش���
  }
  
//  CRC_ResetDR();                                     //��λӲ��CRC���㵥Ԫ
//  crc_val = CRC_CalcCRC((uint32_t)START_OF_FRAME);   //��֡ͷ��־�������
//  while(frame_buf[tx_buf_size])
//  {
//    CRC_CalcCRC((uint32_t)frame_buf[tx_buf_size++]);
//  }
//  crc_val = CRC_CalcCRC((uint32_t)END_OF_FRAME);     //��֡β��־�������
    
    char calc_buf[MAX_FRAME_BUF_NUM] = {0};
    uint16_t calc_buf_cnt = 0;
    calc_buf[calc_buf_cnt++] = START_OF_FRAME;
    while(frame_buf[tx_buf_size])
    {
      calc_buf[calc_buf_cnt++] = frame_buf[tx_buf_size++];
    }
    calc_buf[calc_buf_cnt++] = END_OF_FRAME;       
    crc_val = zhp_crc32(0, calc_buf, calc_buf_cnt);

  
  frame_buf[tx_buf_size++] = (uint8_t)((crc_val & 0xff000000) >> 24);
  frame_buf[tx_buf_size++] = (uint8_t)((crc_val & 0x00ff0000) >> 16);
  frame_buf[tx_buf_size++] = (uint8_t)((crc_val & 0x0000ff00) >> 8);
  frame_buf[tx_buf_size++] = (uint8_t)(crc_val & 0x000000ff);

  memset(final_tx_frame, 0, sizeof(final_tx_frame));
  final_tx_frame[0] = START_OF_FRAME;
  uint16_t data_num_cnt = 1;

  for (uint16_t i = 0; i < tx_buf_size; i++)
  {
    if ( frame_buf[i] != START_OF_FRAME && frame_buf[i] != END_OF_FRAME && frame_buf[i] != ESCAPED_CHARACTER)
    {
      final_tx_frame[data_num_cnt++] = frame_buf[i];
    }
    else if (frame_buf[i] == START_OF_FRAME)
    {
      final_tx_frame[data_num_cnt++] = ESCAPED_CHARACTER;
      final_tx_frame[data_num_cnt++] = ESCAPED_SOF;
    }
    else if (frame_buf[i] == END_OF_FRAME)
    {
      final_tx_frame[data_num_cnt++] = ESCAPED_CHARACTER;
      final_tx_frame[data_num_cnt++] = ESCAPED_EOF;
    }
    else if (frame_buf[i] == ESCAPED_CHARACTER)
    {
      final_tx_frame[data_num_cnt++] = ESCAPED_CHARACTER;
      final_tx_frame[data_num_cnt++] = ESCAPED_ESC;
    }
  }

  final_tx_frame[data_num_cnt++] = END_OF_FRAME;
  final_tx_frame[data_num_cnt] = '\0';
  
  return TRUE;
}

int8_t receiver_decoder_frame(uint8_t rx_char)
{
  static unsigned char rx_cnt = 0;
  if (rx_cnt == 0 && rx_char != START_OF_FRAME) //Ѱ��֡ͷ
  {
    return FALSE;
  }
  else
  {
    rx_frame_buf[rx_cnt++ % MAX_FRAME_BUF_NUM] = rx_char;   //% MAX_FRAME_BUF_NUM��Ϊ�˷�ֹ�����±�Խ��
  }

  if (rx_char != END_OF_FRAME)        //���֡β
  {
    return FALSE;
  }
  else                                //�ɹ���⵽֡β
  {
    uint16_t cnt = 0; 
    uint8_t temp_rx_buf[FRAME_BUR_NUM];

    if (rx_cnt > MAX_FRAME_BUF_NUM)
    {
      return FALSE; //���֡���ȱȻ��滹����˵��֡ͷ�������ˣ����ش���
    }
    
    for (uint16_t i = 1; i < (rx_cnt - 1); i++)    //�ж�֡���Ƿ��б�ת����ַ�����SOF\EOF\ESC��ͬ���غ��ַ�������ת�壩��������ԭ��֡ͷ֡β����Ҫ���ж�
    {
      if (rx_frame_buf[i] != ESCAPED_CHARACTER)
      {
        temp_rx_buf[cnt++] = rx_frame_buf[i];
      }
      else if (rx_frame_buf[i + 1] == ESCAPED_SOF)
      {
        temp_rx_buf[cnt++] = START_OF_FRAME;
        i++;
      }
      else if (rx_frame_buf[i + 1] == ESCAPED_EOF)
      {
        temp_rx_buf[cnt++] = END_OF_FRAME;
        i++;
      }
      else if (rx_frame_buf[i + 1] == ESCAPED_ESC)
      {
        temp_rx_buf[cnt++] = ESCAPED_CHARACTER;
        i++;
      }
    }

    uint32_t calc_crc_val;
    char calc_buf[MAX_FRAME_BUF_NUM] = {0};
    uint16_t calc_buf_cnt = 0;
//    CRC_ResetDR();
//    CRC_CalcCRC((uint32_t)START_OF_FRAME);
//    for (uint16_t i = 0; i < cnt - 4; i++)
//    {
//      CRC_CalcCRC((uint32_t)temp_rx_buf[i]);
//    }
//    calc_crc_val = CRC_CalcCRC((uint32_t)END_OF_FRAME);
    calc_buf[calc_buf_cnt++] = START_OF_FRAME;    
    for (uint16_t i = 0; i < cnt - 4; i++)
    {
      calc_buf[calc_buf_cnt++] =temp_rx_buf[i];
    }
    calc_buf[calc_buf_cnt++] = END_OF_FRAME;
    calc_crc_val = zhp_crc32(0, calc_buf, calc_buf_cnt);
    
    uint32_t received_crc; 
    //������֡�е�4�ֽ�crcֵ��ϳ�һ��32λ��crcУ��ֵ
    received_crc = temp_rx_buf[cnt - 4] << 24 | temp_rx_buf[cnt - 3] << 16 | temp_rx_buf[cnt - 2] << 8 | temp_rx_buf[cnt - 1];
    if(received_crc == calc_crc_val)
    {
       memcpy(final_rx_frame, temp_rx_buf , cnt); 
    }
    else 
    {
      rx_cnt=0;
      return FALSE;
    }
    final_rx_frame[cnt] = '\0';

    rx_cnt=0;
    
    return TRUE;
  }
}

