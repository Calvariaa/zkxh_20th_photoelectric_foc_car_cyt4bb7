#ifndef DEBUG_VOFAPLUS_H_
#define DEBUG_VOFAPLUS_H_

#define DATA_SEND_SIZE 32

#include "zf_common_typedef.h"

#define DATA_SEND_QUEUE_SIZE 32768
#define DATA_SEND_EASE_DIVISOR 16

extern int16_t data_send_list[DATA_SEND_QUEUE_SIZE][3];

extern float data_send[DATA_SEND_SIZE];

void send_vofaplus();
void send_vofaplus_queue();
void data_send_add(int16_t _data1, int16_t _data2, int16_t  _data3);
#endif /* DEBUG_VOFAPLUS_H_ */