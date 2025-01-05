#ifndef DEBUG_VOFAPLUS_H_
#define DEBUG_VOFAPLUS_H_

#define DATA_SEND_SIZE 36

#include "zf_common_typedef.h"

#define DATA_SEND_QUEUE_SIZE 25565
#define DATA_SEND_EASE_DIVISOR 16

extern int16_t data_send_list[DATA_SEND_QUEUE_SIZE][4];

void send_vofaplus();
void send_vofaplus_queue();
void data_send_add(int16_t _data1, int16_t _data2, int16_t _data3, int16_t _data4);
void data_send(uint16_t num, float data);
void data_send_clear();

#endif /* DEBUG_VOFAPLUS_H_ */