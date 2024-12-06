#include "vofaplus.h"

float data_send[DATA_SEND_SIZE] = {0};

void send_vofaplus()
{
    for (int8_t i = 0; i <= DATA_SEND_SIZE; i++)
        printf("%f,", data_send[i]);
    printf("-1.0\r\n");
}