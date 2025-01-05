#include "vofaplus.h"

#include "zf_common_headfile.h"

#if CY_CPU_CORTEX_M0P
#pragma location = 0x28001000
float _data_send[DATA_SEND_SIZE];
#else
#if defined(CY_CORE_CM7_0)
#pragma location = 0x28001000
__no_init float _data_send[DATA_SEND_SIZE];
#elif defined(CY_CORE_CM7_1)
#pragma location = 0x28001000
__no_init float _data_send[DATA_SEND_SIZE];
#else
#error "No core selected"
#endif
#endif

int16_t data_send_list[DATA_SEND_QUEUE_SIZE][4] = {0};

void send_vofaplus()
{
    if (_data_send[0] == 1)
    {
#if (!CY_CPU_CORTEX_M0P)
        SCB_CleanInvalidateDCache_by_Addr(&_data_send, sizeof(_data_send));
#endif
        for (int8_t i = 0; i <= DATA_SEND_SIZE; i++)
            printf("%f,", _data_send[i]);
        printf("-1.0\r\n");
        _data_send[0] = 0;
    }
    else
    {
#if (!CY_CPU_CORTEX_M0P)
        SCB_CleanInvalidateDCache_by_Addr(&_data_send, sizeof(_data_send));
#endif
        for (int8_t i = 0; i <= DATA_SEND_SIZE; i++)
            printf("%f,", _data_send[i]);
        printf("-1.0\r\n");
        _data_send[0] = 0;
    }
}

uint32_t data_send_number = 0;
uint32_t data_send_head = 0;
uint32_t data_send_tail = 0;
void send_vofaplus_queue()
{
    if (data_send_number > 0)
    {
        printf("%d,%d,%d,%d,%d,%d,%d\r\n", data_send_number, data_send_head, data_send_tail, data_send_list[data_send_head][0], data_send_list[data_send_head][1], data_send_list[data_send_head][2], data_send_list[data_send_head][3]);
        data_send_head = (data_send_head + 1) % DATA_SEND_QUEUE_SIZE;
        data_send_number--;
    }
    else
    {
        printf("No Data\r\n");
    }
}

void data_send_clear()
{
#if (!CY_CPU_CORTEX_M0P)
    SCB_CleanInvalidateDCache_by_Addr(&_data_send, sizeof(_data_send));
#endif
}

void data_send(uint16_t num, float data)
{
    _data_send[num] = data;

#if (!CY_CPU_CORTEX_M0P)
// SCB_CleanInvalidateDCache_by_Addr(&_data_send, sizeof(_data_send));
#endif
}

void data_send_add(int16_t _data1, int16_t _data2, int16_t _data3, int16_t _data4)
{
    data_send_list[data_send_tail][0] = _data1;
    data_send_list[data_send_tail][1] = _data2;
    data_send_list[data_send_tail][2] = _data3;
    data_send_list[data_send_tail][3] = _data4;
    data_send_tail = (data_send_tail + 1) % DATA_SEND_QUEUE_SIZE;
    data_send_number++;
}