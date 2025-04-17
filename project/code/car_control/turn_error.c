#include "turn_error.h"
#include "car_control/adc.h"

float turn_error;

void get_sensor()
{
    tube_adc_convert();
    // ccd_convert();
}

/*
 *   传感器采集函数，size为20
 *   void tube_adc_convert()
 *   {
 *       for (uint8_t i = 0; i < sizeof(adc_tube) / sizeof(adc_tube[0]); i++)
 *       {
 *           adc_tube_read_raw[i] = adc_mean_filter_convert(adc_tube[i], 5);
 *       }
 *   }
 */

// // 对adc_tube_read_raw进行归一化，min和max随时更新
// float adc_tube_read_norm[20] = {0};
// void tube_proc()
// {
//     // 归一化
//     static float min = 0;
//     float max = 4095;
//     for (uint8_t i = 0; i < sizeof(adc_tube_read_raw) / sizeof(adc_tube_read_raw[0]); i++)
//     {
//         if (adc_tube_read_raw[i] < min)
//             min = adc_tube_read_raw[i];
//         // if (adc_tube_read_raw[i] > max)
//         //     max = adc_tube_read_raw[i];
//     }
//     for (uint8_t i = 0; i < sizeof(adc_tube_read_raw) / sizeof(adc_tube_read_raw[0]); i++)
//     {
//         adc_tube_read_norm[i] = (adc_tube_read_raw[i] - min) / (max - min);
//     }
// }

bool adc_tube_read_bin[20] = {0};
void get_turn_angle()
{
    get_sensor();
    // 单阈值二值化
    uint16_t thres = 4000;
    for (uint8_t i = 0; i < sizeof(adc_tube_read_raw) / sizeof(adc_tube_read_raw[0]); i++)
    {
        adc_tube_read_bin[i] = adc_tube_read_raw[i] > thres ? 1 : 0;
    }

    // 20个光电管从右到左顺序排列，依次增加权重然后乘算后加给偏差turn_error。前十个应该是正值，后十个应该是负值
    turn_error = 0;
    for (uint8_t i = 0; i < sizeof(adc_tube_read_bin) / sizeof(adc_tube_read_bin[0]); i++)
    {
        turn_error += (i - 10) * adc_tube_read_bin[i];
    }
    
}

