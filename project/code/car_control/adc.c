/*
 * @Author: Calvariaa 17198186+Calvariaa@users.noreply.github.com
 * @Date: 2024-12-29 18:40:50
 * @LastEditors: Calvariaa 17198186+Calvariaa@users.noreply.github.com
 * @LastEditTime: 2025-03-17 20:33:10
 * @FilePath: \zkxh_20th_photoelectric_foc_car_cyt4bb7\project\code\brushless\adc.c
 * @Description: ����Ĭ������,������`customMade`, ��koroFileHeader�鿴���� ��������: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "car_control/adc.h"

#include "sysclk/cy_sysclk.h"
#include "tcpwm/cy_tcpwm_pwm.h"
#include "gpio/cy_gpio.h"
#include "sysint/cy_sysint.h"
#include "trigmux/cy_trigmux.h"
#include "adc/cy_adc.h"
#include "debug/vofaplus.h"
#include "zf_driver_adc.h"
#include "zf_driver_gpio.h"


adc_channel_enum adc_tube[1] = {ADC2_CH06_P18_6};

uint16_t adc_tube_read_raw[20] = {0};


void tube_adc_init()
{
    for (uint8_t i = 0; i < sizeof(adc_tube) / sizeof(adc_tube[0]); i++)
    {
        adc_init(adc_tube[i], ADC_12BIT);
    }
}

void tube_adc_convert()
{
    for (uint8_t i = 0; i < sizeof(adc_tube) / sizeof(adc_tube[0]); i++)
    {
        adc_tube_read_raw[i] = adc_mean_filter_convert(adc_tube[i], 5);
    }
}