/*
 * @Author: Calvariaa 17198186+Calvariaa@users.noreply.github.com
 * @Date: 2024-12-29 18:40:50
 * @LastEditors: Calvariaa 17198186+Calvariaa@users.noreply.github.com
 * @LastEditTime: 2025-03-17 20:33:10
 * @FilePath: \zkxh_20th_photoelectric_foc_car_cyt4bb7\project\code\brushless\adc.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "brushless/adc.h"

#include "sysclk/cy_sysclk.h"
#include "tcpwm/cy_tcpwm_pwm.h"
#include "gpio/cy_gpio.h"
#include "sysint/cy_sysint.h"
#include "trigmux/cy_trigmux.h"
#include "adc/cy_adc.h"
#include "debug/vofaplus.h"
#include "zf_driver_adc.h"
#include "zf_driver_gpio.h"

int16_t adc_abmf_value = 0;
int16_t adc_bbmf_value = 0;
int16_t adc_cbmf_value = 0;

int16_t adc_test_mid[3] = {0};
int16_t adc_global_value_last = 0;
int16_t adc_global_value = 0;

void motor_bldc_adc_init()
{
    adc_init(ADC_ABMF, ADC_12BIT);
    adc_init(ADC_BBMF, ADC_12BIT);
    adc_init(ADC_CBMF, ADC_12BIT);
    
    // adc_init(ADC0_CH24_P08_1, ADC_12BIT);
    // adc_init(ADC0_CH25_P08_2, ADC_12BIT);
    // adc_init(ADC0_CH26_P08_3, ADC_12BIT);
    // adc_init(ADC_MID, ADC_12BIT);
}

void bldc_adc_convert()
{
    // adc_abmf_value = adc_convert(ADC_ABMF);
    // adc_bbmf_value = adc_convert(ADC_BBMF);
    // adc_cbmf_value = adc_convert(ADC_CBMF);
    adc_abmf_value = adc_mean_filter_convert(ADC_ABMF,5);
    adc_bbmf_value = adc_mean_filter_convert(ADC_BBMF,5);
    adc_cbmf_value = adc_mean_filter_convert(ADC_CBMF,5);
    // adc_mid_value = adc_mean_filter_convert(ADC_MID, 1) / 2;
}