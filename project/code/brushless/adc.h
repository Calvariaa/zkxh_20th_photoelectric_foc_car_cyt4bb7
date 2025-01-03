
#ifndef _ADC_H
#define _ADC_H

#define ADC_ABMF ADC1_CH28_P15_0
#define ADC_BBMF ADC1_CH29_P15_1
#define ADC_CBMF ADC1_CH30_P15_2
#define ADC_MID ADC1_CH31_P15_3

#include "zf_common_typedef.h"
#include "brushless/motor.h"

extern int16_t adc_abmf_value;
extern int16_t adc_bbmf_value;
extern int16_t adc_cbmf_value;
extern int16_t adc_global_value_last;
extern int16_t adc_global_value;

extern int16_t adc_test_mid[3];

void motor_bldc_adc_init();
void bldc_adc_convert();

#endif /* _ADC_H */