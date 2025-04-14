
#ifndef _ADC_H
#define _ADC_H

// #define ADC_MID ADC1_CH31_P15_3

#include "zf_common_typedef.h"

extern int16_t adc_test_mid[3];
extern uint16_t adc_tube_read_raw[20];

void tube_adc_init();
void tube_adc_convert();

#endif /* _ADC_H */