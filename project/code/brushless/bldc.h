
#ifndef _BLDC_H
#define _BLDC_H

#include "zf_common_typedef.h"
#include "brushless/motor.h"
void bldc_soft_openloop();
void bldc_output(uint8_t hall_now, uint16_t output_duty);
void bldc_svpwm();

#endif /* _BLDC_H */