
#ifndef _BLDC_H
#define _BLDC_H

#include "zf_common_typedef.h"
#include "brushless/motor.h"
#include "brushless/adc.h"

typedef enum
{
    // MOTOR_PRESTART,
    // MOTOR_BUZZ,
    MOTOR_START,
    MOTOR_STOP
} motor_state_t;

typedef struct
{
    uint8_t rotor_n;
    uint16_t time_div;
    uint16_t duty;
    int32_t speed_buf;
    int32_t speed;
    int32_t speed_diff;
    int32_t speed_list[32];
    motor_state_t state;

    float start;
    float set_duty;
} motor_t;

void bldc_commutation(motor_t *);
// void bldc_output(uint8_t hall_now, uint16_t output_duty);
// void bldc_svpwm();

#endif /* _BLDC_H */