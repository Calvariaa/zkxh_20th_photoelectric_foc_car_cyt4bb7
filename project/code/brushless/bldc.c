#include "brushless/bldc.h"
#include "debug/vofaplus.h"
#include "brushless/foc.h"
#include "brushless/buzzer.h"
#include "zf_driver_adc.h"
#include "brushless/pid.h"

uint64_t bldc_timer_50ns = 0;

#define BLDC_SET_DIV_MIN 1
#define BLDC_SET_DIV_MAX 48
#define BLDC_SET_DUTY_START PWM_PRIOD_LOAD * 0.04

motor_t motor = {
    .rotor_n = 1,
    .time_div = BLDC_SET_DIV_MAX,
    .duty = BLDC_SET_DUTY_START,
    .speed_buf = 0,
    .speed = 0,
    .speed_diff = 0,
    .speed_list = {0},
    .state = MOTOR_STOP,
    .start = -1,
    .set_duty = 0};

void bldc_output(uint8_t hall_now, uint16_t output_duty)
{
    switch (hall_now)
    {
    case 1:
        // 开启A相以及B相下桥MOS 关闭B相上桥以及C相MOS
        mos_a_bn_open_middle(output_duty);
        adc_global_value = (adc_cbmf_value - (adc_abmf_value + adc_bbmf_value) / 2);
        break;
    case 2:
        mos_a_cn_open_middle(output_duty);
        adc_global_value = -(adc_bbmf_value - (adc_abmf_value + adc_cbmf_value) / 2);
        break;
    case 3:
        mos_b_cn_open_middle(output_duty);
        adc_global_value = (adc_abmf_value - (adc_bbmf_value + adc_cbmf_value) / 2);
        break;
    case 4:
        mos_b_an_open_middle(output_duty);
        adc_global_value = -(adc_cbmf_value - (adc_bbmf_value + adc_abmf_value) / 2);
        break;
    case 5:
        mos_c_an_open_middle(output_duty);
        adc_global_value = (adc_bbmf_value - (adc_cbmf_value + adc_abmf_value) / 2);
        break;
    case 6:
        mos_c_bn_open_middle(output_duty);
        adc_global_value = -(adc_abmf_value - (adc_cbmf_value + adc_bbmf_value) / 2);
        break;
    default:
        mos_close_middle();
        break;
    }
}

// #define BLDC_TEST
void bldc_commutation(motor_t *__motor)
{
    bldc_timer_50ns++;
    // adc_global_value_last = adc_global_value;
    bldc_adc_convert();

    if (__motor->start < 0)
    {
        __motor->state = MOTOR_STOP;
    }

    switch (__motor->state)
    {
    case MOTOR_START:

#ifdef BLDC_TEST
        if (bldc_timer_50ns % 128 == 0)
        {
            bldc_output(__motor->rotor_n, 500);
            // if ((adc_global_value_last < 0 && adc_global_value >= 0) || (adc_global_value_last > 0 && adc_global_value <= 0))

            __motor->rotor_n++;
            if (__motor->rotor_n == 7)
                __motor->rotor_n = 1;
        }
#else
        if (bldc_timer_50ns % __motor->time_div == 0)
        {
            bldc_output(__motor->rotor_n, __motor->duty);
            // if ((adc_global_value_last < 0 && adc_global_value >= 0) || (adc_global_value_last > 0 && adc_global_value <= 0))
            if (adc_global_value < 0)
            {
                __motor->speed_buf++;
                __motor->rotor_n++;
                // gpio_toggle_level(P20_1);
            }
            if (__motor->rotor_n == 7)
                __motor->rotor_n = 1;
        }
#endif

        // speed measure
        if (bldc_timer_50ns % 256 == 0)
        {
            // __motor->speed_last = __motor->speed;
            for (uint8_t i = 0; i < 31; i++)
            {
                __motor->speed_list[i] = __motor->speed_list[i + 1];
            }
            __motor->speed_list[31] = __motor->speed;
            __motor->speed = __motor->speed_buf;
            __motor->speed_buf = 0;

            __motor->speed_diff = 0;
            for (uint8_t i = 0; i < 31; i++)
            {
                __motor->speed_diff += __motor->speed_list[i + 1] - __motor->speed_list[i];
            }
        }

        if (bldc_timer_50ns % 32 == 0 && __motor->time_div > BLDC_SET_DIV_MIN)
        {
            __motor->time_div--;
        }
        if (__motor->time_div == BLDC_SET_DIV_MIN && bldc_timer_50ns % 16 == 0 && __motor->duty < MINMAX((uint16_t)__motor->set_duty, 0, PWM_PRIOD_LOAD / 1.5))
            __motor->duty++;

        if (bldc_timer_50ns >= 2147483647)
        {
            bldc_timer_50ns = 0;
        }

#ifndef BLDC_TEST
        if ((__motor->time_div == BLDC_SET_DIV_MAX && bldc_timer_50ns >= 20 * 1000 * 2 && __motor->speed < 2) || (__motor->time_div == BLDC_SET_DIV_MIN && bldc_timer_50ns >= 20 * 1000 * 10 && __motor->speed < 2))
        {
            bldc_timer_50ns = 0;
            __motor->duty = 0;
            __motor->state = MOTOR_STOP;
        }
        else
#endif
        break;

    case MOTOR_STOP:
        bldc_timer_50ns++;
        mos_close_middle();

        if (bldc_timer_50ns > 20 * 1000 * 2) // 2s
        {
            __motor->rotor_n = 1;
            __motor->time_div = BLDC_SET_DIV_MAX;
            __motor->duty = BLDC_SET_DUTY_START;
            __motor->speed_buf = 0;
            __motor->speed = 0;
            __motor->speed_diff = 0;
            memset(__motor->speed_list, 0, sizeof(__motor->speed_list));
            __motor->state = MOTOR_START;

            bldc_timer_50ns = 0;
        }

        break;

    default:
        break;
    }

    data_send(19, __motor->time_div);
    data_send(21, __motor->duty);

    data_send(22, adc_global_value);

    data_send(24, __motor->speed);
    data_send(25, __motor->speed_diff);

    // data_send(20, bldc_timer_50ns);

    // data_send_add(adc_abmf_value, adc_global_value, bldc_test1);
    // data_send_add(adc_abmf_value, adc_bbmf_value, adc_cbmf_value, __motor->rotor_n);
}
/*

float bldc_accel = 0.6;
FOC_Parm_Typedef FOC_M = {0};
void bldc_svpwm()
{
    bldc_timer_50ns++;
    if (bldc_timer_50ns % 1024 == 0)
    {
        bldc_timer_50ns = 0;
        if (bldc_accel < 4)
        {
            FOC_M.Park_in.u_q = 4;
            bldc_accel += 0.01;
        }
        // else if (bldc_accel < 10)
        // {
        //     FOC_M.Park_in.u_q += 0.003;
        //     bldc_accel += 0.005;
        // }
    }
    FOC_M.set_angle += ANGLE_TO_RAD(bldc_accel);
    if (FOC_M.set_angle >= pi_2)
    {
        FOC_M.expect_rotations++;
        FOC_M.set_angle -= pi_2;
    }
    if (FOC_M.set_angle < -pi_2)
    {
        FOC_M.expect_rotations--;
        FOC_M.set_angle += pi_2;
    }

    FOC_M.Park_in.u_d = get_ud_freq(&FOC_M, 1000, 1.0);
    FOC_M.Park_in.u_q = MINMAX(FOC_M.Park_in.u_q, -5.5, 5.5);

    FOC_M.V_Clark = iPark_Calc(FOC_M.Park_in, FOC_M.set_angle);

    FOC_M.tool = Tool_Calc(FOC_M.V_Clark);         // 中间变量计算
    FOC_M.N = Electrical_Sector_Judge(FOC_M.tool); // 电角度扇区判断

    FOC_M.Vector = Vector_Calc(FOC_M.tool, FOC_M.N, BUS_VOLTAGE, PWM_PRIOD_LOAD); // 矢量作用时间计算
    FOC_M.Period = PeriodCal(FOC_M.Vector, FOC_M.N, PWM_PRIOD_LOAD);              // 各桥PWM占空比计算

    mos_all_open_middle(FOC_M.Period.AH, FOC_M.Period.BH, FOC_M.Period.CH);
    // data_send(19, FOC_M.Park_in.u_q);
    // data_send(20, bldc_accel);
}
*/
