#include "brushless/bldc.h"
#include "debug/vofaplus.h"
#include "brushless/foc.h"
#include "brushless/buzzer.h"
#include "zf_driver_adc.h"

uint64_t bldc_timer_50ns = 0;

typedef enum
{
    // MOTOR_PRESTART,
    MOTOR_START,
    MOTOR_STOP
} MotorState;

typedef struct
{
    uint8_t rotor_n;
    uint16_t time_div;
    uint16_t duty;
    int32_t speed_buf;
    int32_t speed;
    int32_t speed_diff;
    int32_t speed_list[32];
    MotorState state;
} MotorControl;

MotorControl motor = {
    .rotor_n = 1,
    .time_div = 48,
    .duty = PWM_PRIOD_LOAD / 8,
    .speed_buf = 0,
    .speed = 0,
    .speed_diff = 0,
    .speed_list = {0},
    .state = MOTOR_START,
};

void bldc_output(uint8_t hall_now, uint16_t output_duty)
{
    switch (hall_now)
    {
    case 1:
        // 开启A相以及B相下桥MOS 关闭B相上桥以及C相MOS
        mos_a_bn_open_middle(output_duty);
        adc_global_value = (adc_cbmf_value - adc_abmf_value / 2);
        break;
    case 2:
        mos_a_cn_open_middle(output_duty);
        adc_global_value = -(adc_bbmf_value - adc_abmf_value / 2);
        break;
    case 3:
        mos_b_cn_open_middle(output_duty);
        adc_global_value = (adc_abmf_value - adc_bbmf_value / 2);
        break;
    case 4:
        mos_b_an_open_middle(output_duty);
        adc_global_value = -(adc_cbmf_value - adc_bbmf_value / 2);
        break;
    case 5:
        mos_c_an_open_middle(output_duty);
        adc_global_value = (adc_bbmf_value - adc_cbmf_value / 2);
        break;
    case 6:
        mos_c_bn_open_middle(output_duty);
        adc_global_value = -(adc_abmf_value - adc_cbmf_value / 2);
        break;
    default:
        mos_close_middle();
        break;
    }
    data_send[26] = adc_abmf_value;
    data_send[27] = adc_bbmf_value;
    data_send[28] = adc_cbmf_value;
    data_send[29] = adc_global_value;
}

int8_t bldc_test1 = 1;
void bldc_commutation()
{
    bldc_timer_50ns++;
    // adc_global_value_last = adc_global_value;
    bldc_adc_convert();

    switch (motor.state)
    {
    case MOTOR_START:

        if (bldc_timer_50ns % motor.time_div == 0)
        {
            bldc_output(motor.rotor_n, motor.duty);
            // if ((adc_global_value_last < 0 && adc_global_value >= 0) || (adc_global_value_last > 0 && adc_global_value <= 0))
            if (adc_global_value < 0)
            {
                bldc_test1 = !bldc_test1;

                motor.speed_buf++;
                motor.rotor_n++;
                // gpio_toggle_level(P20_1);
            }
            if (motor.rotor_n == 7)
                motor.rotor_n = 1;
        }

        // speed measure
        if (bldc_timer_50ns % 256 == 0)
        {
            // motor.speed_last = motor.speed;
            for (uint8_t i = 0; i < 31; i++)
            {
                motor.speed_list[i] = motor.speed_list[i + 1];
            }
            motor.speed_list[31] = motor.speed;
            motor.speed = motor.speed_buf;
            motor.speed_buf = 0;

            motor.speed_diff = 0;
            for (uint8_t i = 0; i < 31; i++)
            {
                motor.speed_diff += motor.speed_list[i + 1] - motor.speed_list[i];
            }
        }

        if (bldc_timer_50ns % 32 == 0 && motor.time_div > 1)
        {
            motor.time_div--;
        }
        if (motor.time_div == 1 && bldc_timer_50ns % 16 == 0 && motor.duty < PWM_PRIOD_LOAD / 2)
            motor.duty++;

        if (bldc_timer_50ns >= 65536)
        {
            bldc_timer_50ns = 0;
        }

        if (motor.speed_diff < -30)
        {
            bldc_timer_50ns = 0;
            motor.duty = 0;
            motor.state = MOTOR_STOP;
        }

        break;

    case MOTOR_STOP:
        bldc_timer_50ns++;
        mos_close_middle();

        // if (bldc_timer_50ns > 20 * 1000 * 2) // 2s
        // {
        //     motor.rotor_n = 1;
        //     motor.time_div = 48;
        //     motor.duty = PWM_PRIOD_LOAD / 8;
        //     motor.speed_buf = 0;
        //     motor.speed = 0;
        //     motor.speed_diff = 0;
        //     memset(motor.speed_list, 0, sizeof(motor.speed_list));
        //     motor.state = MOTOR_START;
        // }

        break;

    default:
        break;
    }

    data_send[19] = (float)motor.time_div;
    data_send[20] = (float)bldc_timer_50ns;
    data_send[21] = (float)motor.duty;

    data_send[22] = (float)adc_global_value;
    data_send[23] = (float)bldc_test1;

    data_send_add(adc_abmf_value, adc_global_value, bldc_test1);

    data_send[24] = (float)motor.speed;
    data_send[25] = (float)motor.speed_diff;
}
/*

void bldc_commutation()
{
    bldc_timer_50ns++;
    bldc_adc_convert();

    switch (motor.state)
    {
    case MOTOR_PRESTART:
        if (bldc_timer_50ns % bldc_timer_50ns == 0)
        {
            bldc_output(motor.rotor_n, motor.duty);
                motor.rotor_n++;
            if (motor.rotor_n == 7)
                motor.rotor_n = 1;
        }
        if (bldc_timer_50ns % 128 == 0 && bldc_timer_50ns > 1)
        {
            bldc_timer_50ns--;
        }
        if (bldc_timer_50ns == 1 && bldc_timer_50ns % 32 == 0 && motor.duty < PWM_PRIOD_LOAD - 2000)
            motor.duty++;
        if (bldc_timer_50ns >= 65536)
        {
            bldc_timer_50ns = 0;
        }
        if ()
        {
            motor.state = MOTOR_START;
        }
        break;

    case MOTOR_START:
        if (bldc_timer_50ns % bldc_timer_50ns == 0)
        {
            bldc_output(motor.rotor_n, motor.duty);
            if (adc_global_value < 0)
                motor.rotor_n++;
            if (motor.rotor_n == 7)
                motor.rotor_n = 1;
        }
        if ()
        {
            motor.state = MOTOR_STOP;
        }
        break;

    case MOTOR_STOP:
        // Add motor stop logic here
        break;
    }

    data_send[19] = (float)bldc_timer_50ns;
    data_send[20] = (float)bldc_timer_50ns;
    data_send[21] = (float)motor.duty;
    data_send[22] = (float)adc_global_value < 0;
}

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
    data_send[19] = (float)FOC_M.Park_in.u_q;
    data_send[20] = (float)bldc_accel;
}
*/
