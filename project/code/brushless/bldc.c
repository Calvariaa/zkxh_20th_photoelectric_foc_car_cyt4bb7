#include "brushless/bldc.h"
#include "debug/vofaplus.h"
#include "brushless/foc.h"
#include "brushless/buzzer.h"
#include "zf_driver_adc.h"

uint8_t n = 1;
uint16_t b = 48;
uint64_t t = 1;
uint16_t PRIOD = PWM_PRIOD_LOAD / 8;

#define BLDC_TEST

void bldc_soft_openloop()
{
    t++;
    tcpwm_irq_middle();

    if (t % b == 0)
    {
        bldc_output(n, PRIOD);
        if (adc_global_value < 0)
            n++;
        if (n == 7)
            n = 1;
    }
    if (t % 128 == 0 && b > 1)
    {
        b--;
    }
    if (b == 1 && t % 32 == 0 && PRIOD < PWM_PRIOD_LOAD - 2000)
        PRIOD++;

    if (t >= 65536)
    {
        t = 0;
    }

    data_send[19] = (float)b;
    data_send[20] = (float)t;
    data_send[21] = (float)PRIOD;
    data_send[22] = (float)adc_global_value < 0;
}

float bldc_accel = 0.6;
FOC_Parm_Typedef FOC_M = {0};
void bldc_svpwm()
{
    t++;
    if (t % 1024 == 0)
    {
        t = 0;
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

void bldc_output(uint8_t hall_now, uint16_t output_duty)
{
    switch (hall_now)
    {
    case 1:
        mos_a_bn_open_middle(output_duty);
        adc_mid_value = (adc_abmf_value + adc_bbmf_value) / 2;
        adc_global_value = adc_cbmf_value - adc_mid_value;
        break;
    case 2:
        mos_a_cn_open_middle(output_duty);
        adc_mid_value = (adc_abmf_value + adc_cbmf_value) / 2;
        adc_global_value = adc_bbmf_value - adc_mid_value;
        break;
    case 3:
        mos_b_cn_open_middle(output_duty);
        adc_mid_value = (adc_cbmf_value + adc_bbmf_value) / 2;
        adc_global_value = adc_abmf_value - adc_mid_value;
        break;
    case 4:
        mos_b_an_open_middle(output_duty);
        adc_mid_value = (adc_abmf_value + adc_bbmf_value) / 2;
        adc_global_value = adc_cbmf_value - adc_mid_value;
        break;
    case 5:
        mos_c_an_open_middle(output_duty);
        adc_mid_value = (adc_abmf_value + adc_cbmf_value) / 2;
        adc_global_value = adc_bbmf_value - adc_mid_value;
        break;
    case 6:
        mos_c_bn_open_middle(output_duty);
        adc_mid_value = (adc_cbmf_value + adc_bbmf_value) / 2;
        adc_global_value = adc_abmf_value - adc_mid_value;
        break;
    default:
        mos_close_middle();
        break;
    }
    data_send[25] = adc_abmf_value;
    data_send[26] = adc_bbmf_value;
    data_send[27] = adc_cbmf_value;
    data_send[28] = adc_mid_value;
    data_send[29] = adc_global_value;
}