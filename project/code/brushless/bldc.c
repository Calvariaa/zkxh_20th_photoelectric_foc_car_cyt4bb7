#include "brushless/bldc.h"
#include "debug/vofaplus.h"
#include "brushless/foc.h"

uint8_t n = 1;
uint16_t b = 384;
uint64_t t = 0;
uint16_t PRIOD = PWM_PRIOD_LOAD / 4;
void bldc_soft_openloop()
{
    t++;
    if (t % b == 0)
    {
        if (b > 128)
            b-=2;
        bldc_output(n++, PRIOD);
        if (n == 7)
            n = 1;
    }
    if (t > 20000 && t % 1024 == 0 && (b > 24 && b <= 128))
    {
        if (b > 32)
            PRIOD = PWM_PRIOD_LOAD / 2;
        else
            PRIOD = PWM_PRIOD_LOAD ;
        b--;
    }
    if (t > 20000 && t % 32768 == 0 && (b > 12 && b <= 24))
    {
        b--;
    }

    data_send[19] = (float)b;
    data_send[20] = (float)t;
}

FOC_Parm_Typedef FOC_M = {0};
void bldc_svpwm()
{
    FOC_M.set_angle += ANGLE_TO_RAD(0.04);
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

    FOC_M.Park_in.u_d = 0;
    FOC_M.Park_in.u_q = 2;

    FOC_M.V_Clark = iPark_Calc(FOC_M.Park_in, -FOC_M.set_angle);
    
    FOC_M.tool = Tool_Calc(FOC_M.V_Clark);         // 中间变量计算
    FOC_M.N = Electrical_Sector_Judge(FOC_M.tool); // 电角度扇区判断

    FOC_M.Vector = Vector_Calc(FOC_M.tool, FOC_M.N, BUS_VOLTAGE, PWM_PRIOD_LOAD); // 矢量作用时间计算
    FOC_M.Period = PeriodCal(FOC_M.Vector, FOC_M.N, PWM_PRIOD_LOAD);              // 各桥PWM占空比计算

    mos_all_open_middle(FOC_M.Period.AH, FOC_M.Period.BH, FOC_M.Period.CH);
}

void bldc_output(uint8_t hall_now, uint16_t output_duty)
{
    switch (hall_now)
    {
    case 1:
        mos_a_bn_open_middle(output_duty);
        break; // 1
    case 2:
        mos_a_cn_open_middle(output_duty);
        break; // 2
    case 3:
        mos_b_cn_open_middle(output_duty);
        break; // 3
    case 4:
        mos_b_an_open_middle(output_duty);
        break; // 4
    case 5:
        mos_c_an_open_middle(output_duty);
        break; // 5
    case 6:
        mos_c_bn_open_middle(output_duty);
        break; // 6
    default:
        mos_close_middle();
        break;
    }
}