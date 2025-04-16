/*
 * foc.c
 *
 *  Created on: 2023年3月30日
 *      Author: 11474
 */
#include "zf_common_typedef.h"
#include "fastmath/cos_sin.h"
#include "brushless/foc.h"
#include "brushless/encoder/encoder.h"
#include "brushless/move_filter.h"
#include "brushless/motor.h"
#include "debug/vofaplus.h"

extern bool protect_flag;

uint16 ierror_count = 0u;
#ifdef CURRENTLOOP
//-------------------------------------------------------------------------------------------------------------------
//  @brief      克拉克变换
//  @param      void
//  @return     void
//  @since      none
//-------------------------------------------------------------------------------------------------------------------
clark_variable clark_cacl(adc_struct current)
{
    clark_variable clark;
    clark.Alpha = 1.5 * current.current_a * 2 / 3;
    clark.Beta = (sqrt3 / 2.0) * current.current_a * 2 / 3 + sqrt3 * current.current_b * 2 / 3;

    return clark;
}
//-------------------------------------------------------------------------------------------------------------------
//  @brief      帕克变换
//  @param      clark:克拉克结构体
//  @param      theta:角度
//  @return     帕克变换计算结果
//  @since
//-------------------------------------------------------------------------------------------------------------------
park_variable park_cacl(clark_variable clark, double theta)
{
    park_variable park;

    park.id_ref = clark.Alpha * fast_cos(theta) + clark.Beta * fast_sin(theta);
    park.iq_ref = -clark.Alpha * fast_sin(theta) + clark.Beta * fast_cos(theta);
    move_filter_double_calc(&id_ref_filter, park.id_ref);
    move_filter_double_calc(&iq_ref_filter, park.iq_ref);
    park.id_ref = id_ref_filter.data_average;
    park.iq_ref = iq_ref_filter.data_average;
    return park;
}
#endif

//-------------------------------------------------------------------------------------------------------------------
//  @brief      帕克逆变换
//  @param      park:帕克结构体
//  @param      theta:角度
//  @return     帕克逆变换计算结果
//  @since
//-------------------------------------------------------------------------------------------------------------------
out_variable iPark_Calc(ipark_variable park, double theta)
{
    out_variable u_in;

    u_in.u_alpha = park.u_d * fast_cos(theta) - park.u_q * fast_sin(theta);
    u_in.u_beta = park.u_d * fast_sin(theta) + park.u_q * fast_cos(theta);
    // look3 =park.u_q*10000;
    return u_in;
}
//-------------------------------------------------------------------------------------------------------------------
//  @brief      中间变量计算
//  @param      clark: Alpha & Beta
//  @return     中间变量计算结果
//  @since      x,y,z为中间变量，无实际意义
//-------------------------------------------------------------------------------------------------------------------
Instrument_Typedef Tool_Calc(out_variable V_Clark)
{
    Instrument_Typedef tool;

    tool.x = V_Clark.u_beta;
    tool.y = V_Clark.u_alpha * sqrt3 / 2.0 - V_Clark.u_beta / 2.0;
    tool.z = -V_Clark.u_alpha * sqrt3 / 2.0 - V_Clark.u_beta / 2.0;

    return tool;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      电角度扇区判断
//  @param      temp:用于辅助计算的中间变量
//  @return     N = （1~6）
//              3 1 5 4 6 2 3（N值）
//              1 2 3 4 5 6 1（扇区）
//  @since      x,y,z为中间变量，无实际意义
//-------------------------------------------------------------------------------------------------------------------
uint8 Electrical_Sector_Judge(Instrument_Typedef tool)
{
    uint8 N = 0;

    if (tool.x > 0)
        N = N + 1;
    if (tool.y > 0)
        N = N + 2;
    if (tool.z > 0)
        N = N + 4;

    return N;
}
// int PID[3]={0,0,0};
// 函数合起来

//-------------------------------------------------------------------------------------------------------------------
//  @brief      每个扇区两基矢量作用时间计算
//  @param      tool:用于辅助计算的中间变量
//  @param      N:判断扇区的N值
//  @param      Udc:母线电压
//  @param      T:PWM周期（装载值）
//  @return     各扇区时间计算结果
//  @since      根据扇区分别计算在各扇区中两基矢量的作用时间
//-------------------------------------------------------------------------------------------------------------------
VectorTime_Typedef Vector_Calc(Instrument_Typedef tool, uint8 N, uint8 Udc, uint16 T)
{
    VectorTime_Typedef vector;

    double temp = sqrt3 * T / Udc; // 为了等幅值变换，已乘以2/3

    switch (N)
    {
    case 3: // 扇区1
        vector.ta = temp * tool.y;
        vector.tb = temp * tool.x;
        break;
    case 1: // 扇区2
        vector.ta = -temp * tool.y;
        vector.tb = -temp * tool.z;
        break;
    case 5: // 扇区3
        vector.ta = temp * tool.x;
        vector.tb = temp * tool.z;
        break;
    case 4: // 扇区4
        vector.ta = -temp * tool.x;
        vector.tb = -temp * tool.y;
        break;
    case 6: // 扇区5
        vector.ta = temp * tool.z;
        vector.tb = temp * tool.y;
        break;
    case 2: // 扇区6
        vector.ta = -temp * tool.z;
        vector.tb = -temp * tool.x;
        break;
    default:
        vector.ta = 0;
        vector.tb = 0;
        break;
    }

    return vector;
}

Period_Typedef PeriodCal(VectorTime_Typedef vector, uint8 N, uint16 T)
{
    Period_Typedef period;
    uint16 value1, value2, value3;
    double Ttemp = vector.ta + vector.tb;

    if (Ttemp > T)
    {
        vector.ta = vector.ta / Ttemp * (double)T;
        vector.tb = vector.tb / Ttemp * (double)T;
    }
    value1 = (uint16)(((double)T - vector.ta - vector.tb) / 2.0);
    value2 = (uint16)(value1 + vector.ta);
    value3 = (uint16)(value2 + vector.tb);
    switch (N)
    {
    case 3:
        period.AH = value1;
        period.BH = value2;
        period.CH = value3;
        break;
    case 1:
        period.AH = value2;
        period.BH = value1;
        period.CH = value3;
        break;
    case 5:
        period.AH = value3;
        period.BH = value1;
        period.CH = value2;
        break;
    case 4:
        period.AH = value3;
        period.BH = value2;
        period.CH = value1;
        break;
    case 6:
        period.AH = value2;
        period.BH = value3;
        period.CH = value1;
        break;
    case 2:
        period.AH = value1;
        period.BH = value3;
        period.CH = value2;
        break;
    default:
        period.AH = PWM_PRIOD_LOAD;
        period.BH = PWM_PRIOD_LOAD;
        period.CH = PWM_PRIOD_LOAD;
        break;
    }

    return period;
}

// FOC_Parm_Typedef ccu6_pwm;
//-------------------------------------------------------------------------------------------------------------------
//   @brief      d、q轴PI控制器
//   @param      ref_park:d、q目标值
//   @param      I_park  :d、q实际值
//   @return     d、q输入值
//   @since
//-------------------------------------------------------------------------------------------------------------------
ipark_variable Current_Close_Loop(FOC_Parm_Typedef *__foc_, park_variable I_park)
{
    float error_d, error_q;

    // int slow_startup_count = 0;

    float kp_foc_id = 120;  // 10
    float ki_foc_id = 0.08; // 0.06
    float kp_foc_iq = 7;    // 20
    float ki_foc_iq = 0.08; // 0.06
    // if(slow_startup_count<=100000)
    //{
    // slow_startup_count++;
    // }
    // if(slow_startup_count>=100000)
    //{
    //     kp_foc_id = 7;
    //     ki_foc_id = 0.17;
    //     kp_foc_iq = 7;    ///计时，防止未打开开关前积分累加
    //     ki_foc_iq = 0.17;
    //     slow_startup_count = 199999;
    // }
    // if(slow_startup_count<100000)
    //{
    //      kp_foc_id = 0;
    //      ki_foc_id = 0;
    //      kp_foc_iq = 0;
    //      ki_foc_iq = 0;
    //     __foc_->error_sum_d=0;
    //     __foc_->error_sum_q=0;
    // }
    error_d = __foc_->Ref_Park.u_d - I_park.id_ref;
    error_q = __foc_->Ref_Park.u_q - I_park.iq_ref;

    // look5 = error_d * 10000;
    // look6 = error_q * 10000; // look1-5都是用于上位机查看波形

    // look4 = (error_q)*10000;
    // look3 =I_park.id_ref*10000;
    __foc_->error_sum_d = __foc_->error_sum_d + error_d;
    __foc_->error_sum_q = __foc_->error_sum_q + error_q;

    if (__foc_->error_sum_d > 70)
        __foc_->error_sum_d = 70; // 积分限幅
    if (__foc_->error_sum_d < -70)
        __foc_->error_sum_d = -70; // 积分限幅
    if (__foc_->error_sum_q > 70)
        __foc_->error_sum_q = 70; // 积分限幅
    if (__foc_->error_sum_q < -70)
        __foc_->error_sum_q = -70; // 积分限幅
    //  look5 =__foc_->error_sum_d*100;
    // look6 =__foc_->error_sum_q*100;
    __foc_->Park_in.u_d = kp_foc_id * error_d + ki_foc_id * __foc_->error_sum_d;
    __foc_->Park_in.u_q = kp_foc_iq * error_q + ki_foc_iq * __foc_->error_sum_q;
    if (__foc_->Park_in.u_d >= 4)
    {
        __foc_->Park_in.u_d = 4;
    }
    if (__foc_->Park_in.u_d <= 0 - 4)
    {
        __foc_->Park_in.u_d = 0 - 4;
    }
    if (__foc_->Park_in.u_q <= 0 - 6)
    {
        __foc_->Park_in.u_q = 0 - 6;
    }
    if (__foc_->Park_in.u_q >= 6)
    {
        __foc_->Park_in.u_q = 6;
    }

    // if(__foc_->Park_in.u_q>=6.5)
    //{
    //     __foc_->Park_in.u_q=6.5;
    // }
    // if(__foc_->Park_in.u_d>=1)
    //{
    //     __foc_->Park_in.u_d=1;
    // }
    // if(__foc_->Park_in.u_q<=0-6.5)
    //{
    //     __foc_->Park_in.u_q=0-6.5;
    // }
    // if(__foc_->Park_in.u_d<=0-1)
    //{
    //     __foc_->Park_in.u_d=0-1;
    // }
    // look1 = I_park.id_ref*1000;

    // look3 =I_park.iq_ref*1000;
    // __foc_->Park_in.u_d=1;
    // __foc_->Park_in.u_q=0;
    return __foc_->Park_in;
}

#define SAMPLE_RATE (80000000.f / PWM_PRIOD_LOAD)
float get_ud_freq(FOC_Parm_Typedef *__foc_, uint16_t _frequency, float _amplitude)
{
    if (_frequency == 0)
        return 0;

    __foc_->ud_phase += pi_2 * _frequency / SAMPLE_RATE;
    if (__foc_->ud_phase >= pi_2)
    {
        __foc_->ud_phase -= pi_2;
    }

    if (fast_sin(__foc_->ud_phase) >= 0.0f)
    {
        return _amplitude;
    }
    else
    {
        return -_amplitude;
    }

    // return sin(__foc_->ud_phase) * AMPLITUDE;

    // if (__foc_->ud_phase < PI) {
    //     return AMPLITUDE * (__foc_->ud_phase / pi - 0.5f);
    // } else {
    //     return AMPLITUDE * (1.5f - __foc_->ud_phase / pi);
    // }
}

// #define CURRENTLOOP
// #define TESTMODE

FOC_Parm_Typedef foc_left = {0};
FOC_Parm_Typedef foc_right = {0};

void foc_commutation(FOC_Parm_Typedef *__foc_, encoder_t *__encoder_, pid_param_t *__pid_, void (*__mos_all_open_)(uint16_t, uint16_t, uint16_t))
{
    __encoder_->theta_val = __encoder_->__get_magnet_val_();
    __encoder_->theta_magnet = get_magnet_angle(__encoder_->theta_val, __encoder_->zero_angle);
    __encoder_->theta_elec = get_elec_angle(__encoder_->theta_val, __encoder_->zero_reval);
    __encoder_->full_rotations = get_magnet_angle_rot(__encoder_->theta_magnet, __encoder_);

#ifdef CURRENTLOOP

    // get adc conv

    /*----------*/
    __foc_->I_Clrak = clark_cacl(adc_information);
    /*----------*/
    __foc_->I_Park = park_cacl(__foc_->I_Clrak, theta);

    // data_send(4, __foc_->I_Park.id_ref * 10000);
    // data_send(5, __foc_->I_Park.iq_ref * 10000);

    // look5 =  adc_information.current_a *1000;
    //  look6 =  adc_information.current_b *1000;
    // data_send(3]=(int16) __foc_->I_Park.id_ref*1000);
    // data_send(4]=(int16)  __foc_->I_Park.iq_ref*1000);

    if (fabsf(__foc_->Park_in.u_q) < FOC_UQ_MAX)
        __foc_->set_angle += ANGLE_TO_RAD(0.01);

    // if (ierror_count < 20)
    // {
    //     // set_angle += motor_control.set_speed;
    // }

    // if (ierror_count > 1000)
    // {
    //     set_angle = theta_magnet;
    //     expect_rotations = full_rotations;
    // }

    // data_send(14, ierror_count);
    if (__foc_->set_angle >= pi_2)
    {
        __foc_->expect_rotations++;
        __foc_->set_angle -= pi_2;
    }
    if (__foc_->set_angle < -pi_2)
    {
        __foc_->expect_rotations--;
        __foc_->set_angle += pi_2;
    }

    __foc_->Ref_Park.u_d = get_ud_freq(__foc_, __foc_->foc_ud_freq, __foc_->foc_ud_amp);

    if (!protect_flag)
        __foc_->Ref_Park.u_q = pid_solve(__pid_, (__foc_->set_angle + __foc_->expect_rotations * pi_2) * __encoder_->turn_dir - (__encoder_->theta_magnet + __encoder_->full_rotations * pi_2)) * __encoder_->polarity;
    else
        __foc_->Ref_Park.u_q = 0;

    Current_Close_Loop(&__foc_, __foc_->I_Park);

    // data_send(6, __foc_->Park_in.u_q * 1000);
    // data_send(7,  __foc_->Park_in.u_d * 1000);
#elif defined TESTMODE
    // test
    __foc_->set_angle += ANGLE_TO_RAD(0.01);
    if (__foc_->set_angle >= pi_2)
    {
        __foc_->expect_rotations++;
        __foc_->set_angle -= pi_2;
    }
    if (__foc_->set_angle < -pi_2)
    {
        __foc_->expect_rotations--;
        __foc_->set_angle += pi_2;
    }

    __foc_->Park_in.u_d = 0;
    __foc_->Park_in.u_q = 1;

    // data_send(13, __encoder_->theta_elec - __foc_->set_angle);
    // data_send(14, __foc_->Park_in.u_q);

    __foc_->V_Clark = iPark_Calc(__foc_->Park_in, -__foc_->set_angle);
#else

    __foc_->Park_in.u_d = get_ud_freq(__foc_, __foc_->foc_ud_freq, 0.3);

    // __foc_->Park_in.u_d = get_ud_freq(__foc_, __foc_->foc_ud_freq, __foc_->foc_ud_amp);

    // if (j++ % 50)
    // {
    //     j = 0;
    // }

    // test
    if (fabsf(__foc_->Park_in.u_q) < FOC_UQ_MAX)
        __foc_->set_angle += __foc_->foc_speed;

    // if (ierror_count < 20)
    // {
    //     // set_angle += motor_control.set_speed;
    // }

    // if (ierror_count > 1000)
    // {
    //     set_angle = theta_magnet;
    //     expect_rotations = full_rotations;
    // }
    if (__foc_->set_angle >= pi_2)
    {
        __foc_->expect_rotations++;
        __foc_->set_angle -= pi_2;
    }
    if (__foc_->set_angle < -pi_2)
    {
        __foc_->expect_rotations--;
        __foc_->set_angle += pi_2;
    }

    // move_filter_double_calc(&speed_filter, get_magnet_speed(__encoder_->theta_magnet, __encoder_->full_rotations, __encoder_->theta_magnet_last, __encoder_->full_rotations_last, PWM_PRIOD_LOAD));

    // __foc_->Park_in.u_q = 2;
    if (!protect_flag)
        __foc_->Park_in.u_q = pid_solve(__pid_, (__foc_->set_angle + __foc_->expect_rotations * pi_2) * __encoder_->turn_dir - (__encoder_->theta_magnet + __encoder_->full_rotations * pi_2)) * __encoder_->polarity;
    else
        __foc_->Park_in.u_q = 0;

    //test

    __foc_->V_Clark = iPark_Calc(__foc_->Park_in, __encoder_->theta_elec);

#endif

    __foc_->tool = Tool_Calc(__foc_->V_Clark);         // 中间变量计算
    __foc_->N = Electrical_Sector_Judge(__foc_->tool); // 电角度扇区判断

    __foc_->Vector = Vector_Calc(__foc_->tool, __foc_->N, BUS_VOLTAGE, PWM_PRIOD_LOAD); // 矢量作用时间计算
    __foc_->Period = PeriodCal(__foc_->Vector, __foc_->N, PWM_PRIOD_LOAD);              // 各桥PWM占空比计算

    __mos_all_open_(__foc_->Period.AH, __foc_->Period.BH, __foc_->Period.CH);

    __encoder_->theta_magnet_last = __encoder_->theta_magnet;
    __encoder_->full_rotations_last = __encoder_->full_rotations;
}