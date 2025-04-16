
#include "encoder.h"
#include "as5047p.h"
#include "mt6701ct.h"
#include "debug/vofaplus.h"

uint16_t get_magnet_val_left();
uint16_t get_magnet_val_right();
encoder_t encoder_left = {
    .theta_val = 0,

    .theta_elec = 0,

    .theta_magnet = 0,
    .full_rotations = 0,

    .theta_magnet_last = 0,
    .full_rotations_last = 0,

    .angle_prev = 0,
    .angle_rot_dat = 0,

    .__get_magnet_val_ = get_magnet_val_left,

    // 这个需要改
    .polarity = 1,
    .turn_dir = 1,
    .zero_reval = 4.613,
    .zero_angle = 0,
};
encoder_t encoder_right = {
    .theta_val = 0,

    .theta_elec = 0,

    .theta_magnet = 0,
    .full_rotations = 0,

    .theta_magnet_last = 0,
    .full_rotations_last = 0,

    .angle_prev = 0,
    .angle_rot_dat = 0,

    .__get_magnet_val_ = get_magnet_val_right,

    // 这个需要改
    .polarity = 1,
    .turn_dir = -1,
    .zero_reval = 4.2,
    .zero_angle = 0,
};

////-------------------------------------------------------------------------------------------------------------------
//  @brief      得到转子角度
//  @param      none
//  @return     转子角度（弧度）
//  @since      none
////-------------------------------------------------------------------------------------------------------------------

void encoder_init()
{
    // as5047p_init();
    mt6701ct_init();
}

uint16_t get_magnet_val_left()
{
    return mt6701ct_get_magnet_val_left();
    // return mt6701ct_get_magnet_val_left();
}

uint16_t get_magnet_val_right()
{
    // return as5047p_get_magnet_val_right();
    return mt6701ct_get_magnet_val_right();
}

double get_magnet_angle(uint16_t val, double zero_angle)
{
    double reval;
    val = val % 16384;
    reval = (double)val / 16384 * pi_2;
    reval -= zero_angle;
    if (reval < 0)
    {
        reval += pi_2;
    }
    return reval;
}

int32_t get_magnet_angle_rot(double reval, encoder_t *_enc)
{
    double d_angle = reval - _enc->angle_prev;
    if (fabs(d_angle) > (0.8f * pi_2))
        _enc->angle_rot_dat += (d_angle > 0.f) ? -1 : 1;

    _enc->angle_prev = reval;
    return _enc->angle_rot_dat;
}

void reset_rotations(encoder_t *_enc)
{
    _enc->angle_rot_dat = 0;
}

double get_elec_angle(uint16_t val, double zero_reval)
{
    double reval;
    val = val % 2340;
    reval = (double)val / 2340 * pi_2;
    reval -= zero_reval;
    if (reval < 0)
    {
        reval += pi_2;
    }
    return reval;
}

// 归一化角度
float _normalizeAngle(float angle)
{
    float a = fmod(angle, pi_2);
    return a >= 0 ? a : (a + pi_2);
}

double get_magnet_speed(double reval, int32_t reval_rot, double reval_last, int32_t reval_rot_last, uint32_t T)
{
    return (double)(((reval_rot - reval_rot_last) * pi_2) + (reval - reval_last)) * T;
}

void set_zero_angle(float angle, encoder_t *_enc)
{
    _enc->zero_angle = angle;
}