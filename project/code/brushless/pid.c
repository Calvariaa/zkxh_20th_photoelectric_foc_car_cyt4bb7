#include "brushless/pid.h"
#include "brushless/foc.h"

pid_param_t foc_left_pid = PID_CREATE(2.4, 0.0008, 0., 0.8, 10., 0.8, 0, FOC_UQ_MAX);
pid_param_t foc_right_pid = PID_CREATE(2.4, 0.0008, 0., 0.8, 10., 0.8, 0, FOC_UQ_MAX);

// 常规PID
float pid_solve(pid_param_t *pid, float error)
{
    pid->out_d = (error - pid->out_p) * pid->low_pass + pid->out_d * (1 - pid->low_pass);
    pid->out_p = pid->kp * error;
    pid->out_i += error;

    if (pid->ki != 0)
        pid->out_i = MINMAX(pid->out_i, -pid->i_max, pid->i_max);

    return MINMAX(MINMAX(pid->out_p, -pid->p_max, pid->p_max) + pid->ki * pid->out_i + MINMAX(pid->kd * pid->out_d, -pid->d_max, pid->d_max), -pid->pid_max, pid->pid_max);
}

// 增量式PID
float increment_pid_solve(pid_param_t *pid, float error)
{
    pid->out_d = MINMAX(pid->kd * (error - 2 * pid->pre_error + pid->pre_pre_error), -pid->d_max, pid->d_max);
    pid->out_p = MINMAX(pid->kp * (error - pid->pre_error), -pid->p_max, pid->p_max);
    pid->out_i = MINMAX(pid->ki * error, -pid->i_max, pid->i_max);

    pid->pre_pre_error = pid->pre_error;
    pid->pre_error = error;

    pid->output = pid->out_p + pid->out_i + pid->out_d;

    if (pid->pre_output > 10000)
        if (pid->output > 0)
            pid->output = 0.;
    if (pid->pre_output < -10000)
        if (pid->output < 0)
            pid->output = 0.;

    pid->pre_output = pid->output;

    return pid->output;
}
