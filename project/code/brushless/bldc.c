#include "brushless/bldc.h"

// 霍尔硬件换相序列
int8_t hall_hw_order[7] = {0, 6, 2, 3, 1, 5, 4};

// 霍尔硬件换相序列位置转换
// 例：采集霍尔值为4 带入位置转换数组得出当前位置在6 对应霍尔硬件换相序列的第6个数据
//     当前位置为6 如果我们想要驱动电机旋转则可以输出下一个位置或者上一个位置
//     输出上一个位置则是5 带入霍尔硬件换相序列得出第5个数据值为5 则输出霍尔为5的相位
//     输出下一个位置则是1 带入霍尔硬件换相序列得出第1个数据值为6 则输出霍尔为6的相位
int8_t hall_hw_order_transition[7] = {0, 4, 2, 3, 6, 5, 1};
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     BLDC六步换相输出
// 参数说明     hall_now        当前霍尔值
// 返回参数     void
// 使用示例     bldc_output(1);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void bldc_output(uint8_t hall_now)
{
    uint16_t output_duty = 0;
    int8_t hall_output = 0;

    output_duty = PWM_PRIOD_LOAD / 2;

    // output_duty = func_limit_ab(motor_control.motor_duty - motor_control.motor_duty_offset, 0, PWM_PRIOD_LOAD / 100 * 99);

    if (0 != hall_now) // 当霍尔相位有效时执行相位偏移
    {
        // 用户设置参数为电机换相角度 而输出只能是固定角度 但结合延迟换相可以实现任意角度换相
        // 单相霍尔电角度是60° 所以此处先取有多少相：X = (motor_control.motor_control_angle / 60)
        // 当不满足完整相位时则直接算一整个相位 (motor_control.motor_control_angle % 60) == 0 ? X : X +1

        hall_output = (120 / 60);
        hall_output += ((120 % 60) > 0 ? 1 : 0);

        // if (FORWARD == motor_control.motor_set_dir) // 设置电机正转时执行正转的偏移方向
        // {
        // 加上实际霍尔的位置 可以得出需要输出的霍尔位置是多少
        hall_output = hall_hw_order_transition[hall_now] + hall_output;

        if (6 < hall_output) // 当输出超过最大位置时 减去最大位置实现循环
        {
            hall_output -= 6;
        }

        hall_output = hall_hw_order[hall_output]; // 获取对应位置的霍尔编码（上面是位置偏移计算 计算完成应该根据位置查询正确相位）
        // }
        // else // 设置电机反转时执行反转的偏移方向
        // {
        //     // 减去实际霍尔的位置 可以得出需要输出的霍尔位置是多少
        //     hall_output = hall_hw_order_transition[hall_now] - hall_output;

        //     if (1 > hall_output)
        //     {
        //         hall_output += 6;
        //     }
        //     hall_output = hall_hw_order[hall_output];
        // }
    }

    // 根据计算好的输出相位调用对应的输出函数 使电机旋转至指定位置
    switch (hall_output)
    {
    case 1:
        mos_q5q2_open_middle(output_duty);
        break; // 1
    case 2:
        mos_q1q4_open_middle(output_duty);
        break; // 2
    case 3:
        mos_q5q4_open_middle(output_duty);
        break; // 3
    case 4:
        mos_q3q6_open_middle(output_duty);
        break; // 4
    case 5:
        mos_q3q2_open_middle(output_duty);
        break; // 5
    case 6:
        mos_q1q6_open_middle(output_duty);
        break; // 6
    default:
        mos_close_middle();
        break;
    }
}