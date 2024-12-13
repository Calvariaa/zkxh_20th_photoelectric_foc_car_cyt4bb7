#include "brushless/bldc.h"

// ����Ӳ����������
int8_t hall_hw_order[7] = {0, 6, 2, 3, 1, 5, 4};

// ����Ӳ����������λ��ת��
// �����ɼ�����ֵΪ4 ����λ��ת������ó���ǰλ����6 ��Ӧ����Ӳ���������еĵ�6������
//     ��ǰλ��Ϊ6 ���������Ҫ���������ת����������һ��λ�û�����һ��λ��
//     �����һ��λ������5 �������Ӳ���������еó���5������ֵΪ5 ���������Ϊ5����λ
//     �����һ��λ������1 �������Ӳ���������еó���1������ֵΪ6 ���������Ϊ6����λ
int8_t hall_hw_order_transition[7] = {0, 4, 2, 3, 6, 5, 1};
//-------------------------------------------------------------------------------------------------------------------
// �������     BLDC�����������
// ����˵��     hall_now        ��ǰ����ֵ
// ���ز���     void
// ʹ��ʾ��     bldc_output(1);
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
void bldc_output(uint8_t hall_now)
{
    uint16_t output_duty = 0;
    int8_t hall_output = 0;

    output_duty = PWM_PRIOD_LOAD / 2;

    // output_duty = func_limit_ab(motor_control.motor_duty - motor_control.motor_duty_offset, 0, PWM_PRIOD_LOAD / 100 * 99);

    if (0 != hall_now) // ��������λ��Чʱִ����λƫ��
    {
        // �û����ò���Ϊ�������Ƕ� �����ֻ���ǹ̶��Ƕ� ������ӳٻ������ʵ������ǶȻ���
        // ���������Ƕ���60�� ���Դ˴���ȡ�ж����ࣺX = (motor_control.motor_control_angle / 60)
        // ��������������λʱ��ֱ����һ������λ (motor_control.motor_control_angle % 60) == 0 ? X : X +1

        hall_output = (120 / 60);
        hall_output += ((120 % 60) > 0 ? 1 : 0);

        // if (FORWARD == motor_control.motor_set_dir) // ���õ����תʱִ����ת��ƫ�Ʒ���
        // {
        // ����ʵ�ʻ�����λ�� ���Եó���Ҫ����Ļ���λ���Ƕ���
        hall_output = hall_hw_order_transition[hall_now] + hall_output;

        if (6 < hall_output) // ������������λ��ʱ ��ȥ���λ��ʵ��ѭ��
        {
            hall_output -= 6;
        }

        hall_output = hall_hw_order[hall_output]; // ��ȡ��Ӧλ�õĻ������루������λ��ƫ�Ƽ��� �������Ӧ�ø���λ�ò�ѯ��ȷ��λ��
        // }
        // else // ���õ����תʱִ�з�ת��ƫ�Ʒ���
        // {
        //     // ��ȥʵ�ʻ�����λ�� ���Եó���Ҫ����Ļ���λ���Ƕ���
        //     hall_output = hall_hw_order_transition[hall_now] - hall_output;

        //     if (1 > hall_output)
        //     {
        //         hall_output += 6;
        //     }
        //     hall_output = hall_hw_order[hall_output];
        // }
    }

    // ���ݼ���õ������λ���ö�Ӧ��������� ʹ�����ת��ָ��λ��
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