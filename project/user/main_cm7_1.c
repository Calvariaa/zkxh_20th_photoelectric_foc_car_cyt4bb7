/*********************************************************************************************************************
 * CYT4BB Opensourec Library ���� CYT4BB ��Դ�⣩��һ�����ڹٷ� SDK �ӿڵĵ�������Դ��
 * Copyright (c) 2022 SEEKFREE ��ɿƼ�
 *
 * ���ļ��� CYT4BB ��Դ���һ����
 *
 * CYT4BB ��Դ�� ��������
 * �����Ը��������������ᷢ���� GPL��GNU General Public License���� GNUͨ�ù������֤��������
 * �� GPL �ĵ�3�棨�� GPL3.0������ѡ��ģ��κκ����İ汾�����·�����/���޸���
 *
 * ����Դ��ķ�����ϣ�����ܷ������ã�����δ�������κεı�֤
 * ����û�������������Ի��ʺ��ض���;�ı�֤
 * ����ϸ����μ� GPL
 *
 * ��Ӧ�����յ�����Դ���ͬʱ�յ�һ�� GPL �ĸ���
 * ���û�У������<https://www.gnu.org/licenses/>
 *
 * ����ע����
 * ����Դ��ʹ�� GPL3.0 ��Դ���֤Э�� �����������Ϊ���İ汾
 * �������Ӣ�İ��� libraries/doc �ļ����µ� GPL3_permission_statement.txt �ļ���
 * ���֤������ libraries �ļ����� �����ļ����µ� LICENSE �ļ�
 * ��ӭ��λʹ�ò����������� ���޸�����ʱ���뱣����ɿƼ��İ�Ȩ����������������
 *
 * �ļ�����          main_cm7_1
 * ��˾����          �ɶ���ɿƼ����޹�˾
 * �汾��Ϣ          �鿴 libraries/doc �ļ����� version �ļ� �汾˵��
 * ��������          IAR 9.40.1
 * ����ƽ̨          CYT4BB
 * ��������          https://seekfree.taobao.com/
 *
 * �޸ļ�¼
 * ����              ����                ��ע
 * 2024-1-4       pudding            first version
 ********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "brushless/move_filter.h"
#include "brushless/motor.h"
#include "brushless/foc.h"
#include "debug/vofaplus.h"
#include "brushless/encoder/encoder.h"
#include "brushless/move_filter.h"
#include "brushless/buzzer.h"
#include "fastmath/cos_sin.h"
#include "arm_math.h"

bool protect_flag = 0;
#define LED1 (P19_0)

extern FOC_Parm_Typedef FOC_L;
extern FOC_Parm_Typedef FOC_R;

extern encoder_t encoder_left;
extern encoder_t encoder_right;

extern uint64_t timer_1ms;

int main(void)
{
    clock_init(SYSTEM_CLOCK_250M); // ʱ�����ü�ϵͳ��ʼ��<��ر���>
    debug_info_init();             // ���Դ�����Ϣ��ʼ��

    // �˴���д�û����� ���������ʼ�������

    gpio_init(LED1, GPO, GPIO_LOW, GPO_PUSH_PULL); // ��ʼ�� LED1 ��� Ĭ�ϸߵ�ƽ �������ģʽ

    buzzer_init(1);

    move_filter_double_init(&current_a_filter);
    move_filter_double_init(&current_b_filter);
    move_filter_double_init(&current_c_filter);
    move_filter_double_init(&iq_ref_filter);
    move_filter_double_init(&id_ref_filter);
    move_filter_double_init(&speed_filter);
    encoder_init();

    interrupt_global_disable(); // �ر�ȫ���ж�

    motor_parameter_init(); // ���������ʼ��

    pit_us_init(PIT_CH0, 50); // 20khz
    pit_us_init(PIT_CH1, 50);
    pit_us_init(PIT_CH2, 50);

    pit_ms_init(PIT_CH3, 1);  // 1ms

    interrupt_global_enable(0);

    // play_music();

    buzz_keep_ms(100, 0);
    buzz_keep_ms(140, NOTE_C6);
    buzz_keep_ms(10, 0);

    buzz_keep_ms(140, NOTE_G6);
    buzz_keep_ms(10, 0);

    buzz_keep_ms(140, NOTE_C7);
    buzz_keep_ms(10, 0);

    buzz_ease_ms(180, NOTE_E6, NOTE_F6);
    buzz_ease_ms(100, NOTE_F6, NOTE_A6);
    buzz_keep_ms(10, 0);

    buzz_keep_ms(140, NOTE_G6);
    buzz_keep_ms(10, 0);

    // �˴���д�û����� ���������ʼ�������
    while (true)
    {
        // �˴���д��Ҫѭ��ִ�еĴ���

        gpio_toggle_level(LED1);
        // buzz_keep_ms(1, NOTE_C5);
        // foc_ud_freq = NOTE_C5;
        // �˴���д��Ҫѭ��ִ�еĴ���
        // mos_all_open_left(3000, 1000, 0);
        // foc_commutation();
        // for (uint16 i = 0; i < 4095; i++)
        //     ;
        // spi_write_16bit(SPI_0, 0xffff);

        data_send[1] = (float)FOC_L.Period.AH;
        data_send[2] = (float)FOC_L.Period.BH;
        data_send[3] = (float)FOC_L.Period.CH;

        data_send[4] = (float)FOC_R.Period.AH;
        data_send[5] = (float)FOC_R.Period.BH;
        data_send[6] = (float)FOC_R.Period.CH;

        data_send[7] = (float)FOC_R.Period.AH;
        data_send[8] = (float)FOC_R.Period.BH;
        data_send[9] = (float)FOC_R.Period.CH;

        data_send[10] = (float)encoder_left.theta_val;
        data_send[11] = (float)encoder_right.theta_val;

        data_send[12] = (float)encoder_left.theta_elec;
        data_send[13] = (float)encoder_right.theta_elec;

        // data_send[12] = (float)(FOC_L.set_angle + FOC_L.expect_rotations * pi_2) - (encoder_left.theta_magnet + encoder_left.full_rotations * pi_2);
        // data_send[13] = (float)(FOC_R.set_angle + FOC_R.expect_rotations * pi_2) - (encoder_right.theta_magnet + encoder_right.full_rotations * pi_2);
        data_send[12] = (float)(FOC_L.set_angle);
        data_send[13] = (float)(FOC_R.set_angle);
        
        data_send[16] = (float)FOC_L.Park_in.u_q;
        data_send[17] = (float)FOC_R.Park_in.u_q;

        data_send[18] = (float)speed_filter.data_average;

        send_vofaplus();
    }
}

// **************************** �������� ****************************
