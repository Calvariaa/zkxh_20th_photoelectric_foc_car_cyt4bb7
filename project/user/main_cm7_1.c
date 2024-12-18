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
#include "brushless/adc.h"
#include "fastmath/cos_sin.h"
#include "arm_math.h"

bool protect_flag = 0;
#define LED1 (P19_0)
#define TESTPIN (P20_1)

extern FOC_Parm_Typedef foc_left;
extern FOC_Parm_Typedef foc_right;

extern encoder_t encoder_left;
extern encoder_t encoder_right;

extern uint64_t timer_1ms;

int main(void)
{
    clock_init(SYSTEM_CLOCK_250M); // ʱ�����ü�ϵͳ��ʼ��<��ر���>
    debug_info_init();             // ���Դ�����Ϣ��ʼ��

    // �˴���д�û����� ���������ʼ�������

    gpio_init(LED1, GPO, GPIO_LOW, GPO_PUSH_PULL); // ��ʼ�� LED1 ��� Ĭ�ϸߵ�ƽ �������ģʽ
    gpio_init(TESTPIN, GPO, GPIO_LOW, GPO_PUSH_PULL); // ��ʼ�� LED1 ��� Ĭ�ϸߵ�ƽ �������ģʽ

    buzzer_init(1);

    // move_filter_double_init(&current_a_filter);
    // move_filter_double_init(&current_b_filter);
    // move_filter_double_init(&current_c_filter);
    // move_filter_double_init(&iq_ref_filter);
    // move_filter_double_init(&id_ref_filter);
    // move_filter_double_init(&speed_filter);
    encoder_init();

    interrupt_global_disable(); // �ر�ȫ���ж�

    motor_parameter_init(); // ���������ʼ��

    motor_bldc_adc_init();

    // gpio_init(P20_1, GPI, 0, GPI_PULL_DOWN);
    // gpio_init(P20_3, GPI, 0, GPI_PULL_DOWN);
    // gpio_init(P21_6, GPI, 0, GPI_PULL_DOWN);

    cy_stc_gpio_pin_config_t gpio_pin_config = {0};
    gpio_pin_config.driveMode = CY_GPIO_DM_PULLUP;
    Cy_GPIO_Pin_Init(get_port(P20_1), (P20_1 % 8), &gpio_pin_config);
    Cy_GPIO_Pin_Init(get_port(P20_3), (P20_3 % 8), &gpio_pin_config);
    Cy_GPIO_Pin_Init(get_port(P21_6), (P21_6 % 8), &gpio_pin_config);

    pit_us_init(PIT_CH0, 50); // 20khz
    pit_us_init(PIT_CH1, 50);
    pit_us_init(PIT_CH2, 50);

    pit_ms_init(PIT_CH3, 1); // 1ms

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

        data_send[1] = (float)foc_left.Period.AH;
        data_send[2] = (float)foc_left.Period.BH;
        data_send[3] = (float)foc_left.Period.CH;

        data_send[4] = (float)foc_right.Period.AH;
        data_send[5] = (float)foc_right.Period.BH;
        data_send[6] = (float)foc_right.Period.CH;

        data_send[10] = (float)encoder_left.theta_val;
        data_send[11] = (float)encoder_right.theta_val;

        data_send[12] = (float)encoder_left.theta_elec;
        data_send[13] = (float)encoder_right.theta_elec;

        // data_send[12] = (float)(foc_left.set_angle + foc_left.expect_rotations * pi_2) - (encoder_left.theta_magnet + encoder_left.full_rotations * pi_2);
        // data_send[13] = (float)(foc_right.set_angle + foc_right.expect_rotations * pi_2) - (encoder_right.theta_magnet + encoder_right.full_rotations * pi_2);
        data_send[14] = (float)(foc_left.set_angle);
        data_send[15] = (float)(foc_right.set_angle);

        data_send[16] = (float)foc_left.Park_in.u_q;
        data_send[17] = (float)foc_right.Park_in.u_q;

        // data_send[18] = (float)speed_filter.data_average;

        // data_send[25] = (float)adc_convert(ADC1_CH28_P15_0) - adc_convert(ADC1_CH31_P15_3);
        // data_send[26] = (float)adc_convert(ADC1_CH29_P15_1) - adc_convert(ADC1_CH31_P15_3);
        // data_send[27] = (float)adc_convert(ADC1_CH30_P15_2) - adc_convert(ADC1_CH31_P15_3);
        // data_send[28] = (float)adc_convert(ADC1_CH31_P15_3);

        send_vofaplus();
    }
}

// **************************** �������� ****************************
