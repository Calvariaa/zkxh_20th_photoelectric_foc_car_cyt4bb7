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
#include "foc/move_filter.h"
#include "foc/motor.h"
#include "foc/foc.h"
#include "debug/vofaplus.h"
#include "foc/encoder/encoder.h"

bool protect_flag = 0;
#define LED1 (P19_0)
int main(void)
{
    clock_init(SYSTEM_CLOCK_250M); // ʱ�����ü�ϵͳ��ʼ��<��ر���>
    debug_info_init();             // ���Դ�����Ϣ��ʼ��

    // �˴���д�û����� ���������ʼ�������

    gpio_init(LED1, GPO, GPIO_LOW, GPO_PUSH_PULL); // ��ʼ�� LED1 ��� Ĭ�ϸߵ�ƽ �������ģʽ

    encoder_init();

    interrupt_global_disable(); // �ر�ȫ���ж�

    motor_parameter_init(); // ���������ʼ��

    pit_us_init(PIT_CH0, 50); // �����жϳ�ʼ��
    pit_us_init(PIT_CH1, 50); // �����жϳ�ʼ��
    pit_ms_init(PIT_CH2, 2);  // �����жϳ�ʼ��

    interrupt_global_enable(0);

    // �˴���д�û����� ���������ʼ�������
    while (true)
    {
        // �˴���д��Ҫѭ��ִ�еĴ���

        gpio_toggle_level(LED1);

        // �˴���д��Ҫѭ��ִ�еĴ���
        // mos_all_open_left(3000, 1000, 0);
        // foc_commutation();
        // for (uint16 i = 0; i < 4095; i++)
        //     ;
        // spi_write_16bit(SPI_0, 0xffff);
        send_vofaplus();
    }
}

// **************************** �������� ****************************
