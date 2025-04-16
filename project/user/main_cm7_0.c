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
 * �ļ�����          main_cm7_0
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
#include "debug/vofaplus.h"
#include "car_control/gyro.h"
#include "car_control/adc.h"

#pragma location = 0x280010C8
float memory_from_cm70_to_cm71[7];
// foc���أ������ٶȣ������ٶȣ��������أ������ٶȣ���Ƶע��Ƶ��
#define FOC_LEFT_START memory_from_cm70_to_cm71[0]  // foc�󿪹�
#define FOC_LEFT_SPEED memory_from_cm70_to_cm71[1]  // �����ٶ�
#define FOC_RIGHT_START memory_from_cm70_to_cm71[2] // foc�ҿ���
#define FOC_RIGHT_SPEED memory_from_cm70_to_cm71[3] // �����ٶ�
#define BLDC_START memory_from_cm70_to_cm71[4]      // ��������
#define BLDC_SPEED memory_from_cm70_to_cm71[5]      // �����ٶ�
#define UQ_FREQ memory_from_cm70_to_cm71[6]         // ��Ƶע��Ƶ��

#pragma location = 0x280010C8 + sizeof(memory_from_cm70_to_cm71)
__no_init float memory_from_cm71_to_cm70[4];

int main(void)
{
    clock_init(SYSTEM_CLOCK_250M); // ʱ�����ü�ϵͳ��ʼ��<��ر���>
    debug_info_init();             // ���Դ�����Ϣ��ʼ��
                                   // �˴���д�û����� ���������ʼ�������

    // ips114_init();

    interrupt_global_disable();

    gyro_init();

    tube_adc_init();

    pit_us_init(PIT_CH1, 200); // 0.2ms
    interrupt_global_enable(0);
    // ips114_clear();
    while (true)
    {
        tube_adc_convert();

        data_send(21, (float)imu_data.gyro_z);
        // for (uint8_t i = 0; i < 20; i++)
        //     data_send(i + 1, (float)adc_tube_read_raw[i]);

        data_send_clear();

        FOC_LEFT_START = 1;    // foc�󿪹�
        FOC_LEFT_SPEED = ANGLE_TO_RAD(imu_data.gyro_z * 0.0004); // �����ٶ�
        FOC_RIGHT_START = 1;    // foc�ҿ���
        FOC_RIGHT_SPEED = ANGLE_TO_RAD(imu_data.gyro_z * -0.0004); // �����ٶ�
        BLDC_START = -1;   // ��������
        BLDC_SPEED = 0;    // �����ٶ�
        UQ_FREQ = 0;    // ��Ƶע��Ƶ��

        SCB_CleanInvalidateDCache_by_Addr(&memory_from_cm70_to_cm71, sizeof(memory_from_cm70_to_cm71));
        SCB_CleanInvalidateDCache_by_Addr(&memory_from_cm71_to_cm70, sizeof(memory_from_cm71_to_cm70));
        // read some shit

        // ips114_show_float(0, 0, imu_data.gyro_x, 8, 4);
        // ips114_show_float(0, 16, imu_data.gyro_y, 8, 4);
        // ips114_show_float(0, 32, imu_data.gyro_z, 8, 4);
    }
}

// **************************** �������� ****************************
