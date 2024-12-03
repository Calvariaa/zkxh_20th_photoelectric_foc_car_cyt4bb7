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
* �ļ�����          move_filter
* ��˾����          �ɶ���ɿƼ����޹�˾
* �汾��Ϣ          �鿴 libraries/doc �ļ����� version �ļ� �汾˵��
* ��������          IAR 9.40.1
* ����ƽ̨          CYT4BB
* ��������          https://seekfree.taobao.com/
*
* �޸ļ�¼
* ����              ����                ��ע
* 2024-1-18       pudding            first version
********************************************************************************************************************/

#include "foc/move_filter.h"

move_filter_double_struct speed_filter;
move_filter_double_struct current_a_filter;
move_filter_double_struct current_b_filter;
move_filter_double_struct current_c_filter;
move_filter_double_struct iq_ref_filter;
move_filter_double_struct id_ref_filter;
// move_filter_double_struct
//-------------------------------------------------------------------------------------------------------------------
//   @brief      ����ƽ���˲�����
//   @param      void
//   @return     void
//   @since      ��Ҫ���ڶ������˲����洢Ŀ�����������n�����ݣ������ƽ��ֵ
//-------------------------------------------------------------------------------------------------------------------
void move_filter_calc(move_filter_struct *move_filter, int32 new_data)
{
    // �����µ���ֵ ��ȥ��ĩβ����ֵ ������µĺ�
    move_filter->data_sum = move_filter->data_sum + new_data - move_filter->data_buffer[move_filter->index];
    // ������ƽ��ֵ
    move_filter->data_average = move_filter->data_sum / move_filter->buffer_size;

    // ������д�뻺����
    move_filter->data_buffer[move_filter->index] = new_data;
    move_filter->index++;
    if (move_filter->buffer_size <= move_filter->index)
    {
        move_filter->index = 0;
    }
}

void move_filter_double_calc(move_filter_double_struct *move_filter, double new_data)
{
    // �����µ���ֵ ��ȥ��ĩβ����ֵ ������µĺ�
    move_filter->data_sum = move_filter->data_sum + new_data - move_filter->data_buffer[move_filter->index];
    // ������ƽ��ֵ
    move_filter->data_average = move_filter->data_sum / move_filter->buffer_size;

    // ������д�뻺����
    move_filter->data_buffer[move_filter->index] = new_data;
    move_filter->index++;
    if (move_filter->buffer_size <= move_filter->index)
    {
        move_filter->index = 0;
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ����ƽ���˲���ʼ��
//  @param      void
//  @return     void
//  @since      ��Ҫ���ڶ������˲����洢Ŀ�����������n�����ݣ������ƽ��ֵ
//-------------------------------------------------------------------------------------------------------------------
void move_filter_init(move_filter_struct *move_filter)
{
    move_filter->data_average = 0;
    move_filter->data_sum = 0;
    move_filter->index = 0;
    // ���û�������С
    move_filter->buffer_size = MOVE_AVERAGE_SIZE;

    uint8 i;
    for (i = 0; i < move_filter->buffer_size; i++)
    {
        move_filter->data_buffer[i] = 0;
    }
}

void move_filter_double_init(move_filter_double_struct *move_filter)
{
    move_filter->data_average = 0;
    move_filter->data_sum = 0;
    move_filter->index = 0;
    // ���û�������С
    move_filter->buffer_size = MOVE_AVERAGE_SIZE;

    uint8 i;
    for (i = 0; i < move_filter->buffer_size; i++)
    {
        move_filter->data_buffer[i] = 0;
    }
}
