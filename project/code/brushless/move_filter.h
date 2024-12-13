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
#ifndef _MOVE_FILTER_H
#define _MOVE_FILTER_H

#include "zf_common_typedef.h"

#define MOVE_AVERAGE_SIZE 200 // ���建������С

typedef struct
{
    uint8 index;                          // �±�
    uint8 buffer_size;                    // buffer��С
    int32 data_buffer[MOVE_AVERAGE_SIZE]; // ������
    int32 data_sum;                       // ���ݺ�
    int32 data_average;                   // ����ƽ��ֵ
} move_filter_struct;


void move_filter_init(move_filter_struct *move_average);
void move_filter_calc(move_filter_struct *move_average, int32 new_data);

typedef struct
{
    uint8 index;                           // �±�
    uint8 buffer_size;                     // buffer��С
    double data_buffer[MOVE_AVERAGE_SIZE]; // ������
    double data_sum;                       // ���ݺ�
    double data_average;                   // ����ƽ��ֵ
} move_filter_double_struct;

extern move_filter_double_struct current_a_filter;
extern move_filter_double_struct current_b_filter;
extern move_filter_double_struct current_c_filter;
extern move_filter_double_struct iq_ref_filter;
extern move_filter_double_struct id_ref_filter;
extern move_filter_double_struct speed_filter;

void move_filter_double_init(move_filter_double_struct *move_average);
void move_filter_double_calc(move_filter_double_struct *move_average, double new_data);


#endif 
