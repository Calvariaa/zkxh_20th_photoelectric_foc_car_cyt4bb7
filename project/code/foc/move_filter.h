/*********************************************************************************************************************
* CYT4BB Opensourec Library 即（ CYT4BB 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 CYT4BB 开源库的一部分
*
* CYT4BB 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          move_filter
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          IAR 9.40.1
* 适用平台          CYT4BB
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2024-1-18       pudding            first version
********************************************************************************************************************/
#ifndef _MOVE_FILTER_H
#define _MOVE_FILTER_H

#include "zf_common_typedef.h"

#define MOVE_AVERAGE_SIZE 200 // 定义缓冲区大小

typedef struct
{
    uint8 index;                          // 下标
    uint8 buffer_size;                    // buffer大小
    int32 data_buffer[MOVE_AVERAGE_SIZE]; // 缓冲区
    int32 data_sum;                       // 数据和
    int32 data_average;                   // 数据平均值
} move_filter_struct;


void move_filter_init(move_filter_struct *move_average);
void move_filter_calc(move_filter_struct *move_average, int32 new_data);

typedef struct
{
    uint8 index;                           // 下标
    uint8 buffer_size;                     // buffer大小
    double data_buffer[MOVE_AVERAGE_SIZE]; // 缓冲区
    double data_sum;                       // 数据和
    double data_average;                   // 数据平均值
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
