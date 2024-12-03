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
 * 文件名称          motor
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

#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "zf_common_typedef.h"

#define FPWM (uint16)(20000)          // PWM频率
#define PWM_PRIOD_LOAD (uint16)(4000) // PWM周期装载值
#define DEADTIME_LOAD (10)                      // 死区装载值

#define L_TRIG_OUT_MUX TRIG_OUT_MUX_5_TCPWM_ALL_CNT_TR_IN2

#define L_A_PHASE_PIN_H (P09_0)
#define L_A_PHASE_HSIOM_H (P9_0_TCPWM0_LINE24)
#define L_A_PHASE_PIN_L (P09_1)
#define L_A_PHASE_HSIOM_L (P9_1_TCPWM0_LINE_COMPL24)
#define L_A_PHASE_CLK_DST (PCLK_TCPWM0_CLOCKS24)
#define L_A_PHASE_GRP_CNT (TCPWM0_GRP0_CNT24)
#define L_A_ADDRESS_CNT (0x40580c00 + 0x00000008)

#define L_B_PHASE_PIN_H (P10_0)
#define L_B_PHASE_HSIOM_H (P10_0_TCPWM0_LINE28)
#define L_B_PHASE_PIN_L (P10_1)
#define L_B_PHASE_HSIOM_L (P10_1_TCPWM0_LINE_COMPL28)
#define L_B_PHASE_CLK_DST (PCLK_TCPWM0_CLOCKS28)
#define L_B_PHASE_GRP_CNT (TCPWM0_GRP0_CNT28)
#define L_B_ADDRESS_CNT (0x40580e00 + 0x00000008)

#define L_C_PHASE_PIN_H (P10_2)
#define L_C_PHASE_HSIOM_H (P10_2_TCPWM0_LINE30)
#define L_C_PHASE_PIN_L (P10_3)
#define L_C_PHASE_HSIOM_L (P10_3_TCPWM0_LINE_COMPL30)
#define L_C_PHASE_CLK_DST (PCLK_TCPWM0_CLOCKS30)
#define L_C_PHASE_GRP_CNT (TCPWM0_GRP0_CNT30)
#define L_C_ADDRESS_CNT (0x40580f00 + 0x00000008)



#define R_TRIG_OUT_MUX TRIG_OUT_MUX_5_TCPWM_ALL_CNT_TR_IN3

#define R_A_PHASE_PIN_H (P12_0)
#define R_A_PHASE_HSIOM_H (P12_0_TCPWM0_LINE36)
#define R_A_PHASE_PIN_L (P12_1)
#define R_A_PHASE_HSIOM_L (P12_1_TCPWM0_LINE_COMPL36)
#define R_A_PHASE_CLK_DST (PCLK_TCPWM0_CLOCKS36)
#define R_A_PHASE_GRP_CNT (TCPWM0_GRP0_CNT36)
#define R_A_ADDRESS_CNT (0x40581200 + 0x00000008)

#define R_B_PHASE_PIN_H (P12_2)
#define R_B_PHASE_HSIOM_H (P12_2_TCPWM0_LINE38)
#define R_B_PHASE_PIN_L (P12_3)
#define R_B_PHASE_HSIOM_L (P12_3_TCPWM0_LINE_COMPL38)
#define R_B_PHASE_CLK_DST (PCLK_TCPWM0_CLOCKS38)
#define R_B_PHASE_GRP_CNT (TCPWM0_GRP0_CNT38)
#define R_B_ADDRESS_CNT (0x40581300 + 0x00000008)

#define R_C_PHASE_PIN_H (P12_4)
#define R_C_PHASE_HSIOM_H (P12_4_TCPWM0_LINE40)
#define R_C_PHASE_PIN_L (P12_5)
#define R_C_PHASE_HSIOM_L (P12_5_TCPWM0_LINE_COMPL40)
#define R_C_PHASE_CLK_DST (PCLK_TCPWM0_CLOCKS40)
#define R_C_PHASE_GRP_CNT (TCPWM0_GRP0_CNT40)
#define R_C_ADDRESS_CNT (0x40581400 + 0x00000008)


void motor_parameter_init(void);
void mos_all_open_left(uint16 periodAH, uint16 periodBH, uint16 periodCH);

#endif