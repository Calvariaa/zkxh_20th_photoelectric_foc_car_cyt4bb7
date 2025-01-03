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
#include "system_cyt4bb.h"

// #define FPWM (uint16_t)(20000)          // PWM频率
#define PWM_PRIOD_LOAD (uint16_t)(3000) // PWM周期装载值
// #define PWM_PRIOD_LOAD (uint16_t)(80000000UL / FPWM / 2) // PWM周期装载值
#define DEADTIME_LOAD (10) // 死区装载值

#define TCPWM_COUNT_USE_ISR CPUIntIdx4_IRQn
// foc left
#define L_TRIG_OUT_MUX TRIG_OUT_MUX_5_TCPWM_ALL_CNT_TR_IN2

#define L_A_PHASE_PIN_H (P07_1)
#define L_A_PHASE_HSIOM_H (P7_1_TCPWM0_LINE15)
#define L_A_PHASE_PIN_L (P07_2)
#define L_A_PHASE_HSIOM_L (P7_2_TCPWM0_LINE_COMPL15)
#define L_A_PHASE_CLK_DST (PCLK_TCPWM0_CLOCKS15)
#define L_A_PHASE_GRP_CNT (TCPWM0_GRP0_CNT15)

#define L_B_PHASE_PIN_H (P07_3)
#define L_B_PHASE_HSIOM_H (P7_3_TCPWM0_LINE16)
#define L_B_PHASE_PIN_L (P07_4)
#define L_B_PHASE_HSIOM_L (P7_4_TCPWM0_LINE_COMPL16)
#define L_B_PHASE_CLK_DST (PCLK_TCPWM0_CLOCKS16)
#define L_B_PHASE_GRP_CNT (TCPWM0_GRP0_CNT16)

#define L_C_PHASE_PIN_H (P07_5)
#define L_C_PHASE_HSIOM_H (P7_5_TCPWM0_LINE17)
#define L_C_PHASE_PIN_L (P07_6)
#define L_C_PHASE_HSIOM_L (P7_6_TCPWM0_LINE_COMPL17)
#define L_C_PHASE_CLK_DST (PCLK_TCPWM0_CLOCKS17)
#define L_C_PHASE_GRP_CNT (TCPWM0_GRP0_CNT17)

#define L_COUNTER_IRQn (tcpwm_0_interrupts_18_IRQn)
#define L_COUNTER_PHASE_CLK_DST (PCLK_TCPWM0_CLOCKS18)
#define L_COUNTER_PHASE_GRP_CNT (TCPWM0_GRP0_CNT18)

// foc right
#define R_TRIG_OUT_MUX TRIG_OUT_MUX_5_TCPWM_ALL_CNT_TR_IN3

#define R_A_PHASE_PIN_H (P13_1)
#define R_A_PHASE_HSIOM_H (P13_1_TCPWM0_LINE44)
#define R_A_PHASE_PIN_L (P13_2)
#define R_A_PHASE_HSIOM_L (P13_2_TCPWM0_LINE_COMPL44)
#define R_A_PHASE_CLK_DST (PCLK_TCPWM0_CLOCKS44)
#define R_A_PHASE_GRP_CNT (TCPWM0_GRP0_CNT44)

#define R_B_PHASE_PIN_H (P13_3)
#define R_B_PHASE_HSIOM_H (P13_3_TCPWM0_LINE45)
#define R_B_PHASE_PIN_L (P13_4)
#define R_B_PHASE_HSIOM_L (P13_4_TCPWM0_LINE_COMPL45)
#define R_B_PHASE_CLK_DST (PCLK_TCPWM0_CLOCKS45)
#define R_B_PHASE_GRP_CNT (TCPWM0_GRP0_CNT45)

#define R_C_PHASE_PIN_H (P13_5)
#define R_C_PHASE_HSIOM_H (P13_5_TCPWM0_LINE46)
#define R_C_PHASE_PIN_L (P13_6)
#define R_C_PHASE_HSIOM_L (P13_6_TCPWM0_LINE_COMPL46)
#define R_C_PHASE_CLK_DST (PCLK_TCPWM0_CLOCKS46)
#define R_C_PHASE_GRP_CNT (TCPWM0_GRP0_CNT46)

#define R_COUNTER_IRQn (tcpwm_0_interrupts_47_IRQn)
#define R_COUNTER_PHASE_CLK_DST (PCLK_TCPWM0_CLOCKS47)
#define R_COUNTER_PHASE_GRP_CNT (TCPWM0_GRP0_CNT47)

// brushless
#define M_TRIG_OUT_MUX TRIG_OUT_MUX_5_TCPWM_ALL_CNT_TR_IN4

#define M_A_PHASE_PIN_H (P09_0)
#define M_A_PHASE_HSIOM_H (P9_0_TCPWM0_LINE24)
#define M_A_PHASE_PIN_L (P09_1)
#define M_A_PHASE_HSIOM_L (P9_1_TCPWM0_LINE_COMPL24)
#define M_A_PHASE_CLK_DST (PCLK_TCPWM0_CLOCKS24)
#define M_A_PHASE_GRP_CNT (TCPWM0_GRP0_CNT24)

#define M_B_PHASE_PIN_H (P05_0)
#define M_B_PHASE_HSIOM_H (P5_0_TCPWM0_LINE9)
#define M_B_PHASE_PIN_L (P05_1)
#define M_B_PHASE_HSIOM_L (P5_1_TCPWM0_LINE_COMPL9)
#define M_B_PHASE_CLK_DST (PCLK_TCPWM0_CLOCKS9)
#define M_B_PHASE_GRP_CNT (TCPWM0_GRP0_CNT9)

#define M_C_PHASE_PIN_H (P05_2)
#define M_C_PHASE_HSIOM_H (P5_2_TCPWM0_LINE11)
#define M_C_PHASE_PIN_L (P05_3)
#define M_C_PHASE_HSIOM_L (P5_3_TCPWM0_LINE_COMPL11)
#define M_C_PHASE_CLK_DST (PCLK_TCPWM0_CLOCKS11)
#define M_C_PHASE_GRP_CNT (TCPWM0_GRP0_CNT11)

#define M_COUNTER_IRQn (tcpwm_0_interrupts_10_IRQn)
#define M_COUNTER_PHASE_CLK_DST (PCLK_TCPWM0_CLOCKS10)
#define M_COUNTER_PHASE_GRP_CNT (TCPWM0_GRP0_CNT10)

#define M2_COUNTER_IRQn (tcpwm_0_interrupts_12_IRQn)
#define M2_COUNTER_PHASE_CLK_DST (PCLK_TCPWM0_CLOCKS12)
#define M2_COUNTER_PHASE_GRP_CNT (TCPWM0_GRP0_CNT12)

void motor_parameter_init(void);
void mos_all_open_left(uint16_t periodAH, uint16_t periodBH, uint16_t periodCH);
void mos_all_open_right(uint16_t periodAH, uint16_t periodBH, uint16_t periodCH);
void mos_all_open_middle(uint16_t periodAH, uint16_t periodBH, uint16_t periodCH);

void mos_a_bn_open_middle(uint16_t duty);
void mos_a_cn_open_middle(uint16_t duty);
void mos_b_an_open_middle(uint16_t duty);
void mos_b_cn_open_middle(uint16_t duty);
void mos_c_an_open_middle(uint16_t duty);
void mos_c_bn_open_middle(uint16_t duty);

void mos_close_left(void);
void mos_close_right(void);
void mos_close_middle(void);

void L_tcpwm_irq();
void R_tcpwm_irq();
void M_tcpwm_irq();
void M2_tcpwm_irq();
#endif