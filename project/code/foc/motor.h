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
 * �ļ�����          motor
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

#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "zf_common_typedef.h"
#include "system_cyt4bb.h"

// #define FPWM (uint16)(20000)          // PWMƵ��
#define PWM_PRIOD_LOAD (uint16)(4000) // PWM����װ��ֵ
// #define PWM_PRIOD_LOAD (uint16)(SYSTEM_CLOCK_250M / FPWM / 2) // PWM����װ��ֵ
#define DEADTIME_LOAD (10)                      // ����װ��ֵ

// foc left
#define L_TRIG_OUT_MUX TRIG_OUT_MUX_5_TCPWM_ALL_CNT_TR_IN2

#define L_A_PHASE_PIN_H (P09_0)
#define L_A_PHASE_HSIOM_H (P9_0_TCPWM0_LINE24)
#define L_A_PHASE_PIN_L (P09_1)
#define L_A_PHASE_HSIOM_L (P9_1_TCPWM0_LINE_COMPL24)
#define L_A_PHASE_CLK_DST (PCLK_TCPWM0_CLOCKS24)
#define L_A_PHASE_GRP_CNT (TCPWM0_GRP0_CNT24)

#define L_B_PHASE_PIN_H (P05_0)
#define L_B_PHASE_HSIOM_H (P5_0_TCPWM0_LINE9)
#define L_B_PHASE_PIN_L (P05_1)
#define L_B_PHASE_HSIOM_L (P5_1_TCPWM0_LINE_COMPL9)
#define L_B_PHASE_CLK_DST (PCLK_TCPWM0_CLOCKS9)
#define L_B_PHASE_GRP_CNT (TCPWM0_GRP0_CNT9)

#define L_C_PHASE_PIN_H (P05_2)
#define L_C_PHASE_HSIOM_H (P5_2_TCPWM0_LINE11)
#define L_C_PHASE_PIN_L (P05_3)
#define L_C_PHASE_HSIOM_L (P5_3_TCPWM0_LINE_COMPL11)
#define L_C_PHASE_CLK_DST (PCLK_TCPWM0_CLOCKS11)
#define L_C_PHASE_GRP_CNT (TCPWM0_GRP0_CNT11)

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

//brushless
#define M_TRIG_OUT_MUX TRIG_OUT_MUX_5_TCPWM_ALL_CNT_TR_IN4

#define M_A_PHASE_PIN_H (P07_1)
#define M_A_PHASE_HSIOM_H (P7_1_TCPWM0_LINE15)
#define M_A_PHASE_PIN_L (P07_2)
#define M_A_PHASE_HSIOM_L (P7_2_TCPWM0_LINE_COMPL15)
#define M_A_PHASE_CLK_DST (PCLK_TCPWM0_CLOCKS15)
#define M_A_PHASE_GRP_CNT (TCPWM0_GRP0_CNT15)

#define M_B_PHASE_PIN_H (P07_3)
#define M_B_PHASE_HSIOM_H (P7_3_TCPWM0_LINE16)
#define M_B_PHASE_PIN_L (P07_4)
#define M_B_PHASE_HSIOM_L (P7_4_TCPWM0_LINE_COMPL16)
#define M_B_PHASE_CLK_DST (PCLK_TCPWM0_CLOCKS16)
#define M_B_PHASE_GRP_CNT (TCPWM0_GRP0_CNT16)

#define M_C_PHASE_PIN_H (P07_5)
#define M_C_PHASE_HSIOM_H (P7_5_TCPWM0_LINE17)
#define M_C_PHASE_PIN_L (P07_6)
#define M_C_PHASE_HSIOM_L (P7_6_TCPWM0_LINE_COMPL17)
#define M_C_PHASE_CLK_DST (PCLK_TCPWM0_CLOCKS17)
#define M_C_PHASE_GRP_CNT (TCPWM0_GRP0_CNT17)


void motor_parameter_init(void);
void mos_all_open_left(uint16 periodAH, uint16 periodBH, uint16 periodCH);
void mos_all_open_right(uint16 periodAH, uint16 periodBH, uint16 periodCH);
void mos_all_open_middle(uint16 periodAH, uint16 periodBH, uint16 periodCH);

#endif