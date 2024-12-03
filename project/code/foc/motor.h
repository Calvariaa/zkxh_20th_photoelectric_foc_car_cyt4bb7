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

#define FPWM (uint16)(20000)          // PWMƵ��
#define PWM_PRIOD_LOAD (uint16)(4000) // PWM����װ��ֵ
#define DEADTIME_LOAD (10)                      // ����װ��ֵ

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