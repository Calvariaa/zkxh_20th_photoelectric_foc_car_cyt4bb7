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
 * �ļ�����          isr_arm_7_0
 * ��˾����          �ɶ���ɿƼ����޹�˾
 * �汾��Ϣ          �鿴 libraries/doc �ļ����� version �ļ� �汾˵��
 * ��������          IAR 9.40.1
 * ����ƽ̨          CYT4BB
 * ��������          https://seekfree.taobao.com/
 *
 * �޸ļ�¼
 * ����              ����                ��ע
 * 2024-1-9      pudding            first version
 ********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "brushless/foc.h"
#include "brushless/bldc.h"
#include "brushless/encoder/encoder.h"
#include "debug/vofaplus.h"
#include "brushless/move_filter.h"
#include "brushless/buzzer.h"
#include "brushless/motor.h"
#include "brushless/pid.h"
// **************************** PIT�жϺ��� ****************************

extern FOC_Parm_Typedef foc_left;
extern FOC_Parm_Typedef foc_right;

extern encoder_t encoder_left;
extern encoder_t encoder_right;

uint64 timer_1ms = 0u;
#define START_DELAY_FLAG (timer_1ms < 100)

void L_tcpwm_irq()
{
    Cy_Tcpwm_Counter_ClearTC_Intr(L_COUNTER_PHASE_GRP_CNT);

    if (START_DELAY_FLAG)
        return;

    foc_commutation(&foc_left, &encoder_left, &foc_left_pid, mos_all_open_left);
}

void R_tcpwm_irq()
{
    Cy_Tcpwm_Counter_ClearTC_Intr(R_COUNTER_PHASE_GRP_CNT);

    if (START_DELAY_FLAG)
        return;

    foc_commutation(&foc_right, &encoder_right, &foc_right_pid, mos_all_open_right);
}

void M_tcpwm_irq()
{
    Cy_Tcpwm_Counter_ClearTC_Intr(M_COUNTER_PHASE_GRP_CNT);

    if (START_DELAY_FLAG)
        return;

    bldc_commutation();
}

void pit0_ch0_isr()
{
    pit_isr_flag_clear(PIT_CH0);
    timer_1ms++;

    if (!START_DELAY_FLAG)
        buzz_exec();
    if (timer_1ms == 50)
    {
        set_zero_angle(get_magnet_angle(encoder_left.__get_magnet_val_(), encoder_left.zero_angle), &encoder_left);
        set_zero_angle(get_magnet_angle(encoder_right.__get_magnet_val_(), encoder_right.zero_angle), &encoder_right);
    }

    // if (timer_1ms % 5 == 0)
    //     motor_speed_out();

    // if (fabsf(Park_in.u_q) >= FOC_UQ_MAX)
    // {
    //     if (ierror_count < 2000)
    //         ierror_count++;
    // }
}

void pit0_ch1_isr()
{
    pit_isr_flag_clear(PIT_CH1);
}

void pit0_ch2_isr()
{
    pit_isr_flag_clear(PIT_CH2);
}

void pit0_ch3_isr()
{
    pit_isr_flag_clear(PIT_CH3);
}

void pit0_ch4_isr()
{
    pit_isr_flag_clear(PIT_CH4);
}

void pit0_ch5_isr()
{
    pit_isr_flag_clear(PIT_CH5);
}

void pit0_ch6_isr()
{
    pit_isr_flag_clear(PIT_CH6);
}

void pit0_ch7_isr()
{
    pit_isr_flag_clear(PIT_CH7);
}
// **************************** PIT�жϺ��� ****************************

// **************************** �ⲿ�жϺ��� ****************************
void gpio_0_exti_isr()
{
}

void gpio_1_exti_isr()
{
    if (exti_flag_get(P01_0)) // ʾ��P1_0�˿��ⲿ�ж��ж�
    {
    }
    if (exti_flag_get(P01_1))
    {
    }
}

void gpio_2_exti_isr()
{
    if (exti_flag_get(P02_0))
    {
    }
    if (exti_flag_get(P02_4))
    {
    }
}

void gpio_3_exti_isr()
{
}

void gpio_4_exti_isr()
{
}

void gpio_5_exti_isr()
{
}

void gpio_6_exti_isr()
{
}

void gpio_7_exti_isr()
{
}

void gpio_8_exti_isr()
{
}

void gpio_9_exti_isr()
{
}

void gpio_10_exti_isr()
{
}

void gpio_11_exti_isr()
{
}

void gpio_12_exti_isr()
{
}

void gpio_13_exti_isr()
{
}

void gpio_14_exti_isr()
{
}

void gpio_15_exti_isr()
{
}

void gpio_16_exti_isr()
{
}

void gpio_17_exti_isr()
{
}

void gpio_18_exti_isr()
{
}

void gpio_19_exti_isr()
{
}

void gpio_20_exti_isr()
{
}

void gpio_21_exti_isr()
{
}

void gpio_22_exti_isr()
{
}

void gpio_23_exti_isr()
{
}
// **************************** �ⲿ�жϺ��� ****************************

//// **************************** DMA�жϺ��� ****************************
// void dma_event_callback(void* callback_arg, cyhal_dma_event_t event)
//{
//     CY_UNUSED_PARAMETER(event);
//
//
//
//
// }
//  **************************** DMA�жϺ��� ****************************

// **************************** �����жϺ��� ****************************
// ����0Ĭ����Ϊ���Դ���
void uart0_isr(void)
{
    if (Cy_SCB_GetRxInterruptMask(get_scb_module(UART_0)) & CY_SCB_UART_RX_NOT_EMPTY) // ����0�����ж�
    {
        Cy_SCB_ClearRxInterrupt(get_scb_module(UART_0), CY_SCB_UART_RX_NOT_EMPTY); // ��������жϱ�־λ

#if DEBUG_UART_USE_INTERRUPT       // ������� debug �����ж�
        debug_interrupr_handler(); // ���� debug ���ڽ��մ����� ���ݻᱻ debug ���λ�������ȡ
#endif                             // ����޸��� DEBUG_UART_INDEX ����δ�����Ҫ�ŵ���Ӧ�Ĵ����ж�ȥ
    }
    else if (Cy_SCB_GetTxInterruptMask(get_scb_module(UART_0)) & CY_SCB_UART_TX_DONE) // ����0�����ж�
    {
        Cy_SCB_ClearTxInterrupt(get_scb_module(UART_0), CY_SCB_UART_TX_DONE); // ��������жϱ�־λ
    }
}

void uart1_isr(void)
{
    if (Cy_SCB_GetRxInterruptMask(get_scb_module(UART_1)) & CY_SCB_UART_RX_NOT_EMPTY) // ����1�����ж�
    {
        Cy_SCB_ClearRxInterrupt(get_scb_module(UART_1), CY_SCB_UART_RX_NOT_EMPTY); // ��������жϱ�־λ

        wireless_module_uart_handler();
    }
    else if (Cy_SCB_GetTxInterruptMask(get_scb_module(UART_1)) & CY_SCB_UART_TX_DONE) // ����1�����ж�
    {
        Cy_SCB_ClearTxInterrupt(get_scb_module(UART_1), CY_SCB_UART_TX_DONE); // ��������жϱ�־λ
    }
}

void uart2_isr(void)
{
    if (Cy_SCB_GetRxInterruptMask(get_scb_module(UART_2)) & CY_SCB_UART_RX_NOT_EMPTY) // ����2�����ж�
    {
        Cy_SCB_ClearRxInterrupt(get_scb_module(UART_2), CY_SCB_UART_RX_NOT_EMPTY); // ��������жϱ�־λ

        gnss_uart_callback();
    }
    else if (Cy_SCB_GetTxInterruptMask(get_scb_module(UART_2)) & CY_SCB_UART_TX_DONE) // ����2�����ж�
    {
        Cy_SCB_ClearTxInterrupt(get_scb_module(UART_2), CY_SCB_UART_TX_DONE); // ��������жϱ�־λ
    }
}

void uart3_isr(void)
{
    if (Cy_SCB_GetRxInterruptMask(get_scb_module(UART_3)) & CY_SCB_UART_RX_NOT_EMPTY) // ����3�����ж�
    {
        Cy_SCB_ClearRxInterrupt(get_scb_module(UART_3), CY_SCB_UART_RX_NOT_EMPTY); // ��������жϱ�־λ
    }
    else if (Cy_SCB_GetTxInterruptMask(get_scb_module(UART_3)) & CY_SCB_UART_TX_DONE) // ����3�����ж�
    {
        Cy_SCB_ClearTxInterrupt(get_scb_module(UART_3), CY_SCB_UART_TX_DONE); // ��������жϱ�־λ
    }
}

void uart4_isr(void)
{
    if (Cy_SCB_GetRxInterruptMask(get_scb_module(UART_4)) & CY_SCB_UART_RX_NOT_EMPTY) // ����4�����ж�
    {
        Cy_SCB_ClearRxInterrupt(get_scb_module(UART_4), CY_SCB_UART_RX_NOT_EMPTY); // ��������жϱ�־λ

        uart_receiver_handler(); // ���ڽ��ջ��ص�����
    }
    else if (Cy_SCB_GetTxInterruptMask(get_scb_module(UART_4)) & CY_SCB_UART_TX_DONE) // ����4�����ж�
    {
        Cy_SCB_ClearTxInterrupt(get_scb_module(UART_4), CY_SCB_UART_TX_DONE); // ��������жϱ�־λ
    }
}
// **************************** �����жϺ��� ****************************