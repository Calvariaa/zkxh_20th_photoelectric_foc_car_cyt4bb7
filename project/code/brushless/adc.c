#include "brushless/adc.h"

#include "sysclk/cy_sysclk.h"
#include "tcpwm/cy_tcpwm_pwm.h"
#include "gpio/cy_gpio.h"
#include "sysint/cy_sysint.h"
#include "trigmux/cy_trigmux.h"
#include "adc/cy_adc.h"
#include "debug/vofaplus.h"
#include "zf_driver_adc.h"
#include "zf_driver_gpio.h"

int16_t adc_abmf_value = 0;
int16_t adc_bbmf_value = 0;
int16_t adc_cbmf_value = 0;
int16_t adc_sum_value = 0;
int16_t adc_mid_value = 0;
int16_t adc_global_value_last = 0;
int16_t adc_global_value = 0;

void motor_bldc_adc_init()
{
    adc_init(ADC_ABMF, ADC_12BIT);
    adc_init(ADC_BBMF, ADC_12BIT);
    adc_init(ADC_CBMF, ADC_12BIT);
    adc_init(ADC_MID, ADC_12BIT);
}

void tcpwm_irq_left()
{
}

void tcpwm_irq_right()
{
}

void tcpwm_irq_middle()
{
    adc_abmf_value = adc_mean_filter_convert(ADC_ABMF, 1);
    adc_bbmf_value = adc_mean_filter_convert(ADC_BBMF, 1);
    adc_cbmf_value = adc_mean_filter_convert(ADC_CBMF, 1);
    // adc_mid_value = adc_mean_filter_convert(ADC_MID, 1) / 2;
}

void motor_pwm_interrupt_init()
{

    // Trigger Multiplexer Setting (pass.tr_sar_ch_in[4])
    // Cy_TrigMux_Connect1To1(TRIG_IN_1TO1_7_TCPWM_TO_PASS_CH_TR28, 0ul, TRIGGER_TYPE_PASS_TR_SAR_CH_IN__EDGE, 0ul);
    // Cy_TrigMux_SwTrigger(TRIG_IN_MUX_12_TCPWM_16_TR_OUT124, TRIGGER_TYPE_EDGE, 1ul);

    // Cy_SysClk_PeriphAssignDivider(PCLK_TCPWM0_CLOCKS24, CY_SYSCLK_DIV_16_BIT, 2);
    // Cy_SysClk_PeriphSetDivider(Cy_SysClk_GetClockGroup(PCLK_TCPWM0_CLOCKS24), CY_SYSCLK_DIV_16_BIT, 2, 0); // 80Mhz时钟被10分频为8Mhz
    // Cy_SysClk_PeriphEnableDivider(Cy_SysClk_GetClockGroup(PCLK_TCPWM0_CLOCKS24), CY_SYSCLK_DIV_16_BIT, 2);

    cy_stc_sysint_irq_t tcpwm_irq_middle_cfg_mid;
    tcpwm_irq_middle_cfg_mid = (cy_stc_sysint_irq_t){
        .sysIntSrc = tcpwm_0_interrupts_24_IRQn,
        .intIdx = CPUIntIdx4_IRQn,
        .isEnabled = true,
    };

    interrupt_init(&tcpwm_irq_middle_cfg_mid, tcpwm_irq_middle, 7);
    // Cy_SysInt_InitIRQ(&tcpwm_irq_middle_cfg_24);
    // // Cy_SysInt_SetSystemIrqVector(tcpwm_0_interrupts_15_IRQn, tcpwm_irq_left);
    // // Cy_SysInt_SetSystemIrqVector(tcpwm_0_interrupts_44_IRQn, tcpwm_irq_right);
    // Cy_SysInt_SetSystemIrqVector(tcpwm_0_interrupts_24_IRQn, tcpwm_irq_middle);
    // NVIC_SetPriority(CPUIntIdx4_IRQn, 6ul);
    // NVIC_EnableIRQ(CPUIntIdx4_IRQn);

    // cy_stc_tcpwm_counter_config_t pit_config = {0};

    // pit_config.period = (PWM_PRIOD_LOAD - 1); // pit周期计算：时钟为8M 则计数8为1us
    // pit_config.clockPrescaler = CY_TCPWM_PRESCALER_DIVBY_1;
    // pit_config.runMode = CY_TCPWM_COUNTER_CONTINUOUS;
    // pit_config.countDirection = CY_TCPWM_COUNTER_COUNT_UP;
    // pit_config.compareOrCapture = CY_TCPWM_COUNTER_MODE_COMPARE;
    // pit_config.countInputMode = CY_TCPWM_INPUT_LEVEL;
    // pit_config.countInput = 1uL;
    // pit_config.trigger0EventCfg = CY_TCPWM_COUNTER_OVERFLOW;
    // pit_config.trigger1EventCfg = CY_TCPWM_COUNTER_OVERFLOW;
    // pit_config.reloadInputMode = CY_TCPWM_INPUT_LEVEL; /* NO_EDGE_DET: No edge detection, use trigger as is */
    // pit_config.reloadInput = 9ul;                      /* Select the TCPWM_ALL_CNT_TR_IN[4] */

    // Cy_Tcpwm_Counter_Init((volatile stc_TCPWM_GRP_CNT_t *)&TCPWM0->GRP[0].CNT[10], &pit_config);
    // Cy_Tcpwm_TriggerCapture0((volatile stc_TCPWM_GRP_CNT_t *)&TCPWM0->GRP[0].CNT[10]);
    // Cy_Tcpwm_TriggerStart((volatile stc_TCPWM_GRP_CNT_t *)&TCPWM0->GRP[0].CNT[10]);
    // Cy_Tcpwm_Counter_SetTC_IntrMask((volatile stc_TCPWM_GRP_CNT_t *)&TCPWM0->GRP[0].CNT[10]);
}