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

#include "sysclk/cy_sysclk.h"
#include "tcpwm/cy_tcpwm_pwm.h"
#include "gpio/cy_gpio.h"
#include "sysint/cy_sysint.h"
#include "trigmux/cy_trigmux.h"
#include "zf_common_debug.h"
#include "zf_common_function.h"
#include "zf_driver_gpio.h"
#include "zf_driver_delay.h"
#include "zf_driver_pwm.h"
#include "move_filter.h"
#include "foc/motor.h"
#include "debug/vofaplus.h"

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     电机PWM通道初始化
// 参数说明     void
// 返回参数     void
// 使用示例     motor_pwm_output_init();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------

/* Configuration for U/V/W-phase Timer */
cy_stc_tcpwm_pwm_config_t tcpwm_config_left = // Configure the PWM_DT parameters
    {
        .pwmMode = CY_TCPWM_PWM_MODE_DEADTIME,
        .clockPrescaler = CY_TCPWM_PRESCALER_DIVBY_1,
        .debug_pause = false,
        .countDirection = CY_TCPWM_COUNTER_COUNT_UP,
        /* Set UPDN2 modes */ // _DOWN2
        .cc0MatchMode = CY_TCPWM_PWM_TR_CTRL2_SET,
        .overflowMode = CY_TCPWM_PWM_TR_CTRL2_SET,
        .underflowMode = CY_TCPWM_PWM_TR_CTRL2_NO_CHANGE,
        .cc1MatchMode = CY_TCPWM_PWM_TR_CTRL2_CLEAR,
        .deadTime = DEADTIME_LOAD,     /* Right side dead time */
        .deadTimeComp = DEADTIME_LOAD, /* Left  side dead time */
        .runMode = CY_TCPWM_PWM_CONTINUOUS,
        .period = (PWM_PRIOD_LOAD - 1),
        .period_buff = 0ul,                     /* 2000ul */
        .enablePeriodSwap = false,              /* Auto Reload Period = OFF */
        .enableCompare0Swap = true,             /* Auto Reload CC0 = ON */
        .enableCompare1Swap = true,             /* Auto Reload CC1 = ON */
        .interruptSources = CY_TCPWM_INT_ON_CC, /* Interrupt Mask for TC, CC0/CC1_MATCH (0:OFF, 1:TC, 2:CC0 MATCH, 4:CC1 MATCH, 7:all) */
        .invertPWMOut = 0ul,
        .invertPWMOutN = 0ul,
        .killMode = CY_TCPWM_PWM_STOP_ON_KILL,
        .switchInputMode = CY_TCPWM_INPUT_LEVEL, /* NO_EDGE_DET: No edge detection, use trigger as is */
        .switchInput = 0ul,                      /* Select the constant 0 */
        .reloadInputMode = CY_TCPWM_INPUT_LEVEL, /* NO_EDGE_DET: No edge detection, use trigger as is */
        .reloadInput = 7ul,                      /* Select the TCPWM_ALL_CNT_TR_IN[2] */
        .startInputMode = CY_TCPWM_INPUT_LEVEL,  /* NO_EDGE_DET: No edge detection, use trigger as is */
        .startInput = 0ul,                       /* Select the constant 0 */
        .kill0InputMode = CY_TCPWM_INPUT_LEVEL,  /* NO_EDGE_DET: No edge detection, use trigger as is */
        .kill0Input = 0ul,                       /* Select the constant 0 */
        .kill1InputMode = CY_TCPWM_INPUT_LEVEL,  /* NO_EDGE_DET: No edge detection, use trigger as is */
        .kill1Input = 0ul,                       /* Select the constant 0 */
        .countInputMode = CY_TCPWM_INPUT_LEVEL,  /* NO_EDGE_DET: No edge detection, use trigger as is */
        .countInput = 1ul,                       /* Select the constant 1 */
};

cy_stc_tcpwm_pwm_config_t tcpwm_config_right = // Configure the PWM_DT parameters
    {
        .pwmMode = CY_TCPWM_PWM_MODE_DEADTIME,
        .clockPrescaler = CY_TCPWM_PRESCALER_DIVBY_1,
        .debug_pause = false,
        .countDirection = CY_TCPWM_COUNTER_COUNT_UP,
        /* Set UPDN2 modes */ // _DOWN2
        .cc0MatchMode = CY_TCPWM_PWM_TR_CTRL2_SET,
        .overflowMode = CY_TCPWM_PWM_TR_CTRL2_SET,
        .underflowMode = CY_TCPWM_PWM_TR_CTRL2_NO_CHANGE,
        .cc1MatchMode = CY_TCPWM_PWM_TR_CTRL2_CLEAR,
        .deadTime = DEADTIME_LOAD,     /* Right side dead time */
        .deadTimeComp = DEADTIME_LOAD, /* Left  side dead time */
        .runMode = CY_TCPWM_PWM_CONTINUOUS,
        .period = (PWM_PRIOD_LOAD - 1),
        .period_buff = 0ul,                     /* 2000ul */
        .enablePeriodSwap = false,              /* Auto Reload Period = OFF */
        .enableCompare0Swap = true,             /* Auto Reload CC0 = ON */
        .enableCompare1Swap = true,             /* Auto Reload CC1 = ON */
        .interruptSources = CY_TCPWM_INT_ON_CC, /* Interrupt Mask for TC, CC0/CC1_MATCH (0:OFF, 1:TC, 2:CC0 MATCH, 4:CC1 MATCH, 7:all) */
        .invertPWMOut = 0ul,
        .invertPWMOutN = 0ul,
        .killMode = CY_TCPWM_PWM_STOP_ON_KILL,
        .switchInputMode = CY_TCPWM_INPUT_LEVEL, /* NO_EDGE_DET: No edge detection, use trigger as is */
        .switchInput = 0ul,                      /* Select the constant 0 */
        .reloadInputMode = CY_TCPWM_INPUT_LEVEL, /* NO_EDGE_DET: No edge detection, use trigger as is */
        .reloadInput = 8ul,                      /* Select the TCPWM_ALL_CNT_TR_IN[3] */
        .startInputMode = CY_TCPWM_INPUT_LEVEL,  /* NO_EDGE_DET: No edge detection, use trigger as is */
        .startInput = 0ul,                       /* Select the constant 0 */
        .kill0InputMode = CY_TCPWM_INPUT_LEVEL,  /* NO_EDGE_DET: No edge detection, use trigger as is */
        .kill0Input = 0ul,                       /* Select the constant 0 */
        .kill1InputMode = CY_TCPWM_INPUT_LEVEL,  /* NO_EDGE_DET: No edge detection, use trigger as is */
        .kill1Input = 0ul,                       /* Select the constant 0 */
        .countInputMode = CY_TCPWM_INPUT_LEVEL,  /* NO_EDGE_DET: No edge detection, use trigger as is */
        .countInput = 1ul,                       /* Select the constant 1 */
};

cy_stc_tcpwm_pwm_config_t tcpwm_config_middle = // Configure the PWM_DT parameters
    {
        .pwmMode = CY_TCPWM_PWM_MODE_DEADTIME,
        .clockPrescaler = CY_TCPWM_PRESCALER_DIVBY_1,
        .debug_pause = false,
        .countDirection = CY_TCPWM_COUNTER_COUNT_UP,
        /* Set UPDN2 modes */ // _DOWN2
        .cc0MatchMode = CY_TCPWM_PWM_TR_CTRL2_SET,
        .overflowMode = CY_TCPWM_PWM_TR_CTRL2_SET,
        .underflowMode = CY_TCPWM_PWM_TR_CTRL2_NO_CHANGE,
        .cc1MatchMode = CY_TCPWM_PWM_TR_CTRL2_CLEAR,
        .deadTime = DEADTIME_LOAD,     /* Right side dead time */
        .deadTimeComp = DEADTIME_LOAD, /* Left  side dead time */
        .runMode = CY_TCPWM_PWM_CONTINUOUS,
        .period = (PWM_PRIOD_LOAD - 1),
        .period_buff = 0ul,                     /* 2000ul */
        .enablePeriodSwap = false,              /* Auto Reload Period = OFF */
        .enableCompare0Swap = true,             /* Auto Reload CC0 = ON */
        .enableCompare1Swap = true,             /* Auto Reload CC1 = ON */
        .interruptSources = CY_TCPWM_INT_ON_CC, /* Interrupt Mask for TC, CC0/CC1_MATCH (0:OFF, 1:TC, 2:CC0 MATCH, 4:CC1 MATCH, 7:all) */
        .invertPWMOut = 0ul,
        .invertPWMOutN = 0ul,
        .killMode = CY_TCPWM_PWM_STOP_ON_KILL,
        .switchInputMode = CY_TCPWM_INPUT_LEVEL, /* NO_EDGE_DET: No edge detection, use trigger as is */
        .switchInput = 0ul,                      /* Select the constant 0 */
        .reloadInputMode = CY_TCPWM_INPUT_LEVEL, /* NO_EDGE_DET: No edge detection, use trigger as is */
        .reloadInput = 9ul,                      /* Select the TCPWM_ALL_CNT_TR_IN[4] */
        .startInputMode = CY_TCPWM_INPUT_LEVEL,  /* NO_EDGE_DET: No edge detection, use trigger as is */
        .startInput = 0ul,                       /* Select the constant 0 */
        .kill0InputMode = CY_TCPWM_INPUT_LEVEL,  /* NO_EDGE_DET: No edge detection, use trigger as is */
        .kill0Input = 0ul,                       /* Select the constant 0 */
        .kill1InputMode = CY_TCPWM_INPUT_LEVEL,  /* NO_EDGE_DET: No edge detection, use trigger as is */
        .kill1Input = 0ul,                       /* Select the constant 0 */
        .countInputMode = CY_TCPWM_INPUT_LEVEL,  /* NO_EDGE_DET: No edge detection, use trigger as is */
        .countInput = 1ul,                       /* Select the constant 1 */
};
void motor_pwm_output_init(gpio_pin_enum __A_PHASE_PIN_H, en_hsiom_sel_t __A_PHASE_HSIOM_H, gpio_pin_enum __A_PHASE_PIN_L, en_hsiom_sel_t __A_PHASE_HSIOM_L, volatile en_clk_dst_t __A_PHASE_CLK_DST, volatile stc_TCPWM_GRP_CNT_t *__A_PHASE_GRP_CNT,
                           gpio_pin_enum __B_PHASE_PIN_H, en_hsiom_sel_t __B_PHASE_HSIOM_H, gpio_pin_enum __B_PHASE_PIN_L, en_hsiom_sel_t __B_PHASE_HSIOM_L, volatile en_clk_dst_t __B_PHASE_CLK_DST, volatile stc_TCPWM_GRP_CNT_t *__B_PHASE_GRP_CNT,
                           gpio_pin_enum __C_PHASE_PIN_H, en_hsiom_sel_t __C_PHASE_HSIOM_H, gpio_pin_enum __C_PHASE_PIN_L, en_hsiom_sel_t __C_PHASE_HSIOM_L, volatile en_clk_dst_t __C_PHASE_CLK_DST, volatile stc_TCPWM_GRP_CNT_t *__C_PHASE_GRP_CNT,
                           cy_stc_tcpwm_pwm_config_t *__cy_stc_tcpwm_pwm_config, uint32_t __trigLine

)
{
    cy_stc_gpio_pin_config_t pwm_pin_config;
    memset(&pwm_pin_config, 0, sizeof(pwm_pin_config));

    pwm_pin_config.driveMode = CY_GPIO_DM_STRONG; // 配置PWM输出引脚模式为强驱动模式
    // 初始化A相上桥引脚
    pwm_pin_config.hsiom = __A_PHASE_HSIOM_H;
    Cy_GPIO_Pin_Init(get_port(__A_PHASE_PIN_H), (__A_PHASE_PIN_H % 8), &pwm_pin_config);
    // 初始化A相下桥引脚
    pwm_pin_config.hsiom = __A_PHASE_HSIOM_L;
    Cy_GPIO_Pin_Init(get_port(__A_PHASE_PIN_L), (__A_PHASE_PIN_L % 8), &pwm_pin_config);
    // 初始化B相上桥引脚
    pwm_pin_config.hsiom = __B_PHASE_HSIOM_H;
    Cy_GPIO_Pin_Init(get_port(__B_PHASE_PIN_H), (__B_PHASE_PIN_H % 8), &pwm_pin_config);
    // 初始化B相下桥引脚
    pwm_pin_config.hsiom = __B_PHASE_HSIOM_L;
    Cy_GPIO_Pin_Init(get_port(__B_PHASE_PIN_L), (__B_PHASE_PIN_L % 8), &pwm_pin_config);
    // 初始化C相上桥引脚
    pwm_pin_config.hsiom = __C_PHASE_HSIOM_H;
    Cy_GPIO_Pin_Init(get_port(__C_PHASE_PIN_H), (__C_PHASE_PIN_H % 8), &pwm_pin_config);
    // 初始化C相下桥引脚
    pwm_pin_config.hsiom = __C_PHASE_HSIOM_L;
    Cy_GPIO_Pin_Init(get_port(__C_PHASE_PIN_L), (__C_PHASE_PIN_L % 8), &pwm_pin_config);

    Cy_SysClk_PeriphAssignDivider(__A_PHASE_CLK_DST, CY_SYSCLK_DIV_16_BIT, 2);
    Cy_SysClk_PeriphSetDivider(Cy_SysClk_GetClockGroup(__A_PHASE_CLK_DST), CY_SYSCLK_DIV_16_BIT, 2, 0);
    Cy_SysClk_PeriphEnableDivider(Cy_SysClk_GetClockGroup(__A_PHASE_CLK_DST), CY_SYSCLK_DIV_16_BIT, 2);

    Cy_SysClk_PeriphAssignDivider(__B_PHASE_CLK_DST, CY_SYSCLK_DIV_16_BIT, 2);
    Cy_SysClk_PeriphSetDivider(Cy_SysClk_GetClockGroup(__B_PHASE_CLK_DST), CY_SYSCLK_DIV_16_BIT, 2, 0);
    Cy_SysClk_PeriphEnableDivider(Cy_SysClk_GetClockGroup(__B_PHASE_CLK_DST), CY_SYSCLK_DIV_16_BIT, 2);

    Cy_SysClk_PeriphAssignDivider(__C_PHASE_CLK_DST, CY_SYSCLK_DIV_16_BIT, 2);
    Cy_SysClk_PeriphSetDivider(Cy_SysClk_GetClockGroup(__C_PHASE_CLK_DST), CY_SYSCLK_DIV_16_BIT, 2, 0);
    Cy_SysClk_PeriphEnableDivider(Cy_SysClk_GetClockGroup(__C_PHASE_CLK_DST), CY_SYSCLK_DIV_16_BIT, 2);

    Cy_Tcpwm_Pwm_Init(__A_PHASE_GRP_CNT, __cy_stc_tcpwm_pwm_config);
    Cy_Tcpwm_Pwm_Enable(__A_PHASE_GRP_CNT);
    Cy_Tcpwm_TriggerStart(__A_PHASE_GRP_CNT);

    Cy_Tcpwm_Pwm_Init(__B_PHASE_GRP_CNT, __cy_stc_tcpwm_pwm_config);
    Cy_Tcpwm_Pwm_Enable(__B_PHASE_GRP_CNT);
    Cy_Tcpwm_TriggerStart(__B_PHASE_GRP_CNT);

    Cy_Tcpwm_Pwm_Init(__C_PHASE_GRP_CNT, __cy_stc_tcpwm_pwm_config);
    Cy_Tcpwm_Pwm_Enable(__C_PHASE_GRP_CNT);
    Cy_Tcpwm_TriggerStart(__C_PHASE_GRP_CNT);

    /* Synchronize all counters */
    Cy_TrigMux_SwTrigger(__trigLine, TRIGGER_TYPE_EDGE, 1ul); /*Output the Reload signal to TCPWM_ALL_CNT_TR_IN[2] */
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     三相通道占空比配置
// 参数说明     a_duty       A相占空比信息 最大值 PWM_PRIOD_LOAD (motor.h头文件有宏定义)
// 参数说明     b_duty       A相占空比信息 最大值 PWM_PRIOD_LOAD (motor.h头文件有宏定义)
// 参数说明     c_duty       A相占空比信息 最大值 PWM_PRIOD_LOAD (motor.h头文件有宏定义)
// 返回参数     void
// 使用示例     motor_duty_set(200, 300, 400);
// 备注信息     更新比较器并且将计数器清零
//-------------------------------------------------------------------------------------------------------------------
void motor_duty_set(uint16 a_duty, volatile stc_TCPWM_GRP_CNT_t *__A_PHASE_GRP_CNT,
                    uint16 b_duty, volatile stc_TCPWM_GRP_CNT_t *__B_PHASE_GRP_CNT,
                    uint16 c_duty, volatile stc_TCPWM_GRP_CNT_t *__C_PHASE_GRP_CNT,
                    uint32_t __trigLine)
{
    __A_PHASE_GRP_CNT->unCC0.u32Register = (PWM_PRIOD_LOAD - a_duty) / 2;
    __A_PHASE_GRP_CNT->unCC1.u32Register = (PWM_PRIOD_LOAD + a_duty) / 2;
    __B_PHASE_GRP_CNT->unCC0.u32Register = (PWM_PRIOD_LOAD - b_duty) / 2;
    __B_PHASE_GRP_CNT->unCC1.u32Register = (PWM_PRIOD_LOAD + b_duty) / 2;
    __C_PHASE_GRP_CNT->unCC0.u32Register = (PWM_PRIOD_LOAD - c_duty) / 2;
    __C_PHASE_GRP_CNT->unCC1.u32Register = (PWM_PRIOD_LOAD + c_duty) / 2;

    // Cy_TrigMux_SwTrigger(__trigLine, TRIGGER_TYPE_EDGE, 1ul); /*Output the Reload signal to TCPWM_ALL_CNT_TR_IN[2] */
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     三相通道使能选择
// 参数说明     a_channel       0：关闭A相输出通道  1：开启A相输出通道
// 参数说明     b_channel       0：关闭B相输出通道  1：开启B相输出通道
// 参数说明     c_channel       0：关闭C相输出通道  1：开启C相输出通道
// 返回参数     void
// 使用示例     motor_channel_set(1,0,1);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void motor_channel_set(uint8 a_channel, volatile stc_TCPWM_GRP_CNT_t *__A_PHASE_GRP_CNT,
                       uint8 b_channel, volatile stc_TCPWM_GRP_CNT_t *__B_PHASE_GRP_CNT,
                       uint8 c_channel, volatile stc_TCPWM_GRP_CNT_t *__C_PHASE_GRP_CNT)
{
    if (a_channel) // 当A相通道使能则开启通道
    {
        __A_PHASE_GRP_CNT->unCTRL.u32Register |= 0x80000000;  // 使能定时器
        __A_PHASE_GRP_CNT->unTR_CMD.u32Register = 0x00000004; // 启动计数
    }
    else // 否则关闭A相通道使能
    {
        __A_PHASE_GRP_CNT->unCTRL.u32Register &= 0x7FFFFFFF; // 关闭A相通道使能
    }
    if (b_channel) // 当B相通道使能则开启通道
    {
        __B_PHASE_GRP_CNT->unCTRL.u32Register |= 0x80000000;  // 使能定时器
        __B_PHASE_GRP_CNT->unTR_CMD.u32Register = 0x00000004; // 启动计数
    }
    else // 否则关闭B相通道使能
    {
        __B_PHASE_GRP_CNT->unCTRL.u32Register &= 0x7FFFFFFF; // 关闭B相通道使能
    }
    if (c_channel) // 当C相通道使能则开启通道
    {
        __C_PHASE_GRP_CNT->unCTRL.u32Register |= 0x80000000;  // 使能定时器
        __C_PHASE_GRP_CNT->unTR_CMD.u32Register = 0x00000004; // 启动计数
    }
    else // 否则关闭C相通道使能
    {
        __C_PHASE_GRP_CNT->unCTRL.u32Register &= 0x7FFFFFFF; // 关闭C相通道使能
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     开启所有MOS并输出占空比
// 参数说明     void
// 返回参数     void
// 使用示例     mos_all_open();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void mos_all_open_left(uint16 periodAH, uint16 periodBH, uint16 periodCH)
{
    motor_channel_set(1, L_A_PHASE_GRP_CNT,
                      1, L_B_PHASE_GRP_CNT,
                      1, L_C_PHASE_GRP_CNT);
    motor_duty_set(periodAH, L_A_PHASE_GRP_CNT,
                   periodBH, L_B_PHASE_GRP_CNT,
                   periodCH, L_C_PHASE_GRP_CNT,
                   L_TRIG_OUT_MUX);
}

void mos_all_open_right(uint16 periodAH, uint16 periodBH, uint16 periodCH)
{
    motor_channel_set(1, R_A_PHASE_GRP_CNT,
                      1, R_B_PHASE_GRP_CNT,
                      1, R_C_PHASE_GRP_CNT);
    motor_duty_set(periodAH, R_A_PHASE_GRP_CNT,
                   periodBH, R_B_PHASE_GRP_CNT,
                   periodCH, R_C_PHASE_GRP_CNT,
                   R_TRIG_OUT_MUX);
}

void mos_all_open_middle(uint16 periodAH, uint16 periodBH, uint16 periodCH)
{
    motor_channel_set(1, M_A_PHASE_GRP_CNT,
                      1, M_B_PHASE_GRP_CNT,
                      1, M_C_PHASE_GRP_CNT);
    motor_duty_set(periodAH, M_A_PHASE_GRP_CNT,
                   periodBH, M_B_PHASE_GRP_CNT,
                   periodCH, M_C_PHASE_GRP_CNT,
                   M_TRIG_OUT_MUX);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     关闭所有MOS
// 参数说明     void
// 返回参数     void
// 使用示例     mos_close();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void mos_close_left(void)
{
    motor_channel_set(0, L_A_PHASE_GRP_CNT,
                      0, L_B_PHASE_GRP_CNT,
                      0, L_C_PHASE_GRP_CNT);
    motor_duty_set(0, L_A_PHASE_GRP_CNT,
                   0, L_B_PHASE_GRP_CNT,
                   0, L_C_PHASE_GRP_CNT,
                   L_TRIG_OUT_MUX);
}

void mos_close_right(void)
{
    motor_channel_set(0, R_A_PHASE_GRP_CNT,
                      0, R_B_PHASE_GRP_CNT,
                      0, R_C_PHASE_GRP_CNT);
    motor_duty_set(0, R_A_PHASE_GRP_CNT,
                   0, R_B_PHASE_GRP_CNT,
                   0, R_C_PHASE_GRP_CNT,
                   R_TRIG_OUT_MUX);
}

void mos_close_middle(void)
{
    motor_channel_set(0, M_A_PHASE_GRP_CNT,
                      0, M_B_PHASE_GRP_CNT,
                      0, M_C_PHASE_GRP_CNT);
    motor_duty_set(0, M_A_PHASE_GRP_CNT,
                   0, M_B_PHASE_GRP_CNT,
                   0, M_C_PHASE_GRP_CNT,
                   M_TRIG_OUT_MUX);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     电机参数初始化
// 参数说明     void
// 返回参数     void
// 使用示例     motor_parameter_init();
// 备注信息     此处将完成电机控制通道初始化、转速输出通道初始化、基本控制参数配置
//-------------------------------------------------------------------------------------------------------------------
void motor_parameter_init(void)
{
    motor_pwm_output_init(L_A_PHASE_PIN_H, L_A_PHASE_HSIOM_H, L_A_PHASE_PIN_L, L_A_PHASE_HSIOM_L, L_A_PHASE_CLK_DST, L_A_PHASE_GRP_CNT,
                          L_B_PHASE_PIN_H, L_B_PHASE_HSIOM_H, L_B_PHASE_PIN_L, L_B_PHASE_HSIOM_L, L_B_PHASE_CLK_DST, L_B_PHASE_GRP_CNT,
                          L_C_PHASE_PIN_H, L_C_PHASE_HSIOM_H, L_C_PHASE_PIN_L, L_C_PHASE_HSIOM_L, L_C_PHASE_CLK_DST, L_C_PHASE_GRP_CNT,
                          &tcpwm_config_left, L_TRIG_OUT_MUX

    );
    motor_pwm_output_init(R_A_PHASE_PIN_H, R_A_PHASE_HSIOM_H, R_A_PHASE_PIN_L, R_A_PHASE_HSIOM_L, R_A_PHASE_CLK_DST, R_A_PHASE_GRP_CNT,
                          R_B_PHASE_PIN_H, R_B_PHASE_HSIOM_H, R_B_PHASE_PIN_L, R_B_PHASE_HSIOM_L, R_B_PHASE_CLK_DST, R_B_PHASE_GRP_CNT,
                          R_C_PHASE_PIN_H, R_C_PHASE_HSIOM_H, R_C_PHASE_PIN_L, R_C_PHASE_HSIOM_L, R_C_PHASE_CLK_DST, R_C_PHASE_GRP_CNT,
                          &tcpwm_config_right, R_TRIG_OUT_MUX

    );
    motor_pwm_output_init(M_A_PHASE_PIN_H, M_A_PHASE_HSIOM_H, M_A_PHASE_PIN_L, M_A_PHASE_HSIOM_L, M_A_PHASE_CLK_DST, M_A_PHASE_GRP_CNT,
                          M_B_PHASE_PIN_H, M_B_PHASE_HSIOM_H, M_B_PHASE_PIN_L, M_B_PHASE_HSIOM_L, M_B_PHASE_CLK_DST, M_B_PHASE_GRP_CNT,
                          M_C_PHASE_PIN_H, M_C_PHASE_HSIOM_H, M_C_PHASE_PIN_L, M_C_PHASE_HSIOM_L, M_C_PHASE_CLK_DST, M_C_PHASE_GRP_CNT,
                          &tcpwm_config_middle, M_TRIG_OUT_MUX

    );
}
