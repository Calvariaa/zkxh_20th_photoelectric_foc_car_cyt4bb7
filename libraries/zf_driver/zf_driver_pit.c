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
* 文件名称          zf_driver_pit
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          IAR 9.40.1
* 适用平台          CYT4BB
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2024-1-9       pudding            first version
********************************************************************************************************************/

#include "sysclk/cy_sysclk.h"
#include "tcpwm/cy_tcpwm_pwm.h"
#include "zf_common_interrupt.h"
#include "zf_driver_pit.h"

#define PIT_USE_ISR     CPUIntIdx2_IRQn

void pit0_ch0_isr();
void pit0_ch1_isr();
void pit0_ch2_isr();
void pit0_ch3_isr();
void pit0_ch4_isr();
void pit0_ch5_isr();
void pit0_ch6_isr();
void pit0_ch7_isr();

void (*pit_isr_func[8])() = {pit0_ch0_isr, pit0_ch1_isr, pit0_ch2_isr, pit0_ch3_isr, pit0_ch4_isr, pit0_ch5_isr, pit0_ch6_isr, pit0_ch7_isr};


//-------------------------------------------------------------------------------------------------------------------
//  函数简介      pit关闭
//  参数说明      pit_index        	选择PIT模块
//  返回参数      void
//  使用示例      pit_isr_flag_clear(PIT_CH0); // 关闭 TCPWM2 通道0的计时器
//  备注信息
//-------------------------------------------------------------------------------------------------------------------
void pit_isr_flag_clear (pit_index_enum pit_index)
{
    Cy_Tcpwm_Counter_ClearTC_Intr((volatile stc_TCPWM_GRP_CNT_t*) &TCPWM0->GRP[2].CNT[pit_index]);
}

//-------------------------------------------------------------------------------------------------------------------
//  函数简介      失能pit中断
//  参数说明      pit_index      	选择PIT模块
//  返回参数      void
//  使用示例      pit_disable(PIT_CH0); // 禁止 TCPWM2 通道0的中断
//  备注信息
//-------------------------------------------------------------------------------------------------------------------
void pit_disable (pit_index_enum pit_index)
{
    Cy_Tcpwm_Counter_Disable((volatile stc_TCPWM_GRP_CNT_t*) &TCPWM0->GRP[2].CNT[pit_index]);
}

//-------------------------------------------------------------------------------------------------------------------
//  函数简介      使能pit中断
//  参数说明      pit_index       	选择PIT模块
//  返回参数      void
//  使用示例      pit_enable(PIT_CH0);  // 开启 TCPWM2 通道0的中断
//  备注信息
//-------------------------------------------------------------------------------------------------------------------
void pit_enable (pit_index_enum pit_index)
{
    Cy_Tcpwm_Counter_Enable((volatile stc_TCPWM_GRP_CNT_t*) &TCPWM0->GRP[2].CNT[pit_index]);
}

//-------------------------------------------------------------------------------------------------------------------
//  函数简介      禁止所有pit中断
//  返回参数      void
//  使用示例      pit_all_close();
//  备注信息
//-------------------------------------------------------------------------------------------------------------------
void pit_all_close (void)
{
    pit_disable(PIT_CH0);
    pit_disable(PIT_CH1);
    pit_disable(PIT_CH2);
    pit_disable(PIT_CH3);
    pit_disable(PIT_CH4);
    pit_disable(PIT_CH5);
    pit_disable(PIT_CH6);
    pit_disable(PIT_CH7);
}

//-------------------------------------------------------------------------------------------------------------------
//  函数简介      pit初始化
//  参数说明      pit_index       	选择PIT模块
//  参数说明      time            	周期时间
//  返回参数      void
//  使用示例      pit_init(PIT_CH0, 5000);      // 设置周期中断5000us
//  备注信息      请使用.h文件中 带时间单位的宏定义函数
//-------------------------------------------------------------------------------------------------------------------
void pit_init (pit_index_enum pit_index, uint32 time)
{
    cy_stc_tcpwm_counter_config_t pit_config  = {0};
    cy_stc_sysint_irq_t           irq_cfg;
    
    Cy_SysClk_PeriphAssignDivider((en_clk_dst_t)((uint32)pit_index + (uint32)PCLK_TCPWM0_CLOCKS512), (cy_en_divider_types_t)CY_SYSCLK_DIV_16_BIT, 0ul);
    Cy_SysClk_PeriphSetDivider(Cy_SysClk_GetClockGroup((en_clk_dst_t)((uint32)pit_index + (uint32)PCLK_TCPWM0_CLOCKS512)), (cy_en_divider_types_t)CY_SYSCLK_DIV_16_BIT, 0ul, 9u); // 80Mhz时钟被10分频为8Mhz
    Cy_SysClk_PeriphEnableDivider(Cy_SysClk_GetClockGroup((en_clk_dst_t)((uint32)pit_index + (uint32)PCLK_TCPWM0_CLOCKS512)), (cy_en_divider_types_t)CY_SYSCLK_DIV_16_BIT, 0ul);
    
    irq_cfg.sysIntSrc  = (cy_en_intr_t)((uint32)tcpwm_0_interrupts_512_IRQn + (uint32)pit_index); 
    irq_cfg.intIdx     = PIT_USE_ISR                                    ;
    irq_cfg.isEnabled  = true                                          ;
    interrupt_init(&irq_cfg, pit_isr_func[pit_index], 3)                ;
    
    pit_config.period             = time * 8                           ;        // pit周期计算：时钟为8M 则计数8为1us
    pit_config.clockPrescaler     = CY_TCPWM_PRESCALER_DIVBY_1         ;
    pit_config.runMode            = CY_TCPWM_COUNTER_CONTINUOUS        ; 
    pit_config.countDirection     = CY_TCPWM_COUNTER_COUNT_UP          ;
    pit_config.compareOrCapture   = CY_TCPWM_COUNTER_MODE_COMPARE      ;
    pit_config.countInputMode     = CY_TCPWM_INPUT_LEVEL               ;
    pit_config.countInput         = 1uL                                ;
    pit_config.trigger0EventCfg   = CY_TCPWM_COUNTER_OVERFLOW          ;
    pit_config.trigger1EventCfg   = CY_TCPWM_COUNTER_OVERFLOW          ;
    
    Cy_Tcpwm_Counter_Init((volatile stc_TCPWM_GRP_CNT_t*) &TCPWM0->GRP[2].CNT[pit_index], &pit_config);
    Cy_Tcpwm_Counter_Enable((volatile stc_TCPWM_GRP_CNT_t*) &TCPWM0->GRP[2].CNT[pit_index]);
    Cy_Tcpwm_TriggerStart((volatile stc_TCPWM_GRP_CNT_t*) &TCPWM0->GRP[2].CNT[pit_index]);  
    Cy_Tcpwm_Counter_SetTC_IntrMask((volatile stc_TCPWM_GRP_CNT_t*) &TCPWM0->GRP[2].CNT[pit_index]);
}
