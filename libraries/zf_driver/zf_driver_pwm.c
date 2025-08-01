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
* 文件名称          zf_driver_pwm
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          IAR 9.40.1
* 适用平台          CYT4BB
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2024-1-8       pudding            first version
********************************************************************************************************************/

#include "sysclk/cy_sysclk.h"
#include "tcpwm/cy_tcpwm_pwm.h"
#include "gpio/cy_gpio.h"
#include "zf_driver_gpio.h"
#include "zf_common_debug.h"
#include "zf_driver_pwm.h"


#define PWM_CLK_FREQ       (CY_INITIAL_TARGET_PERI_FREQ / 10)                            // TCPWM模块时钟 默认8M

//cyhal_pwm_t pwm_obj[TCPWM_CH_NUM];
//uint32 pwm_freq_save[TCPWM_CH_NUM];


//-------------------------------------------------------------------------------------------------------------------
//  函数简介     获取端口参数
//  返回参数     gpio_pin_enum
//  使用示例     get_pwm_pin(TCPWM_CH00_P03_1);
//  备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
static gpio_pin_enum get_pwm_pin (pwm_channel_enum pwm_channel)
{
    gpio_pin_enum pwm_pin;

    switch(pwm_channel)
    {
        case TCPWM_CH00_P03_1 : pwm_pin = P03_1;   break;
        case TCPWM_CH00_P06_1 : pwm_pin = P06_1;   break;
        case TCPWM_CH01_P03_0 : pwm_pin = P03_0;   break;
        case TCPWM_CH01_P06_3 : pwm_pin = P06_3;   break;
        case TCPWM_CH02_P06_5 : pwm_pin = P06_5;   break;
        case TCPWM_CH03_P02_4 : pwm_pin = P02_4;   break;
        case TCPWM_CH03_P06_7 : pwm_pin = P06_7;   break;
        case TCPWM_CH04_P02_3 : pwm_pin = P02_3;   break;
        case TCPWM_CH04_P04_0 : pwm_pin = P04_0;   break;
        case TCPWM_CH05_P02_2 : pwm_pin = P02_2;   break;
        case TCPWM_CH05_P04_1 : pwm_pin = P04_1;   break;
        case TCPWM_CH06_P02_1 : pwm_pin = P02_1;   break;
        case TCPWM_CH07_P02_0 : pwm_pin = P02_0;   break;
        case TCPWM_CH09_P05_0 : pwm_pin = P05_0;   break;
        case TCPWM_CH10_P05_1 : pwm_pin = P05_1;   break;
        case TCPWM_CH11_P05_2 : pwm_pin = P05_2;   break;
        case TCPWM_CH11_P01_1 : pwm_pin = P01_1;   break;
        case TCPWM_CH12_P05_3 : pwm_pin = P05_3;   break;
        case TCPWM_CH12_P01_0 : pwm_pin = P01_0;   break;
        case TCPWM_CH13_P05_4 : pwm_pin = P05_4;   break;
        case TCPWM_CH13_P00_3 : pwm_pin = P00_3;   break;
        case TCPWM_CH14_P00_2 : pwm_pin = P00_2;   break;
        case TCPWM_CH15_P07_1 : pwm_pin = P07_1;   break;
        case TCPWM_CH16_P07_3 : pwm_pin = P07_3;   break;
        case TCPWM_CH17_P07_5 : pwm_pin = P07_5;   break;
        case TCPWM_CH17_P00_1 : pwm_pin = P00_1;   break;
        case TCPWM_CH18_P07_7 : pwm_pin = P07_7;   break;
        case TCPWM_CH18_P00_0 : pwm_pin = P00_0;   break;
        case TCPWM_CH19_P08_0 : pwm_pin = P08_0;   break;
        case TCPWM_CH20_P08_1 : pwm_pin = P08_1;   break;
        case TCPWM_CH21_P08_2 : pwm_pin = P08_2;   break;
        case TCPWM_CH22_P08_3 : pwm_pin = P08_3;   break;
        case TCPWM_CH22_P23_7 : pwm_pin = P23_7;   break;
        case TCPWM_CH24_P09_0 : pwm_pin = P09_0;   break;
        case TCPWM_CH25_P09_1 : pwm_pin = P09_1;   break;
        case TCPWM_CH25_P23_4 : pwm_pin = P23_4;   break;
        case TCPWM_CH26_P19_1 : pwm_pin = P19_1;   break;
        case TCPWM_CH27_P19_2 : pwm_pin = P19_2;   break;
        case TCPWM_CH28_P10_0 : pwm_pin = P10_0;   break;
        case TCPWM_CH28_P19_3 : pwm_pin = P19_3;   break;
        case TCPWM_CH28_P22_6 : pwm_pin = P22_6;   break;
        case TCPWM_CH29_P10_1 : pwm_pin = P10_1;   break;
        case TCPWM_CH29_P19_4 : pwm_pin = P19_4;   break;
        case TCPWM_CH29_P22_5 : pwm_pin = P22_5;   break;
        case TCPWM_CH30_P10_2 : pwm_pin = P10_2;   break;
        case TCPWM_CH30_P20_0 : pwm_pin = P20_0;   break;
        case TCPWM_CH30_P22_4 : pwm_pin = P22_4;   break;
        case TCPWM_CH31_P10_3 : pwm_pin = P10_3;   break;
        case TCPWM_CH31_P22_3 : pwm_pin = P22_3;   break;
        case TCPWM_CH32_P10_4 : pwm_pin = P10_4;   break;
        case TCPWM_CH32_P22_2 : pwm_pin = P22_2;   break;
        case TCPWM_CH33_P22_1 : pwm_pin = P22_1;   break;
        case TCPWM_CH34_P21_5 : pwm_pin = P21_5;   break;
        case TCPWM_CH36_P12_0 : pwm_pin = P12_0;   break;
        case TCPWM_CH36_P21_6 : pwm_pin = P21_6;   break;
        case TCPWM_CH37_P12_1 : pwm_pin = P12_1;   break;
        case TCPWM_CH37_P21_5 : pwm_pin = P21_5;   break;
        case TCPWM_CH38_P12_2 : pwm_pin = P12_2;   break;
        case TCPWM_CH39_P12_3 : pwm_pin = P12_3;   break;
        case TCPWM_CH39_P21_3 : pwm_pin = P21_3;   break;
        case TCPWM_CH40_P12_4 : pwm_pin = P12_4;   break;
        case TCPWM_CH40_P21_2 : pwm_pin = P21_2;   break;
        case TCPWM_CH41_P12_5 : pwm_pin = P12_5;   break;
        case TCPWM_CH41_P21_1 : pwm_pin = P21_1;   break;
        case TCPWM_CH42_P21_0 : pwm_pin = P21_0;   break;
        case TCPWM_CH44_P13_1 : pwm_pin = P13_1;   break;
        case TCPWM_CH45_P13_3 : pwm_pin = P13_3;   break;
        case TCPWM_CH46_P13_5 : pwm_pin = P13_5;   break;
        case TCPWM_CH47_P13_7 : pwm_pin = P13_7;   break;
        case TCPWM_CH47_P20_3 : pwm_pin = P20_3;   break;
        case TCPWM_CH48_P14_0 : pwm_pin = P14_0;   break;
        case TCPWM_CH48_P20_2 : pwm_pin = P20_2;   break;
        case TCPWM_CH49_P14_1 : pwm_pin = P14_1;   break;
        case TCPWM_CH49_P20_1 : pwm_pin = P20_1;   break;
        case TCPWM_CH50_P18_7 : pwm_pin = P18_7;   break;
        case TCPWM_CH51_P18_6 : pwm_pin = P18_6;   break;
        case TCPWM_CH52_P14_4 : pwm_pin = P14_4;   break;
        case TCPWM_CH52_P18_5 : pwm_pin = P18_5;   break;
        case TCPWM_CH53_P14_5 : pwm_pin = P14_5;   break;
        case TCPWM_CH53_P18_4 : pwm_pin = P18_4;   break;
        case TCPWM_CH54_P18_3 : pwm_pin = P18_3;   break;
        case TCPWM_CH55_P18_2 : pwm_pin = P18_2;   break;
        case TCPWM_CH56_P15_0 : pwm_pin = P15_0;   break;
        case TCPWM_CH57_P15_1 : pwm_pin = P15_1;   break;
        case TCPWM_CH57_P17_4 : pwm_pin = P17_4;   break;
        case TCPWM_CH58_P15_2 : pwm_pin = P15_2;   break;
        case TCPWM_CH58_P17_3 : pwm_pin = P17_3;   break;
        case TCPWM_CH59_P15_3 : pwm_pin = P15_3;   break;
        case TCPWM_CH59_P17_2 : pwm_pin = P17_2;   break;
        case TCPWM_CH59_P11_2 : pwm_pin = P11_2;   break;
        case TCPWM_CH60_P11_1 : pwm_pin = P11_1;   break;
        case TCPWM_CH60_P17_1 : pwm_pin = P17_1;   break;
        case TCPWM_CH61_P11_0 : pwm_pin = P11_0;   break;
        case TCPWM_CH61_P17_0 : pwm_pin = P17_0;   break;
        default: zf_assert(0); break;
    }
    return pwm_pin;
}

//-------------------------------------------------------------------------------------------------------------------
//  函数简介     获取端口参数
//  返回参数     gpio_pin_enum
//  使用示例     get_pwm_pin(TCPWM_CH00_P03_1);
//  备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
static en_hsiom_sel_t get_pwm_pin_hsiom (pwm_channel_enum pwm_channel)
{
    en_hsiom_sel_t pin_hsiom;

    switch(pwm_channel)
    {
        case TCPWM_CH00_P03_1 : pin_hsiom = P3_1_TCPWM0_LINE0;     break;
        case TCPWM_CH00_P06_1 : pin_hsiom = P6_1_TCPWM0_LINE0;     break;
        case TCPWM_CH01_P03_0 : pin_hsiom = P3_0_TCPWM0_LINE1;     break;
        case TCPWM_CH01_P06_3 : pin_hsiom = P6_3_TCPWM0_LINE1;     break;
        case TCPWM_CH02_P06_5 : pin_hsiom = P6_5_TCPWM0_LINE2;     break;
        case TCPWM_CH03_P02_4 : pin_hsiom = P2_4_TCPWM0_LINE3;     break;
        case TCPWM_CH03_P06_7 : pin_hsiom = P6_7_TCPWM0_LINE3;     break;
        case TCPWM_CH04_P02_3 : pin_hsiom = P2_3_TCPWM0_LINE4;     break;
        case TCPWM_CH04_P04_0 : pin_hsiom = P4_0_TCPWM0_LINE4;     break;
        case TCPWM_CH05_P02_2 : pin_hsiom = P2_2_TCPWM0_LINE5;     break;
        case TCPWM_CH05_P04_1 : pin_hsiom = P4_1_TCPWM0_LINE5;     break;
        case TCPWM_CH06_P02_1 : pin_hsiom = P2_1_TCPWM0_LINE6;     break;
        case TCPWM_CH07_P02_0 : pin_hsiom = P2_0_TCPWM0_LINE7;     break;
        case TCPWM_CH09_P05_0 : pin_hsiom = P5_0_TCPWM0_LINE9;     break;
        case TCPWM_CH10_P05_1 : pin_hsiom = P5_1_TCPWM0_LINE10;    break;
        case TCPWM_CH11_P05_2 : pin_hsiom = P5_2_TCPWM0_LINE11;    break;
        case TCPWM_CH11_P01_1 : pin_hsiom = P1_1_TCPWM0_LINE11;    break;
        case TCPWM_CH12_P05_3 : pin_hsiom = P5_3_TCPWM0_LINE12;    break;
        case TCPWM_CH12_P01_0 : pin_hsiom = P1_0_TCPWM0_LINE12;    break;
        case TCPWM_CH13_P05_4 : pin_hsiom = P5_4_TCPWM0_LINE13;    break;
        case TCPWM_CH13_P00_3 : pin_hsiom = P0_3_TCPWM0_LINE13;    break;
        case TCPWM_CH14_P00_2 : pin_hsiom = P0_2_TCPWM0_LINE14;    break;
        case TCPWM_CH15_P07_1 : pin_hsiom = P7_1_TCPWM0_LINE15;    break;
        case TCPWM_CH16_P07_3 : pin_hsiom = P7_3_TCPWM0_LINE16;    break;
        case TCPWM_CH17_P07_5 : pin_hsiom = P7_5_TCPWM0_LINE17;    break;
        case TCPWM_CH17_P00_1 : pin_hsiom = P0_1_TCPWM0_LINE17;    break;
        case TCPWM_CH18_P07_7 : pin_hsiom = P7_7_TCPWM0_LINE18;    break;
        case TCPWM_CH18_P00_0 : pin_hsiom = P0_0_TCPWM0_LINE18;    break;
        case TCPWM_CH19_P08_0 : pin_hsiom = P8_0_TCPWM0_LINE19;    break;
        case TCPWM_CH20_P08_1 : pin_hsiom = P8_1_TCPWM0_LINE20;    break;
        case TCPWM_CH21_P08_2 : pin_hsiom = P8_2_TCPWM0_LINE21;    break;
        case TCPWM_CH22_P08_3 : pin_hsiom = P8_3_TCPWM0_LINE22;    break;
        case TCPWM_CH22_P23_7 : pin_hsiom = P23_7_TCPWM0_LINE22;   break;
        case TCPWM_CH24_P09_0 : pin_hsiom = P9_0_TCPWM0_LINE24;    break;
        case TCPWM_CH25_P09_1 : pin_hsiom = P9_1_TCPWM0_LINE25;    break;
        case TCPWM_CH25_P23_4 : pin_hsiom = P23_4_TCPWM0_LINE25;   break;
        case TCPWM_CH26_P19_1 : pin_hsiom = P19_1_TCPWM0_LINE26;   break;
        case TCPWM_CH27_P19_2 : pin_hsiom = P19_2_TCPWM0_LINE27;   break;
        case TCPWM_CH28_P10_0 : pin_hsiom = P10_0_TCPWM0_LINE28;   break;
        case TCPWM_CH28_P19_3 : pin_hsiom = P19_3_TCPWM0_LINE28;   break;
        case TCPWM_CH28_P22_6 : pin_hsiom = P22_6_TCPWM0_LINE28;   break;
        case TCPWM_CH29_P10_1 : pin_hsiom = P10_1_TCPWM0_LINE29;   break;
        case TCPWM_CH29_P19_4 : pin_hsiom = P19_4_TCPWM0_LINE29;   break;
        case TCPWM_CH29_P22_5 : pin_hsiom = P22_5_TCPWM0_LINE29;   break;
        case TCPWM_CH30_P10_2 : pin_hsiom = P10_2_TCPWM0_LINE30;   break;
        case TCPWM_CH30_P20_0 : pin_hsiom = P20_0_TCPWM0_LINE30;   break;
        case TCPWM_CH30_P22_4 : pin_hsiom = P22_4_TCPWM0_LINE30;   break;
        case TCPWM_CH31_P10_3 : pin_hsiom = P10_3_TCPWM0_LINE31;   break;
        case TCPWM_CH31_P22_3 : pin_hsiom = P22_3_TCPWM0_LINE31;   break;
        case TCPWM_CH32_P10_4 : pin_hsiom = P10_4_TCPWM0_LINE32;   break;
        case TCPWM_CH32_P22_2 : pin_hsiom = P22_2_TCPWM0_LINE32;   break;
        case TCPWM_CH33_P22_1 : pin_hsiom = P22_1_TCPWM0_LINE33;   break;
        case TCPWM_CH34_P21_5 : pin_hsiom = P21_5_TCPWM0_LINE34;   break;
        case TCPWM_CH36_P12_0 : pin_hsiom = P12_0_TCPWM0_LINE36;   break;
        case TCPWM_CH36_P21_6 : pin_hsiom = P21_6_TCPWM0_LINE36;   break;
        case TCPWM_CH37_P12_1 : pin_hsiom = P12_1_TCPWM0_LINE37;   break;
        case TCPWM_CH37_P21_5 : pin_hsiom = P21_5_TCPWM0_LINE37;   break;
        case TCPWM_CH38_P12_2 : pin_hsiom = P12_2_TCPWM0_LINE38;   break;
        case TCPWM_CH39_P12_3 : pin_hsiom = P12_3_TCPWM0_LINE39;   break;
        case TCPWM_CH39_P21_3 : pin_hsiom = P21_3_TCPWM0_LINE39;   break;
        case TCPWM_CH40_P12_4 : pin_hsiom = P12_4_TCPWM0_LINE40;   break;
        case TCPWM_CH40_P21_2 : pin_hsiom = P21_2_TCPWM0_LINE40;   break;
        case TCPWM_CH41_P12_5 : pin_hsiom = P12_5_TCPWM0_LINE41;   break;
        case TCPWM_CH41_P21_1 : pin_hsiom = P21_1_TCPWM0_LINE41;   break;
        case TCPWM_CH42_P21_0 : pin_hsiom = P21_0_TCPWM0_LINE42;   break;
        case TCPWM_CH44_P13_1 : pin_hsiom = P13_1_TCPWM0_LINE44;   break;
        case TCPWM_CH45_P13_3 : pin_hsiom = P13_3_TCPWM0_LINE45;   break;
        case TCPWM_CH46_P13_5 : pin_hsiom = P13_5_TCPWM0_LINE46;   break;
        case TCPWM_CH47_P13_7 : pin_hsiom = P13_7_TCPWM0_LINE47;   break;
        case TCPWM_CH47_P20_3 : pin_hsiom = P20_3_TCPWM0_LINE47;   break;
        case TCPWM_CH48_P14_0 : pin_hsiom = P14_0_TCPWM0_LINE48;   break;
        case TCPWM_CH48_P20_2 : pin_hsiom = P20_2_TCPWM0_LINE48;   break;
        case TCPWM_CH49_P14_1 : pin_hsiom = P14_1_TCPWM0_LINE49;   break;
        case TCPWM_CH49_P20_1 : pin_hsiom = P20_1_TCPWM0_LINE49;   break;
        case TCPWM_CH50_P18_7 : pin_hsiom = P18_7_TCPWM0_LINE50;   break;
        case TCPWM_CH51_P18_6 : pin_hsiom = P18_6_TCPWM0_LINE51;   break;
        case TCPWM_CH52_P14_4 : pin_hsiom = P14_4_TCPWM0_LINE52;   break;
        case TCPWM_CH52_P18_5 : pin_hsiom = P18_5_TCPWM0_LINE52;   break;
        case TCPWM_CH53_P14_5 : pin_hsiom = P14_5_TCPWM0_LINE53;   break;
        case TCPWM_CH53_P18_4 : pin_hsiom = P18_4_TCPWM0_LINE53;   break;
        case TCPWM_CH54_P18_3 : pin_hsiom = P18_3_TCPWM0_LINE54;   break;
        case TCPWM_CH55_P18_2 : pin_hsiom = P18_2_TCPWM0_LINE55;   break;
        case TCPWM_CH56_P15_0 : pin_hsiom = P15_0_TCPWM0_LINE56;   break;
        case TCPWM_CH57_P15_1 : pin_hsiom = P15_1_TCPWM0_LINE57;   break;
        case TCPWM_CH57_P17_4 : pin_hsiom = P17_4_TCPWM0_LINE57;   break;
        case TCPWM_CH58_P15_2 : pin_hsiom = P15_2_TCPWM0_LINE58;   break;
        case TCPWM_CH58_P17_3 : pin_hsiom = P17_3_TCPWM0_LINE58;   break;
        case TCPWM_CH59_P15_3 : pin_hsiom = P15_3_TCPWM0_LINE59;   break;
        case TCPWM_CH59_P17_2 : pin_hsiom = P17_2_TCPWM0_LINE59;   break;
        case TCPWM_CH59_P11_2 : pin_hsiom = P11_2_TCPWM0_LINE59;   break;
        case TCPWM_CH60_P11_1 : pin_hsiom = P11_1_TCPWM0_LINE60;   break;
        case TCPWM_CH60_P17_1 : pin_hsiom = P17_1_TCPWM0_LINE60;   break;
        case TCPWM_CH61_P11_0 : pin_hsiom = P11_0_TCPWM0_LINE61;   break;
        case TCPWM_CH61_P17_0 : pin_hsiom = P17_0_TCPWM0_LINE61;   break;
        default: zf_assert(0); break;
    }
    return pin_hsiom;
}

//-------------------------------------------------------------------------------------------------------------------
//  函数简介     获取端口通道号
//  返回参数     cyhal_gpio_t
//  使用示例     get_pwm_pin(TCPWM_CH00_P03_1);
//  备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
static uint8 get_pwm_ch (pwm_channel_enum pwm_channel)
{
    uint8 pwm_ch;

    switch(pwm_channel)
    {
        case TCPWM_CH00_P03_1 : pwm_ch = 0 ;   break;
        case TCPWM_CH00_P06_1 : pwm_ch = 0 ;   break;
        case TCPWM_CH01_P03_0 : pwm_ch = 1 ;   break;
        case TCPWM_CH01_P06_3 : pwm_ch = 1 ;   break;
        case TCPWM_CH02_P06_5 : pwm_ch = 2 ;   break;
        case TCPWM_CH03_P02_4 : pwm_ch = 3 ;   break;
        case TCPWM_CH03_P06_7 : pwm_ch = 3 ;   break;
        case TCPWM_CH04_P02_3 : pwm_ch = 4 ;   break;
        case TCPWM_CH04_P04_0 : pwm_ch = 4 ;   break;
        case TCPWM_CH05_P02_2 : pwm_ch = 5 ;   break;
        case TCPWM_CH05_P04_1 : pwm_ch = 5 ;   break;
        case TCPWM_CH06_P02_1 : pwm_ch = 6 ;   break;
        case TCPWM_CH07_P02_0 : pwm_ch = 7 ;   break;
        case TCPWM_CH09_P05_0 : pwm_ch = 9 ;   break;
        case TCPWM_CH10_P05_1 : pwm_ch = 10;   break;
        case TCPWM_CH11_P05_2 : pwm_ch = 11;   break;
        case TCPWM_CH11_P01_1 : pwm_ch = 11;   break;
        case TCPWM_CH12_P05_3 : pwm_ch = 12;   break;
        case TCPWM_CH12_P01_0 : pwm_ch = 12;   break;
        case TCPWM_CH13_P05_4 : pwm_ch = 13;   break;
        case TCPWM_CH13_P00_3 : pwm_ch = 13;   break;
        case TCPWM_CH14_P00_2 : pwm_ch = 14;   break;
        case TCPWM_CH15_P07_1 : pwm_ch = 15;   break;
        case TCPWM_CH16_P07_3 : pwm_ch = 16;   break;
        case TCPWM_CH17_P07_5 : pwm_ch = 17;   break;
        case TCPWM_CH17_P00_1 : pwm_ch = 17;   break;
        case TCPWM_CH18_P07_7 : pwm_ch = 18;   break;
        case TCPWM_CH18_P00_0 : pwm_ch = 18;   break;
        case TCPWM_CH19_P08_0 : pwm_ch = 19;   break;
        case TCPWM_CH20_P08_1 : pwm_ch = 20;   break;
        case TCPWM_CH21_P08_2 : pwm_ch = 21;   break;
        case TCPWM_CH22_P08_3 : pwm_ch = 22;   break;
        case TCPWM_CH22_P23_7 : pwm_ch = 22;   break;
        case TCPWM_CH24_P09_0 : pwm_ch = 24;   break;
        case TCPWM_CH25_P09_1 : pwm_ch = 25;   break;
        case TCPWM_CH25_P23_4 : pwm_ch = 25;   break;
        case TCPWM_CH26_P19_1 : pwm_ch = 26;   break;
        case TCPWM_CH27_P19_2 : pwm_ch = 27;   break;
        case TCPWM_CH28_P10_0 : pwm_ch = 28;   break;
        case TCPWM_CH28_P19_3 : pwm_ch = 28;   break;
        case TCPWM_CH28_P22_6 : pwm_ch = 28;   break;
        case TCPWM_CH29_P10_1 : pwm_ch = 29;   break;
        case TCPWM_CH29_P19_4 : pwm_ch = 29;   break;
        case TCPWM_CH29_P22_5 : pwm_ch = 29;   break;
        case TCPWM_CH30_P10_2 : pwm_ch = 30;   break;
        case TCPWM_CH30_P20_0 : pwm_ch = 30;   break;
        case TCPWM_CH30_P22_4 : pwm_ch = 30;   break;
        case TCPWM_CH31_P10_3 : pwm_ch = 31;   break;
        case TCPWM_CH31_P22_3 : pwm_ch = 31;   break;
        case TCPWM_CH32_P10_4 : pwm_ch = 32;   break;
        case TCPWM_CH32_P22_2 : pwm_ch = 32;   break;
        case TCPWM_CH33_P22_1 : pwm_ch = 33;   break;
        case TCPWM_CH34_P21_5 : pwm_ch = 34;   break;
        case TCPWM_CH36_P12_0 : pwm_ch = 36;   break;
        case TCPWM_CH36_P21_6 : pwm_ch = 36;   break;
        case TCPWM_CH37_P12_1 : pwm_ch = 37;   break;
        case TCPWM_CH37_P21_5 : pwm_ch = 37;   break;
        case TCPWM_CH38_P12_2 : pwm_ch = 38;   break;
        case TCPWM_CH39_P12_3 : pwm_ch = 39;   break;
        case TCPWM_CH39_P21_3 : pwm_ch = 39;   break;
        case TCPWM_CH40_P12_4 : pwm_ch = 40;   break;
        case TCPWM_CH40_P21_2 : pwm_ch = 40;   break;
        case TCPWM_CH41_P12_5 : pwm_ch = 41;   break;
        case TCPWM_CH41_P21_1 : pwm_ch = 41;   break;
        case TCPWM_CH42_P21_0 : pwm_ch = 42;   break;
        case TCPWM_CH44_P13_1 : pwm_ch = 44;   break;
        case TCPWM_CH45_P13_3 : pwm_ch = 45;   break;
        case TCPWM_CH46_P13_5 : pwm_ch = 46;   break;
        case TCPWM_CH47_P13_7 : pwm_ch = 47;   break;
        case TCPWM_CH47_P20_3 : pwm_ch = 47;   break;
        case TCPWM_CH48_P14_0 : pwm_ch = 48;   break;
        case TCPWM_CH48_P20_2 : pwm_ch = 48;   break;
        case TCPWM_CH49_P14_1 : pwm_ch = 49;   break;
        case TCPWM_CH49_P20_1 : pwm_ch = 49;   break;
        case TCPWM_CH50_P18_7 : pwm_ch = 50;   break;
        case TCPWM_CH51_P18_6 : pwm_ch = 51;   break;
        case TCPWM_CH52_P14_4 : pwm_ch = 52;   break;
        case TCPWM_CH52_P18_5 : pwm_ch = 52;   break;
        case TCPWM_CH53_P14_5 : pwm_ch = 53;   break;
        case TCPWM_CH53_P18_4 : pwm_ch = 53;   break;
        case TCPWM_CH54_P18_3 : pwm_ch = 54;   break;
        case TCPWM_CH55_P18_2 : pwm_ch = 55;   break;
        case TCPWM_CH56_P15_0 : pwm_ch = 56;   break;
        case TCPWM_CH57_P15_1 : pwm_ch = 57;   break;
        case TCPWM_CH57_P17_4 : pwm_ch = 57;   break;
        case TCPWM_CH58_P15_2 : pwm_ch = 58;   break;
        case TCPWM_CH58_P17_3 : pwm_ch = 58;   break;
        case TCPWM_CH59_P15_3 : pwm_ch = 59;   break;
        case TCPWM_CH59_P17_2 : pwm_ch = 59;   break;
        case TCPWM_CH59_P11_2 : pwm_ch = 59;   break;
        case TCPWM_CH60_P11_1 : pwm_ch = 60;   break;
        case TCPWM_CH60_P17_1 : pwm_ch = 60;   break;
        case TCPWM_CH61_P11_0 : pwm_ch = 61;   break;
        case TCPWM_CH61_P17_0 : pwm_ch = 61;   break;
        default: zf_assert(0); pwm_ch = 0;
    }
    return pwm_ch;
}

//-------------------------------------------------------------------------------------------------------------------
//  函数简介      关闭所有通道的PWM输出
//  返回参数      void
//  使用示例      pwm_all_channel_close();
//  备注信息
//-------------------------------------------------------------------------------------------------------------------
void pwm_all_channel_close (void)
{
    for(int i = 0; i < 63; i ++)
    {
        TCPWM0->GRP[0].CNT[i].unCTRL.u32Register &= 0x7fffffff;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     PWM占空比设置
// 参数说明     pin             选择 PWM 引脚
// 参数说明     duty            设置占空比
// 返回参数     void
// 使用示例     pwm_set_duty(TCPWM1_CH00_P03_1, 5000); // 设置占空比为百分之5000/PWM_DUTY_MAX*100
// 备注信息     PWM_DUTY_MAX 宏定义在zf_driver_pwm.h  默认为10000
//-------------------------------------------------------------------------------------------------------------------
void pwm_set_duty (pwm_channel_enum pwmch, uint32 duty)
{
    uint16 period = 0;
    uint16 compare = 0;
  
    period = TCPWM0->GRP[0].CNT[get_pwm_ch(pwmch)].unPERIOD.u32Register;
    
    compare = period * duty / 10000;
  
    TCPWM0->GRP[0].CNT[get_pwm_ch(pwmch)].unCC0_BUFF.u32Register = compare;

    TCPWM0->GRP[0].CNT[get_pwm_ch(pwmch)].unTR_CMD.u32Register |= 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     PWM 初始化
// 参数说明     pin             选择 PWM 引脚
// 参数说明     freq            设置频率 同个模块只有最后一次设置生效
// 参数说明     duty            设置占空比
// 返回参数     void
// 使用示例     pwm_init(TCPWM_CH00_P03_1, 50, 1000);   // ATOM 0模块的通道7 使用P02_7引脚输出PWM  PWM频率50HZ  占空比百分之1000/PWM_DUTY_MAX*100
// 备注信息     PWM_DUTY_MAX 宏定义在zf_driver_pwm.h  默认为10000
//-------------------------------------------------------------------------------------------------------------------
void pwm_init (pwm_channel_enum pwmch, uint32 freq, uint32 duty)
{	
    cy_stc_gpio_pin_config_t  pwm_pin_config     = {0};
    cy_stc_tcpwm_pwm_config_t tcpwm_pwm_config;
    uint32 tcpwm_prescaler_div                   = 0;
    
    memset(&tcpwm_pwm_config, 0, sizeof(tcpwm_pwm_config));
    
    pwm_pin_config.driveMode                     = CY_GPIO_DM_STRONG;
    pwm_pin_config.hsiom                         = get_pwm_pin_hsiom(pwmch);
    Cy_GPIO_Pin_Init(get_port(get_pwm_pin(pwmch)), (get_pwm_pin(pwmch) % 8), &pwm_pin_config);
    
    Cy_SysClk_PeriphAssignDivider((en_clk_dst_t)(get_pwm_ch(pwmch) + PCLK_TCPWM0_CLOCKS0), (cy_en_divider_types_t)CY_SYSCLK_DIV_16_BIT, 0ul);
    Cy_SysClk_PeriphSetDivider(Cy_SysClk_GetClockGroup((en_clk_dst_t)(get_pwm_ch(pwmch) + PCLK_TCPWM0_CLOCKS0)), (cy_en_divider_types_t)CY_SYSCLK_DIV_16_BIT, 0ul, 9u); // 80Mhz时钟被10分频为8Mhz
    Cy_SysClk_PeriphEnableDivider(Cy_SysClk_GetClockGroup((en_clk_dst_t)(get_pwm_ch(pwmch) + PCLK_TCPWM0_CLOCKS0)), (cy_en_divider_types_t)CY_SYSCLK_DIV_16_BIT, 0ul);
    
    tcpwm_prescaler_div = (PWM_CLK_FREQ/freq)>>16;
    if(PWM_CLK_FREQ % (freq<<16)) tcpwm_prescaler_div ++;
    if      (1   >= tcpwm_prescaler_div)  tcpwm_pwm_config.clockPrescaler     = CY_TCPWM_PRESCALER_DIVBY_1  ;
    else if (2   >= tcpwm_prescaler_div)  tcpwm_pwm_config.clockPrescaler     = CY_TCPWM_PRESCALER_DIVBY_2  ;
    else if (4   >= tcpwm_prescaler_div)  tcpwm_pwm_config.clockPrescaler     = CY_TCPWM_PRESCALER_DIVBY_4  ;
    else if (8   >= tcpwm_prescaler_div)  tcpwm_pwm_config.clockPrescaler     = CY_TCPWM_PRESCALER_DIVBY_8  ;
    else if (16  >= tcpwm_prescaler_div)  tcpwm_pwm_config.clockPrescaler     = CY_TCPWM_PRESCALER_DIVBY_16 ;
    else if (32  >= tcpwm_prescaler_div)  tcpwm_pwm_config.clockPrescaler     = CY_TCPWM_PRESCALER_DIVBY_32 ;
    else if (64  >= tcpwm_prescaler_div)  tcpwm_pwm_config.clockPrescaler     = CY_TCPWM_PRESCALER_DIVBY_64 ;
    else if (128 >= tcpwm_prescaler_div)  tcpwm_pwm_config.clockPrescaler     = CY_TCPWM_PRESCALER_DIVBY_128;
    else
    {
        zf_assert(0) ;                                                          // 频率范围溢出
    }
    
    tcpwm_pwm_config.pwmMode            = CY_TCPWM_PWM_MODE_PWM                 ;
    tcpwm_pwm_config.cc0MatchMode       = CY_TCPWM_PWM_TR_CTRL2_CLEAR           ;
    tcpwm_pwm_config.overflowMode       = CY_TCPWM_PWM_TR_CTRL2_SET             ;
    tcpwm_pwm_config.underflowMode      = CY_TCPWM_PWM_TR_CTRL2_NO_CHANGE       ;
    tcpwm_pwm_config.cc1MatchMode       = CY_TCPWM_PWM_TR_CTRL2_NO_CHANGE       ;
    tcpwm_pwm_config.runMode            = CY_TCPWM_PWM_CONTINUOUS               ;
    tcpwm_pwm_config.period             = ((PWM_CLK_FREQ / (0x01 << tcpwm_pwm_config.clockPrescaler) / freq) - 1);
    tcpwm_pwm_config.compare0           = (tcpwm_pwm_config.period + 1) * duty / 10000;
    tcpwm_pwm_config.killMode           = CY_TCPWM_PWM_STOP_ON_KILL             ;
    tcpwm_pwm_config.countInputMode     = CY_TCPWM_INPUT_LEVEL                  ;
    tcpwm_pwm_config.countInput         = 1uL                                   ;
    tcpwm_pwm_config.enableCompare0Swap = true                                  ;
    
    Cy_Tcpwm_Pwm_Init((volatile stc_TCPWM_GRP_CNT_t*) &TCPWM0->GRP[0].CNT[get_pwm_ch(pwmch)], &tcpwm_pwm_config);
    Cy_Tcpwm_Pwm_Enable((volatile stc_TCPWM_GRP_CNT_t*) &TCPWM0->GRP[0].CNT[get_pwm_ch(pwmch)]);
    Cy_Tcpwm_TriggerStart((volatile stc_TCPWM_GRP_CNT_t*) &TCPWM0->GRP[0].CNT[get_pwm_ch(pwmch)]);
}


