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
 * 文件名称          main_cm7_1
 * 公司名称          成都逐飞科技有限公司
 * 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
 * 开发环境          IAR 9.40.1
 * 适用平台          CYT4BB
 * 店铺链接          https://seekfree.taobao.com/
 *
 * 修改记录
 * 日期              作者                备注
 * 2024-1-4       pudding            first version
 ********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "brushless/move_filter.h"
#include "brushless/motor.h"
#include "brushless/foc.h"
#include "debug/vofaplus.h"
#include "brushless/encoder/encoder.h"
#include "brushless/move_filter.h"
#include "brushless/buzzer.h"
#include "brushless/adc.h"
#include "fastmath/cos_sin.h"
#include "arm_math.h"

bool protect_flag = 0;
#define TESTPIN (P20_1)

extern FOC_Parm_Typedef foc_left;
extern FOC_Parm_Typedef foc_right;

extern encoder_t encoder_left;
extern encoder_t encoder_right;

extern uint64_t timer_1ms;

int main(void)
{
    clock_init(SYSTEM_CLOCK_250M); // 时钟配置及系统初始化<务必保留>
    debug_info_init();             // 调试串口信息初始化

    // 此处编写用户代码 例如外设初始化代码等

    gpio_init(TESTPIN, GPO, GPIO_LOW, GPO_PUSH_PULL); // 初始化 LED1 输出 默认高电平 推挽输出模式

    buzzer_init(1);

    // move_filter_double_init(&current_a_filter);
    // move_filter_double_init(&current_b_filter);
    // move_filter_double_init(&current_c_filter);
    // move_filter_double_init(&iq_ref_filter);
    // move_filter_double_init(&id_ref_filter);
    // move_filter_double_init(&speed_filter);
    encoder_init();

    // 关闭全局中断
    interrupt_global_disable();

    // init ipc
    // init_ipc_cm71();

    // 电机参数初始化
    motor_parameter_init();

    motor_bldc_adc_init();

    adc_init(ADC_ABMF, ADC_12BIT);
    adc_init(ADC_BBMF, ADC_12BIT);
    adc_init(ADC_CBMF, ADC_12BIT);

    cy_stc_gpio_pin_config_t gpio_pin_config = {0};
    gpio_pin_config.driveMode = CY_GPIO_DM_PULLUP;
    Cy_GPIO_Pin_Init(get_port(P20_1), (P20_1 % 8), &gpio_pin_config);
    Cy_GPIO_Pin_Init(get_port(P20_3), (P20_3 % 8), &gpio_pin_config);
    Cy_GPIO_Pin_Init(get_port(P21_6), (P21_6 % 8), &gpio_pin_config);

    // pit_us_init(PIT_CH0, 50); // 20khz
    // pit_us_init(PIT_CH1, 50);
    // pit_us_init(PIT_CH2, 50);

    pit_ms_init(PIT_CH0, 1); // 1ms

    interrupt_global_enable(0);

    // play_music();

    buzz_keep_ms(100, 0, &buzz_left);
    buzz_keep_ms(140, NOTE_C6, &buzz_left);
    buzz_keep_ms(10, 0, &buzz_left);

    buzz_keep_ms(140, NOTE_G6, &buzz_left);
    buzz_keep_ms(10, 0, &buzz_left);

    buzz_keep_ms(140, NOTE_C7, &buzz_left);
    buzz_keep_ms(10, 0, &buzz_left);

    buzz_ease_ms(180, NOTE_E6, NOTE_F6, &buzz_left);
    buzz_ease_ms(100, NOTE_F6, NOTE_A6, &buzz_left);
    buzz_keep_ms(10, 0, &buzz_left);

    buzz_keep_ms(140, NOTE_G6, &buzz_left);
    buzz_keep_ms(10, 0, &buzz_left);

    buzz_keep_ms(100, 0, &buzz_right);
    buzz_keep_ms(140, NOTE_C6, &buzz_right);
    buzz_keep_ms(10, 0, &buzz_right);

    buzz_keep_ms(140, NOTE_G6, &buzz_right);
    buzz_keep_ms(10, 0, &buzz_right);

    buzz_keep_ms(140, NOTE_C7, &buzz_right);
    buzz_keep_ms(10, 0, &buzz_right);

    buzz_ease_ms(180, NOTE_E6, NOTE_F6, &buzz_right);
    buzz_ease_ms(100, NOTE_F6, NOTE_A6, &buzz_right);
    buzz_keep_ms(10, 0, &buzz_right);

    buzz_keep_ms(140, NOTE_G6, &buzz_right);
    buzz_keep_ms(10, 0, &buzz_right);

    //
    // buzz_keep_ms(240, NOTE_C6, &buzz_left);
    // buzz_keep_ms(240, NOTE_E6, &buzz_right);
    // buzz_keep_ms(10, 0, &buzz_left);
    // buzz_keep_ms(10, 0, &buzz_right);

    // buzz_keep_ms(240, NOTE_E6, &buzz_left);
    // buzz_keep_ms(240, NOTE_G6, &buzz_right);
    // buzz_keep_ms(10, 0, &buzz_left);
    // buzz_keep_ms(10, 0, &buzz_right);

    // buzz_keep_ms(240, NOTE_G6, &buzz_left);
    // buzz_keep_ms(240, NOTE_C7, &buzz_right);
    // buzz_keep_ms(10, 0, &buzz_left);
    // buzz_keep_ms(10, 0, &buzz_right);

    // 此处编写用户代码 例如外设初始化代码等
    while (true)
    {

        // send_vofaplus_queue();
        // continue;

        data_send(4, (float)foc_left.Period.AH);
        data_send(5, (float)foc_left.Period.BH);
        data_send(6, (float)foc_left.Period.CH);

        data_send(7, (float)foc_right.Period.AH);
        data_send(8, (float)foc_right.Period.BH);
        data_send(9, (float)foc_right.Period.CH);

        data_send(10, (float)encoder_left.theta_elec*100);
        data_send(11, (float)encoder_right.theta_elec*100);

        data_send(12, (float)fast_sin(encoder_left.theta_elec));
        // data_send(13, (float)arm_sin_f32(encoder_left.theta_elec));
        // data_send(12, encoder_left.theta_elec;
        // data_send(13, encoder_right.theta_elec;

        // data_send(12, (foc_left.set_angle + foc_left.expect_rotations * pi_2) - (encoder_left.theta_magnet + encoder_left.full_rotations * pi_2);
        // data_send(13, (foc_right.set_angle + foc_right.expect_rotations * pi_2) - (encoder_right.theta_magnet + encoder_right.full_rotations * pi_2);

        // data_send(14, ierror_count);

        data_send(14, (float)foc_left.set_angle);
        data_send(15, (float)foc_right.set_angle);

        data_send(16, (float)foc_left.Park_in.u_q);
        data_send(17, (float)foc_right.Park_in.u_q);
        // data_send(18, speed_filter.data_average;

        // data_send(25, adc_convert(ADC1_CH28_P15_0) - adc_convert(ADC1_CH31_P15_3);
        // data_send(26, adc_convert(ADC1_CH29_P15_1) - adc_convert(ADC1_CH31_P15_3);
        // data_send(27, adc_convert(ADC1_CH30_P15_2) - adc_convert(ADC1_CH31_P15_3);
        // data_send(28, adc_convert(ADC1_CH31_P15_3);

        data_send(26, adc_abmf_value);
        data_send(27, adc_bbmf_value);
        data_send(28, adc_cbmf_value);
        data_send(29, adc_global_value);

        data_send(30, (float)adc_test_mid[0]);
        data_send(31, (float)adc_test_mid[1]);
        data_send(32, (float)adc_test_mid[2]);
        data_send(33, (adc_test_mid[0] + adc_test_mid[1] + adc_test_mid[2] - 2048 * 3));

        data_send_clear();
        // send_vofaplus();
    }
}

// **************************** 代码区域 ****************************
