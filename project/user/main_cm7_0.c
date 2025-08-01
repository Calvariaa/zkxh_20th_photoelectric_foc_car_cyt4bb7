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
 * 文件名称          main_cm7_0
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
#include "debug/vofaplus.h"
#include "car_control/gyro.h"
#include "car_control/adc.h"
#include "car_control/turn_error.h"

#pragma location = 0x28026024
float memory_from_cm70_to_cm71[7];
// foc开关，左轮速度，右轮速度，涵道开关，涵道速度，高频注入频率

#define FOC_LEFT_START memory_from_cm70_to_cm71[0]  // foc左开关
#define FOC_LEFT_SPEED memory_from_cm70_to_cm71[1]  // 左轮速度
#define FOC_RIGHT_START memory_from_cm70_to_cm71[2] // foc右开关
#define FOC_RIGHT_SPEED memory_from_cm70_to_cm71[3] // 右轮速度
#define BLDC_START memory_from_cm70_to_cm71[4]      // 涵道开关
#define BLDC_SPEED memory_from_cm70_to_cm71[5]      // 涵道速度
#define UQ_FREQ memory_from_cm70_to_cm71[6]         // 高频注入频率

#pragma location = 0x28026024 + sizeof(memory_from_cm70_to_cm71)
__no_init float memory_from_cm71_to_cm70[4];

int main(void)
{
    clock_init(SYSTEM_CLOCK_250M); // 时钟配置及系统初始化<务必保留>
    debug_info_init();             // 调试串口信息初始化
                                   // 此处编写用户代码 例如外设初始化代码等

    // ips114_init();

    interrupt_global_disable();

    gyro_init();

    key_init(1);

    tube_adc_init();

    pit_us_init(PIT_CH1, 200); // 0.2ms
    pit_ms_init(PIT_CH2, 1);   // 1ms

    interrupt_global_enable(0);
    // ips114_clear();

    bool start_foc = false;
    bool start_bldc = false;
    while (true)
    {
        data_send(1, (float)imu_data.gyro_z);
        data_send(2, (float)turn_error);
        // for (uint8_t i = 0; i < 20; i++)
        //     data_send(i + 1, (float)adc_tube_read_raw[i]);

        data_send_clear();
        UQ_FREQ = 0; // 高频注入频率

        if (key_get_state(KEY_LEFT) == KEY_SHORT_PRESS)
        {
            key_clear_state(KEY_LEFT);
            start_foc = false;
        }
        if (key_get_state(KEY_RIGHT) == KEY_SHORT_PRESS)
        {
            key_clear_state(KEY_RIGHT);
            start_foc = true;
        }
        if (key_get_state(KEY_UP) == KEY_SHORT_PRESS)
        {
            key_clear_state(KEY_UP);
            start_bldc = false;
        }
        if (key_get_state(KEY_DOWN) == KEY_SHORT_PRESS)
        {
            key_clear_state(KEY_DOWN);
            start_bldc = true;
        }

        if (start_foc)
        {
            FOC_LEFT_START = 1;                                                      // foc左开关
            FOC_LEFT_SPEED = imu_data.gyro_z * 0.0002 + (float)turn_error * -0.003;  // 左轮速度
            FOC_RIGHT_START = 1;                                                     // foc右开关
            FOC_RIGHT_SPEED = imu_data.gyro_z * -0.0002 + (float)turn_error * 0.003; // 右轮速度
        }
        else
        {
            FOC_LEFT_START = 1;  // foc左开关
            FOC_LEFT_SPEED = 0;   // 左轮速度
            FOC_RIGHT_START = 1; // foc右开关
            FOC_RIGHT_SPEED = 0;  // 右轮速度
        }


        if (start_bldc)
        {

            BLDC_START = 1;    // 涵道开关
            BLDC_SPEED = 3000; // 涵道速度
        }
        else
        {
            BLDC_START = -1; // 涵道开关
            BLDC_SPEED = 0;  // 涵道速度
        }

        SCB_CleanInvalidateDCache_by_Addr(&memory_from_cm70_to_cm71, sizeof(memory_from_cm70_to_cm71));
        SCB_CleanInvalidateDCache_by_Addr(&memory_from_cm71_to_cm70, sizeof(memory_from_cm71_to_cm70));
        // read some shit

        // ips114_show_float(0, 0, imu_data.gyro_x, 8, 4);
        // ips114_show_float(0, 16, imu_data.gyro_y, 8, 4);
        // ips114_show_float(0, 32, imu_data.gyro_z, 8, 4);
    }
}

// **************************** 代码区域 ****************************
