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
* 文件名称          zf_driver_soft_spi
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

#ifndef _zf_driver_soft_spi_h_
#define _zf_driver_soft_spi_h_

#include "zf_common_typedef.h"
#include "zf_driver_gpio.h"

#define SOFT_SPI_PIN_NULL       (0xFFFF)                                // 用于区分是否分配引脚


typedef struct //枚举模块号
{
    uint8       mode        :6;                                         // SPI 模式
    uint8       use_miso    :1;                                         // 是否使用 MISO 引脚
    uint8       use_cs      :1;                                         // 是否使用 CS 引脚
}spi_config_info_struct;

typedef struct
{
    spi_config_info_struct  config;                                     // 配置整体数据
    gpio_pin_enum           sck_pin;                                    // 用于记录对应的引脚编号
    gpio_pin_enum           mosi_pin;                                   // 用于记录对应的引脚编号
    gpio_pin_enum           miso_pin;                                   // 用于记录对应的引脚编号
    gpio_pin_enum           cs_pin;                                     // 用于记录对应的引脚编号
    uint32                  delay;                                      // 模拟 SPI 软延时时长
}soft_spi_info_struct;

//==================================================SOFT_SPI 基础函数====================================================
void        soft_spi_write_8bit                 (soft_spi_info_struct *soft_spi_obj, const uint8 data);
void        soft_spi_write_8bit_array           (soft_spi_info_struct *soft_spi_obj, const uint8 *data, uint32 len);

void        soft_spi_write_16bit                (soft_spi_info_struct *soft_spi_obj, const uint16 data);
void        soft_spi_write_16bit_array          (soft_spi_info_struct *soft_spi_obj, const uint16 *data, uint32 len);

void        soft_spi_write_8bit_register        (soft_spi_info_struct *soft_spi_obj, const uint8 register_name, const uint8 data);
void        soft_spi_write_8bit_registers       (soft_spi_info_struct *soft_spi_obj, const uint8 register_name, const uint8 *data, uint32 len);

void        soft_spi_write_16bit_register       (soft_spi_info_struct *soft_spi_obj, const uint16 register_name, const uint16 data);
void        soft_spi_write_16bit_registers      (soft_spi_info_struct *soft_spi_obj, const uint16 register_name, const uint16 *data, uint32 len);

uint8       soft_spi_read_8bit                  (soft_spi_info_struct *soft_spi_obj);
void        soft_spi_read_8bit_array            (soft_spi_info_struct *soft_spi_obj, uint8 *data, uint32 len);

uint16      soft_spi_read_16bit                 (soft_spi_info_struct *soft_spi_obj);
void        soft__array           (soft_spi_info_struct *soft_spi_obj, uint16 *data, uint32 len);

uint8       soft_spi_read_8bit_register         (soft_spi_info_struct *soft_spi_obj, const uint8 register_name);
void        soft_spi_read_8bit_registers        (soft_spi_info_struct *soft_spi_obj, const uint8 register_name, uint8 *data, uint32 len);

uint16      soft_spi_read_16bit_register        (soft_spi_info_struct *soft_spi_obj, const uint16 register_name);
void        soft_spi_read_16bit_registers       (soft_spi_info_struct *soft_spi_obj, const uint16 register_name, uint16 *data, uint32 len);

void        soft_spi_transfer_8bit              (soft_spi_info_struct *soft_spi_obj, const uint8 *write_buffer, uint8 *read_buffer, uint32 len);
void        soft_spi_transfer_16bit             (soft_spi_info_struct *soft_spi_obj, const uint16 *write_buffer, uint16 *read_buffer, uint32 len);

void        soft_spi_init                       (soft_spi_info_struct *soft_spi_obj, uint8 mode, uint32 delay, gpio_pin_enum sck_pin, gpio_pin_enum mosi_pin, uint32 miso_pin, uint32 cs_pin);
//==================================================SOFT_SPI 基础函数====================================================

#endif

