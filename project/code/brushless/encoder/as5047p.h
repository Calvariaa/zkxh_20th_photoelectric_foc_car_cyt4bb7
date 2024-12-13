#ifndef _AS5047P_H
#define _AS5047P_H

#include "brushless/foc.h"
#include "zf_common_typedef.h"

// --- Volatile registers
#define AS5047P_NOP 0x0000   // 无操作指令
#define AS5047P_ERRFL 0x0001 //
#define AS5047P_PROG 0x0003  // 对芯片进行编程的寄存器地址
#define AS5047P_DIAAGC 0x3FFC
#define AS5047P_MAG 0x3FFD      //
#define AS5047P_ANGLEUNC 0x3FFE //
#define AS5047P_ANGLECOM 0x3FFF // 存放具有动态角度误差补偿角度信息的寄存器地址
// --- Non-volatile registers
#define AS5047P_ZPOSM 0x0016 // 存放磁铁初始位置的高8位地址
#define AS5047P_ZPOSL 0x0017 // 存放磁铁初始位置的低8位地址
#define AS5047P_SETTINGS1 0x0018
#define AS5047P_SETTINGS2 0x0019
// --- Fields in registers
#define AS5047P_ERRFL_PARERR (1 << 2)
#define AS5047P_ERRFL_INVCOMM (1 << 1)
#define AS5047P_ERRFL_FRERR (1 << 0)
#define AS5047P_PROG_PROGVER (1 << 6)
#define AS5047P_PROG_PROGOTP (1 << 3)
#define AS5047P_PROG_OTPREF (1 << 2)
#define AS5047P_PROG_PROGEN (1 << 0)
#define AS5047P_DIAAGC_MAGL (1 << 11)
#define AS5047P_DIAAGC_MAGH (1 << 10)
#define AS5047P_DIAAGC_COF (1 << 9)
#define AS5047P_DIAAGC_LF (1 << 8)
#define AS5047P_DIAAGC_AGC (0x00FF << 0)
#define AS5047P_MAG_CMAG (0x3FFF << 0)
#define AS5047P_ANGLEUNC_CORDICANG (0x3FFF << 0)
#define AS5047P_ANGLECOM_DAECANG (0x3FFF << 0)
#define AS5047P_ZPOSM_ZPOSM (0x00FF << 0)
#define AS5047P_ZPOSL_COMP_H_ERR_EN (1 << 7)
#define AS5047P_ZPOSL_COMP_I_ERR_EN (1 << 6)
#define AS5047P_ZPOSL_ZPOSL (0x003F << 0)
#define AS5047P_SETTINGS1_BIT0 (1 << 0)
#define AS5047P_SETTINGS1_NOISESET (1 << 1)
#define AS5047P_SETTINGS1_DIR (1 << 2)
#define AS5047P_SETTINGS1_UVW_ABI (1 << 3)
#define AS5047P_SETTINGS1_DAECDIS (1 << 4)
#define AS5047P_SETTINGS1_ABIBIN (1 << 5)
#define AS5047P_SETTINGS1_DATASEL (1 << 6)
#define AS5047P_SETTINGS1_PWMON (1 << 7)
#define AS5047P_SETTINGS2_UVWPP (0x0007 << 0)
#define AS5047P_SETTINGS2_HYS (0x0003 << 3)
#define AS5047P_SETTINGS2_ABIRES (0x0007 << 5)
// --- R & W command
#define AS5047P_SPI_R (1 << 14)
#define AS5047P_SPI_W (0) //( 0 << 14)

#ifdef AS5047P_ABI_Part_EN
#define AS5047P_TIM (TIM4_ENCODER)
#define AS5047P_B (TIM4_ENCODER_CH1_P02_8) // 接模块B
#define AS5047P_A (TIM4_ENCODER_CH2_P00_9) // 接模块A
#endif

// void as5047p_printf_test(void);
void as5047p_init();
uint16_t as5047p_get_magnet_val_left();
uint16_t as5047p_get_magnet_val_right();
#endif /*_AS5047P_H*/
