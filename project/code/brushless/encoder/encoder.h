#ifndef _ENCODER_H
#define _ENCODER_H

#include "brushless/foc.h"
#include "zf_common_typedef.h"
#include "zf_driver_gpio.h"
#include "brushless/encoder/as5047p.h"
#include "brushless/encoder/mt6701ct.h"

//====================================================硬件 SPI 驱动====================================================
#define ENC_SPI_SPEED (10 * 1000 * 1000) // 硬件 SPI 速率
#define SPI_FREQ CY_INITIAL_TARGET_PERI_FREQ // 串口模块时钟 默认80M

/* Master Settings */
#define SCB_MISO_DRIVE_MODE CY_GPIO_DM_HIGHZ
#define SCB_MOSI_DRIVE_MODE CY_GPIO_DM_STRONG_IN_OFF
#define SCB_CLK_DRIVE_MODE CY_GPIO_DM_STRONG_IN_OFF
#define SCB_SEL0_DRIVE_MODE CY_GPIO_DM_STRONG_IN_OFF

// custom config

// // L
// #define L_SPI_TYPE (SCB7)
// #define L_SPI_CLOCK (PCLK_SCB7_CLOCK)

// #define L_CLK_HSIOM (P2_2_SCB7_SPI_CLK)
// #define L_CLK_GPIO (P02_2)

// #define L_MOSI_HSIOM (P2_1_SCB7_SPI_MOSI)
// #define L_MOSI_GPIO (P02_1)

// #define L_MISO_HSIOM (P2_0_SCB7_SPI_MISO)
// #define L_MISO_GPIO (P02_0)

// #define L_CS_HSIOM (NULL)
// #define L_CS_GPIO (P02_3)

// // R
// #define R_SPI_TYPE (SCB7)
// #define R_SPI_CLOCK (PCLK_SCB7_CLOCK)

// #define R_CLK_HSIOM (P2_2_SCB7_SPI_CLK)
// #define R_CLK_GPIO (P02_2)

// #define R_MOSI_HSIOM (P2_1_SCB7_SPI_MOSI)
// #define R_MOSI_GPIO (P02_1)

// #define R_MISO_HSIOM (P2_0_SCB7_SPI_MISO)
// #define R_MISO_GPIO (P02_0)

// #define R_CS_HSIOM (NULL)
// #define R_CS_GPIO (P02_3)

// L
#define L_SPI_TYPE (SCB3)
#define L_SPI_CLOCK (PCLK_SCB3_CLOCK)

#define L_CLK_HSIOM (P18_3_SCB3_SPI_CLK)
#define L_CLK_GPIO (P18_3)

#define L_MOSI_HSIOM (P18_2_SCB3_SPI_MOSI)
#define L_MOSI_GPIO (P18_2)

#define L_MISO_HSIOM (P18_1_SCB3_SPI_MISO)
#define L_MISO_GPIO (P18_1)

#define L_CS_HSIOM (NULL)
#define L_CS_GPIO (P18_4)

// R
#define R_SPI_TYPE (SCB4)
#define R_SPI_CLOCK (PCLK_SCB4_CLOCK)

#define R_CLK_HSIOM (P6_2_SCB4_SPI_CLK)
#define R_CLK_GPIO (P06_2)

#define R_MOSI_HSIOM (P6_1_SCB4_SPI_MOSI)
#define R_MOSI_GPIO (P06_1)

#define R_MISO_HSIOM (P6_0_SCB4_SPI_MISO)
#define R_MISO_GPIO (P06_0)

#define R_CS_HSIOM (NULL)
#define R_CS_GPIO (P06_3)
//====================================================硬件 SPI 驱动====================================================

void encoder_init();
double get_magnet_angle(uint16_t val, double zero_angle);
int32_t get_magnet_angle_rot(double reval, encoder_t *_enc);
double get_elec_angle(uint16_t val, double zero_angle);
float _normalizeAngle(float angle);
double get_magnet_speed(double reval, int32_t reval_rot, double reval_last, int32_t reval_rot_last, uint32_t T);

void set_zero_angle(float angle, encoder_t *_enc);
void reset_rotations(encoder_t *_enc);

extern double theta_elec;
extern uint16_t theta_val;

extern double theta_magnet;
extern int32_t full_rotations;

extern double theta_magnet_last;
extern int32_t full_rotations_last;

#endif /* _ENCODER_H */