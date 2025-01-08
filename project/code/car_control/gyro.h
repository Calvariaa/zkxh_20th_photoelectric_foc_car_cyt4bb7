#ifndef __GYRO_H
#define __GYRO_H

#include "zf_common_typedef.h"

#ifndef _PI
  #define _PI               3.14159265358979f
#endif
#define _180_PI 57.29577951308232
#define _PI_180 0.017453292519943

#ifndef ANGLE_TO_RAD
#define ANGLE_TO_RAD(x) ((x) * _PI_180) // ???????
#endif
#ifndef RAD_TO_ANGLE
#define RAD_TO_ANGLE(x) ((x) * _180_PI) // ???????
#endif

#define USE_IMU660RA
// #define USE_MPU6050
// #define USE_IMU963RA

#define PRINT_MACRO(x) "#### " #x " IS DEFINED ####"

#ifdef USE_IMU660RA
#include "zf_device_imu660ra.h"
#pragma message(PRINT_MACRO(USE_IMU660RA))
#define imu_init() imu660ra_init()
#define imu_get_acc() imu660ra_get_acc()
#define imu_get_gyro() imu660ra_get_gyro()
#define imu_gyro_transition(gyro_value) imu660ra_gyro_transition(gyro_value)
#define imu_gyro_x imu660ra_gyro_x
#define imu_gyro_y imu660ra_gyro_y
#define imu_gyro_z imu660ra_gyro_z
#define imu_acc_x imu660ra_acc_x
#define imu_acc_y imu660ra_acc_y
#define imu_acc_z imu660ra_acc_z

#elif defined USE_MPU6050
#include "zf_device_mpu6050.h"
#pragma message(PRINT_MACRO(USE_MPU6050))
#define imu_init() mpu6050_init()
#define imu_get_acc() mpu6050_get_acc()
#define imu_get_gyro() mpu6050_get_gyro()
#define imu_gyro_transition(gyro_value) mpu6050_gyro_transition(gyro_value)
#define imu_gyro_x mpu6050_gyro_x
#define imu_gyro_y mpu6050_gyro_y
#define imu_gyro_z mpu6050_gyro_z
#define imu_acc_x mpu6050_acc_x
#define imu_acc_y mpu6050_acc_y
#define imu_acc_z mpu6050_acc_z

#elif defined USE_IMU963RA
#include "zf_device_imu963ra.h"
#pragma message(PRINT_MACRO(USE_IMU963RA))
#define imu_init() imu963ra_init()
#define imu_get_acc() imu963ra_get_acc()
#define imu_get_gyro() imu963ra_get_gyro()
#define imu_gyro_transition(gyro_value) imu963ra_gyro_transition(gyro_value)
#define imu_gyro_x imu963ra_gyro_x
#define imu_gyro_y imu963ra_gyro_y
#define imu_gyro_z imu963ra_gyro_z
#define imu_acc_x imu963ra_acc_x
#define imu_acc_y imu963ra_acc_y
#define imu_acc_z imu963ra_acc_z

#else
#error "???????define??"
#endif

typedef struct
{
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float acc_x;
    float acc_y;
    float acc_z;
    float mag_x;
    float mag_y;
    float mag_z;
} imu_param_t;

typedef struct
{
    float q0;
    float q1;
    float q2;
    float q3;
} quater_param_t;

typedef struct
{
    float pitch; // ??pitch
    float roll;  // ??roll
    float yaw;   // ?????????
} euler_param_t;

typedef struct
{
    float Xdata;
    float Ydata;
    float Zdata;
} imu_offset_param_t;

extern imu_param_t imu_data;
extern euler_param_t eulerAngle;
extern imu_offset_param_t gyro_offset;

void gyro_init();
void gyro_offset_init();
void gyro_update_AHRS(float gx, float gy, float gz, float ax, float ay, float az);
void gyro_get_values();
void gyro_get_euler_angles();

#endif /* __GYRO_H */