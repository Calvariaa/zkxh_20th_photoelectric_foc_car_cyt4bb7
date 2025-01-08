#include "car_control/gyro.h"
#include "car_control/pid.h"
#include <fastmath/cos_sin.h>
#include <math.h>

#define delta_T 1e-5f // 100us计算一次

float I_ex, I_ey, I_ez; // 误差积分

quater_param_t Q_info = {1, 0, 0}; // 全局四元数
euler_param_t eulerAngle;          // 欧拉角

imu_param_t imu_data;
imu_offset_param_t gyro_offset;

bool gyro_offset_init_flag = false;

float fast_sqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

/* maximum relative error about 3.6e-5 */
float fast_atan2f(float y, float x)
{
    float a, r, s, t, c, q, ax, ay, mx, mn;
    ax = fabsf(x);
    ay = fabsf(y);
    mx = fmaxf(ay, ax);
    mn = fminf(ay, ax);
    a = mn / mx;
    /* Minimax polynomial approximation to atan(a) on [0,1] */
    s = a * a;
    c = s * a;
    q = s * s;
    r = 0.024840285f * q + 0.18681418f;
    t = -0.094097948f * q - 0.33213072f;
    r = r * s + t;
    r = r * c + a;
    /* Map to full circle */
    if (ay > ax)
        r = 1.57079637f - r;
    if (x < 0)
        r = 3.14159274f - r;
    if (y < 0)
        r = -r;
    return r;
}

float fast_asin(double x)
{
    return atan2(x, fast_cos(x));
}

void gyro_init()
{
    if (imu_init())
    {
        ips200_show_string(0, 0, "gyro init failed");
        system_delay_ms(1000);
    }
}

/////////陀螺仪零飘初始化, 放中断
void gyro_offset_init()
{
    if (gyro_offset_init_flag)
        return;
    static int16_t i = 0;

    if (i == 0)
    {
        gyro_offset.Xdata = 0;
        gyro_offset.Ydata = 0;
        gyro_offset.Zdata = 0;
    }

    imu_get_acc();
    imu_get_gyro();

    gyro_offset.Xdata += (float)imu_gyro_x;
    gyro_offset.Ydata += (float)imu_gyro_y;
    gyro_offset.Zdata += (float)imu_gyro_z;

    i++;

    if (i >= 1000)
    {

        gyro_offset.Xdata /= 1000;
        gyro_offset.Ydata /= 1000;
        gyro_offset.Zdata /= 1000;

        gyro_offset_init_flag = true;
    }
}
void gyro_offset_static_init()
{
    if (gyro_offset_init_flag)
        return;

    gyro_offset.Xdata = 0.87;
    gyro_offset.Ydata = -3.53;
    gyro_offset.Zdata = -4.11;

    gyro_offset_init_flag = true;
}

#define alpha 0.3f

// 转化为实际物理值
void gyro_get_values()
{
    if (!gyro_offset_init_flag)
        return;

    imu_get_acc();
    imu_get_gyro();

    // 一阶低通滤波，单位g/s
    imu_data.acc_x = (((float)imu_acc_x) * alpha) * 8 / 4096 + imu_data.acc_x * (1 - alpha);
    imu_data.acc_y = (((float)imu_acc_y) * alpha) * 8 / 4096 + imu_data.acc_y * (1 - alpha);
    imu_data.acc_z = (((float)imu_acc_z) * alpha) * 8 / 4096 + imu_data.acc_z * (1 - alpha);

    // 陀螺仪角度转弧度
    imu_data.gyro_x = ANGLE_TO_RAD(imu_gyro_transition((float)imu_gyro_x - gyro_offset.Xdata));
    imu_data.gyro_y = ANGLE_TO_RAD(imu_gyro_transition((float)imu_gyro_y - gyro_offset.Ydata));
    imu_data.gyro_z = ANGLE_TO_RAD(imu_gyro_transition((float)imu_gyro_z - gyro_offset.Zdata));

// 磁力计数据转换为实际物理值
#ifdef USE_IMU963RA
    imu_data.mag_x = ((float)imu_mag_x) / 3000;
    imu_data.mag_y = ((float)imu_mag_y) / 3000;
    imu_data.mag_z = ((float)imu_mag_z) / 3000;
#endif
}

// 互补滤波
void gyro_update_AHRS(float gx, float gy, float gz, float ax, float ay, float az)
{
    float halfT = 0.5 * delta_T;
    float vx, vy, vz; // 当前的机体坐标系上的重力单位向量
    float ex, ey, ez; // 四元数计算值与加速度计测量值的误差
    float q0 = Q_info.q0;
    float q1 = Q_info.q1;
    float q2 = Q_info.q2;
    float q3 = Q_info.q3;
    float q0q0 = q0 * q0;
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;
    float delta_2 = 0;

    // 对加速度数据进行归一化 得到单位加速度
    float norm = fast_sqrt(ax * ax + ay * ay + az * az);
    ax = ax * norm;
    ay = ay * norm;
    az = az * norm;

    // 根据当前四元数的姿态值来估算出各重力分量。用于和加速计实际测量出来的各重力分量进行对比，从而实现对四轴姿态的修正
    vx = 2 * (q1q3 - q0q2);
    vy = 2 * (q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;
    // vz = (q0*q0-0.5f+q3 * q3) * 2;

    // 叉积来计算估算的重力和实际测量的重力这两个重力向量之间的误差。
    ex = ay * vz - az * vy;
    ey = az * vx - ax * vz;
    ez = ax * vy - ay * vx;

    // 用叉乘误差来做PI修正陀螺零偏，
    // 通过调节 gyro_euler.kp，gyro_euler.ki 两个参数，
    // 可以控制加速度计修正陀螺仪积分姿态的速度。
    I_ex += halfT * ex; // integral error scaled by Ki
    I_ey += halfT * ey;
    I_ez += halfT * ez;

    gx = gx + gyro_euler.kp * ex + gyro_euler.ki * I_ex;
    gy = gy + gyro_euler.kp * ey + gyro_euler.ki * I_ey;
    gz = gz + gyro_euler.kp * ez + gyro_euler.ki * I_ez;

    /*数据修正完成，下面是四元数微分方程*/

    // 四元数微分方程，其中halfT为测量周期的1/2，gx gy gz为陀螺仪角速度，以下都是已知量，这里使用了一阶龙哥库塔求解四元数微分方程
    q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * halfT;
    q1 = q1 + (q0 * gx + q2 * gz - q3 * gy) * halfT;
    q2 = q2 + (q0 * gy - q1 * gz + q3 * gx) * halfT;
    q3 = q3 + (q0 * gz + q1 * gy - q2 * gx) * halfT;

    // 整合四元数率    四元数微分方程  四元数更新算法，二阶毕卡法
    // delta_2 = (2 * halfT * gx) * (2 * halfT * gx) + (2 * halfT * gy) * (2 * halfT * gy) + (2 * halfT * gz) * (2 * halfT * gz);
    // q0 = (1 - delta_2 / 8) * q0 + (-q1 * gx - q2 * gy - q3 * gz) * halfT;
    // q1 = (1 - delta_2 / 8) * q1 + (q0 * gx + q2 * gz - q3 * gy) * halfT;
    // q2 = (1 - delta_2 / 8) * q2 + (q0 * gy - q1 * gz + q3 * gx) * halfT;
    // q3 = (1 - delta_2 / 8) * q3 + (q0 * gz + q1 * gy - q2 * gx) * halfT;

    // normalise quaternion
    norm = fast_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    Q_info.q0 = q0 * norm;
    Q_info.q1 = q1 * norm;
    Q_info.q2 = q2 * norm;
    Q_info.q3 = q3 * norm;
}

/*把四元数转换成欧拉角*/
void gyro_get_euler_angles()
{
    if (!gyro_offset_init_flag)
        return;

    // 采集陀螺仪数据
    imu_get_gyro();
    imu_get_acc();
    // imu_get_mag();

    gyro_get_values();
    gyro_update_AHRS(imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z, imu_data.acc_x, imu_data.acc_y, imu_data.acc_z);
    // gyro_update_AHRS_mag(imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z, imu_data.acc_x, imu_data.acc_y, imu_data.acc_z, imu_data.mag_x, imu_data.mag_y, imu_data.mag_z);
    float q0 = Q_info.q0;
    float q1 = Q_info.q1;
    float q2 = Q_info.q2;
    float q3 = Q_info.q3;

    // 四元数计算欧拉角
    eulerAngle.pitch = fast_asin(-2 * q1 * q3 + 2 * q0 * q2) * _180_PI;                                 // pitch
    eulerAngle.roll = fast_atan2f(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * _180_PI; // roll
    eulerAngle.yaw = fast_atan2f(2 * q1 * q2 + 2 * q0 * q3, -2 * q2 * q2 - 2 * q3 * q3 + 1) * _180_PI;  // yaw

    // eulerAngle.pitch = (fast_atan2f(2 * q2 * q3 + 2 * q0 * q1, q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3)) * _180_PI; //-180~180
    // eulerAngle.roll = (fast_asin(2 * q0 * q2 - 2 * q1 * q3)) * _180_PI;                                          //-90~90
    // eulerAngle.yaw = (fast_atan2f(2 * q1 * q2 + 2 * q0 * q3, q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3)) * _180_PI;   // 0~360

    // eulerAngle.pitch = asin(-2 * q1 * q3 + 2 * q0 * q2) * 180 / _PI;                                 // pitch
    // eulerAngle.roll = atan2f(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 180 / _PI; // roll
    // eulerAngle.yaw = atan2f(2 * q1 * q2 + 2 * q0 * q3, -2 * q2 * q2 - 2 * q3 * q3 + 1) * 180 / _PI;  // yaw

    /*   姿态限制
    if (eulerAngle.roll > 90 || eulerAngle.roll < -90)
    {
        if (eulerAngle.pitch > 0)
        {
            eulerAngle.pitch = 180 - eulerAngle.pitch;
        }
        if (eulerAngle.pitch < 0)
        {
            eulerAngle.pitch = -(180 + eulerAngle.pitch);
        }
    }

    if (eulerAngle.yaw > 360)
    {
        eulerAngle.yaw -= 360;
    }
    else if (eulerAngle.yaw < 0)
    {
        eulerAngle.yaw += 360;
    }
    */
}
