#include "IMU.h"

// ================================================================
//  全局变量
// ================================================================
volatile float yaw_angle = 0.0f;   // 累积偏航角（度）
volatile float yaw_rate  = 0.0f;   // 当前角速度（度/秒）

static float s_gyro_z_offset = 0.0f;  // 陀螺仪 Z 轴零偏（标定值）
static float s_yaw_rate_filtered = 0.0f;  // 滤波后的角速度

#define GYRO_Z_DEADBAND  0.15f  // 角速度死区（度/秒），小于此值归零
#define YAW_CALIB_COUNT  200    // 零偏标定采样次数（200次 * 5ms ≈ 1秒）
#define YAW_CALIB_VAR_MAX 0.1f  // 标定方差上限，超过说明有抖动需重来
#define YAW_RATE_FILTER  0.8f   // 低通滤波系数 (0~1)，值越大越平滑

// ================================================================
//  函数接口：IMU 初始化 + 零偏标定
// ================================================================
void imu_init(void)
{
    // 初始化 IMU660RC（禁用四元数模式，使用原始数据模式）
    while (1)
    {
        if (imu660rc_init(IMU660RC_QUARTERNION_DISABLE))
            printf("\r\n IMU660RC init error.");
        else
            break;
    }

    // 零偏标定：静止状态采集，带方差检测
    // 此时小车必须静止不动！
    for (int retry = 0; retry < 3; retry++)
    {
        float sum = 0.0f, sum_sq = 0.0f;
        for (int i = 0; i < YAW_CALIB_COUNT; i++)
        {
            imu660rc_get_gyro();
            float val = -imu660rc_gyro_transition(imu660rc_gyro_z);
            sum    += val;
            sum_sq += val * val;
            system_delay_ms(5);
        }
        float mean     = sum / (float)YAW_CALIB_COUNT;
        float variance = sum_sq / (float)YAW_CALIB_COUNT - mean * mean;

        if (variance < YAW_CALIB_VAR_MAX)
        {
            s_gyro_z_offset = mean;
            break;
        }
        // 方差过大，抖动严重，重试
        if (retry == 2)
            s_gyro_z_offset = mean; // 第3次强制用当前值
    }

    // 清零
    yaw_angle = 0.0f;
    yaw_rate  = 0.0f;

    // 配置 5ms PIT 中断（CCU60_CH1），在 isr.c 中调用 imu_update()
    pit_init(CCU60_CH1, 5000);
}

// ================================================================
//  函数接口：Yaw 数据更新（在 5ms PIT 中断中调用）
// ================================================================
void imu_update(void)
{
    // 读取陀螺仪原始数据
    imu660rc_get_gyro();

    // 转换为度/秒，Z轴取反（芯片朝下安装）
    float raw_rate = -imu660rc_gyro_transition(imu660rc_gyro_z);

    // 减去零偏
    raw_rate -= s_gyro_z_offset;

    // 死区处理：角速度太小认为是噪声
    if (raw_rate > -GYRO_Z_DEADBAND && raw_rate < GYRO_Z_DEADBAND)
        raw_rate = 0.0f;

    // 低通滤波
    s_yaw_rate_filtered = s_yaw_rate_filtered * YAW_RATE_FILTER + raw_rate * (1.0f - YAW_RATE_FILTER);
    yaw_rate = s_yaw_rate_filtered;

    // 累积角度：角速度 * 时间（5ms = 0.005s）
    yaw_angle += yaw_rate * 0.005f;

    // 归一化到 [-180, 180]
    if (yaw_angle > 180.0f)
        yaw_angle -= 360.0f;
    else if (yaw_angle < -180.0f)
        yaw_angle += 360.0f;
}

// ================================================================
//  函数接口：清零 yaw_angle
// ================================================================
void imu_reset_yaw(void)
{
    yaw_angle = 0.0f;
}
