#include "IMU.h"  /* 包含IMU模块头文件，提供IMU660RC驱动接口和模块内部声明 */

/*
 * IMU偏航角积分模块
 *
 * 传感器：IMU660RC，通过SPI0总线通信，使用原始陀螺仪数据
 * 输出约定：
 *   yaw_angle > 0：左转（逆时针偏航角增大）
 *   yaw_angle < 0：右转（顺时针偏航角减小）
 *
 * 本模块仅负责测量偏航角，不直接驱动电机。
 * 偏航角通过陀螺仪Z轴角速度积分得到，用于直角弯HARD阶段的角度退出判断。
 */

volatile float yaw_angle = 0.0f;          /* 当前偏航角（度），正值=左转，负值=右转，范围[-180, 180] */
volatile float yaw_rate  = 0.0f;          /* 当前滤波后的偏航角速度（度/秒），用于调试和显示 */
volatile uint8 imu_ready = 0u;            /* IMU就绪标志：0=未初始化完成，1=初始化成功可使用 */
volatile uint8 imu_error = 1u;            /* IMU错误标志：0=校准正常，1=校准失败（方差过大） */
volatile int16 imu_offset_dps10 = 0;      /* 陀螺仪零偏值（单位：0.1度/秒），用于TFT显示调试 */

/* ==================== 时间常量定义 ==================== */
#define IMU_UPDATE_PERIOD_US     5000u    /* IMU更新周期（微秒），即5ms，对应PIT中断周期 */
#define IMU_UPDATE_DT_SEC        0.005f   /* IMU更新周期（秒），5ms=0.005s，用于角度积分 */

/* ==================== 初始化与校准参数 ==================== */
#define IMU_INIT_RETRY           3u       /* IMU硬件初始化最大重试次数 */
#define IMU_CALIB_RETRY          3u       /* 零偏校准最大重试次数，每次取方差最小的结果 */
#define IMU_CALIB_DISCARD        20u      /* 校准开始前丢弃的采样数（让传感器稳定） */
#define IMU_CALIB_SAMPLES        180u     /* 校准采样数（180*5ms=900ms，约1秒） */
#define IMU_CALIB_DELAY_MS       5u       /* 校准采样间隔（毫秒），与IMU_UPDATE_PERIOD一致 */

/* ==================== 信号处理参数 ==================== */
#define IMU_Z_SIGN               (-1.0f)  /* Z轴符号翻转系数：-1表示安装方向导致需要取反 */
#define IMU_RATE_DEADBAND_DPS    0.35f    /* 角速度死区阈值（度/秒），低于此值视为静止噪声归零 */
#define IMU_RATE_FILTER_ALPHA    0.65f    /* 一阶低通滤波器系数（0~1），越大越平滑但响应越慢 */
#define IMU_RATE_LIMIT_DPS       1200.0f  /* 角速度限幅值（度/秒），防止异常大值导致积分跳变 */
#define IMU_CALIB_VAR_MAX        6.0f     /* 校准方差上限（(度/秒)^2），超过此值认为校准期间车在动 */
#define IMU_RESET_SKIP_SAMPLES   2u       /* 重置偏航角后跳过的IMU采样数，防止重置后立即被积分覆盖 */

/* ==================== 模块内部静态变量 ==================== */
static float s_gyro_z_offset = 0.0f;       /* 陀螺仪Z轴零偏值（度/秒），校准时测量并存储 */
static float s_yaw_rate_filtered = 0.0f;    /* 滤波后的偏航角速度（度/秒），一阶低通滤波输出 */
static uint8 s_reset_skip = 0u;             /* 重置跳过计数器：非0时跳过积分，防止重置后立即被覆盖 */

/**
 * imu_absf() - 计算浮点数的绝对值
 *
 * @v  输入浮点数
 * 返回值：v的绝对值（非负浮点数）
 */
static float imu_absf(float v)
{
    return (v >= 0.0f) ? v : -v;  /* 非负直接返回，负数取反后返回 */
}

/**
 * imu_clampf() - 将浮点数限制在[min_v, max_v]范围内
 *
 * @v      输入浮点数
 * @min_v  下限值
 * @max_v  上限值
 * 返回值：限幅后的浮点数
 */
static float imu_clampf(float v, float min_v, float max_v)
{
    if (v < min_v) return min_v;  /* 低于下限，返回下限值 */
    if (v > max_v) return max_v;  /* 高于上限，返回上限值 */
    return v;                     /* 在范围内，原样返回 */
}

/**
 * imu_read_z_rate_dps() - 读取IMU陀螺仪Z轴角速度（度/秒）
 *
 * 通过SPI读取IMU660RC陀螺仪原始数据，转换为度/秒单位。
 * 应用IMU_Z_SIGN符号翻转以匹配安装方向。
 *
 * 返回值：Z轴角速度（度/秒），已应用符号翻转
 */
static float imu_read_z_rate_dps(void)
{
    imu660rc_get_gyro();                                        /* 通过SPI读取IMU660RC陀螺仪原始数据到全局缓冲区 */
    return IMU_Z_SIGN * imu660rc_gyro_transition(imu660rc_gyro_z);  /* 将原始数据转换为度/秒并应用符号翻转 */
}

/**
 * imu_clear_state() - 清除IMU模块的运行时状态
 *
 * 将偏航角、角速度、滤波器状态全部归零，并设置重置跳过标志。
 * 在初始化和重置偏航角时调用。
 */
static void imu_clear_state(void)
{
    yaw_angle = 0.0f;              /* 偏航角归零 */
    yaw_rate = 0.0f;               /* 偏航角速度归零 */
    s_yaw_rate_filtered = 0.0f;    /* 滤波器输出归零 */
    s_reset_skip = IMU_RESET_SKIP_SAMPLES;  /* 设置跳过标志，防止下一个IMU周期立即积分覆盖 */
}

/**
 * imu_normalize_yaw() - 将偏航角归一化到[-180, 180]度范围内
 *
 * 偏航角积分后可能超出[-180, 180]范围，需要归一化。
 * 使用简单的加减360度方法实现周期性归一化。
 */
static void imu_normalize_yaw(void)
{
    if (yaw_angle > 180.0f)         /* 偏航角超过+180度 */
        yaw_angle -= 360.0f;        /* 减去360度，回到-180附近 */
    else if (yaw_angle < -180.0f)   /* 偏航角低于-180度 */
        yaw_angle += 360.0f;        /* 加上360度，回到+180附近 */
}

/**
 * imu_calibrate_zero() - 陀螺仪Z轴零偏校准
 *
 * 在车辆静止状态下，多次采样陀螺仪Z轴角速度，计算均值作为零偏。
 * 使用多次重试机制，每次重试先丢弃DISCARD个不稳定采样，
 * 再采集SAMPLES个采样计算均值和方差。选择方差最小的一次作为最终结果。
 *
 * 校准成功条件：最佳方差 <= IMU_CALIB_VAR_MAX（6.0 (dps)^2）
 * 即使校准失败（方差过大），仍使用最佳偏移值，但返回错误标志。
 *
 * 返回值：0=校准成功（方差在允许范围内），1=校准失败（方差过大）
 */
static uint8 imu_calibrate_zero(void)
{
    float best_offset = 0.0f;        /* 最佳零偏值（所有重试中方差最小的均值） */
    float best_var = 1000000.0f;     /* 最佳方差（初始设为极大值，用于比较） */
    uint8 best_valid = 0u;           /* 是否找到过有效的校准结果：0=无，1=有 */

    for (uint8 retry = 0u; retry < IMU_CALIB_RETRY; retry++)  /* 多次重试校准，取方差最小的结果 */
    {
        float sum = 0.0f;       /* 采样值累加和，用于计算均值 */
        float sum_sq = 0.0f;    /* 采样值平方累加和，用于计算方差 */

        /* 丢弃前IMU_CALIB_DISCARD个采样，让传感器在当前状态下稳定 */
        for (uint16 i = 0u; i < IMU_CALIB_DISCARD; i++)  /* 丢弃不稳定采样 */
        {
            (void)imu_read_z_rate_dps();          /* 读取并丢弃，不使用结果 */
            system_delay_ms(IMU_CALIB_DELAY_MS);  /* 等待一个采样周期 */
        }

        /* 采集IMU_CALIB_SAMPLES个采样，累加值和平方值 */
        for (uint16 i = 0u; i < IMU_CALIB_SAMPLES; i++)  /* 正式采集校准数据 */
        {
            float v = imu_read_z_rate_dps();      /* 读取当前角速度（度/秒） */
            sum += v;                              /* 累加角速度值 */
            sum_sq += v * v;                       /* 累加角速度平方值 */
            system_delay_ms(IMU_CALIB_DELAY_MS);  /* 等待一个采样周期 */
        }

        {
            float sample_count = (float)IMU_CALIB_SAMPLES;           /* 采样数转为浮点 */
            float mean = sum / sample_count;                          /* 计算均值 = 总和/采样数 */
            float variance = sum_sq / sample_count - mean * mean;     /* 计算方差 = E(X^2) - E(X)^2 */
            if (variance < 0.0f) variance = 0.0f;                    /* 防止浮点精度误差导致负方差 */

            if (variance < best_var)       /* 本次方差比历史最佳更小 */
            {
                best_var = variance;       /* 更新最佳方差 */
                best_offset = mean;        /* 更新最佳零偏值 */
                best_valid = 1u;           /* 标记至少有一次有效结果 */
            }

            if (variance <= IMU_CALIB_VAR_MAX)  /* 方差在允许范围内，校准质量足够好 */
                break;                           /* 提前退出，无需继续重试 */
        }
    }

    if (!best_valid)      /* 从未得到有效校准结果（理论上不会发生） */
        return 1u;        /* 返回校准失败 */

    s_gyro_z_offset = best_offset;                              /* 存储最终零偏值到模块静态变量 */
    imu_offset_dps10 = (int16)(best_offset * 10.0f);           /* 转换为0.1度/秒单位存储，供TFT显示 */

    /* 即使校准期间车辆可能移动了，仍使用最佳偏移值，让TFT能立即暴露问题 */
    return (best_var <= IMU_CALIB_VAR_MAX) ? 0u : 1u;          /* 方差在范围内返回0（成功），否则返回1（失败） */
}

/**
 * imu_init() - IMU模块初始化
 *
 * 按以下步骤初始化IMU：
 *   1. 清除模块状态变量
 *   2. 尝试初始化IMU660RC硬件（最多重试3次）
 *   3. 硬件初始化成功后进行零偏校准
 *   4. 启动5ms周期PIT中断，开始周期性角速度积分
 *
 * 即使校准失败（方差过大），仍启用偏航角输出，让TFT能暴露问题。
 */
void imu_init(void)
{
    imu_ready = 0u;                   /* 清除就绪标志，表示初始化尚未完成 */
    imu_error = 1u;                   /* 设置错误标志，默认为有错误 */
    s_gyro_z_offset = 0.0f;          /* 清零陀螺仪零偏值 */
    imu_offset_dps10 = 0;            /* 清零显示用零偏值 */
    imu_clear_state();               /* 清除偏航角、角速度、滤波器等运行时状态 */

    for (uint8 retry = 0u; retry < IMU_INIT_RETRY; retry++)  /* 多次重试硬件初始化 */
    {
        if (imu660rc_init(IMU660RC_QUARTERNION_DISABLE) == 0u)  /* 硬件初始化成功（返回0=成功） */
        {
            system_delay_ms(50);  /* 初始化成功后等待50ms，让传感器完成内部自检 */

            /*
             * 在车辆静止状态下进行零偏校准。如果方差过大（说明校准期间车在动），
             * 仍使用最佳偏移值，保持偏航角输出使能，让TFT能立即暴露问题。
             */
            imu_error = imu_calibrate_zero();  /* 执行零偏校准，结果存入imu_error */
            imu_clear_state();                 /* 校准完成后清除运行时状态，从零开始积分 */
            imu_ready = 1u;                    /* 设置就绪标志，允许imu_update()正常工作 */
            pit_init(CCU60_CH1, IMU_UPDATE_PERIOD_US);  /* 启动CCU60通道1的5ms周期PIT中断 */
            return;                            /* 初始化成功，直接返回 */
        }

        system_delay_ms(100);  /* 硬件初始化失败，等待100ms后重试 */
    }

    /* 所有重试均失败 */
    imu_ready = 0u;  /* 就绪标志保持为0，imu_update()不会执行积分 */
    imu_error = 1u;  /* 错误标志保持为1 */
}

/**
 * imu_update() - IMU周期性更新函数（5ms中断调用）
 *
 * 每5ms由CCU60_CH1 PIT中断调用一次，执行以下处理：
 *   1. 读取陀螺仪Z轴角速度并减去零偏
 *   2. 限幅到[-1200, 1200]度/秒
 *   3. 死区滤波：低于0.35度/秒的噪声归零
 *   4. 处理重置跳过：重置后的前2个周期不积分
 *   5. 一阶低通滤波（alpha=0.65）
 *   6. 积分累加到偏航角
 *   7. 归一化到[-180, 180]度
 */
void imu_update(void)
{
    float rate;  /* 当前原始角速度（减去零偏后、滤波前） */

    if (!imu_ready)     /* IMU未初始化完成 */
        return;         /* 跳过本次更新，不执行任何操作 */

    rate = imu_read_z_rate_dps() - s_gyro_z_offset;  /* 读取角速度并减去校准得到的零偏值 */
    rate = imu_clampf(rate, -IMU_RATE_LIMIT_DPS, IMU_RATE_LIMIT_DPS);  /* 限幅到[-1200, 1200]度/秒，防止异常大值 */

    if (imu_absf(rate) < IMU_RATE_DEADBAND_DPS)  /* 角速度绝对值低于死区阈值（0.35度/秒） */
        rate = 0.0f;                               /* 视为静止噪声，归零处理 */

    if (s_reset_skip > 0u)  /* 重置跳过计数器非0，表示刚执行过偏航角重置 */
    {
        s_reset_skip--;             /* 跳过计数器递减 */
        s_yaw_rate_filtered = 0.0f; /* 清零滤波器输出 */
        yaw_rate = 0.0f;            /* 清零显示用角速度 */
        return;                     /* 跳过本次积分，防止重置后立即被旧数据覆盖 */
    }

    s_yaw_rate_filtered =
        s_yaw_rate_filtered * IMU_RATE_FILTER_ALPHA +   /* 保留65%的历史滤波值（平滑作用） */
        rate * (1.0f - IMU_RATE_FILTER_ALPHA);           /* 混入35%的新采样值（跟踪作用） */

    yaw_rate = s_yaw_rate_filtered;                               /* 更新全局角速度变量（供调试显示） */
    yaw_angle += yaw_rate * IMU_UPDATE_DT_SEC;                   /* 角速度乘以时间步长（0.005s），累加到偏航角 */
    imu_normalize_yaw();                                          /* 归一化偏航角到[-180, 180]度范围 */
}

/**
 * imu_reset_yaw() - 重置偏航角为零
 *
 * 清除所有运行时状态（偏航角、角速度、滤波器），并设置跳过标志
 * 使下一个IMU周期不执行积分。这防止了重置后立即被残留的积分值覆盖。
 *
 * 由直角弯HARD阶段退出时调用，为下一次转弯做准备。
 */
void imu_reset_yaw(void)
{
    imu_clear_state();  /* 清除偏航角、角速度、滤波器状态，并设置跳过标志 */
}
