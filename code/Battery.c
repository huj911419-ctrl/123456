/**
 * ========================================================================
 * Battery.c - 电池电压监测模块
 * ========================================================================
 * 功能：通过ADC采集锂电池电压，实现低电量检测和警示。
 *
 * 硬件连接：
 *   ADC通道：ADC0_CH11（P11_11引脚）
 *   电池电压经过分压电阻后接入ADC，分压比由 BAT_CAL_NUM/BAT_CAL_DEN 校准
 *
 * 电压计算公式：
 *   实际电压 = ADC值 * 最大量程(36.3V) / ADC满量程(4096) * 校准系数(1045/1000)
 *
 * 低电量保护：
 *   电压 <= 11.5V 时触发低电量标志，LED全部点亮警示
 *   电压 >= 11.7V 时解除低电量标志（滞回防抖）
 *
 * 滤波方案：
 *   一阶IIR低通滤波（移位实现），滤波系数 alpha = 1/16
 *   消除ADC采样噪声，避免电压读数跳变
 * ======================================================================== */
#include "Battery.h"

/* ========================================================================
 * 配置参数
 * ======================================================================== */

#define BATTERY_ADC_CH      ADC0_CH11_A11   /* ADC通道：ADC0的第11通道，对应P11_11引脚 */
#define BAT_ADC_FULL_SCALE  4096u           /* ADC满量程值：12位ADC = 2^12 = 4096 */
#define BAT_VOLT_X10_MAX    363u            /* 电压最大量程x10：36.3V -> 363 */
#define BAT_AVG_SAMPLES     32u             /* 每次采样的平均次数：32次硬件平均滤波 */
#define BATTERY_LOW_X10     115u            /* 低电量阈值x10：11.5V -> 115 */
#define BATTERY_LOW_REL_X10 117u            /* 低电量恢复阈值x10：11.7V -> 117（滞回） */
#define BATTERY_CHECK_DIV   1u              /* 检测分频系数：每1次调用执行1次采样 */
#define BAT_FILTER_SHIFT    4u              /* 滤波移位数：4位 = 除以16（alpha=1/16） */
#define BAT_CAL_NUM         1045u           /* 校准分子：实际电压/ADC电压 = 1045/1000 */
#define BAT_CAL_DEN         1000u           /* 校准分母 */

/* ========================================================================
 * 模块内部静态变量
 * ======================================================================== */

static float s_bat_voltage = 0.0f;          /* 滤波后的电池电压（浮点，单位：伏特） */
static uint16 s_bat_adc = 0u;               /* 滤波后的ADC值（12位，0~4095） */
static uint16 s_bat_voltage_x10 = 0u;       /* 滤波后的电池电压x10（整数，单位：0.1V） */
static uint8  s_bat_low = 0u;               /* 低电量标志：0=正常，1=低电量 */
static uint16 s_bat_cnt = 0u;               /* 分频计数器，用于控制采样频率 */
static uint32 s_bat_adc_filt = 0u;          /* IIR滤波器累加值（定点，左移BAT_FILTER_SHIFT位） */
static uint8  s_bat_filt_valid = 0u;        /* 滤波器有效标志：0=首次采样，1=滤波器已初始化 */

/**
 * @brief 执行一次电池电压采样和处理
 *
 * 处理流程：
 *   1. ADC采样：对电池电压进行32次硬件平均采样，得到12位ADC值
 *   2. IIR低通滤波：new_filt = old_filt + raw - (old_filt >> 4)
 *      等价于：new = old * (1 - 1/16) + raw * (1/16) = old * 0.9375 + raw * 0.0625
 *   3. 电压计算：将ADC值转换为实际电压（含校准系数）
 *   4. 低电量检测：带滞回的电压阈值判断
 *   5. LED警示：低电量时点亮所有LED
 */
static void battery_sample_once(void)
{
    uint16 raw_adc;                         /* ADC原始采样值 */
    uint32 mv10;                            /* 电压x10的中间计算值 */

    /* 步骤1：ADC采样，32次硬件平均滤波 */
    raw_adc = adc_mean_filter_convert(BATTERY_ADC_CH, BAT_AVG_SAMPLES);

    /* 步骤2：一阶IIR低通滤波（定点实现，避免浮点运算） */
    if (!s_bat_filt_valid)
    {
        /* 首次采样：直接用原始值初始化滤波器 */
        s_bat_adc_filt = ((uint32)raw_adc << BAT_FILTER_SHIFT); /* 放大16倍存储 */
        s_bat_filt_valid = 1u;              /* 标记滤波器已初始化 */
    }
    else
    {
        /* 后续采样：IIR滤波更新 */
        s_bat_adc_filt += (uint32)raw_adc;  /* 加入新采样值 */
        s_bat_adc_filt -= (s_bat_adc_filt >> BAT_FILTER_SHIFT); /* 减去旧值的1/16 */
        /* 等价于：filt = filt * (15/16) + raw * (1/16) */
    }

    /* 将滤波后的定点值还原为12位ADC值（四舍五入） */
    s_bat_adc = (uint16)((s_bat_adc_filt + (1u << (BAT_FILTER_SHIFT - 1u))) >> BAT_FILTER_SHIFT);

    /* 步骤3：将ADC值转换为实际电压x10 */
    /* 公式：电压x10 = ADC值 * 最大量程x10 / ADC满量程 * 校准系数 */
    mv10 = ((uint32)s_bat_adc * (uint32)BAT_VOLT_X10_MAX +
            (uint32)(BAT_ADC_FULL_SCALE / 2u)) /           /* +2048 四舍五入 */
           (uint32)BAT_ADC_FULL_SCALE;                      /* 除以4096 */
    mv10 = (mv10 * (uint32)BAT_CAL_NUM + (uint32)(BAT_CAL_DEN / 2u)) / /* 校准系数1045/1000 */
           (uint32)BAT_CAL_DEN;
    s_bat_voltage_x10 = (uint16)mv10;       /* 存储整数电压x10 */
    s_bat_voltage = (float)s_bat_voltage_x10 * 0.1f; /* 转换为浮点电压 */

    /* 步骤4：带滞回的低电量检测 */
    if (s_bat_low)
    {
        /* 当前已处于低电量状态，等待电压恢复到11.7V以上才解除 */
        if (s_bat_voltage_x10 >= BATTERY_LOW_REL_X10) /* >= 11.7V */
        {
            s_bat_low = 0u;                 /* 解除低电量标志 */
        }
    }
    else if (s_bat_voltage_x10 <= BATTERY_LOW_X10) /* <= 11.5V */
    {
        s_bat_low = 1u;                     /* 设置低电量标志 */
    }

    /* 步骤5：低电量时点亮所有LED作为警示 */
    if (s_bat_low)
    {
        gpio_set_level(P33_4, GPIO_HIGH);   /* 红色LED亮 */
        gpio_set_level(P33_5, GPIO_HIGH);   /* 黄色LED亮 */
        gpio_set_level(P33_6, GPIO_HIGH);   /* 绿色LED亮 */
    }
}

/**
 * @brief 电池监测模块初始化
 *
 * 初始化ADC通道并执行第一次采样。
 * 在系统启动时调用一次。
 */
void battery_init(void)
{
    adc_init(BATTERY_ADC_CH, ADC_12BIT);    /* 初始化ADC通道，12位精度 */
    battery_sample_once();                  /* 执行首次采样，初始化滤波器 */
}

/**
 * @brief 周期性电池电压检测
 *
 * 在主循环中调用，内部通过分频计数器控制采样频率。
 * 分频系数由 BATTERY_CHECK_DIV 定义。
 */
void battery_check(void)
{
    s_bat_cnt++;                            /* 分频计数器递增 */
    if (s_bat_cnt < BATTERY_CHECK_DIV)      /* 未达到分频阈值 */
        return;                             /* 跳过本次采样 */
    s_bat_cnt = 0u;                         /* 重置计数器 */

    battery_sample_once();                  /* 执行一次电池采样 */
}

/**
 * @brief 获取电池电压（浮点数）
 * @return 电池电压值（伏特），如12.0表示12V
 */
float battery_get_voltage(void)
{
    return s_bat_voltage;
}

/**
 * @brief 获取电池电压x10（整数）
 * @return 电池电压x10（0.1伏特），如120表示12.0V
 * 避免浮点运算，用于TFT显示和整数比较
 */
uint16 battery_get_voltage_x10(void)
{
    return s_bat_voltage_x10;
}

/**
 * @brief 获取ADC原始值（滤波后）
 * @return 12位ADC值（0~4095），用于调试确认ADC工作正常
 */
uint16 battery_get_adc(void)
{
    return s_bat_adc;
}

/**
 * @brief 获取低电量标志
 * @return 0=电量正常，1=电量过低（<=11.5V）
 */
uint8 battery_is_low(void)
{
    return s_bat_low;
}
