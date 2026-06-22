/**
 * ========================================================================
 * Function.c - 通用工具函数实现
 * ========================================================================
 */
#include "Function.h"

/**
 * @brief 将int16值限制在 [b, c] 范围内
 *
 * 使用CLAMP宏实现，等价于：
 *   if (a < b) return b;
 *   if (a > c) return c;
 *   return a;
 *
 * 典型用途：
 *   - 电机PWM占空比限幅
 *   - 传感器数值范围保护
 *   - PID输出限幅
 *
 * @param a 待限幅的int16值
 * @param b 下限值
 * @param c 上限值
 * @return 限幅后的int16值，保证在 [b, c] 范围内
 */
int16 Limit_int16(int16 a, int16 b, int16 c)
{
    return (int16)CLAMP(a, b, c);
}
