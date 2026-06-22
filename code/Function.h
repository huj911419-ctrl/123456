/**
 * ========================================================================
 * Function.h - 通用工具函数头文件
 * ========================================================================
 * 提供基础的数值限幅（CLAMP）宏和函数。
 * 在整个项目中广泛用于防止数值越界。
 * ======================================================================== */
#ifndef CODE_FUNCTION_H_
#define CODE_FUNCTION_H_

#include "zf_common_headfile.h"

/**
 * CLAMP宏 - 将值限制在 [b, c] 范围内
 *
 * 实现逻辑：
 *   若 a < b，返回 b（下限保护）
 *   若 a > c，返回 c（上限保护）
 *   否则返回 a（在范围内）
 *
 * 注意：参数顺序为 CLAMP(下限, 值, 上限)，与常见的 MIN/MAX 宏不同。
 *
 * @param a 下限值
 * @param b 待限幅的值
 * @param c 上限值
 * @return 限幅后的值，范围 [a, c]
 *
 * 使用示例：
 *   int x = CLAMP(0, sensor_value, 1023);  // 将传感器值限制在0~1023
 */
#define CLAMP(a, b, c) ((b) < (a) ? (a) : ((b) > (c) ? (c) : (b)))

/**
 * Limit_int16 - 将int16值限制在指定范围内
 *
 * CLAMP宏的函数封装版本，适用于int16类型。
 * 在需要函数指针或避免宏副作用的场景下使用。
 *
 * @param a 待限幅的int16值
 * @param b 下限（int16）
 * @param c 上限（int16）
 * @return 限幅后的int16值
 */
int16 Limit_int16(int16 a, int16 b, int16 c);

#endif /* CODE_FUNCTION_H_ */
