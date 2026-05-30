#include "Function.h"

/*
 * Limit_int16 — 将 int16 值限制在 [b, c] 范围内
 * @a: 待限幅的值
 * @b: 下限
 * @c: 上限
 * 返回: 限幅后的值
 */
int16 Limit_int16(int16 a, int16 b, int16 c)
{
    return (int16)CLAMP(a, b, c);
}
