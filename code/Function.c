#include "Function.h"

/**
 * @file    Function.c
 * @brief   通用工具函数
 */



/*********************************************************************************************************************
 * @function:Limit_int( int a, int b, int c )
 * Description: 限制大小范围a-c. b为输入
 * Input:
 * Output:
 * Return:
 * Others:
 *********************************************************************************************************************/
int Limit_int(int a, int b, int c)
{
    if ((b >= a) && (b <= c))
    {
        return b;
    }
    else if (b < a)
    {
        return a;
    }
    else if (b > c)
    {
        return c;
    }
    return 0;
}

/*********************************************************************************************************************
 * @function:Limit_uint8( uint8 a, uint8 b, uint8 c )
 * Description: 限制大小范围a-c. b为输入
 * Input:
 * Output:
 * Return:
 * Others:
 *********************************************************************************************************************/
uint8 Limit_uint8(uint8 a, uint8 b, uint8 c)
{
    if ((b >= a) && (b <= c))
    {
        return b;
    }
    else if (b < a)
    {
        return a;
    }
    else if (b > c)
    {
        return c;
    }
    return 0;
}

/*********************************************************************************************************************
 * @function:Limit_float( float a, float b, float c )
 * Description: 限制大小范围a-c. b为输入
 * Input:
 * Output:
 * Return:
 * Others:
 *********************************************************************************************************************/
float Limit_float(float a, float b, float c)
{
    if ((b >= a) && (b <= c))
    {
        return b;
    }
    else if (b < a)
    {
        return a;
    }
    else if (b > c)
    {
        return c;
    }
    return 0;
}

/*********************************************************************************************************************
 * @function:Limit_int16( int16 a, int16 b, int16 c )
 * Description: 限制大小范围a-c. b为输入
 * Input:
 * Output:
 * Return:
 * Others:
 *********************************************************************************************************************/
int16 Limit_int16(int16 a, int16 b, int16 c)
{
    if ((b >= a) && (b <= c))
    {
        return b;
    }
    else if (b < a)
    {
        return a;
    }
    else if (b > c)
    {
        return c;
    }
    return 0;
}

/*********************************************************************************************************************
 * @function:Limit_int32( int32 a, int32 b, int32 c )
 * Description: 限制大小范围a-c. b为输入
 * Input:
 * Output:
 * Return:
 * Others:
 *********************************************************************************************************************/
int32 Limit_int32(int32 a, int32 b, int32 c)
{
    if ((b >= a) && (b <= c))
    {
        return b;
    }
    else if (b < a)
    {
        return a;
    }
    else if (b > c)
    {
        return c;
    }
    return 0;
}

/*********************************************************************************************************************
 * @function:Abs_int( int a )
 * Description: 取绝对值 int
 * Input:
 * Output:
 * Return:
 * Others:
 *********************************************************************************************************************/
int Abs_int(int a)
{
    if (a < 0)
    {
        return -a;
    }
    return a;
}

/*********************************************************************************************************************
 * @function:Abs_float( float a )
 * Description: 取绝对值 float
 * Input:
 * Output:
 * Return:
 * Others:
 *********************************************************************************************************************/
float Abs_float(float a)
{
    if (a < 0)
    {
        return -a;
    }
    return a;
}