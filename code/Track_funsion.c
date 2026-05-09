#include "Track_funsion.h"

/* ========================================================================
 * 全局变量定义
 * ======================================================================== */

/* 赛道融合检测结果结构体 */
TrackFusion_t g_tf;

/* 二值化图像数组，基于原始mt9v03x_image[][]灰度图转换而来
 * 255 = 白像素(赛道)  0 = 黑像素(背景) */
uint8 bin_image[TF_IMG_H][TF_IMG_W];

/* ========================================================================
 * 静态内部变量 - 大津法计算需要使用的临时数组
 * ======================================================================== */

/* 大津法计数间隔，每隔多少帧计算一次阈值 */
static uint8 s_otsu_cnt = 0;

/* 直方图数组，统计每个灰度值的像素个数 */
static uint16 s_hist[256];

/* 归一化直方图，各灰度值概率 */
static float s_P[256];

/* 累积概率 P(0~i) */
static float s_PK[256];

/* 累积灰度矩 M(0~i) */
static float s_Mk[256];

/* ========================================================================
 * 函数: calc_otsu
 * 功能: 使用大津法(Otsu)计算自适应二值化阈值
 * 说明: 通过最大化类间方差来找到最佳分割阈值
 * 返回: 最佳阈值(0~255)
 * ======================================================================== */
static uint8 calc_otsu(void)
{
    float imgsize = (float)((uint32)TF_IMG_H * TF_IMG_W);

    /* 清零直方图 */
    for (uint16 i = 0; i < 256; i++)
        s_hist[i] = 0;

    /* 统计各灰度值像素个数 */
    for (uint8 i = 0; i < TF_IMG_H; i++)
        for (uint8 j = 0; j < TF_IMG_W; j++)
            s_hist[mt9v03x_image[i][j]]++;

    /* 计算归一化直方图 */
    for (uint16 i = 0; i < 256; i++)
        s_P[i] = s_hist[i] / imgsize;

    /* 初始化累积概率和灰度矩 */
    s_PK[0] = s_P[0];
    s_Mk[0] = 0.0f;
    for (uint16 i = 1; i < 256; i++)
    {
        s_PK[i] = s_PK[i - 1] + s_P[i];
        s_Mk[i] = s_Mk[i - 1] + (float)i * s_P[i];
    }

    /* 寻找最大方差对应的阈值 */
    float max_sigma = 0.0f;
    uint8 best = 128u;
    float Mg = s_Mk[255];  /* 全局灰度矩 */

    for (uint16 i = 1; i < 255; i++)
    {
        float pk = s_PK[i];
        if (pk < 1e-6f || pk > 1.0f - 1e-6f)
            continue;
        float mk = s_Mk[i];
        float sigma = (mk - Mg * pk) * (mk - Mg * pk) / (pk * (1.0f - pk));
        if (sigma > max_sigma)
        {
            max_sigma = sigma;
            best = (uint8)i;
        }
    }
    return best;
}

/* ========================================================================
 * 函数: binarize_image
 * 功能: 图像二值化
 * 说明: 根据阈值将灰度图转换为二值图
 *       灰度值 < 阈值 → 255(白/赛道)
 *       灰度值 >= 阈值 → 0(黑/背景)
 * ======================================================================== */
static void binarize_image(void)
{
    for (uint16 i = 0; i < TF_IMG_H; i++)
        for (uint16 j = 0; j < TF_IMG_W; j++)
            bin_image[i][j] = (mt9v03x_image[i][j] > g_tf.threshold) ? 255u : 0u;
}

/* ========================================================================
 * 函数: is_white
 * 功能: 判断指定位置的像素是否为白色(赛道)
 * 参数: row - 图像行坐标(0~119)
 *       col - 图像列坐标(0~187)
 * 返回: 1=白色, 0=黑色
 * ======================================================================== */
static inline uint8 is_white(uint8 row, int16 col)
{
    if (col < 0 || col >= TF_IMG_W)
        return 0u;
    return (bin_image[row][col] == 255u) ? 1u : 0u;
}

/* ========================================================================
 * 函数: is_left_edge / is_right_edge
 * 功能: 检测左/右边缘
 * 说明: 边缘定义为:
 *       左边缘: col-1=黑, col=白, col+1=白 (左->右颜色变白)
 *       右边缘: col-1=白, col=白, col+1=黑 (左->右颜色变黑)
 * 参数: row - 图像行坐标
 *       col - 图像列坐标
 * 返回: 1=是边缘, 0=不是边缘
 * ======================================================================== */
static inline uint8 is_left_edge(uint8 row, int16 col)
{
    if (col < 1 || col >= TF_IMG_W - 1)
        return 0u;
    return (!is_white(row, col - 1) && is_white(row, col) && is_white(row, col + 1)) ? 1u : 0u;
}

static inline uint8 is_right_edge(uint8 row, int16 col)
{
    if (col < 1 || col >= TF_IMG_W - 1)
        return 0u;
    return (is_white(row, col - 1) && is_white(row, col) && !is_white(row, col + 1)) ? 1u : 0u;
}

/* ========================================================================
 * 函数: scan_left_edge_right / scan_left_edge_left
 * 功能: 从左到右/从右到左扫描寻找左边缘
 * 参数: row   - 图像行坐标
 *       start - 起始列
 *       end   - 结束列
 * 返回: 找到的左边缘列坐标，或TF_INVALID(-1)表示未找到
 * ======================================================================== */
static int16 scan_left_edge_right(uint8 row, int16 start, int16 end)
{
    if (start < 1)
        start = 1;
    if (end >= TF_IMG_W - 1)
        end = TF_IMG_W - 2;
    if (start > end)
        return TF_INVALID;
    for (int16 c = start; c <= end; c++)
        if (is_left_edge(row, c))
            return c;
    return TF_INVALID;
}

static int16 scan_left_edge_left(uint8 row, int16 start, int16 end)
{
    if (start >= TF_IMG_W - 1)
        start = TF_IMG_W - 2;
    if (end < 1)
        end = 1;
    if (start < end)
        return TF_INVALID;
    for (int16 c = start; c >= end; c--)
        if (is_left_edge(row, c))
            return c;
    return TF_INVALID;
}

/* ========================================================================
 * 函数: scan_right_edge_left / scan_right_edge_right
 * 功能: 从左到右/从右到左扫描寻找右边缘
 * 参数: row   - 图像行坐标
 *       start - 起始列
 *       end   - 结束列
 * 返回: 找到的右边缘列坐标，或TF_INVALID(-1)表示未找到
 * ======================================================================== */
static int16 scan_right_edge_left(uint8 row, int16 start, int16 end)
{
    if (start >= TF_IMG_W - 1)
        start = TF_IMG_W - 2;
    if (end < 1)
        end = 1;
    if (start < end)
        return TF_INVALID;
    for (int16 c = start; c >= end; c--)
        if (is_right_edge(row, c))
            return c;
    return TF_INVALID;
}

static int16 scan_right_edge_right(uint8 row, int16 start, int16 end)
{
    if (start < 1)
        start = 1;
    if (end >= TF_IMG_W - 1)
        end = TF_IMG_W - 2;
    if (start > end)
        return TF_INVALID;
    for (int16 c = start; c <= end; c++)
        if (is_right_edge(row, c))
            return c;
    return TF_INVALID;
}

/* ========================================================================
 * 函数: valid_pair
 * 功能: 验证左右边缘对是否有效
 * 说明: 检查左右边缘之间的距离是否在合理范围内
 * 参数: lb - 左边缘列坐标
 *       rb - 右边缘列坐标
 * 返回: 1=有效配对, 0=无效配对
 * ======================================================================== */
static inline uint8 valid_pair(int16 lb, int16 rb)
{
    if (lb == TF_INVALID || rb == TF_INVALID)
        return 0u;
    int16 w = rb - lb;
    return (w >= TF_MIN_TRACK_WIDTH && w <= TF_MAX_TRACK_WIDTH) ? 1u : 0u;
}

/* ========================================================================
 * 函数: find_jidian1
 * 功能: 在起始行寻找赛道的左右边界(基线)
 * 说明: 首先在图像中心1/4、1/2、3/4处���找白块，然后扩展到完整边界
 * 返回: 1=找到有效边界, 0=未找到有效边界
 * ======================================================================== */
static uint8 find_jidian1(void)
{
    const uint8 row = TF_JIDIAN_ROW;
    const int16 mid = TF_IMG_CENTER;
    const int16 qtr = TF_IMG_W / 4;
    const int16 tqtr = TF_IMG_W * 3 / 4;
    int16 lb = TF_INVALID, rb = TF_INVALID;

    /* 策略1: 检测图像中心区域 */
    if (is_white(row, mid - 1) && is_white(row, mid) && is_white(row, mid + 1))
    {
        lb = scan_left_edge_left(row, mid, 1);
        rb = scan_right_edge_right(row, mid, TF_IMG_W - 2);
    }
    /* 策略2: 检测1/4处 */
    else if (is_white(row, qtr - 1) && is_white(row, qtr) && is_white(row, qtr + 1))
    {
        lb = scan_left_edge_left(row, qtr, 1);
        rb = scan_right_edge_right(row, qtr, TF_IMG_W - 2);
    }
    /* 策略3: 检测3/4处 */
    else if (is_white(row, tqtr - 1) && is_white(row, tqtr) && is_white(row, tqtr + 1))
    {
        lb = scan_left_edge_left(row, tqtr, 1);
        rb = scan_right_edge_right(row, tqtr, TF_IMG_W - 2);
    }
    /* 策略4: 全面扫描 */
    else
    {
        lb = scan_left_edge_right(row, 1, TF_IMG_W - 2);
        if (lb != TF_INVALID)
            rb = scan_right_edge_left(row, TF_IMG_W - 2, lb + TF_MIN_TRACK_WIDTH);
    }

    if (!valid_pair(lb, rb))
        return 0u;

    g_tf.left_jidian = (uint8)lb;
    g_tf.right_jidian = (uint8)rb;
    return 1u;
}

/* ========================================================================
 * 函数: search_row_edges
 * 功能: 根据上一行的边缘位置，搜索当前行的左右边缘
 * 说明: 在上一行边缘位置的邻域内搜索，允许一定范围的偏差
 * 参数: row     - 当前行坐标
 *       prev_lb - 上一行左边缘位置
 *       prev_rb - 上一行右边缘位置
 *       out_lb  - 输出找到的左边缘位置
 *       out_rb  - 输出找到的右边缘位置
 * 返回: 1=找到有效边缘, 0=未找到有效边缘
 * ======================================================================== */
static uint8 search_row_edges(uint8 row, int16 prev_lb, int16 prev_rb,
                              int16 *out_lb, int16 *out_rb)
{
    int16 lb = TF_INVALID, rb = TF_INVALID;
    const int16 R = TF_LOCAL_RANGE;
    const int16 MID = TF_IMG_CENTER;

    /* 搜索左边缘: 优先邻域搜索，失败则全局搜索 */
    lb = scan_left_edge_right(row, prev_lb - R, prev_lb + R);
    if (lb == TF_INVALID)
        lb = scan_left_edge_left(row, prev_lb + R, prev_lb - R);
    if (lb == TF_INVALID)
        lb = scan_left_edge_left(row, MID, 1);

    /* 搜索右边缘 */
    rb = scan_right_edge_left(row, prev_rb + R, prev_rb - R);
    if (rb == TF_INVALID)
        rb = scan_right_edge_right(row, prev_rb - R, prev_rb + R);
    if (rb == TF_INVALID)
        rb = scan_right_edge_right(row, MID, TF_IMG_W - 2);

    *out_lb = lb;
    *out_rb = rb;
    return valid_pair(lb, rb);
}

/* ========================================================================
 * 函数: track_fusion_init
 * 功能: 赛道融合检测初始化
 * 说明: 初始化所有全局变量和数组
 * ======================================================================== */
void track_fusion_init(void)
{
    for (uint8 i = 0; i < TF_IMG_H; i++)
    {
        g_tf.left_edge[i] = TF_INVALID;
        g_tf.right_edge[i] = TF_INVALID;
        g_tf.center_line[i] = TF_INVALID;
        g_tf.row_valid[i] = 0u;
    }
    g_tf.error = 0;
    g_tf.lookahead_error = 0;
    g_tf.error_trend = 0;
    g_tf.valid_row_count = 0u;
    g_tf.line_lost = 0u;
    g_tf.left_jidian = (uint8)TF_IMG_CENTER;
    g_tf.right_jidian = (uint8)TF_IMG_CENTER;
    g_tf.threshold = 128u;
    s_otsu_cnt = (uint8)TF_OTSU_INTERVAL;

    for (uint16 i = 0; i < TF_IMG_H; i++)
        for (uint16 j = 0; j < TF_IMG_W; j++)
            bin_image[i][j] = 0u;
}

/* ========================================================================
 * 函数: track_fusion_update
 * 功能: 赛道融合检测主函数，每帧调用一次
 * 说明: 处理流程:
 *       1. 每隔N帧更新一次二值化阈值(大津法)
 *       2. 图像二值化
 *       3. 寻找起始行边界
 *       4. 从起始行向上搜索各行边界
 *       5. 加权平均计算中心线偏差
 * ======================================================================== */
void track_fusion_update(void)
{
    /* 0. 清空本帧数据 */
    for (uint8 i = 0; i < TF_IMG_H; i++)
    {
        g_tf.left_edge[i] = TF_INVALID;
        g_tf.right_edge[i] = TF_INVALID;
        g_tf.center_line[i] = TF_INVALID;
        g_tf.row_valid[i] = 0u;
    }
    g_tf.error = 0;
    g_tf.valid_row_count = 0u;
    g_tf.line_lost = 0u;

    /* 1. 自适应计算二值化阈值 */
    s_otsu_cnt++;
    if (s_otsu_cnt >= TF_OTSU_INTERVAL)
    {
        s_otsu_cnt = 0u;
        int16 raw = (int16)calc_otsu() + threshold_bias;
        if (raw < 20)
            raw = 20;
        if (raw > 240)
            raw = 240;
        g_tf.threshold = (uint8)raw;
    }

    /* 2. 图像二值化 */
    binarize_image();

    /* 3. 寻找基线 */
    if (!find_jidian1())
    {
        g_tf.line_lost = 1u;
        return;
    }

    const uint8 jrow = TF_JIDIAN_ROW;
    int16 prev_lb = (int16)g_tf.left_jidian;
    int16 prev_rb = (int16)g_tf.right_jidian;

    g_tf.left_edge[jrow] = prev_lb;
    g_tf.right_edge[jrow] = prev_rb;
    g_tf.center_line[jrow] = (prev_lb + prev_rb) / 2;
    g_tf.row_valid[jrow] = 1u;
    g_tf.valid_row_count = 1u;

    /* 4. 向上搜索边缘 */
    uint8 miss_streak = 0u;
    int16 end_row = (int16)TF_SEARCH_END_ROW;

    for (int16 row = (int16)jrow - 1; row >= end_row; row--)
    {
        int16 lb, rb;
        if (search_row_edges((uint8)row, prev_lb, prev_rb, &lb, &rb))
        {
            g_tf.left_edge[row] = lb;
            g_tf.right_edge[row] = rb;
            g_tf.center_line[row] = (lb + rb) / 2;
            g_tf.row_valid[row] = 1u;
            g_tf.valid_row_count++;
            prev_lb = lb;
            prev_rb = rb;
            miss_streak = 0u;
        }
        else
        {
            g_tf.row_valid[row] = 0u;
            if (++miss_streak >= TF_MAX_MISS_ROWS)
                break;
        }
    }

    /* 5. 计算加权中心线偏差 */
    if (g_tf.valid_row_count == 0u)
    {
        g_tf.line_lost = 1u;
        return;
    }

    int32 weighted_sum = 0, weight_total = 0;
    for (int16 row = (int16)jrow; row >= end_row; row--)
    {
        if (g_tf.row_valid[row])
        {
            int32 w = (int32)(row - end_row + 1);
            weighted_sum += (int32)g_tf.center_line[row] * w;
            weight_total += w;
        }
    }

    int16 avg_center = (weight_total > 0)
                           ? (int16)(weighted_sum / weight_total)
                           : (int16)TF_IMG_CENTER;

    g_tf.error = -(avg_center - (int16)TF_IMG_CENTER);
    g_tf.line_lost = 0u;

    /* 6. 计算前瞻行偏差（中上部行，用于弯道趋势预判） */
    {
        int32 la_sum = 0, la_wtotal = 0;
        for (int16 row = TF_LOOKAHEAD_START_ROW; row >= TF_LOOKAHEAD_END_ROW; row--)
        {
            if (g_tf.row_valid[row])
            {
                int32 w = (int32)(TF_LOOKAHEAD_START_ROW - row + 1);
                la_sum += (int32)g_tf.center_line[row] * w;
                la_wtotal += w;
            }
        }
        int16 la_center = (la_wtotal > 0)
                              ? (int16)(la_sum / la_wtotal)
                              : (int16)TF_IMG_CENTER;
        g_tf.lookahead_error = -(la_center - (int16)TF_IMG_CENTER);
        g_tf.error_trend = g_tf.lookahead_error - g_tf.error;
    }
}

/* ========================================================================
 * 全局变量: g_ra_flag
 * 功能: 直角弯标志
 * 说明: 0 = 正常直道
 *       1 = 右直角弯(右转)
 *       2 = 左直角弯(左转)
 *       3 = 十字/T字路口
 * ======================================================================== */
uint8 g_ra_flag = 0u;

/* 直角检测内部计数器 */
static uint8 s_ra_confirm_cnt = 0u;
static uint8 s_ra_candidate = 0u;
static uint8 s_ra_timeout_cnt = 0u;

/* ========================================================================
 * 函数: right_angle_detect
 * 功能: 直角弯检测
 * 说明: 检测原理:
 *       1. 统计底部若干行的边线状态
 *       2. 如果只剩左边线 → 右直角
 *       3. 如果只剩右边线 → 左直角
 *       4. 如果两边都丢失 → 十字/T字路口
 *       
 *       防抖机制:
 *       1. 需要连续2帧确认才认为是直角
 *       2. 如果超过50帧自动退出直角状态
 * ======================================================================== */
void right_angle_detect(void)
{
    /* 丢失赛道时复位所有状态 */
    if (g_tf.line_lost)
    {
        s_ra_confirm_cnt = 0u;
        s_ra_candidate = 0u;
        s_ra_timeout_cnt = 0u;
        g_ra_flag = 0u;
        return;
    }

    /* 统计底部边缘丢失情况 */
    uint8 right_lost = 0u;
    uint8 left_lost = 0u;

    for (int16 i = (int16)TF_JIDIAN_ROW;
         i > (int16)TF_JIDIAN_ROW - (int16)RA_CHECK_ROWS; i--)
    {
        if (!g_tf.row_valid[i])
            continue;

        if (g_tf.right_edge[i] >= (int16)(TF_IMG_W - RA_EDGE_MARGIN))
            right_lost++;

        if (g_tf.left_edge[i] <= (int16)RA_EDGE_MARGIN)
            left_lost++;
    }

    /* 本帧检测结果 */
    uint8 this_frame = 0u;

    if (right_lost >= RA_LOST_THRESH && left_lost >= RA_LOST_THRESH)
        this_frame = 3u;
    else if (right_lost >= RA_LOST_THRESH)
        this_frame = 1u;
    else if (left_lost >= RA_LOST_THRESH)
        this_frame = 2u;

    /* 状态确认逻辑 */
    if (this_frame != 0u && this_frame == s_ra_candidate)
    {
        if (s_ra_confirm_cnt < RA_CONFIRM_FRAMES)
            s_ra_confirm_cnt++;

        if (s_ra_confirm_cnt >= RA_CONFIRM_FRAMES)
        {
            g_ra_flag = this_frame;

            s_ra_timeout_cnt++;
            if (s_ra_timeout_cnt >= RA_TIMEOUT_FRAMES)
            {
                g_ra_flag = 0u;
                s_ra_confirm_cnt = 0u;
                s_ra_candidate = 0u;
                s_ra_timeout_cnt = 0u;
            }
        }
    }
    else if (this_frame != 0u)
    {
        s_ra_candidate = this_frame;
        s_ra_confirm_cnt = 1u;
        s_ra_timeout_cnt = 0u;
    }
    else
    {
        s_ra_confirm_cnt = 0u;
        s_ra_candidate = 0u;
        s_ra_timeout_cnt = 0u;
        g_ra_flag = 0u;
    }
}
/* ========================================================================
 * 全局变量: g_ra_pre_flag
 * 功能: 直角预判标志（远处检测，用于提前减速）
 * 说明: 0 = 正常
 *       1 = 远处看到直角，开始减速
 * ======================================================================== */
uint8 g_ra_pre_flag = 0u;

/* ========================================================================
 * 函数: right_angle_pre_detect
 * 功能: 直角预判检测（看图像中上部，直角出现就返回1）
 * 说明: 检测区域：行 RA_PRE_START_ROW ~ RA_PRE_END_ROW（图像中上部）
 *       统计每行左右半幅的白色像素数量
 *       某半幅白色像素超过阈值 → 说明横向白线进入视野 → 准备减速
 * ======================================================================== */
void right_angle_pre_detect(void)
{
    static uint8 s_on_cnt = 0;
    static uint8 s_off_cnt = 0;

    uint8 right_lost = 0u;
    uint8 left_lost = 0u;

    for (int16 i = (int16)RA_PRE_START_ROW;
         i > (int16)RA_PRE_END_ROW; i--)
    {
        if (!g_tf.row_valid[i])
            continue;

        if (g_tf.right_edge[i] >= (int16)(TF_IMG_W - RA_PRE_EDGE_MARGIN))
            right_lost++;

        if (g_tf.left_edge[i] <= (int16)RA_PRE_EDGE_MARGIN)
            left_lost++;
    }

    uint8 detected = (right_lost >= RA_PRE_LOST_THRESH ||
                      left_lost >= RA_PRE_LOST_THRESH) ? 1u : 0u;

    if (detected)
    {
        s_on_cnt++;
        s_off_cnt = 0;
        if (s_on_cnt >= 2)
            g_ra_pre_flag = 1u;
    }
    else
    {
        s_off_cnt++;
        s_on_cnt = 0;
        if (s_off_cnt >= 5)
            g_ra_pre_flag = 0u;
    }
}
