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
    if (row >= TF_IMG_H || col < 0 || col >= TF_IMG_W)
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
IntersectionResult_t g_inter_result;

/* 拐点法路口检测状态机变量 */
static uint8 s_inter_lock_cnt = 0u;
static uint8 s_inter_cooldown_cnt = 0u;
static uint8 s_inter_confirm_cnt = 0u;
static uint8 s_inter_candidate = 0u;

/* ========================================================================
 * 函数: count_white_on_edge
 * 功能: 统计矩形框一条边上的白色像素数
 * 参数: row_start / row_end - 边的行范围（水平边二者相等，垂直边二者不等）
 *       col_start / col_end - 边的列范围
 * 返回: 白像素累计数
 * ======================================================================== */
static uint8 count_white_on_edge(uint8 row_start, uint8 row_end,
                                 uint8 col_start, uint8 col_end)
{
    uint8 cnt = 0u;
    if (row_start == row_end)
    {
        for (uint8 c = col_start; c <= col_end; c++)
            if (is_white(row_start, (int16)c))
                cnt++;
    }
    else
    {
        for (uint8 r = row_start; r <= row_end; r++)
            if (is_white(r, (int16)col_start))
                cnt++;
    }
    return cnt;
}

/* ========================================================================
 * 函数: find_inflection_points
 * 功能: 从图像底部向上扫描，寻找左右边线的跳变点(拐点)
 * 说明: 相邻行边线位置跳变 > INTER_JUMP_THRESH 列即判定为拐点
 *       拐点 row = 跳变之前的行（较低行），col = 跳变前的边线位置
 * ======================================================================== */
static void find_inflection_points(InflectionPoint_t *left_ip,
                                   InflectionPoint_t *right_ip)
{
    left_ip->valid = 0u;
    left_ip->row = 0;
    left_ip->col = 0;
    right_ip->valid = 0u;
    right_ip->row = 0;
    right_ip->col = 0;

    for (int16 row = (int16)TF_JIDIAN_ROW - 1;
         row >= (int16)TF_SEARCH_END_ROW; row--)
    {
        if (!g_tf.row_valid[row] || !g_tf.row_valid[row + 1])
            continue;

        if (!left_ip->valid)
        {
            int16 jump = g_tf.left_edge[row] - g_tf.left_edge[row + 1];
            if (jump < 0) jump = -jump;
            if (jump > INTER_JUMP_THRESH)
            {
                left_ip->valid = 1u;
                left_ip->row = row + 1;
                left_ip->col = g_tf.left_edge[row + 1];
            }
        }

        if (!right_ip->valid)
        {
            int16 jump = g_tf.right_edge[row] - g_tf.right_edge[row + 1];
            if (jump < 0) jump = -jump;
            if (jump > INTER_JUMP_THRESH)
            {
                right_ip->valid = 1u;
                right_ip->row = row + 1;
                right_ip->col = g_tf.right_edge[row + 1];
            }
        }

        if (left_ip->valid && right_ip->valid)
            break;
    }
}

/* ========================================================================
 * 函数: detect_intersection
 * 功能: 拐点法路口检测（替代旧 right_angle_detect）
 *
 * 状态机: READY → LOCKED → COOLDOWN → READY
 *   READY:    扫描拐点，2帧确认后设置 g_ra_flag，进入 LOCKED
 *   LOCKED:   锁定 g_ra_flag 不清零，等待 Pid.c 动作完成后清零
 *   COOLDOWN: 冷却期，防止立即重触发
 *
 * g_ra_flag: 0=正常 1=右转 2=左转 3=十字
 * Pid.c 负责清零 g_ra_flag（动作完成后），检测与控制分离
 * ======================================================================== */
void detect_intersection(void)
{
    /* 丢线时全部复位 */
    if (g_tf.line_lost)
    {
        g_ra_flag = 0u;
        s_inter_lock_cnt = 0u;
        s_inter_cooldown_cnt = 0u;
        s_inter_confirm_cnt = 0u;
        s_inter_candidate = 0u;
        return;
    }

    /* ---- COOLDOWN 阶段 ---- */
    if (s_inter_cooldown_cnt > 0u)
    {
        s_inter_cooldown_cnt++;
        if (s_inter_cooldown_cnt >= INTER_COOLDOWN_FRAMES)
        {
            s_inter_cooldown_cnt = 0u;
            s_inter_confirm_cnt = 0u;
            s_inter_candidate = 0u;
        }
        return;
    }

    /* ---- LOCKED 阶段：g_ra_flag 被锁定，等待 Pid.c 清零 ---- */
    if (s_inter_lock_cnt > 0u)
    {
        if (g_ra_flag == 0u)
        {
            /* Pid.c 已清零 → 进入 COOLDOWN */
            s_inter_lock_cnt = 0u;
            s_inter_cooldown_cnt = 1u;
        }
        else
        {
            s_inter_lock_cnt++;
            if (s_inter_lock_cnt >= INTER_MAX_LOCK_FRAMES)
            {
                /* 超时保护：强制清零 */
                g_ra_flag = 0u;
                s_inter_lock_cnt = 0u;
                s_inter_cooldown_cnt = 1u;
            }
        }
        return;
    }

    /* ---- READY 阶段：扫描拐点 ---- */
    InflectionPoint_t left_ip, right_ip;
    find_inflection_points(&left_ip, &right_ip);

    g_inter_result.left_ip = left_ip;
    g_inter_result.right_ip = right_ip;

    /* 拐点行号约束：必须靠近车身底部（≥INTER_IP_MIN_ROW）才触发，防止远处提前拐弯 */
    {
        uint8 ip_close = 0u;
        if (left_ip.valid && left_ip.row >= (int16)INTER_IP_MIN_ROW)
            ip_close = 1u;
        if (right_ip.valid && right_ip.row >= (int16)INTER_IP_MIN_ROW)
            ip_close = 1u;
        if (!ip_close)
        {
            s_inter_confirm_cnt = 0u;
            s_inter_candidate = 0u;
            g_inter_result.detected_type = 0u;
            return;
        }
    }

    uint8 detected = 0u;
    g_inter_result.box_road_count = 0u;
    g_inter_result.box_road_mask = 0u;
    g_inter_result.box_top = 0u;
    g_inter_result.box_bottom = 0u;
    g_inter_result.box_left = 0u;
    g_inter_result.box_right = 0u;

    if (left_ip.valid && right_ip.valid)
    {
        /* 双侧拐点：构建矩形框，统计4条边上的道路像素 */
        uint8 box_top = (left_ip.row < right_ip.row)
                            ? (uint8)left_ip.row : (uint8)right_ip.row;
        uint8 box_bottom = (left_ip.row > right_ip.row)
                               ? (uint8)left_ip.row : (uint8)right_ip.row;
        uint8 box_left = (left_ip.col < right_ip.col)
                             ? (uint8)left_ip.col : (uint8)right_ip.col;
        uint8 box_right = (left_ip.col > right_ip.col)
                              ? (uint8)left_ip.col : (uint8)right_ip.col;

        g_inter_result.box_top = box_top;
        g_inter_result.box_bottom = box_bottom;
        g_inter_result.box_left = box_left;
        g_inter_result.box_right = box_right;

        if ((box_right - box_left) >= INTER_MIN_BOX_SIZE &&
            (box_bottom - box_top) >= INTER_MIN_BOX_SIZE)
        {
            if (count_white_on_edge(box_top, box_top,
                                    box_left, box_right) >= INTER_BOX_EDGE_COUNT)
                { g_inter_result.box_road_count++;
                  g_inter_result.box_road_mask |= 0x01u; }
            if (count_white_on_edge(box_bottom, box_bottom,
                                    box_left, box_right) >= INTER_BOX_EDGE_COUNT)
                { g_inter_result.box_road_count++;
                  g_inter_result.box_road_mask |= 0x02u; }
            if (count_white_on_edge(box_top, box_bottom,
                                    box_left, box_left) >= INTER_BOX_EDGE_COUNT)
                { g_inter_result.box_road_count++;
                  g_inter_result.box_road_mask |= 0x04u; }
            if (count_white_on_edge(box_top, box_bottom,
                                    box_right, box_right) >= INTER_BOX_EDGE_COUNT)
                { g_inter_result.box_road_count++;
                  g_inter_result.box_road_mask |= 0x08u; }
        }

        /* 根据框边道路判定方向 */
        if (g_inter_result.box_road_count >= 3u)
            detected = 3u;
        else if ((g_inter_result.box_road_mask & 0x02u) &&
                 (g_inter_result.box_road_mask & 0x08u))
            detected = 1u;
        else if ((g_inter_result.box_road_mask & 0x02u) &&
                 (g_inter_result.box_road_mask & 0x04u))
            detected = 2u;
    }
    else if (right_ip.valid)
    {
        detected = 1u;
    }
    else if (left_ip.valid)
    {
        detected = 2u;
    }

    g_inter_result.detected_type = detected;

    /* ---- 确认防抖：连续 INTER_CONFIRM_FRAMES 帧相同才触发 ---- */
    if (detected != 0u && detected == s_inter_candidate)
    {
        s_inter_confirm_cnt++;
        if (s_inter_confirm_cnt >= INTER_CONFIRM_FRAMES)
        {
            g_ra_flag = detected;
            s_inter_lock_cnt = 1u;
            s_inter_confirm_cnt = 0u;
            s_inter_candidate = 0u;
        }
    }
    else if (detected != 0u)
    {
        s_inter_candidate = detected;
        s_inter_confirm_cnt = 1u;
    }
    else
    {
        s_inter_confirm_cnt = 0u;
        s_inter_candidate = 0u;
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
