/* 包含巡线融合模块头文件，定义了常量、结构体和接口声明  */
#include "Track_funsion.h"
/* 包含PID控制头文件，用于获取单边巡线模式变量 g_post_edge_side  */
#include "Pid.h"

/* ========================================================================
 * 全局变量定义
 * ========================================================================  */

/* 巡线融合主结构体，存储每帧的边缘、中线、误差等所有巡线结果  */
TrackFusion_t g_tf;
static TrackFusion_t s_tf_work;
static TrackFusion_t * const s_tf_publish_target = &g_tf;
#define g_tf s_tf_work

void track_fusion_publish(void)
{
    uint32 irq_state = interrupt_global_disable();
    *s_tf_publish_target = s_tf_work;
    interrupt_global_enable(irq_state);
}

/* 二值化后的图像数据（压缩后 94x60），255=白（赛道），0=黑（背景） */
uint8 Image_Binarize[TF_IMG_H][TF_IMG_W];

/* 压缩后的灰度图像数据（94x60），由原始 188x120 最近邻下采样得到  */
uint8 image_0[COMP_H][COMP_W];

/* 当前使用的二值化阈值，初始Otsu 最低阈值 75  */
uint16 Image_Threshold = (uint16)TF_OTSU_MIN_THRESHOLD;

/* 阈值偏移量，通过菜单调节，正值提高阈值（更少白色），负值降低阈值（更多白色） */
int16 threshold_bias = 30;

/* TFT图像诊断：判断是否真的处理到新帧，以及二值图里是否有白色像素  */
volatile uint8 g_tf_image_ready = 0u;
volatile uint32 g_tf_frame_count = 0u;
volatile uint16 g_tf_white_count = 0u;
uint8 g_corner_fill_active = 0u;
int16 g_corner_fill_center[TF_IMG_H];
uint8 g_corner_fill_valid[TF_IMG_H];

/* 未降噪二值图临时缓冲区，降噪后直接写入 Image_Binarize  */
static uint8 s_bin_tmp[TF_IMG_H][TF_IMG_W];

/* ---- Otsu 内部变量 ----  */

/* Otsu 帧计数器，每 TF_OTSU_INTERVAL 帧执行一次 Otsu 自适应阈值计算  */
static uint8 s_otsu_cnt = 0;

/* Otsu 灰度直方图，统计 0~255 各灰度级的像素数 */
static uint16 s_hist[256];

/* ---- 丢线检测与拐点相关变量 ----  */

/* 行扫描过程中首次丢失行的行号1 表示未丢失。用于路弯道检 */
static int16 s_first_miss_row = -1;

/* 最后一个有效行的中点列号，当某行丢失时用于回退参 */
static int16 s_last_valid_center = TF_IMG_CENTER;

/* Post-turn frames that skip pre-detect and intersection detect.  */
static volatile uint8 s_inter_post_turn_suppress_cnt = 0u;

/* 基准行行号（基点行），即从底部开始搜索赛道的起始 */
static int16 s_jidian_row = TF_JIDIAN_ROW;

/* 动态赛道半宽度（像素），根据有效边缘对自动更新，用于单边模式补全缺失边 */
static int16 s_track_half_width = TRACK_HALF_WIDTH;
static int16 s_tf_error_filtered = 0;
static int16 s_tf_lookahead_filtered = 0;
static uint8 s_tf_error_filter_valid = 0u;

/* 中点列号环形缓冲区大小，用于路口检测时选取倒数N 个有效中 */
#define IP_COL_BUF_SIZE 8

/* 中点列号环形缓冲区，存储最近若干帧的有效中点列 */
static int16 s_center_buf[IP_COL_BUF_SIZE];

/* 中点缓冲区已存元素计数（不超IP_COL_BUF_SIZE */
static uint8 s_center_buf_cnt = 0u;

/* 左侧探针列号（像素），用于在图像左侧边缘附近检测白色像素（路口分支 */
int16 ip_left_col = 9;

/* 右侧探针列号（像素），用于在图像右侧边缘附近检测白色像素（路口分支 */
int16 ip_right_col = 84;

/* 中点缓冲区偏移量=取最近（倒数个）=取倒数个，以此类推  */
int16 ip_col_offset = 2;

/* ---- 前向声明 ----  */

/* 前向声明：侧面探针白色检测（检测某行指定列附近是否有足够白色像素）  */
static uint8 side_probe_has_white(int16 row, int16 center_col);

/* 前向声明：对称分量检测（判断某行是否为三极管等干扰的对称结构 */
static uint8 symmetric_component_at_row(int16 row, int16 center_col);

static uint8 inter_side_branch_has_road(int16 row,
                                        int16 center_col,
                                        uint8 side);

static uint8 inter_side_branch_strong_has_road(int16 row,
                                               int16 center_col,
                                               uint8 side);
static uint8 inter_side_edge_reach_has_road(int16 row,
                                            int16 center_col,
                                            uint8 side);

static uint8 inter_side_crossing_has_road(int16 top,
                                          int16 bottom,
                                          int16 left,
                                          int16 right,
                                          uint8 side);

static uint8 inter_vertical_band_has_road(int16 center_col,
                                          int16 row_start,
                                          int16 row_end,
                                          uint8 min_white,
                                          uint8 min_streak);
static uint8 row_white_count(int16 row, int16 col_start, int16 col_end);
static uint8 max_white_streak_on_row(int16 row, int16 col_start, int16 col_end);
static uint8 glare_pre_guard_row(int16 row);

static void build_inter_box(int16 cx, int16 cy,
                            int16 *top, int16 *bottom,
                            int16 *left, int16 *right);

static uint8 inline_fast_pass_view(void);
static uint8 inline_component_pre_guard(int16 far_center_span);
static uint8 straight_inline_view(void);
static uint8 s_fast_pass_cache = 0u;
static uint8 s_fast_pass_cache_ok = 0u;
static uint8 s_straight_inline_cache = 0u;
static uint8 s_straight_inline_cache_ok = 0u;
static uint8 inline_fast_pass_view_cached(void)
{
    if (!s_fast_pass_cache_ok)
    {
        s_fast_pass_cache = inline_fast_pass_view();
        s_fast_pass_cache_ok = 1u;
    }
    return s_fast_pass_cache;
}

static uint8 straight_inline_view_cached(void)
{
    if (!s_straight_inline_cache_ok)
    {
        s_straight_inline_cache = straight_inline_view();
        s_straight_inline_cache_ok = 1u;
    }
    return s_straight_inline_cache;
}

static uint8 single_edge_route_ra_dir(void);

static uint8 valid_rows_in_range(int16 row_start, int16 row_end);

static void update_image_diagnostics(void);
static uint8 fast_vision_mode(void);
static uint8 fast_aux_row_allowed(int16 row);
static void copy_binary_image(void);

static uint8 corner_real_line_takeover_ready(int16 jrow, int16 end_row);
static void corner_fill_apply(int16 jrow, int16 end_row);

/* 前向声明：int16 取绝对 */
static int16 abs_i16(int16 v);

/* 前向声明：清除路口检测结 */
static void clear_inter_result(void);

/**
 * @brief int16 阈值值钳位到 [20, 240] 范围并转uint16
 *
 * 作用：保证二值化阈值不会过低（全白）或过高（全黑），提供基本的数值安全性
 *
 * @param value 输入阈值值（int16 类型
 * @return 钳位后的 uint16 阈值，范围 [20, 240]
  */
static uint16 clamp_threshold_i16(int16 value)
{
    /* 若值低于下0，强制设0，防止阈值过低导致二值化全白  */
    if (value < 20) value = 20;
    /* 若值高于上40，强制设40，防止阈值过高导致二值化全黑  */
    if (value > 240) value = 240;
    /* 将钳位后int16 值转uint16 类型返回  */
    return (uint16)value;
}

static void reset_track_error_filter(void)
{
    s_tf_error_filtered = 0;
    s_tf_lookahead_filtered = 0;
    s_tf_error_filter_valid = 0u;
}

static uint8 fast_vision_mode(void)
{
    if (base_speed < (int16)TF_FAST_VISION_SPEED)
        return 0u;
    if (g_post_edge_side != EDGE_BOTH)
        return 0u;
    if (g_ra_flag != 0u)
        return 0u;
    return 1u;
}

static uint8 fast_aux_row_allowed(int16 row)
{
    if (!fast_vision_mode())
        return 1u;
#if TF_FAST_AUX_ROW_STEP <= 1u
    (void)row;
    return 1u;
#else
    return (((uint16)row % (uint16)TF_FAST_AUX_ROW_STEP) == 0u) ? 1u : 0u;
#endif
}

static int16 filter_track_signal(int16 raw, int16 *state, int16 max_step)
{
    int16 delta;

    if (s_tf_error_filter_valid == 0u)
    {
        *state = raw;
        return raw;
    }

    delta = raw - *state;
    if (delta > max_step)
        delta = max_step;
    else if (delta < -max_step)
        delta = (int16)-max_step;

    *state = (int16)(*state + delta);
    return *state;
}

/* ========================================================================
 * 图像压缩88x120 -> 94x60（最近邻下采样）
 * ========================================================================  */

/**
 * @brief 图像压缩：将原始 188x120 灰度图像通过最近邻下采样压缩为 94x60
 *
 * 采用最简单的隔行隔列抽样方式（步长为2），速度极快，适合嵌入式实时处理
 * 压缩后图像存image_0[COMP_H][COMP_W]，后续所有处理基于此压缩图像
 *
 * 算法：对原始图像行取1行、每2列取1列，等价nearest-neighbor 缩放
 * 原始 188x120 -> 压缩94x60，像素数减少1/4，显著降低后续处理计算量
  */
static void compress_image(void)
{
    /* 外层循环：遍历压缩后图像的每一行（0 COMP_H-1，即 0~59 */
    for (uint8 i = 0u; i < COMP_H; i++)
    {
        /* 目标行指针，指向压缩后图像第 i 行的起始位置  */
        uint8 *dst = image_0[i];
        /* 源行指针，指向原始图像第 i*2 行（步长，隔行取样）  */
        const uint8 *row0 = mt9v03x_image[(uint16)i * 2u];
        const uint8 *row1 = mt9v03x_image[(uint16)i * 2u + 1u];

        /* 内层循环：遍历压缩后图像的每一列（0 COMP_W-1，即 0~93 */
        for (uint8 j = 0u; j < COMP_W; j++)
        {
            /* 隔列抽样，取原始图像j*2 列的像素值赋给压缩图 */
            uint16 col = (uint16)j * 2u;
            dst[j] = (uint8)(((uint16)row0[col] + row0[col + 1u] +
                              row1[col] + row1[col + 1u] + 2u) >> 2);
        }
    }
}

/* ========================================================================
 * Otsu 自适应阈值算法（基于压缩94x60 图像
 * ========================================================================  */

/**
 * @brief Otsu 自适应阈值算法（基于压缩94x60 图像
 *
 * Otsu 算法（大津法）通过最大化类间方差自动选取最佳二值化阈值
 * 实现步骤
 *   1. 统计灰度直方s_hist[256]
 *   2. 遍历所有可能阈t (0~255)，计算前背景的类间方
 *   3. 选取使类间方差最大的 t 作为阈
 *   4. 阈值不低于 TF_OTSU_MIN_THRESHOLD5），防止低对比度场景阈值过
 *   5. 加上菜单可调threshold_bias 偏移
 *
 * @return 计算得到的二值化阈值（uint16），范围 [20, 240]
  */
static uint16 calc_otsu(void)
{
    /* 图像总像素数 = 94 * 60 = 5640，用于计算前背景的权重比 */
    uint32 total = (uint32)COMP_H * COMP_W;
    /* 所有像素灰度值的加权总和，用于快速计算前景均 */
    uint32 sum = 0u;
    /* 默认阈值（最低保护值），当 Otsu 计算结果过低时使用此 */
    uint16 threshold = (uint16)TF_OTSU_MIN_THRESHOLD;

    /* ---- 步骤1：清零直方图并统计各灰度级出现次----  */
    /* 256 个灰度级的计数全部清 */
    for (uint16 i = 0; i < 256; i++) s_hist[i] = 0;
    /* 遍历压缩图像的每个像素，累加对应灰度级的直方图计算  */
    for (uint8 i = 0; i < COMP_H; i++)
        for (uint8 j = 0; j < COMP_W; j++)
            s_hist[image_0[i][j]]++;

    /* ---- 步骤2：计算总灰度加权和 sum = sigma(t * hist[t]) ----  */
    /* 遍历所有灰度级，计算加权总和，用于后续快速求前景均 */
    for (uint16 t = 0; t < 256; t++)
        sum += (uint32)t * s_hist[t];

    /* 背景类（灰度<= t）的灰度加权和，逐步累加  */
    uint32 sumB = 0u;
    /* 背景类的像素数（权重），逐步累加  */
    uint32 wB = 0u;
    /* 记录遍历过程中遇到的最大类间方差得到  */
    uint32 max_score = 0u;

    /* ---- 步骤3：遍历所有阈值，寻找使类间方差最大的阈----  */
    for (uint16 t = 0; t < 256; t++)
    {
        /* 累加当前灰度t 的像素数到背景类  */
        wB += s_hist[t];
        /* 若背景类为空（尚无像素），跳过当前阈 */
        if (wB == 0) continue;

        /* 前景类像素数 = 总像素数 - 背景类像素数  */
        uint32 wF = total - wB;
        /* 若前景类为空（所有像素都在背景中），后续阈值无意义，提前退 */
        if (wF == 0) break;

        /* 累加当前灰度级的加权和到背景） */
        sumB += (uint32)t * s_hist[t];

        /* 计算背景类的灰度均 */
        uint32 meanB = sumB / wB;
        /* 计算前景类的灰度均 */
        uint32 meanF = (sum - sumB) / wF;
        /* 计算背景与前景均值之 */
        int32 diff = (int32)meanB - (int32)meanF;
        /* 取均值差的绝对 */
        uint32 diff_abs = (uint32)((diff < 0) ? -diff : diff);
        /* 计算类间权重，右位防止乘法溢 */
        uint32 weight = (wB * wF) >> 8;
        /* 类间方差得分 = 权重 * 均值差平方  */
        uint32 score = weight * diff_abs * diff_abs;

        /* 若当前得分超过历史最大值，更新最优阈 */
        if (score > max_score)
        {
            /* 记录新的最大得到  */
            max_score = score;
            /* 记录当前阈值为最优阈 */
            threshold = t;
        }
    }

    /* 阈值下限保护：防止低对比度场景阈值过低导致大量噪 */
    if (threshold < (uint16)TF_OTSU_MIN_THRESHOLD)
        threshold = (uint16)TF_OTSU_MIN_THRESHOLD;

    /* 加上菜单可调的阈值偏移量，钳位到 [20, 240] 后返 */
    return clamp_threshold_i16((int16)threshold + threshold_bias);
}

/* ========================================================================
 * 图像二值化（基于压缩后图像
 * ========================================================================  */

/**
 * @brief 图像二值化：将压缩后的灰度图像转为黑白二值图
 *
 * 遍历 image_0 中每个像素，灰度值大于阈值设为白(255)，否则设为黑(0)
 * 结果先存s_bin_tmp，降噪后再写入 Image_Binarize，供后续边缘扫描使用
 * 二值化是将连续灰度信息简化为0/1的关键步骤，大幅简化后续边缘检测逻辑
  */
static void binarize_image(void)
{
    /* 外层循环：遍历压缩图像的每一 */
    for (uint16 i = 0; i < COMP_H; i++)
    {
        const uint8 *src = image_0[i];
        uint8 *dst = s_bin_tmp[i];
        /* 内层循环：遍历压缩图像的每一 */
        for (uint16 j = 0; j < COMP_W; j++)
            /* 灰度值大于阈值设为白255)，否则设为黑0)，完成二值化  */
            dst[j] = (src[j] > Image_Threshold) ? Image_WHITE : Image_BLACK;
    }
}

/* ========================================================================
 * 3x3 邻域双缓冲降噪（二值化图像
 * ========================================================================  */

/**
 * @brief 3x3 邻域双缓冲降噪，消除二值化图像中的孤立噪点
 *
 * 对每个像素检查其 3x3 邻域内的白色像素数量
 *   - 若当前像素为白色：邻域白色数 >= TF_DENOISE_WHITE_MIN(4) 则保留，否则变黑（去白噪点）
 *   - 若当前像素为黑色：邻域白色数 >= TF_DENOISE_BLACK_FILL(7) 则变白（填补黑噪点），否则保
 *   - 图像边界像素不做处理，直接保留原
 *
 * 使用双缓冲：s_bin_tmp 读取未降噪二值图，直接写入 Image_Binarize
 * 避免在计算过程中读写同一数组导致结果不一致，同时省掉整幅图拷贝
  */
static void denoise_binary_image(void)
{
    uint16 white_count = 0u;

    /* 外层循环：遍历二值化图像的每一 */
    for (uint8 i = 0u; i < COMP_H; i++)
    {
        const uint8 *src = s_bin_tmp[i];
        uint8 *dst = Image_Binarize[i];

        if (i == 0u || i >= (uint8)(COMP_H - 1u))
        {
            for (uint8 j = 0u; j < COMP_W; j++)
            {
                dst[j] = src[j];
                if (dst[j] == Image_WHITE)
                    white_count++;
            }
            continue;
        }

        const uint8 *prev = s_bin_tmp[(uint8)(i - 1u)];
        const uint8 *next = s_bin_tmp[(uint8)(i + 1u)];

        dst[0] = src[0];
        if (dst[0] == Image_WHITE)
            white_count++;

        /* 内层循环：遍历二值化图像的每一 */
        uint8 colsum[COMP_W];
        for (uint8 c = 0u; c < COMP_W; c++)
            colsum[c] = (uint8)((prev[c] == Image_WHITE) + (src[c] == Image_WHITE) + (next[c] == Image_WHITE));

        uint8 sum3 = (uint8)(colsum[0] + colsum[1] + colsum[2]);
        for (uint8 j = 1u; j < (uint8)(COMP_W - 1u); j++)
        {
            uint8 white_cnt = sum3;

            if (src[j] == Image_WHITE)
                dst[j] = (white_cnt >= TF_DENOISE_WHITE_MIN) ? Image_WHITE : Image_BLACK;
            else
                dst[j] = (white_cnt >= TF_DENOISE_BLACK_FILL) ? Image_WHITE : Image_BLACK;

            if (dst[j] == Image_WHITE)
                white_count++;

            if (j + 2u < COMP_W)
                sum3 = (uint8)(sum3 - colsum[j - 1u] + colsum[j + 2u]);
        }

        dst[COMP_W - 1u] = src[COMP_W - 1u];
        if (dst[COMP_W - 1u] == Image_WHITE)
            white_count++;
    }

    g_tf_white_count = white_count;
}

static void copy_binary_image(void)
{
    uint16 white_count = 0u;

    for (uint8 i = 0u; i < COMP_H; i++)
    {
        const uint8 *src = s_bin_tmp[i];
        uint8 *dst = Image_Binarize[i];
        for (uint8 j = 0u; j < COMP_W; j++)
        {
            uint8 v = src[j];
            dst[j] = v;
            if (v == Image_WHITE)
                white_count++;
        }
    }

    g_tf_white_count = white_count;
}

static void update_image_diagnostics(void)
{
    g_tf_image_ready = 1u;
    g_tf_frame_count++;
}

/* ========================================================================
 * 二值化图像辅助函数
 * ========================================================================  */

/**
 * @brief 判断指定坐标是否为白色像素（带边界检查）
 *
 * 提供安全的像素访问接口，越界时返回黑色（0），避免数组越界访问
 *
 * @param row 行号（int16 类型，支持负数检查）
 * @param col 列号（int16 类型，支持负数检查）
 * @return 1=白色像素=黑色像素或坐标越
  */
static inline uint8 is_white(int16 row, int16 col)
{
    /* 越界检查：行号或列号超出图像范围时返回0（视为黑色）  */
    if (row < 0 || row >= TF_IMG_H || col < 0 || col >= TF_IMG_W)
        return 0u;
    /* 在范围内，判断像素值是否等于白255)，是返回1，否返回0  */
    return (Image_Binarize[row][col] == Image_WHITE) ? 1u : 0u;
}

static uint8 inline_component_ip_guard(const InflectionPoint_t *ip)
{
    int16 b_top, b_bottom, b_left, b_right;
    uint8 top_valid_rows;
    uint8 side_evidence;
    uint8 center_forward_has;

    if (!ip->valid)
        return 0u;
    if (g_tf.line_lost != 0u)
        return 0u;
    if (g_tf.valid_row_count < INTER_INLINE_COMPONENT_VALID_ROWS_MIN)
        return 0u;
    if (abs_i16(g_tf.error) > INTER_STRAIGHT_ERR_LIMIT)
        return 0u;
    if (abs_i16(g_tf.lookahead_error) > INTER_STRAIGHT_LA_ERR_LIMIT)
        return 0u;

    top_valid_rows = valid_rows_in_range((int16)INTER_FAST_PASS_TOP_START_ROW,
                                         (int16)INTER_FAST_PASS_TOP_END_ROW);
    if (top_valid_rows < INTER_INLINE_COMPONENT_TOP_VALID_MIN)
        return 0u;

    build_inter_box(ip->col, ip->row, &b_top, &b_bottom, &b_left, &b_right);

    center_forward_has = inter_vertical_band_has_road(
        ip->col,
        b_top,
        (int16)(ip->row - 2),
        INTER_INLINE_CENTER_MIN_WHITE,
        INTER_INLINE_CENTER_MIN_STREAK);
    if (!center_forward_has)
        return 0u;

    /* Resistor-like straight components: stable line, high white area, fixed near-bottom IP.  */
    if (ip->row >= (int16)INTER_INLINE_COMPONENT_BOTTOM_ROW_MIN &&
        ip->row <= (int16)INTER_INLINE_COMPONENT_BOTTOM_ROW_MAX &&
        g_tf.valid_row_count >= INTER_INLINE_COMPONENT_BOTTOM_VALID_MIN &&
        top_valid_rows >= INTER_INLINE_COMPONENT_BOTTOM_TOP_MIN &&
        g_tf_white_count >= INTER_INLINE_COMPONENT_WHITE_MIN)
    {
        return 1u;
    }

    if (!inline_fast_pass_view_cached())
        return 0u;

    side_evidence =
        (inter_side_crossing_has_road(b_top, b_bottom, b_left, b_right, 1u) ||
         inter_side_crossing_has_road(b_top, b_bottom, b_left, b_right, 2u) ||
         inter_side_branch_strong_has_road(ip->row, ip->col, 1u) ||
         inter_side_branch_strong_has_road(ip->row, ip->col, 2u)) ? 1u : 0u;
    if (side_evidence)
        return 0u;

    return 1u;
}

/**
 * @brief 判断指定位置是否为左边缘（黑->白跳变，即赛道左边界
 *
 * 左边缘定义：左边像素为黑，当前和右边像素为白
 * 这代表赛道从背景进入赛道的跳变点（赛道左边界）
 * 通过检查三个连续像素的黑白关系来检测边缘跳变
 *
 * @param row 行号
 * @param col 列号
 * @return 1=是左边缘=不是或越
  */
static inline uint8 is_left_edge(int16 row, int16 col)
{
    /* 边界列不检测（需要访col-1 col+1，避免越界）  */
    if (row < 0 || row >= TF_IMG_H || col < 1 || col >= TF_IMG_W - 1) return 0u;
    const uint8 *line = Image_Binarize[row];
    /* 左边缘条件：col-1为黑 col为白 col+1为白（黑->白跳变）  */
    return (line[col - 1] != Image_WHITE &&
            line[col]     == Image_WHITE &&
            line[col + 1] == Image_WHITE) ? 1u : 0u;
}

/**
 * @brief 判断指定位置是否为右边缘（白->黑跳变，即赛道右边界
 *
 * 右边缘定义：左边和当前像素为白，右边像素为黑
 * 这代表赛道从赛道进入背景的跳变点（赛道右边界）
 * 通过检查三个连续像素的黑白关系来检测边缘跳变
 *
 * @param row 行号
 * @param col 列号
 * @return 1=是右边缘=不是或越
  */
static inline uint8 is_right_edge(int16 row, int16 col)
{
    /* 边界列不检测（需要访col-1 col+1，避免越界）  */
    if (row < 0 || row >= TF_IMG_H || col < 1 || col >= TF_IMG_W - 1) return 0u;
    const uint8 *line = Image_Binarize[row];
    /* 右边缘条件：col-1为白 col为白 col+1为黑（白->黑跳变）  */
    return (line[col - 1] == Image_WHITE &&
            line[col]     == Image_WHITE &&
            line[col + 1] != Image_WHITE) ? 1u : 0u;
}

/* ========================================================================
 * 边缘扫描函数（带边界安全保护
 * ========================================================================  */

/**
 * @brief start 向右扫描寻找左边缘（>白跳变点
 *
 * 在指定行中，start 列向 end 列逐列扫描，找到第一个左边缘即返回
 * 左边= 左边黑、当前白、右边白
 *
 * @param row   行号
 * @param start 起始列号（含
 * @param end   结束列号（含
 * @return 找到的边缘列号，未找到返TF_INVALID(-1)
  */
static int16 scan_left_edge_right(int16 row, int16 start, int16 end)
{
    /* 将起始列钳位>= 1（左边缘检测需要访col-1 */
    if (start < 1) start = 1;
    /* 将结束列钳位<= TF_IMG_W-2（需要访col+1 */
    if (end >= TF_IMG_W - 1) end = TF_IMG_W - 2;
    /* 若起始列大于结束列，搜索范围无效，直接返 */
    if (start > end) return TF_INVALID;
    /* 从左到右逐列扫描，找到第一个左边缘即返回其列号  */
    for (int16 c = start; c <= end; c++)
        if (is_left_edge(row, c)) return c;
    /* 扫描完毕未找到左边缘，返回无效标 */
    return TF_INVALID;
}

/**
 * @brief start 向左扫描寻找左边缘（>白跳变点
 *
 * 在指定行中，start 列向 end 列逐列反向扫描，找到第一个左边缘即返回
 * 用于从中心向左搜索赛道左边界
 *
 * @param row   行号
 * @param start 起始列号（含），扫描方向为从右向
 * @param end   结束列号（含
 * @return 找到的边缘列号，未找到返TF_INVALID(-1)
  */
static int16 scan_left_edge_left(int16 row, int16 start, int16 end)
{
    /* 将起始列钳位<= TF_IMG_W-2（右边界保护 */
    if (start >= TF_IMG_W - 1) start = TF_IMG_W - 2;
    /* 将结束列钳位>= 1（左边缘检测需要访col-1 */
    if (end < 1) end = 1;
    /* 若起始列小于结束列（反向范围无效），直接返回  */
    if (start < end) return TF_INVALID;
    /* 从右到左逐列扫描，找到第一个左边缘即返回其列号  */
    for (int16 c = start; c >= end; c--)
        if (is_left_edge(row, c)) return c;
    /* 扫描完毕未找到左边缘，返回无效标 */
    return TF_INVALID;
}

/**
 * @brief start 向左扫描寻找右边缘（>黑跳变点
 *
 * 在指定行中，start 列向 end 列逐列反向扫描，找到第一个右边缘即返回
 * 右边= 左边白、当前白、右边黑
 *
 * @param row   行号
 * @param start 起始列号（含），扫描方向为从右向
 * @param end   结束列号（含
 * @return 找到的边缘列号，未找到返TF_INVALID(-1)
  */
static int16 scan_right_edge_left(int16 row, int16 start, int16 end)
{
    /* 将起始列钳位<= TF_IMG_W-2（右边缘检测需要访col+1 */
    if (start >= TF_IMG_W - 1) start = TF_IMG_W - 2;
    /* 将结束列钳位>= 1  */
    if (end < 1) end = 1;
    /* 若起始列小于结束列（反向范围无效），直接返回  */
    if (start < end) return TF_INVALID;
    /* 从右到左逐列扫描，找到第一个右边缘即返回其列号  */
    for (int16 c = start; c >= end; c--)
        if (is_right_edge(row, c)) return c;
    /* 扫描完毕未找到右边缘，返回无效标 */
    return TF_INVALID;
}

/**
 * @brief start 向右扫描寻找右边缘（>黑跳变点
 *
 * 在指定行中，start 列向 end 列逐列扫描，找到第一个右边缘即返回
 * 用于从中心向右搜索赛道右边界
 *
 * @param row   行号
 * @param start 起始列号（含
 * @param end   结束列号（含
 * @return 找到的边缘列号，未找到返TF_INVALID(-1)
  */
static int16 scan_right_edge_right(int16 row, int16 start, int16 end)
{
    /* 将起始列钳位>= 1  */
    if (start < 1) start = 1;
    /* 将结束列钳位<= TF_IMG_W-2（右边缘检测需要访col+1 */
    if (end >= TF_IMG_W - 1) end = TF_IMG_W - 2;
    /* 若起始列大于结束列，搜索范围无效，直接返 */
    if (start > end) return TF_INVALID;
    /* 从左到右逐列扫描，找到第一个右边缘即返回其列号  */
    for (int16 c = start; c <= end; c++)
        if (is_right_edge(row, c)) return c;
    /* 扫描完毕未找到右边缘，返回无效标 */
    return TF_INVALID;
}

/* ---- 前向声明（函数定义在后面----  */

/* 前向声明：将边缘列号钳位到合法范[1, TF_IMG_W-2]  */
static int16 clamp_edge_col(int16 col);

/* 前向声明：返回当前有效的赛道半宽度（带上下限保护 */
static int16 active_track_half_width(void);

/**
 * @brief 在指定行中，ref_col 为中心查找最近的白色段（赛道区域
 *
 * 搜索策略
 *   1. 先检ref_col 处是否为白色
 *   2. 若不是，向两侧交替扩展搜索最近的白色像素作为种子
 *   3. 找到种子后向左右扩展得到完整的白色段 [left, right]
 *   4. 对白色段宽度进行合法性检查：
 *      - 过宽 TF_MAX_TRACK_WIDTH）：可能是噪声区域，返回失败
 *      - 过窄 TF_MIN_TRACK_WIDTH）：使用半宽度补
 *
 * @param row      行号
 * @param ref_col  参考列号（搜索起点
 * @param out_lb   输出参数：白色段左边界列
 * @param out_rb   输出参数：白色段右边界列
 * @return 1=找到有效白色段，0=未找
  */
static uint8 find_white_segment_near_col(int16 row, int16 ref_col,
                                         int16 *out_lb, int16 *out_rb)
{
    /* 种子白色像素的列号，初始为无效 */
    int16 seed = TF_INVALID;
    /* 白色段的左边界、右边界、中心和半宽 */
    int16 left, right, center, half;

    /* 将参考列钳位到合法范[1, TF_IMG_W-2]，避免越界访 */
    if (ref_col < 1) ref_col = 1;
    if (ref_col > (int16)(TF_IMG_W - 2)) ref_col = (int16)(TF_IMG_W - 2);

    /* 策略1：直接检查参考列是否为白色像 */
    if (is_white(row, ref_col))
    {
        /* 参考列为白色，直接作为种子  */
        seed = ref_col;
    }
    else
    {
        /* 策略2：从参考列向两侧交替扩展搜索最近的白色像素  */
        for (int16 d = 1; d < TF_IMG_W; d++)
        {
            /* 计算左侧候选列  */
            int16 l = ref_col - d;
            /* 计算右侧候选列  */
            int16 r = ref_col + d;

            /* 检查左侧候选列是否在范围内且为白色  */
            if (l >= 1 && is_white(row, l))
            {
                /* 找到左侧白色种子，记录并退出搜 */
                seed = l;
                break;
            }
            /* 检查右侧候选列是否在范围内且为白色  */
            if (r <= (int16)(TF_IMG_W - 2) && is_white(row, r))
            {
                /* 找到右侧白色种子，记录并退出搜 */
                seed = r;
                break;
            }
        }
    }

    /* 若未找到任何白色像素种子，整行无赛道，返回失 */
    if (seed == TF_INVALID)
        return 0u;

    /* 从种子位置向左扩展，找到白色段的左边 */
    left = seed;
    right = seed;
    /* 向左扩展：只要左边像素仍为白色就继续左移  */
    while (left > 1 && is_white(row, left - 1))
        left--;
    /* 向右扩展：只要右边像素仍为白色就继续右移  */
    while (right < (int16)(TF_IMG_W - 2) && is_white(row, right + 1))
        right++;

    /* 白色段宽度过大（超过 TF_MAX_TRACK_WIDTH=30），判定为噪声区域（如大面积白色背景），返回失败  */
    if ((right - left) > TF_MAX_TRACK_WIDTH)
        return 0u;

    /* 白色段宽度过小（不足 TF_MIN_TRACK_WIDTH=4），使用动态半宽度补全  */
    if ((right - left) < TF_MIN_TRACK_WIDTH)
    {
        /* 计算白色段中心列 */
        center = (int16)((left + right) / 2);
        /* 获取当前有效的赛道半宽度  */
        half = active_track_half_width();
        /* 以中心为基准，向左右各扩展半宽度，得到补全后的边 */
        left = clamp_edge_col((int16)(center - half));
        right = clamp_edge_col((int16)(center + half));
    }

    /* 输出补全/确认后的白色段边 */
    *out_lb = left;
    *out_rb = right;
    /* 返回成功标志  */
    return 1u;
}

/**
 * @brief 判断左右边缘对是否构成合法赛道宽
 *
 * 检查边缘列号是否有效，且宽度在 [TF_MIN_TRACK_WIDTH(4), TF_MAX_TRACK_WIDTH(30)] 范围内
 * 用于过滤噪声边缘对和异常宽度的检测结果
 *
 * @param lb 左边缘列
 * @param rb 右边缘列
 * @return 1=合法边缘对，0=非法（边缘无效或宽度超限
  */
static inline uint8 valid_pair(int16 lb, int16 rb)
{
    /* 若任一边缘为无效-1)，直接判定为非法  */
    if (lb == TF_INVALID || rb == TF_INVALID) return 0u;
    /* 计算赛道宽度（右边缘 - 左边缘）  */
    int16 w = rb - lb;
    /* 宽度在合法范围内返回1，否则返  */
    return (w >= TF_MIN_TRACK_WIDTH && w <= TF_MAX_TRACK_WIDTH) ? 1u : 0u;
}

/**
 * @brief 将边缘列号钳位到合法范围 [1, TF_IMG_W-2]
 *
 * 边缘列不能为 0 TF_IMG_W-1，因is_left_edge/is_right_edge 需要访问相邻列（col-1 col+1）
 * 钳位[1, TF_IMG_W-2] 保证所有边缘检测函数不会越界
 *
 * @param col 输入列号
 * @return 钳位后的列号，范[1, TF_IMG_W-2]
  */
static int16 clamp_edge_col(int16 col)
{
    /* 下限保护：列号不能小  */
    if (col < 1) return 1;
    /* 上限保护：列号不能大TF_IMG_W-2  */
    if (col > (int16)(TF_IMG_W - 2)) return (int16)(TF_IMG_W - 2);
    /* 在合法范围内，直接返回原 */
    return col;
}

/**
 * @brief 将中线列号钳位到合法范围 [0, TF_IMG_W-1]
 *
 * 中线列号可以取图像最边缘的列 TF_IMG_W-1），
 * 不像边缘列那样需要访问相邻列
 *
 * @param col 输入列号
 * @return 钳位后的列号，范[0, TF_IMG_W-1]
  */
static int16 clamp_center_col(int16 col)
{
    /* 下限保护：列号不能小  */
    if (col < 0) return 0;
    /* 上限保护：列号不能大TF_IMG_W-1  */
    if (col > (int16)(TF_IMG_W - 1)) return (int16)(TF_IMG_W - 1);
    /* 在合法范围内，直接返回原 */
    return col;
}

/**
 * @brief 返回当前有效的赛道半宽度（带上下限保护）
 *
 * s_track_half_width 在运行过程中通过 update_track_half_width() 自动更新
 * 此函数将其限制在 [TRACK_HALF_WIDTH_MIN(4), TRACK_HALF_WIDTH_MAX(14)] 范围内
 * 半宽度用于单边模式下补全缺失的边缘
 *
 * @return 钳位后的赛道半宽度（像素），范围 [4, 14]
  */
static int16 active_track_half_width(void)
{
    /* 若半宽度低于最小4)，返回最小 */
    if (s_track_half_width < TRACK_HALF_WIDTH_MIN)
        return TRACK_HALF_WIDTH_MIN;
    /* 若半宽度超过最大14)，返回最大 */
    if (s_track_half_width > TRACK_HALF_WIDTH_MAX)
        return TRACK_HALF_WIDTH_MAX;
    /* 在合法范围内，直接返回当前 */
    return s_track_half_width;
}

/**
 * @brief 根据新发现的边缘对更新动态赛道半宽度
 *
 * 采用一阶低通滤波（权重 3:1）平滑更新：
 *   s_track_half_width = (旧* 3 + 新+ 2) / 4
 * 这样单帧噪声不会剧烈改变半宽度，同时能跟踪赛道宽度的缓慢变化
 * +2 用于四舍五入，提高精度
 *
 * @param lb 左边缘列
 * @param rb 右边缘列
  */
static void update_track_half_width(int16 lb, int16 rb)
{
    /* 计算当前帧的赛道宽度（右边缘 - 左边缘）  */
    int16 w = rb - lb;
    /* 当前帧的半宽 */
    int16 half;

    /* 宽度不合法（超出赛道宽度范围），不更新半宽度  */
    if (w < TF_MIN_TRACK_WIDTH || w > TF_MAX_TRACK_WIDTH)
        return;

    /* 计算半宽度（宽度除以2 */
    half = w / 2;
    /* 钳位半宽度到合法范围下限  */
    if (half < TRACK_HALF_WIDTH_MIN)
        half = TRACK_HALF_WIDTH_MIN;
    /* 钳位半宽度到合法范围上限  */
    if (half > TRACK_HALF_WIDTH_MAX)
        half = TRACK_HALF_WIDTH_MAX;

    /* 低通滤波更新：旧值权，新值权2 为四舍五 */
    s_track_half_width = (int16)((s_track_half_width * 3 + half + 2) / 4);
}

/**
 * @brief 根据单边巡线模式归一化边缘对
 *
 * 单边巡线模式下（g_post_edge_side EDGE_LEFT EDGE_RIGHT），
 * 仅使用一侧的有效边缘，另一侧通过半宽度推算：
 *   - EDGE_LEFT（左侧巡线）：仅用左边缘，右边缘 = 左边+ 2*半宽
 *   - EDGE_RIGHT（右侧巡线）：仅用右边缘，左边缘 = 右边- 2*半宽
 *   - EDGE_BOTH（双边模式）：正常检查边缘对合法性，并更新半宽度
 *
 * 此函数同时负责更新动态半宽度（仅在双边模式下）
 *
 * @param lb 输入/输出：左边缘列号（指针，可被修改
 * @param rb 输入/输出：右边缘列号（指针，可被修改
 * @return 1=有效边缘对，0=无效
  */
static uint8 normalize_edges_for_mode(int16 *lb, int16 *rb)
{
    /* 获取当前有效的赛道半宽度  */
    int16 half = active_track_half_width();

    /* 单边左侧巡线模式：只需左边缘有效，右边缘由半宽度推 */
    if (g_post_edge_side == EDGE_LEFT)
    {
        if (*lb == TF_INVALID)
        {
            if (*rb == TF_INVALID)
                return 0u;
            *lb = clamp_edge_col((int16)(*rb - half * 2));
        }
        else
        {
            *rb = clamp_edge_col((int16)(*lb + half * 2));
        }
        return (*rb > *lb) ? 1u : 0u;
    }

    /* 单边右侧巡线模式：只需右边缘有效，左边缘由半宽度推 */
    if (g_post_edge_side == EDGE_RIGHT)
    {
        if (*rb == TF_INVALID)
        {
            if (*lb == TF_INVALID)
                return 0u;
            *rb = clamp_edge_col((int16)(*lb + half * 2));
        }
        else
        {
            *lb = clamp_edge_col((int16)(*rb - half * 2));
        }
        return (*rb > *lb) ? 1u : 0u;
    }

    /* 双边模式（EDGE_BOTH）：正常检查边缘对合法 */
    if (!valid_pair(*lb, *rb))
        return 0u;

    /* 合法边缘对，更新动态半宽度（仅双边模式下更新）  */
    update_track_half_width(*lb, *rb);
    /* 返回成功标志  */
    return 1u;
}

/**
 * @brief 根据边缘对计算赛道中线列
 *
 * 单边巡线模式下，中线 = 活跃边缘 + 半宽度偏移：
 *   - EDGE_LEFT：中= 左边+ 半宽度（靠右侧推算赛道中心）
 *   - EDGE_RIGHT：中= 右边- 半宽度（靠左侧推算赛道中心）
 *   - EDGE_BOTH：中= (左边+ 右边 / 2（取两侧边缘中点
 *
 * @param lb 左边缘列
 * @param rb 右边缘列
 * @return 钳位后的中线列号，范[0, TF_IMG_W-1]
  */
static int16 calc_center_from_edges(int16 lb, int16 rb)
{
    /* 获取当前有效的赛道半宽度  */
    int16 half = active_track_half_width();

    /* 单边左侧模式：中= 左边+ 半宽 */
    if (g_post_edge_side == EDGE_LEFT)
        return clamp_center_col((int16)(lb + half));

    /* 单边右侧模式：中= 右边- 半宽 */
    if (g_post_edge_side == EDGE_RIGHT)
        return clamp_center_col((int16)(rb - half));

    /* 双边模式：中= 左右边缘中点  */
    return clamp_center_col((int16)((lb + rb) / 2));
}

/* ========================================================================
 * 基准行（基点）检—在底部行搜索赛道
 * ========================================================================  */

/**
 * @brief 设置基准行（基点）的行号和边缘数
 *
 * 将找到的基准行信息写入全局状态：
 *   - s_jidian_row：基准行行号
 *   - g_tf.left_jidian：左边缘列号
 *   - g_tf.right_jidian：右边缘列号
 *
 * @param row 基准行行
 * @param lb  左边缘列
 * @param rb  右边缘列
  */
static void set_jidian_row_data(int16 row, int16 lb, int16 rb)
{
    /* 记录基准行的行号  */
    s_jidian_row = row;
    /* 记录基准行的左边缘列号（转为uint8存储 */
    g_tf.left_jidian = (uint8)lb;
    /* 记录基准行的右边缘列号（转为uint8存储 */
    g_tf.right_jidian = (uint8)rb;
}

/**
 * @brief 在指定行搜索赛道边缘对（基点搜索
 *
 * 搜索策略（按优先级依次尝试）
 *   1. 检查图像中心列(col=47)附近是否有白色区域，从中心向两侧找边
 *   2. 检1/4 col=23)附近是否有白色区
 *   3. 检3/4 col=70)附近是否有白色区
 *   4. 从左到右扫描第一个左边缘，再从右到左扫描对应的右边缘
 * 最后通过 normalize_edges_for_mode() 根据单边/双边模式归一化边缘对
 *
 * @param row    行号
 * @param out_lb 输出参数：左边缘列号
 * @param out_rb 输出参数：右边缘列号
 * @return 1=找到有效边缘对，0=未找
  */
static uint8 find_jidian_at_row(int16 row, int16 *out_lb, int16 *out_rb)
{
    /* 图像中心列号 = 47  */
    const int16 mid = (int16)TF_IMG_CENTER;
    /* 图像 1/4 列号 = 23  */
    const int16 qtr = (int16)(TF_IMG_W / 4);
    /* 图像 3/4 列号 = 70  */
    const int16 tqtr = (int16)(TF_IMG_W * 3 / 4);
    /* 初始化左右边缘为无效 */
    int16 lb = TF_INVALID, rb = TF_INVALID;
    *out_lb = TF_INVALID;
    *out_rb = TF_INVALID;

    /* 策略1：检查中心列(mid)附近3个像素是否为白色（赛道区域）  */
    if (is_white(row, mid - 1) && is_white(row, mid) && is_white(row, mid + 1))
    {
        /* 从中心向左扫描左边缘  */
        lb = scan_left_edge_left(row, mid, 1);
        /* 从中心向右扫描右边缘  */
        rb = scan_right_edge_right(row, mid, TF_IMG_W - 2);
    }
    /* 策略2：检1/4 qtr)附近是否为白 */
    else if (is_white(row, qtr - 1) && is_white(row, qtr) && is_white(row, qtr + 1))
    {
        /* 1/4 列向左扫描左边缘  */
        lb = scan_left_edge_left(row, qtr, 1);
        /* 1/4 列向右扫描右边缘  */
        rb = scan_right_edge_right(row, qtr, TF_IMG_W - 2);
    }
    /* 策略3：检3/4 tqtr)附近是否为白 */
    else if (is_white(row, tqtr - 1) && is_white(row, tqtr) && is_white(row, tqtr + 1))
    {
        /* 3/4 列向左扫描左边缘  */
        lb = scan_left_edge_left(row, tqtr, 1);
        /* 3/4 列向右扫描右边缘  */
        rb = scan_right_edge_right(row, tqtr, TF_IMG_W - 2);
    }
    /* 策略4：以上位置均无白色，从左到右全扫描找左边 */
    else
    {
        /* 从图像左端向右扫描，找第一个左边缘  */
        lb = scan_left_edge_right(row, 1, TF_IMG_W - 2);
        /* 若找到左边缘，从图像右端向左扫描找对应的右边 */
        if (lb != TF_INVALID)
            rb = scan_right_edge_left(row, TF_IMG_W - 2, lb + TF_MIN_TRACK_WIDTH);
    }

    /* 根据单边/双边模式归一化边缘对（可能推算缺失边缘或更新半宽度）  */
    if (g_post_edge_side != EDGE_BOTH)
    {
        if (lb == TF_INVALID)
            lb = scan_left_edge_right(row, 1, TF_IMG_W - 2);
        if (rb == TF_INVALID)
            rb = scan_right_edge_left(row, TF_IMG_W - 2, 1);
    }

    if (!normalize_edges_for_mode(&lb, &rb)) return 0u;

    /* 输出找到的边缘对  */
    *out_lb = lb;
    *out_rb = rb;
    /* 返回成功标志  */
    return 1u;
}

/**
 * @brief 基点搜索主函数：从底部向上搜索赛道基准行
 *
 * TF_JIDIAN_ROW(56) 向上搜索TF_JIDIAN_MIN_ROW(45)，找到第一
 * 有效边缘对即作为基准行。基准行是逐行向上扫描的起点
 * 如果在所有候选行中都找不到有效边缘对，判定为丢线
 *
 * @return 1=找到基准行，0=未找到（判定丢线
  */
static uint8 find_jidian1(void)
{
    /* 初始化左右边缘为无效 */
    int16 lb = TF_INVALID;
    int16 rb = TF_INVALID;

    /* 从底部行(56)向上搜索到最小基准行(45)，共搜索12 */
    for (int16 row = (int16)TF_JIDIAN_ROW;
         row >= (int16)TF_JIDIAN_MIN_ROW; row--)
    {
        /* 在当前行尝试搜索赛道边缘 */
        if (find_jidian_at_row(row, &lb, &rb))
        {
            /* 找到有效边缘对，设置基准行数据并返回成功  */
            set_jidian_row_data(row, lb, rb);
            return 1u;
        }
    }

    /* 所有候选行均未找到有效边缘对，判定丢线  */
    if (g_post_edge_side != EDGE_BOTH)
    {
        for (int16 row = (int16)(TF_JIDIAN_MIN_ROW - 1);
             row >= (int16)TF_SINGLE_EDGE_JIDIAN_MIN_ROW; row--)
        {
            if (find_jidian_at_row(row, &lb, &rb))
            {
                set_jidian_row_data(row, lb, rb);
                return 1u;
            }
        }
    }

    return 0u;
}

/* ========================================================================
 * 逐行边缘搜索（基于上一行的局部范围搜索）
 * ========================================================================  */

/**
 * @brief 逐行边缘搜索：在指定行中基于上一行的边缘位置搜索新的边缘
 *
 * 搜索策略（三级回退机制）：
 *   1. 局部搜索：以上一行边缘为中心，左右各 TF_LOCAL_RANGE(10) 像素范围内搜
 *   2. 反向搜索：若局部未找到，扩大范围反向扫描（防止边缘跳变
 *   3. 全局搜索：若仍未找到，从图像中心向两侧全扫描
 *   4. 白色段搜索：以上一行中点为参考，查找最近的白色段作为最后手
 *
 * 多级回退机制保证在弯道、路口等边缘大幅移动场景下仍能可靠跟踪
 *
 * @param row     当前行号
 * @param prev_lb 上一行左边缘列号（作为搜索参考）
 * @param prev_rb 上一行右边缘列号（作为搜索参考）
 * @param out_lb  输出参数：当前行左边缘列
 * @param out_rb  输出参数：当前行右边缘列
 * @return 1=找到有效边缘对，0=未找
  */
static uint8 search_row_edges(int16 row, int16 prev_lb, int16 prev_rb,
                              int16 *out_lb, int16 *out_rb)
{
    /* 初始化左右边缘为无效 */
    int16 lb = TF_INVALID, rb = TF_INVALID;
    *out_lb = TF_INVALID;
    *out_rb = TF_INVALID;
    /* 局部搜索范±10 像素  */
    const int16 R = TF_LOCAL_RANGE;
    /* 图像中心列号  */
    const int16 MID = TF_IMG_CENTER;

    /* ---- 左边缘搜索：局部范-> 反向 -> 全局 ----  */
    /* 级：以上一行左边缘为中心，向右扫描局部范 */
    lb = scan_left_edge_right(row, prev_lb - R, prev_lb + R);
    /* 级：局部未找到，反向扫描（从右到左 */
    if (lb == TF_INVALID) lb = scan_left_edge_left(row, prev_lb + R, prev_lb - R);
    /* 级：仍未找到，从中心向左全局扫描  */
    if (lb == TF_INVALID) lb = scan_left_edge_left(row, MID, 1);

    /* ---- 右边缘搜索：局部范-> 反向 -> 全局 ----  */
    /* 级：以上一行右边缘为中心，向左扫描局部范 */
    rb = scan_right_edge_left(row, prev_rb + R, prev_rb - R);
    /* 级：局部未找到，反向扫描（从左到右 */
    if (rb == TF_INVALID) rb = scan_right_edge_right(row, prev_rb - R, prev_rb + R);
    /* 级：仍未找到，从中心向右全局扫描  */
    if (rb == TF_INVALID) rb = scan_right_edge_right(row, MID, TF_IMG_W - 2);

    /* 若边缘扫描未得到合法对，尝试白色段搜索作为最后手 */
    if (!normalize_edges_for_mode(&lb, &rb))
    {
        /* 计算上一行中点作为白色段搜索的参考列  */
        int16 ref_center = (int16)((prev_lb + prev_rb) / 2);
        /* 以上一行中点为中心，查找最近的白色） */
        if (!find_white_segment_near_col(row, ref_center, &lb, &rb))
            return 0u;
        /* 再次归一化边缘对  */
        if (!normalize_edges_for_mode(&lb, &rb))
            return 0u;
    }

    /* 输出找到的边缘对  */
    *out_lb = lb;
    *out_rb = rb;
    /* 返回成功标志  */
    return 1u;
}

/* ========================================================================
 * 初始
 * ========================================================================  */

/**
 * @brief 巡线融合模块初始
 *
 * 清零所有边中线/有效行数据，重置误差、基点、阈值等全局状态
 * 在系统启动或重新开始比赛时调用，将所有状态恢复到初始值
  */
void track_fusion_init(void)
{
    /* 循环变量  */
    uint16 i, j;

    /* 清零每行的边缘、中线和有效标志  */
    for (i = 0; i < TF_IMG_H; i++)
    {
        /* 左边缘设为无效 */
        g_tf.left_edge[i] = TF_INVALID;
        /* 右边缘设为无效 */
        g_tf.right_edge[i] = TF_INVALID;
        /* 中线设为无效 */
        g_tf.center_line[i] = TF_INVALID;
        /* 有效标志清零  */
        g_tf.row_valid[i] = 0u;
        g_corner_fill_center[i] = TF_INVALID;
        g_corner_fill_valid[i] = 0u;
    }
    g_corner_fill_active = 0u;
    /* 横向偏差清零  */
    g_tf.error = 0;
    /* 前瞻误差清零  */
    g_tf.lookahead_error = 0;
    /* 误差趋势清零  */
    g_tf.error_trend = 0;
    /* 有效行计数清 */
    g_tf.valid_row_count = 0u;
    /* 丢线标志清零  */
    g_tf.line_lost = 0u;
    /* 基点左边缘默认设为图像中 */
    g_tf.left_jidian = (uint8)TF_IMG_CENTER;
    /* 基点右边缘默认设为图像中 */
    g_tf.right_jidian = (uint8)TF_IMG_CENTER;
    /* 基点行重置为默认底部 */
    s_jidian_row = (int16)TF_JIDIAN_ROW;
    /* 清除路口检测结 */
    clear_inter_result();
    /* 阈值重置为 Otsu 最低保护 */
    Image_Threshold = (uint16)TF_OTSU_MIN_THRESHOLD;
    /* 阈值偏移量清零  */
    threshold_bias = 0;
    s_inter_post_turn_suppress_cnt = 0u;
    g_tf_image_ready = 0u;
    g_tf_frame_count = 0u;
    g_tf_white_count = 0u;
    /* Otsu 计数器设为间隔值，确保首帧即触Otsu 计算  */
    s_otsu_cnt = (uint8)TF_OTSU_INTERVAL;
    /* 赛道半宽度重置为默认 */
    s_track_half_width = TRACK_HALF_WIDTH;
    reset_track_error_filter();

    /* 清零压缩图像、二值化图像和临时缓冲区  */
    for (i = 0; i < COMP_H; i++)
        for (j = 0; j < COMP_W; j++)
        {
            /* 二值化图像清零（黑色）  */
            Image_Binarize[i][j] = Image_BLACK;
            /* 压缩灰度图像清零  */
            image_0[i][j] = 0u;
            /* 降噪临时缓冲区清 */
            s_bin_tmp[i][j] = Image_BLACK;
        }

    track_fusion_publish();
}

/* ========================================================================
 * 加权中线误差计算
 * ========================================================================  */

/**
 * @brief 计算指定行范围内的加权中线列
 *
 * 对指定范围内的有效行，按距离加权计算平均中线位置
 * 权重分配模式
 *   - top_heavy=0（底部权重高）：越靠start_row（底部近相机）权重越大，用于正常巡线
 *   - top_heavy=1（顶部权重高）：越靠end_row（顶部远相机）权重越大，用于前瞻弯道预测
 *
 * @param start_row 起始行号（包含，行号大的一端）
 * @param end_row   结束行号（包含，行号小的一端）
 * @param top_heavy 0=底部加权（近处优先）=顶部加权（远处优先）
 * @return 加权平均中线列号，无有效行时返回 TF_IMG_CENTER(47)
  */
static int16 calc_weighted_center(int16 start_row, int16 end_row, uint8 top_heavy)
{
    /* 加权中线列号总和  */
    int32 sum = 0;
    /* 权重总和  */
    int32 wtotal = 0;
    /* start_row end_row（行号递减）遍 */
    for (int16 row = start_row; row >= end_row; row--)
    {
        /* 仅处理有效行（边缘对合法的行 */
        uint8 valid = g_tf.row_valid[row];
        int16 center = g_tf.center_line[row];
        if (g_corner_fill_active != 0u && g_corner_fill_valid[row] != 0u)
        {
            valid = 1u;
            center = g_corner_fill_center[row];
        }
        if (valid)
        {
            int32 w = top_heavy
                ? (int32)(start_row - row + 1)
                : (int32)(row - end_row + 1);
            sum += (int32)center * w;
            wtotal += w;
        }
    }
    /* 有有效行则返回加权平均值，否则返回图像中心  */
    return (wtotal > 0) ? (int16)(sum / wtotal) : (int16)TF_IMG_CENTER;
}

/* ========================================================================
 * 每帧更新主函数：压缩 -> Otsu -> 二值化 -> 降噪 -> 边缘扫描
 * ========================================================================  */

/**
 * @brief 每帧巡线融合主函数（在主循环中调用）
 *
 * 完整处理流程
 *   1. 清除上一帧数
 *   2. 压缩图像 188x120 -> 94x60
 *   3. 计算/使用二值化阈值（Otsu 自适应或固定阈值，TF_FIXED_THRESHOLD_ENABLE 控制
 *   4. 二值化（灰-> 黑白
 *   5. 3x3 邻域降噪（消除孤立噪点）
 *   6. 在底部行搜索基准（基点），失败则判定丢线
 *   7. 从基点向上逐行扫描边缘，同时进行对称分量检测和分支检
 *   8. 计算加权横向偏差（底部加权，用于正常巡线控制
 *   9. 计算前瞻误差（顶部加权，用于弯道预测和速度规划
 *  10. 误差值乘缩放到原始坐标系，兼PID 参数
  */
void track_fusion_update(void)
{
    uint8 fast_mode = fast_vision_mode();

    s_fast_pass_cache_ok = 0u;
    s_straight_inline_cache_ok = 0u;
    /* ---- 步骤1：清除上一帧的逐行数据 ----  */
    for (uint8 i = 0; i < TF_IMG_H; i++)
    {
        /* 左边缘设为无 */
        g_tf.left_edge[i] = TF_INVALID;
        /* 右边缘设为无 */
        g_tf.right_edge[i] = TF_INVALID;
        /* 中线设为无效  */
        g_tf.center_line[i] = TF_INVALID;
        /* 有效标志清零  */
        g_tf.row_valid[i] = 0u;
        g_corner_fill_center[i] = TF_INVALID;
        g_corner_fill_valid[i] = 0u;
    }
    g_corner_fill_active = 0u;
    /* 横向偏差清零  */
    g_tf.error = 0;
    /* 前瞻误差清零  */
    g_tf.lookahead_error = 0;
    /* 误差趋势清零  */
    g_tf.error_trend = 0;
    /* 有效行计数清 */
    g_tf.valid_row_count = 0u;
    /* 丢线标志清零  */
    g_tf.line_lost = 0u;
    /* 清除对称分量标志（三极管干扰检测）  */
    g_sym_component_flag = 0u;
    s_first_miss_row = -1;
    s_center_buf_cnt = 0u;
    s_last_valid_center = TF_IMG_CENTER;

    /* ---- 步骤2：压188x120 -> 94x60 ----  */
    compress_image();

#if TF_FIXED_THRESHOLD_ENABLE
    /* ---- 步骤3（固定阈值模式）：使用固定阈+ 菜单偏移 ----  */
    /* 适合光照稳定的比赛环境，跳过 Otsu 计算以提高帧 */
    Image_Threshold = clamp_threshold_i16(
        (int16)TF_FIXED_THRESHOLD_VALUE + threshold_bias);
#else
    /* ---- 步骤3（自适应阈值模式）：每TF_OTSU_INTERVAL 帧执行一次 Otsu 计算 ----  */
    /* 递增 Otsu 帧计数器  */
    s_otsu_cnt++;
    /* 计数器达到间隔阈值时，执Otsu 计算并重置计数器  */
    if (s_otsu_cnt >= TF_OTSU_INTERVAL)
    {
        /* 重置计数 */
        s_otsu_cnt = 0u;
        /* 计算自适应阈 */
        {
            uint16 new_th = calc_otsu();
            uint16 old_th = Image_Threshold;
            if (new_th > old_th + 6u)
                new_th = old_th + 6u;
            else if (new_th + 6u < old_th)
                new_th = old_th - 6u;
            Image_Threshold = new_th;
        }
    }
#endif

    /* ---- 步骤4：二值化（灰-> 黑白----  */
    binarize_image();

    /* ---- 步骤5x3 邻域降噪（消除孤立噪点） ----  */
    if (fast_mode &&
        TF_FAST_DENOISE_DIV > 1u &&
        ((g_tf_frame_count % TF_FAST_DENOISE_DIV) != 0u))
    {
        copy_binary_image();
    }
    else
    {
        denoise_binary_image();
    }
    update_image_diagnostics();

    /* ---- 步骤6：在底部行搜索赛道基准（基点----  */
    /* 若未找到基点，判定丢线并提前返回  */
    if (!find_jidian1())
    {
        /* 设置丢线标志  */
        g_tf.line_lost = 1u;
        reset_track_error_filter();
        /* 丢线状态下不进行后续处理，直接返回  */
        return;
    }

    /* 基点行号（从底部开始向上扫描的起点 */
    const int16 jrow = s_jidian_row;
    /* 上一行左边缘列号，初始化为基点左边缘  */
    int16 prev_lb = (int16)g_tf.left_jidian;
    /* 上一行右边缘列号，初始化为基点右边缘  */
    int16 prev_rb = (int16)g_tf.right_jidian;

    /* 设置基点行的边缘和中线数据到全局结构 */
    g_tf.left_edge[jrow] = prev_lb;
    g_tf.right_edge[jrow] = prev_rb;
    /* 根据边缘对计算基点行的中线列 */
    g_tf.center_line[jrow] = calc_center_from_edges(prev_lb, prev_rb);
    /* 标记基点行为有效  */
    g_tf.row_valid[jrow] = 1u;
    /* 有效行计数初始化  */
    g_tf.valid_row_count = 1u;

    /* ---- 步骤7：从基点向上逐行扫描边缘 ----  */
    /* 连续丢失行计数器，用于判断何时停止向上扫 */
    uint8 miss_streak = 0u;
    /* 扫描终止行号（行，图像顶部附近）  */
    int16 end_row = (int16)TF_SEARCH_END_ROW;
    /* 重置首次丢失行为未丢失状 */
    s_first_miss_row = -1;
    /* 清空中点环形缓冲区计算  */
    s_center_buf_cnt = 0u;
    /* 初始化最后有效中点为基点行的中线列号  */
    s_last_valid_center = g_tf.center_line[jrow];

    /* 从基点行上一行开始，向上（行号递减）逐行扫描到终止行  */
    for (int16 row = jrow - 1; row >= end_row; row--)
    {
        /* 当前行的左右边缘列号  */
        int16 lb, rb;
        /* 在当前行基于上一行边缘位置搜索新的边缘对  */
        if (search_row_edges(row, prev_lb, prev_rb, &lb, &rb))
        {
            /* 找到有效边缘对，计算当前行的原始中线列号  */
            int16 raw_center = calc_center_from_edges(lb, rb);
            /* 检查当前行是否存在对称分量（三极管干扰等）  */
            uint8 sym_row = symmetric_component_at_row(row, raw_center);
            uint8 left_branch_near = 0u;
            uint8 right_branch_near = 0u;
            uint8 left_probe_near = 0u;
            uint8 right_probe_near = 0u;
            uint8 y_branch_row = 0u;
            uint8 side_pair_near = 0u;
            uint8 probe_noise_row = 0u;
            uint8 inline_component_row = 0u;
            /* 是否为分支（路口）行标志  */
            uint8 branch_row = 0u;

            /* 若检测到对称分量，设置全局标志  */
            if (sym_row)
                g_sym_component_flag = 1u;

            /* 分支检测：检查图像左右边缘附近是否有白色延伸（路口分支）  */
            /* 条件：行>= INTER_MIN_MISS_ROW(28) 左或右探针有白色 不是对称分量  */
            if (row >= (int16)INTER_MIN_MISS_ROW &&
                (!fast_mode ||
                 row >= (int16)INTER_BOX_START_ROW ||
                 fast_aux_row_allowed(row)))
            {
                left_probe_near = side_probe_has_white(row, ip_left_col);
                right_probe_near = side_probe_has_white(row, ip_right_col);

                if (!fast_mode ||
                    left_probe_near ||
                    right_probe_near ||
                    g_ra_pre_flag != 0u)
                {
                    left_branch_near = inter_side_branch_has_road(row, raw_center, 2u);
                    right_branch_near = inter_side_branch_has_road(row, raw_center, 1u);
                }

                y_branch_row = (left_branch_near && right_branch_near) ? 1u : 0u;
                side_pair_near = ((left_probe_near && right_probe_near) || y_branch_row) ? 1u : 0u;

                if (side_pair_near &&
                    g_ra_pre_flag == 0u &&
                    abs_i16((int16)(raw_center - s_last_valid_center)) <= INTER_INLINE_CENTER_DELTA &&
                    inter_vertical_band_has_road(raw_center,
                        (int16)(row - INTER_INLINE_AHEAD_ROWS),
                        (int16)(row - 2),
                        INTER_INLINE_CENTER_MIN_WHITE,
                        INTER_INLINE_CENTER_MIN_STREAK))
                {
                    inline_component_row = 1u;
                    g_sym_component_flag = 1u;
                }

                if ((left_probe_near || right_probe_near) &&
                    !left_branch_near && !right_branch_near &&
                    g_ra_pre_flag == 0u &&
                    abs_i16((int16)(raw_center - s_last_valid_center)) <= INTER_INLINE_CENTER_DELTA &&
                    inter_vertical_band_has_road(raw_center,
                        (int16)(row - INTER_INLINE_AHEAD_ROWS),
                        (int16)(row - 2),
                        INTER_INLINE_CENTER_MIN_WHITE,
                        INTER_INLINE_CENTER_MIN_STREAK))
                {
                    probe_noise_row = 1u;
                    g_sym_component_flag = 1u;
                }

                if (!inline_component_row &&
                    !probe_noise_row &&
                    (left_probe_near ||
                     right_probe_near ||
                     y_branch_row) &&
                    (!sym_row || y_branch_row))
                {
                    /* 标记当前行为分支 */
                    branch_row = 1u;
                    /* 若是首次检测到分支行，记录其行 */
                    if (s_first_miss_row < 0)
                        s_first_miss_row = row;
                }
            }

            /* 根据是否为分支行，采用不同的数据更新策略  */
            if (branch_row)
            {
                /* 分支行只用于路口识别，不写入普通边中线，防
                 * 三极管、T口、侧向分支把循迹中线拉歪
                 * 旧逻辑直接 break，会让分支行上方（含前瞻0~35行）
                 * 全部停扫，入弯瞬valid_rows 塌到1、前瞻丢失，导致
                 * T斜入弯冷启动转不动。改为跳过该行继续向上扫
                 * 保住前瞻；连续丢失过多仍miss_streak 兜底停止
                 * s_first_miss_row 已在上方 branch 检测时记录
                 * 路口识别不受影响 */
#if TF_BRANCH_ROW_KEEP_SCAN
                g_tf.left_edge[row] = TF_INVALID;
                g_tf.right_edge[row] = TF_INVALID;
                g_tf.row_valid[row] = 0u;
                miss_streak++;
                if (miss_streak >= TF_MAX_MISS_ROWS)
                    break;
                continue;
#else
                break;
#endif
            }
            else
            {
                /* 正常行：使用当前行实际检测到的边缘和中线  */
                g_tf.left_edge[row] = lb;
                g_tf.right_edge[row] = rb;
                g_tf.center_line[row] = raw_center;
                /* 更新上一行边缘参考值（供下一行搜索使用）  */
                prev_lb = lb;
                prev_rb = rb;
                /* 更新最后有效中 */
                s_last_valid_center = raw_center;
                /* 将有效中点存入环形缓冲区（用于路口检测时选取稳定中点 */
                s_center_buf[s_center_buf_cnt % IP_COL_BUF_SIZE] = raw_center;
                /* 递增缓冲区计算  */
                s_center_buf_cnt++;
            }

            /* 标记当前行为有效  */
            g_tf.row_valid[row] = 1u;
            /* 有效行计数递增  */
            g_tf.valid_row_count++;
            /* 找到有效行，重置连续丢失计数 */
            miss_streak = 0u;
        }
        else
        {
            /* 该行未找到有效边缘对，记录为无效  */
            g_tf.left_edge[row] = TF_INVALID;
            g_tf.right_edge[row] = TF_INVALID;
            /* 标记当前行为无效  */
            g_tf.row_valid[row] = 0u;
            /* 连续丢失计数递增  */
            miss_streak++;
            /* 连续丢失2行时记录首次丢失行（用于路口检测判断）  */
            if (miss_streak == 2 && s_first_miss_row < 0)
                /* row+1 = 丢失前的最后一个有效行  */
                s_first_miss_row = row + 1;
            /* 连续丢失超过上限(5，停止向上扫 */
            if (miss_streak >= TF_MAX_MISS_ROWS) break;
        }
    }

    /* 若无任何有效行，判定丢线  */
    if (g_tf.valid_row_count == 0u)
    {
        /* 设置丢线标志  */
        g_tf.line_lost = 1u;
        reset_track_error_filter();
        /* 丢线状态下直接返回  */
        return;
    }

    /* ---- 步骤8：计算加权横向偏差（底部加权，用于正常巡线控制） ----  */
    /* 计算所有有效行的加权平均中线列号（底部行权重更高）  */
    corner_fill_apply(jrow, end_row);

    int16 avg_center = calc_weighted_center(jrow, end_row, 0);
    int16 raw_error;
    int16 raw_lookahead;
    /* 横向偏差 = -(平均中线 - 图像中心)，负号使得左偏为正、右偏为 */
    raw_error = -(avg_center - (int16)TF_IMG_CENTER);
    /* 有有效行，清除丢线标 */
    g_tf.line_lost = 0u;

    /* ---- 步骤9：计算前瞻误差（顶部加权，用于弯道预测和速度规划----  */
    /* 使用局部作用域限制 la_center 变量的生命周 */
    {
        /* 计算前瞻区域(5~20)的加权平均中线（顶部行权重更高）  */
        int16 la_center = calc_weighted_center(
            (int16)TF_LOOKAHEAD_START_ROW, (int16)TF_LOOKAHEAD_END_ROW, 1);
        /* 前瞻误差 = -(前瞻中线 - 图像中心)  */
        raw_lookahead = -(la_center - (int16)TF_IMG_CENTER);
    }

    if (s_inter_post_turn_suppress_cnt >
        (INTER_POST_TURN_SUPPRESS_FRAMES - INTER_POST_TURN_FAR_BLOCK_FRAMES))
    {
        raw_lookahead = 0;
        s_tf_lookahead_filtered = 0;
    }

    g_tf.error = filter_track_signal(raw_error,
                                     &s_tf_error_filtered,
                                     TF_ERROR_FILTER_MAX_STEP);
    g_tf.lookahead_error = filter_track_signal(raw_lookahead,
                                               &s_tf_lookahead_filtered,
                                               TF_LOOKAHEAD_FILTER_MAX_STEP);
    s_tf_error_filter_valid = 1u;
    g_tf.error_trend = g_tf.lookahead_error - g_tf.error;

    /* ---- 步骤10：误差值乘，缩放到原始图像坐标----  */
    /* 原始图像 188x120，压缩后 94x60，乘回到原始坐标尺度  */
    /* 这样 PID 参数可以基于原始图像坐标调整，不受压缩影 */
    g_tf.error = g_tf.error * 2;
    g_tf.lookahead_error = g_tf.lookahead_error * 2;
    g_tf.error_trend = g_tf.error_trend * 2;
}

/* ========================================================================
 * 直角/弯道预检测（远场检测，用于提前减速）
 * ========================================================================  */

/* 直角/弯道预检测标志，1=检测到远场边缘丢失（提前减速信号）  */
uint8 g_ra_pre_flag = 0u;
uint8 g_ra_pre_dir = 0u;
uint8 g_ra_pre_slow_flag = 0u;
static int16 s_single_edge_ra_row = INTER_BOX_START_ROW;

static uint8 single_edge_side_evidence(uint8 side)
{
    uint8 probe_hits = 0u;
    uint8 branch_hits = 0u;
    uint8 strong_hits = 0u;
    int16 center = s_last_valid_center;
    int16 probe_col;

    if (center < 0 || center >= (int16)TF_IMG_W)
        center = (int16)TF_IMG_CENTER;

    if (g_tf.line_lost != 0u || g_tf.valid_row_count < 18u)
        return 0u;

    if (side == 1u)
        probe_col = ip_right_col;
    else if (side == 2u)
        probe_col = ip_left_col;
    else
        return 0u;

    for (int16 row = (int16)RA_PRE_START_ROW;
         row >= (int16)INTER_BOX_START_ROW; row--)
    {
        if (side_probe_has_white(row, probe_col))
        {
            if (probe_hits < 255u) probe_hits++;
        }
        if (inter_side_branch_has_road(row, center, side))
        {
            if (branch_hits < 255u) branch_hits++;
        }
        if (inter_side_branch_strong_has_road(row, center, side))
        {
            if (strong_hits < 255u) strong_hits++;
        }

        if (strong_hits >= 2u ||
            (strong_hits >= 1u && branch_hits >= 2u) ||
            (branch_hits >= 4u && probe_hits >= 3u))
        {
            s_single_edge_ra_row = row;
            return 1u;
        }
    }

    return 0u;
}

static uint8 s_single_edge_startup = 0u;  /* 单边启动冷却：只在刚进单边时用一次  */
static uint16 s_single_edge_ra_gap = 0u;  /* 单边触发RA后的冷却：每次RA后独立  */
static uint32 s_se_gap_tick = 0u;          /* 帧内去重：每帧只减一次  */
static uint32 s_single_edge_dir_cache_frame = 0u;
static uint8 s_single_edge_dir_cache_side = EDGE_BOTH;
static uint8 s_single_edge_dir_cache_valid = 0u;
static uint8 s_single_edge_dir_cache_value = 0u;

#define SINGLE_EDGE_STARTUP_FRAMES 8u     /* 刚进单边：8帧冷却  */
#define SINGLE_EDGE_RA_GAP_FRAMES 24u     /* 单边触发RA后冷却帧数  */

static uint8 cache_single_edge_route_dir(uint8 value)
{
    s_single_edge_dir_cache_frame = g_tf_frame_count;
    s_single_edge_dir_cache_side = g_post_edge_side;
    s_single_edge_dir_cache_valid = 1u;
    s_single_edge_dir_cache_value = value;
    return value;
}

static uint8 single_edge_route_ra_dir(void)
{
    if (s_single_edge_dir_cache_valid &&
        s_single_edge_dir_cache_frame == g_tf_frame_count &&
        s_single_edge_dir_cache_side == g_post_edge_side)
    {
        return s_single_edge_dir_cache_value;
    }

    if (g_post_edge_side == EDGE_BOTH)
    {
        s_single_edge_startup = 0u;
        s_single_edge_ra_gap = 0u;
        s_se_gap_tick = 0u;
        s_single_edge_dir_cache_valid = 0u;
        return cache_single_edge_route_dir(0u);
    }

    /* 帧内去重：每帧只减一次gap计数器  */
    if (s_se_gap_tick != g_tf_frame_count)
    {
        s_se_gap_tick = g_tf_frame_count;
        if (s_single_edge_startup < SINGLE_EDGE_STARTUP_FRAMES)
            s_single_edge_startup++;
        else if (s_single_edge_ra_gap > 0u)
            s_single_edge_ra_gap--;
    }

    /* 刚进单边：8帧冷却，只这一次  */
    if (s_single_edge_startup < SINGLE_EDGE_STARTUP_FRAMES)
        return cache_single_edge_route_dir(0u);

    /* 触发RA后：10帧冷却，和启动8帧无关  */
    if (s_single_edge_ra_gap > 0u)
        return cache_single_edge_route_dir(0u);

    s_single_edge_ra_row = INTER_BOX_START_ROW;

    if (route_next_flag_is(1u) && single_edge_side_evidence(1u))
    {
        if (g_sym_component_flag != 0u &&
            ((uint16)s_single_edge_ra_row * 2u) <
            (uint16)INTER_SINGLE_EDGE_COMPONENT_ARM_IP_ROW)
        {
            return cache_single_edge_route_dir(0u);
        }
        s_single_edge_ra_gap = SINGLE_EDGE_RA_GAP_FRAMES;
        return cache_single_edge_route_dir(1u);
    }
    if (route_next_flag_is(2u) && single_edge_side_evidence(2u))
    {
        if (g_sym_component_flag != 0u &&
            ((uint16)s_single_edge_ra_row * 2u) <
            (uint16)INTER_SINGLE_EDGE_COMPONENT_ARM_IP_ROW)
        {
            return cache_single_edge_route_dir(0u);
        }
        s_single_edge_ra_gap = SINGLE_EDGE_RA_GAP_FRAMES;
        return cache_single_edge_route_dir(2u);
    }

    return cache_single_edge_route_dir(0u);
}

/**
 * @brief 直角/弯道预检测函数（远场检测，用于提前减速）
 *
 * 在行 46~28 范围内检查左右边缘是否接近图像边界（距离 < RA_PRE_EDGE_MARGIN=5 像素）
 * 若一侧边缘丢失行>= RA_PRE_LOST_THRESH(2)，则设置 g_ra_pre_flag 提前减速
 *
 * 特殊处理：若检测到对称分量（三极管干扰等），则清除标志，防止误触发减速
 *
 * 使用滞回滤波防止抖动
 *   - 连续 2 帧检测到 -> 置位（快速响应）
 *   - 连续 5 帧未检测到 -> 清除（缓慢释放，防止抖动
  */

/* ========================================================================
 * Fast RA rough detect + control inflection row
 * ======================================================================== */

static uint8 s_fast_right_cnt = 0u;
static uint8 s_fast_left_cnt = 0u;
static uint8 s_fast_complex_cnt = 0u;
static uint8 s_fast_none_cnt = 0u;

static uint8 ip_col_is_white(uint8 row, int16 col)
{
    if (row >= TF_IMG_H)
        return 0u;
    if (col < 0 || col >= (int16)TF_IMG_W)
        return 0u;
    return (Image_Binarize[row][col] == Image_WHITE) ? 1u : 0u;
}

static uint8 ip_count_front_white(uint8 row, int16 center)
{
    int16 x;
    uint8 cnt = 0u;

    for (x = center - IP_FAST_FRONT_HALF_WIN;
         x <= center + IP_FAST_FRONT_HALF_WIN;
         x++)
    {
        if (ip_col_is_white(row, x))
            cnt++;
    }

    return cnt;
}

static uint8 ip_white_run_left(uint8 row, int16 start_col)
{
    int16 x;
    uint8 len = 0u;

    for (x = start_col; x >= 0; x--)
    {
        if (ip_col_is_white(row, x))
        {
            len++;
        }
        else
        {
            if (len > 0u)
                break;
        }
    }

    return len;
}

static uint8 ip_white_run_right(uint8 row, int16 start_col)
{
    int16 x;
    uint8 len = 0u;

    for (x = start_col; x < (int16)TF_IMG_W; x++)
    {
        if (ip_col_is_white(row, x))
        {
            len++;
        }
        else
        {
            if (len > 0u)
                break;
        }
    }

    return len;
}

static void update_fast_ra_feature(void)
{
#if IP_FAST_RA_ENABLE
    uint8 y;
    uint8 front_rows = 0u;
    uint8 left_rows = 0u;
    uint8 right_rows = 0u;
    uint8 raw_left = 0u;
    uint8 raw_right = 0u;
    uint8 raw_front = 0u;
    uint8 raw_type = 0u;
    uint8 score = 0u;
    uint8 reason = 0u;
    uint8 dir = 0u;

    for (y = IP_FAST_SCAN_TOP;
         y <= IP_FAST_SCAN_BOTTOM;
         y += IP_FAST_SCAN_STEP)
    {
        int16 c;
        uint8 front_white;
        uint8 left_len;
        uint8 right_len;

        if (y >= TF_IMG_H)
            break;

        if (g_tf.center_line[y] != TF_INVALID)
            c = g_tf.center_line[y];
        else
            c = TF_IMG_CENTER;

        front_white = ip_count_front_white(y, c);
        if (front_white >= IP_FAST_FRONT_WHITE_MIN)
            front_rows++;

        left_len = ip_white_run_left(y, c - IP_FAST_BRANCH_GAP);
        right_len = ip_white_run_right(y, c + IP_FAST_BRANCH_GAP);

        if (left_len >= IP_FAST_BRANCH_MIN_LEN)
            left_rows++;
        if (right_len >= IP_FAST_BRANCH_MIN_LEN)
            right_rows++;
    }

    if (front_rows >= IP_FAST_FRONT_ROW_MIN)
        raw_front = 1u;
    if (left_rows >= IP_FAST_BRANCH_ROW_MIN)
        raw_left = 1u;
    if (right_rows >= IP_FAST_BRANCH_ROW_MIN)
        raw_right = 1u;

    if (raw_left && raw_right)
        raw_type = 5u;
    else if (!raw_front && raw_right && !raw_left)
        raw_type = 1u;
    else if (!raw_front && raw_left && !raw_right)
        raw_type = 2u;
    else
        raw_type = 0u;

    if (raw_type == 1u)
    {
        if (s_fast_right_cnt < 255u) s_fast_right_cnt++;
        if (s_fast_left_cnt > 0u) s_fast_left_cnt--;
        if (s_fast_complex_cnt > 0u) s_fast_complex_cnt--;
        s_fast_none_cnt = 0u;
    }
    else if (raw_type == 2u)
    {
        if (s_fast_left_cnt < 255u) s_fast_left_cnt++;
        if (s_fast_right_cnt > 0u) s_fast_right_cnt--;
        if (s_fast_complex_cnt > 0u) s_fast_complex_cnt--;
        s_fast_none_cnt = 0u;
    }
    else if (raw_type == 5u)
    {
        if (s_fast_complex_cnt < 255u) s_fast_complex_cnt++;
        if (s_fast_right_cnt > 0u) s_fast_right_cnt--;
        if (s_fast_left_cnt > 0u) s_fast_left_cnt--;
        s_fast_none_cnt = 0u;
    }
    else
    {
        if (s_fast_none_cnt < 255u) s_fast_none_cnt++;
        if (s_fast_none_cnt >= IP_FAST_LOST_DECAY_FRAMES)
        {
            if (s_fast_right_cnt > 0u) s_fast_right_cnt--;
            if (s_fast_left_cnt > 0u) s_fast_left_cnt--;
            if (s_fast_complex_cnt > 0u) s_fast_complex_cnt--;
        }
    }

    g_fast_ra_left = raw_left;
    g_fast_ra_right = raw_right;
    g_fast_ra_front = raw_front;

    if (s_fast_right_cnt >= IP_FAST_CONFIRM_FRAMES)
        g_fast_ra_type = 1u;
    else if (s_fast_left_cnt >= IP_FAST_CONFIRM_FRAMES)
        g_fast_ra_type = 2u;
    else if (s_fast_complex_cnt >= IP_FAST_CONFIRM_FRAMES)
        g_fast_ra_type = 5u;
    else
        g_fast_ra_type = 0u;

    g_ip_visual_row = g_ip_max_row;
    if (g_ip_visual_row > g_ip_ctrl_row)
    {
        g_ip_ctrl_row = g_ip_visual_row;
        g_ip_ctrl_score = 100u;
        g_ip_ctrl_reason = 0x01u;
        return;
    }

    if (g_tf.line_lost != 0u)
    {
        if (g_ip_ctrl_row > IP_CTRL_ROW_DECAY)
            g_ip_ctrl_row -= IP_CTRL_ROW_DECAY;
        else
            g_ip_ctrl_row = 0u;
        return;
    }

    if (g_ra_pre_flag != 0u)
    {
        score += IP_CTRL_PRE_SCORE;
        reason |= 0x02u;
        if (g_ra_pre_dir == 1u || g_ra_pre_dir == 2u)
            dir = g_ra_pre_dir;
    }

    if (g_fast_ra_type == 1u)
    {
        score += IP_CTRL_BRANCH_SCORE;
        score += IP_CTRL_FRONT_LOST_SCORE;
        dir = 1u;
        reason |= 0x04u;
    }
    else if (g_fast_ra_type == 2u)
    {
        score += IP_CTRL_BRANCH_SCORE;
        score += IP_CTRL_FRONT_LOST_SCORE;
        dir = 2u;
        reason |= 0x08u;
    }
    else if (g_fast_ra_type == 5u)
    {
        score += IP_CTRL_BOTH_BRANCH_SCORE;
        dir = 3u;
        reason |= 0x10u;
    }

    if (!raw_front && (raw_left || raw_right))
    {
        score += IP_CTRL_FRONT_LOST_SCORE;
        reason |= 0x20u;
    }

    if (score >= IP_CTRL_SCORE_MIN && dir != 0u)
    {
        uint16 next_row;

        g_ip_ctrl_dir = dir;
        g_ip_ctrl_score = score;
        g_ip_ctrl_reason = reason;

        next_row = (uint16)g_ip_ctrl_row + IP_CTRL_ROW_STEP;
        if (next_row > IP_CTRL_ROW_MAX)
            next_row = IP_CTRL_ROW_MAX;
        g_ip_ctrl_row = (uint8)next_row;
    }
    else
    {
        if (g_ip_ctrl_row > IP_CTRL_ROW_DECAY)
        {
            g_ip_ctrl_row -= IP_CTRL_ROW_DECAY;
        }
        else
        {
            g_ip_ctrl_row = 0u;
            g_ip_ctrl_dir = 0u;
            g_ip_ctrl_score = 0u;
            g_ip_ctrl_reason = 0u;
        }
    }
#endif
}

void right_angle_pre_detect(void)
{
    /* 连续检测到的帧计数（用于滞回置位判断）  */
    static uint8 s_on_cnt = 0;
    /* 连续未检测到的帧计数（用于滞回清除判断）  */
    static uint8 s_off_cnt = 0;
    static uint8 s_slow_off_cnt = 0u;

    /* 右边缘接近图像右边界的行数计算  */
    uint8 right_lost = 0u;
    /* 左边缘接近图像左边界的行数计算  */
    uint8 left_lost = 0u;
    /* 存在对称分量的行数计算  */
    uint8 sym_component_rows = 0u;
    uint8 far_open_rows = 0u;
    int16 far_min_center = TF_IMG_W;
    int16 far_max_center = -1;

    if (s_inter_post_turn_suppress_cnt >
        (INTER_POST_TURN_SUPPRESS_FRAMES - INTER_POST_TURN_BLOCK_FRAMES))
    {
        s_on_cnt = 0u;
        s_off_cnt = 0u;
        s_slow_off_cnt = 0u;
        g_ra_pre_flag = 0u;
        g_ra_pre_dir = 0u;
        g_ra_pre_slow_flag = 0u;
        return;
    }
    uint8 post_turn_guard =
        (s_inter_post_turn_suppress_cnt > 0u) ? 1u : 0u;

    /* 近场保留原来的贴边判断；远场用边线打开/中线跳变提前降速 */
    for (int16 i = (int16)RA_PRE_START_ROW;
         i >= (int16)RA_PRE_FAR_END_ROW; i--)
    {
        /* 跳过无效行（无有效边缘对的行 */
        if (!g_tf.row_valid[i])
            continue;
        if (glare_pre_guard_row(i))
        {
            g_sym_component_flag = 1u;
            continue;
        }
        if (s_inter_post_turn_suppress_cnt >
            (INTER_POST_TURN_SUPPRESS_FRAMES - INTER_POST_TURN_FAR_BLOCK_FRAMES) &&
            i <= (int16)RA_PRE_FAR_START_ROW)
            continue;

        /* 检查该行是否为对称分量（三极管干扰等）  */
        if (fast_aux_row_allowed(i) &&
            symmetric_component_at_row(i, g_tf.center_line[i]))
        {
            /* 对称分量行计数递增  */
            sym_component_rows++;
            /* 设置全局对称分量标志  */
            g_sym_component_flag = 1u;
        }

        if (i > (int16)RA_PRE_END_ROW)
        {
            /* 右边缘接近图像右边界（列>= TF_IMG_W-5）视为丢 */
            if (g_tf.right_edge[i] >= (int16)(TF_IMG_W - RA_PRE_EDGE_MARGIN))
                right_lost++;

            /* 左边缘接近图像左边界（列<= 5）视为丢 */
            if (g_tf.left_edge[i] <= (int16)RA_PRE_EDGE_MARGIN)
                left_lost++;
        }

        if (i <= (int16)RA_PRE_FAR_START_ROW)
        {
            int16 width = (int16)(g_tf.right_edge[i] - g_tf.left_edge[i]);

            if (g_tf.center_line[i] < far_min_center)
                far_min_center = g_tf.center_line[i];
            if (g_tf.center_line[i] > far_max_center)
                far_max_center = g_tf.center_line[i];

            if (width >= (int16)RA_PRE_FAR_WIDTH_MIN &&
                (g_tf.left_edge[i] <= (int16)RA_PRE_FAR_LEFT_COL ||
                 g_tf.right_edge[i] >= (int16)RA_PRE_FAR_RIGHT_COL))
            {
                far_open_rows++;
            }
        }
    }

    /* 判断是否检测到远场边缘丢失：至少一侧丢失行数达到阈2)  */
    uint8 detected = (right_lost >= RA_PRE_LOST_THRESH ||
                      left_lost >= RA_PRE_LOST_THRESH) ? 1u : 0u;
    uint8 pre_dir = 0u;
    int16 far_center_span =
        (far_max_center >= far_min_center) ?
        (int16)(far_max_center - far_min_center) : 0;
    uint8 far_slow_detected =
        (far_open_rows >= RA_PRE_FAR_OPEN_ROWS ||
         (g_tf.valid_row_count <= RA_PRE_VALID_ROWS_LOW &&
          (far_open_rows > 0u ||
           far_center_span >= (int16)RA_PRE_CENTER_SPAN_MIN))) ? 1u : 0u;
    uint8 single_edge_ra_dir = single_edge_route_ra_dir();
    uint8 route_direct_dir = 0u;
    uint8 far_component_like =
        (far_open_rows >= RA_PRE_COMPONENT_OPEN_ROWS &&
         far_center_span >= (int16)RA_PRE_COMPONENT_CENTER_SPAN &&
         g_tf.valid_row_count >= RA_PRE_COMPONENT_VALID_ROWS) ? 1u : 0u;
    uint8 route_direct_has_geom =
        (far_open_rows >= RA_PRE_FAR_OPEN_ROWS ||
         far_center_span >= (int16)RA_PRE_CENTER_SPAN_MIN) ? 1u : 0u;
    uint8 route_direct_straight_guard =
        (g_tf.line_lost == 0u &&
         g_tf.valid_row_count >= RA_PRE_ROUTE_VALID_ROWS_MIN &&
         far_open_rows == 0u &&
         far_center_span < (int16)RA_PRE_CENTER_SPAN_MIN &&
         abs_i16(g_tf.error) <= RA_PRE_ROUTE_STRAIGHT_ERR_MAX &&
         abs_i16(g_tf.lookahead_error) <= RA_PRE_ROUTE_STRAIGHT_LA_MAX) ? 1u : 0u;



    if (far_component_like)
    {
        far_slow_detected = 0u;
        g_sym_component_flag = 1u;
    }

    if (route_next_flag_is(1u))
        route_direct_dir = 1u;
    else if (route_next_flag_is(2u))
        route_direct_dir = 2u;

    if (single_edge_ra_dir == 0u &&
        route_direct_straight_guard != 0u)
    {
        detected = 0u;
        far_slow_detected = 0u;
    }

    if (single_edge_ra_dir != 0u)
    {
        detected = 1u;
        far_slow_detected = 1u;
        pre_dir = single_edge_ra_dir;
    }
    else if (route_direct_dir != 0u &&
             g_tf.line_lost == 0u &&
             g_tf.valid_row_count <= RA_PRE_ROUTE_VALID_ROWS &&
             g_tf.valid_row_count >= RA_PRE_ROUTE_VALID_ROWS_MIN &&
             abs_i16(g_tf.error) <= RA_PRE_ROUTE_ERR_MAX &&
             abs_i16(g_tf.lookahead_error) <= RA_PRE_ROUTE_LA_MAX &&
             far_component_like == 0u &&
             route_direct_straight_guard == 0u &&
             route_direct_has_geom != 0u)
    {
        detected = 1u;
        far_slow_detected = 1u;
        pre_dir = route_direct_dir;
    }

    if ((detected || far_slow_detected) &&
        single_edge_ra_dir == 0u &&
        route_direct_dir == 0u &&
        inline_fast_pass_view_cached())
    {
        s_on_cnt = 0u;
        s_off_cnt = 0u;
        s_slow_off_cnt = 0u;
        g_ra_pre_flag = 0u;
        g_ra_pre_dir = 0u;
        g_ra_pre_slow_flag = 0u;
        g_sym_component_flag = 1u;
        return;
    }

    if (single_edge_ra_dir == 0u)
    {
        if (right_lost >= RA_PRE_LOST_THRESH &&
            right_lost > left_lost &&
            route_direct_straight_guard == 0u)
        {
            pre_dir = 1u;
        }
        else if (left_lost >= RA_PRE_LOST_THRESH &&
                 left_lost > right_lost &&
                 route_direct_straight_guard == 0u)
        {
            pre_dir = 2u;
        }
    }

    if (single_edge_ra_dir == 0u &&
        route_direct_dir == 0u &&
        pre_dir == 0u &&
        (detected || far_slow_detected) &&
        (inline_fast_pass_view_cached() ||
         (g_tf.line_lost == 0u &&
          g_tf.valid_row_count >= RA_PRE_ROUTE_VALID_ROWS &&
          abs_i16(g_tf.error) <= RA_PRE_ROUTE_ERR_MAX &&
          abs_i16(g_tf.lookahead_error) <= RA_PRE_ROUTE_LA_MAX &&
          far_center_span < (int16)RA_PRE_CENTER_SPAN_MIN)))
    {
        detected = 0u;
        far_slow_detected = 0u;
        s_on_cnt = 0u;
        s_slow_off_cnt = RA_PRE_SLOW_OFF_FRAMES;
        g_ra_pre_flag = 0u;
        g_ra_pre_dir = 0u;
        g_ra_pre_slow_flag = 0u;
    }
    if (post_turn_guard &&
        (pre_dir == 0u || !route_next_flag_is((uint8)pre_dir)))
    {
        detected = 0u;
        far_slow_detected = 0u;
    }

#if INTER_COMPONENT_PRE_GUARD_ENABLE
    if ((detected || far_slow_detected) &&
        single_edge_ra_dir == 0u &&
        (pre_dir == 0u || !route_next_flag_is((uint8)pre_dir)) &&
        inline_component_pre_guard(far_center_span))
    {
        detected = 0u;
        far_slow_detected = 0u;
        s_on_cnt = 0u;
        s_off_cnt = 0u;
        s_slow_off_cnt = RA_PRE_SLOW_OFF_FRAMES;
        g_ra_pre_flag = 0u;
        g_ra_pre_dir = 0u;
        g_ra_pre_slow_flag = 0u;
        g_sym_component_flag = 1u;
        return;
    }
#endif

    /* 若对称分量行数足够多(>=2)，判定为干扰（如三极管），清除标志并返回  */
    if (single_edge_ra_dir == 0u &&
        route_direct_dir == 0u &&
        sym_component_rows >= INTER_SYM_PRE_ROWS)
    {
        /* 清除检测标 */
        detected = 0u;
        /* 重置置位计数 */
        s_on_cnt = 0u;
        /* 重置清除计数 */
        s_off_cnt = 0u;
        s_slow_off_cnt = 0u;
        /* 清除预检测标 */
        g_ra_pre_flag = 0u;
        g_ra_pre_dir = 0u;
        g_ra_pre_slow_flag = 0u;
        /* 干扰情况下直接返 */
        return;
    }

    if (detected || far_slow_detected)
    {
        g_ra_pre_slow_flag = 1u;
        s_slow_off_cnt = 0u;
    }
    else
    {
        if (s_slow_off_cnt < 255u)
            s_slow_off_cnt++;
        if (s_slow_off_cnt >= RA_PRE_SLOW_OFF_FRAMES)
            g_ra_pre_slow_flag = 0u;
    }

    /* 滞回滤波：防止检测结果在边界处反复抖 */
    if (detected)
    {
        /* 检测到边缘丢失，递增置位计数 */
        s_on_cnt++;
        /* 重置清除计数 */
        s_off_cnt = 0;
        /* 连续确认后再置直角预检测，避免单帧边缘抖动触发 */
        if (s_on_cnt >= RA_PRE_CONFIRM_FRAMES ||
            single_edge_ra_dir != 0u ||
            route_direct_dir != 0u)
        {
            g_ra_pre_flag = 1u;
            g_ra_pre_dir = pre_dir;
        }
    }
    else
    {
        /* 未检测到边缘丢失，递增清除计数 */
        s_off_cnt++;
        /* 重置置位计数 */
        s_on_cnt = 0;
        /* 连续未检测到5帧，清除预检测标 */
        if (s_off_cnt >= 5)
        {
            g_ra_pre_flag = 0u;
            g_ra_pre_dir = 0u;
        }
    }
}

/* ========================================================================
 * 路口检测：拐点扫描 + 检测框 + 类型分类
 *
 * 方法：当逐行扫描丢失赛道时，从丢失行向上扫描寻找图像左右边缘的白色像素
 * 左边缘有白色 -> 左转弯，拐点在赛道右
 * 右边缘有白色 -> 右转弯，拐点在赛道左
 * 两侧有白-> 十字路口，拐点在中心
 * ========================================================================  */

/* 直角/路口标志~5），detect_intersection() 设置，Pid.c 中的 RA 状态机使用  */
/* 含义=右转 2=左转 3=T型左 4=T型右 5=十字  */
uint8 g_ra_flag = 0u;

/* 对称分量标志=当前帧检测到对称分量（三极管干扰等）  */
uint8 g_sym_component_flag = 0u;

/* 路口检测结果结构体，存储拐点坐标、检测框边界和路口类 */
IntersectionResult_t g_inter_result;

/* 拐点最大行号（像素2），用于 RA 状态机判断何时进入转弯阶段  */
uint8 g_ip_max_row = 0u;
uint8 g_ip_visual_row = 0u;
uint8 g_ip_ctrl_row = 0u;
uint8 g_ip_ctrl_dir = 0u;
uint8 g_ip_ctrl_score = 0u;
uint8 g_ip_ctrl_reason = 0u;
uint8 g_fast_ra_type = 0u;
uint8 g_fast_ra_left = 0u;
uint8 g_fast_ra_right = 0u;
uint8 g_fast_ra_front = 0u;

/* 路口锁定帧计数器0 表示正在锁定状态（RA 正在执行转弯动作 */
static volatile uint8 s_inter_lock_cnt = 0u;

/* 路口冷却帧计数器，锁定结束后冷却期内不再检测新路口，防止重复触 */
static volatile uint8 s_inter_cooldown_cnt = 0u;

/* 框候选有效标志，1=当前有活跃的候选检测框  */
static uint8 s_box_candidate_valid = 0u;

/* 框锁定有效标志，1=候选框已稳定足够帧数，锁定生效可用于分 */
static uint8 s_box_lock_valid = 0u;

/* 框位置稳定帧计数（候选框连续在相同位置的帧数 */
static uint8 s_box_stable_cnt = 0u;

/* 上一帧候选框的中心列号，用于判断框是否发生移 */
static int16 s_box_last_cx = 0;

/* 上一帧候选框的中心行号，用于判断框是否发生移 */
static int16 s_box_last_cy = 0;

/* 锁定后的框中心列号，分类时使用此稳定位置而非移动中的候选位 */
static int16 s_box_lock_cx = 0;

/* 锁定后的框中心行号，分类时使用此稳定位置  */
static int16 s_box_lock_cy = 0;

/* 路口类型投票缓冲区，环形存储最INTER_TYPE_VOTE_FRAMES(2) 帧的检测结 */
static uint8 s_type_vote[INTER_TYPE_VOTE_FRAMES];

/* 投票缓冲区写入索引（环形缓冲区当前位置）  */
static uint8 s_type_vote_idx = 0u;

/* 投票缓冲区已存帧数（不超INTER_TYPE_VOTE_FRAMES */
static uint8 s_type_vote_cnt = 0u;

/**
 * @brief 侧面探针白色检测：检查指定行 center_col 附近 5 像素宽区域内是否有足够白
 *
 * 用于检测图像左右边缘附近是否有白色延伸（路弯道分支）
 * 检测区域为 [center_col-2, center_col+2]，共5个像素
 * 白色像素 >= 3 即判定有白色延伸，表示该侧可能存在分支道路
 *
 * @param row        行号
 * @param center_col 探针中心列号
 * @return 1=有足够白色（>=3个）=
  */
static uint8 side_probe_has_white(int16 row, int16 center_col)
{
    /* 探针左边界（中心2 */
    int16 start = center_col - 2;
    /* 探针右边界（中心2 */
    int16 end = center_col + 2;
    /* 白色像素计数 */
    uint8 white_cnt = 0u;

    /* 行号越界检 */
    if (row < 0 || row >= TF_IMG_H) return 0u;
    /* 列号左边界保护，不能小于0  */
    if (start < 0) start = 0;
    /* 列号右边界保护，不能超过图像宽度  */
    if (end >= TF_IMG_W) end = TF_IMG_W - 1;

    /* 遍历探针区域内每个像素，统计白色像素 */
    for (int16 col = start; col <= end; col++)
    {
        /* 检查当前像素是否为白色  */
        if (Image_Binarize[row][col] == Image_WHITE)
            white_cnt++;
    }

    /* 5个像素中至少3个白色，判定为有白色延伸  */
    return (white_cnt >= 3u) ? 1u : 0u;
}

static uint8 row_white_count(int16 row, int16 col_start, int16 col_end)
{
    uint8 white_count = 0u;

    if (row < 0 || row >= TF_IMG_H) return 0u;
    if (col_start < 0) col_start = 0;
    if (col_end >= TF_IMG_W) col_end = TF_IMG_W - 1;
    if (col_start > col_end) return 0u;

    for (int16 col = col_start; col <= col_end; col++)
    {
        if (Image_Binarize[row][col] == Image_WHITE)
            white_count++;
    }

    return white_count;
}

/**
 * @brief 计算指定行指定列范围内最长连续白色像素段长度
 *
 * 用于路口检测中判断是否存在足够长的白色道路延伸
 * 连续白色段表示赛道区域，其长度可用于区分真实道路分支和噪声
 *
 * @param row       行号
 * @param col_start 起始列号
 * @param col_end   结束列号
 * @return 最长连续白色段长度（像素数
  */
static uint8 max_white_streak_on_row(int16 row, int16 col_start, int16 col_end)
{
    /* 当前连续白色像素计数  */
    uint8 streak = 0u;
    /* 遇到的最长连续白色段长度  */
    uint8 max_streak = 0u;

    /* 行号越界检 */
    if (row < 0 || row >= TF_IMG_H) return 0u;
    /* 列号左边界保 */
    if (col_start < 0) col_start = 0;
    /* 列号右边界保 */
    if (col_end >= TF_IMG_W) col_end = TF_IMG_W - 1;
    /* 起始列大于结束列，范围无 */
    if (col_start > col_end) return 0u;

    /* 遍历指定列范围内的每个像 */
    for (int16 col = col_start; col <= col_end; col++)
    {
        /* 检查当前像素是否为白色  */
        if (Image_Binarize[row][col] == Image_WHITE)
        {
            /* 白色像素，连续计数递增  */
            streak++;
            /* 更新最大连续长 */
            if (streak > max_streak)
                max_streak = streak;
        }
        else
        {
            /* 遇到黑色像素，重置连续白色计算  */
            streak = 0u;
        }
    }

    /* 返回找到的最长连续白色段长度  */
    return max_streak;
}

static uint8 glare_pre_guard_row(int16 row)
{
#if TF_GLARE_PRE_GUARD_ENABLE
    uint8 white_count = row_white_count(row, 0, TF_IMG_W - 1);
    uint8 max_streak = max_white_streak_on_row(row, 0, TF_IMG_W - 1);

    if (white_count >= TF_GLARE_PRE_ROW_WHITE_MIN &&
        max_streak >= TF_GLARE_PRE_ROW_STREAK_MIN)
    {
        return 1u;
    }
#else
    (void)row;
#endif
    return 0u;
}

/**
 * @brief 对称分量形状检测：判断左右两侧白色像素是否构成对称结构
 *
 * 用于识别三极管等元器件干扰。此类干扰在图像中表现为左右对称的白色区域，
 * 与真实路弯道的不对称特征不同，可以通过形状分析过滤
 *
 * 判定条件（全部满足才判定为对称分量）
 *   1. 左右探针行差 <= INTER_SYM_ROW_DELTA(3)（左右对称高度接近）
 *   2. 中心列偏<= INTER_SYM_CENTER_DELTA(8)（中心接近图像中心）
 *   3. 左右两侧外延白色段长< INTER_SYM_BRANCH_STREAK(18)（非真实分支道路
 *
 * @param left_row   左侧探针行号
 * @param right_row  右侧探针行号
 * @param center_col 中心列号
 * @return 1=是对称分量（干扰），0=不是
  */

static float bezier_cubic_eval(float p0, float p1, float p2, float p3, float t)
{
    float u = 1.0f - t;
    return u * u * u * p0 +
           3.0f * u * u * t * p1 +
           3.0f * u * t * t * p2 +
           t * t * t * p3;
}

static int16 clamp_i16_range(int16 value, int16 low, int16 high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

static uint8 corner_fill_dir_hint(void)
{
    uint8 dir = ra_exit_bezier_turn_dir();

    return (dir != 0u) ? dir : ra_curve_turn_dir();
}

static uint8 corner_find_branch_center_col(int16 row,
                                           int16 center_col,
                                           uint8 side,
                                           int16 *out_col)
{
    int16 col_start;
    int16 col_end;
    int16 best_start = -1;
    int16 best_end = -1;
    int16 cur_start = -1;
    uint8 cur_len = 0u;
    uint8 best_len = 0u;

    if (row < 0 || row >= (int16)TF_IMG_H)
        return 0u;

    if (side == 1u)
    {
        col_start = center_col + CORNER_FILL_BRANCH_GAP;
        col_end = (int16)(TF_IMG_W - 1);
    }
    else
    {
        col_start = 0;
        col_end = center_col - CORNER_FILL_BRANCH_GAP;
    }

    col_start = clamp_i16_range(col_start, 0, (int16)(TF_IMG_W - 1));
    col_end = clamp_i16_range(col_end, 0, (int16)(TF_IMG_W - 1));
    if (col_start > col_end)
        return 0u;

    for (int16 col = col_start; col <= col_end; col++)
    {
        if (Image_Binarize[row][col] == Image_WHITE)
        {
            if (cur_len == 0u)
                cur_start = col;
            if (cur_len < 255u)
                cur_len++;
            if (cur_len > best_len)
            {
                best_len = cur_len;
                best_start = cur_start;
                best_end = col;
            }
        }
        else
        {
            cur_len = 0u;
            cur_start = -1;
        }
    }

    if (best_len < (uint8)CORNER_FILL_BRANCH_MIN_STREAK)
        return 0u;

    if (side == 1u)
    {
        if (best_end < (int16)(TF_IMG_W - CORNER_FILL_EDGE_MARGIN))
            return 0u;
    }
    else
    {
        if (best_start > (int16)CORNER_FILL_EDGE_MARGIN)
            return 0u;
    }

    *out_col = clamp_center_col((int16)((best_start + best_end) / 2));
    return 1u;
}

static uint8 corner_find_branch_anchor(int16 base_row,
                                       int16 center_col,
                                       uint8 side,
                                       int16 *out_row,
                                       int16 *out_col)
{
    int16 top = (int16)(base_row - CORNER_FILL_EXIT_SCAN_ROWS);
    int16 bottom = (int16)(base_row + CORNER_FILL_EXIT_SCAN_ROWS);

    if (top < (int16)TF_SEARCH_END_ROW)
        top = (int16)TF_SEARCH_END_ROW;
    if (bottom >= (int16)TF_IMG_H)
        bottom = (int16)(TF_IMG_H - 1);

    for (int16 row = base_row; row <= bottom; row++)
    {
        if (corner_find_branch_center_col(row, center_col, side, out_col))
        {
            *out_row = row;
            return 1u;
        }
    }

    for (int16 row = (int16)(base_row - 1); row >= top; row--)
    {
        if (corner_find_branch_center_col(row, center_col, side, out_col))
        {
            *out_row = row;
            return 1u;
        }
    }

    return 0u;
}

static int16 corner_find_entry_row(int16 start_row, int16 bottom_row)
{
    if (start_row < 0) start_row = 0;
    if (start_row >= (int16)TF_IMG_H) start_row = (int16)(TF_IMG_H - 1);
    if (bottom_row >= (int16)TF_IMG_H) bottom_row = (int16)(TF_IMG_H - 1);

    for (int16 row = start_row; row <= bottom_row; row++)
    {
        if (g_tf.row_valid[row] && g_tf.center_line[row] != TF_INVALID)
            return row;
    }

    return -1;
}

static uint8 corner_real_line_takeover_ready(int16 jrow, int16 end_row)
{
    int16 raw_center;
    int16 la_center;
    int16 raw_error;
    int16 raw_lookahead;
    int16 raw_trend;

    uint8 exit_bezier = (ra_exit_bezier_turn_dir() != 0u) ? 1u : 0u;

    if (exit_bezier == 0u)
    {
        if (ra_curve_yaw_takeover_ready() == 0u)
            return 0u;
        if (g_tf.valid_row_count < RA_CURVE_PID_LINE_VALID_ROWS)
            return 0u;
    }
    else
    {
        if (g_tf.valid_row_count < RA_RECOVER_VALID_ROWS)
            return 0u;
    }
    if (valid_rows_in_range((int16)TF_LOOKAHEAD_END_ROW,
                            (int16)TF_LOOKAHEAD_START_ROW) <
        (uint8)CORNER_FILL_TAKEOVER_LA_ROWS)
        return 0u;

    raw_center = calc_weighted_center(jrow, end_row, 0);
    la_center = calc_weighted_center((int16)TF_LOOKAHEAD_START_ROW,
                                     (int16)TF_LOOKAHEAD_END_ROW,
                                     1u);
    raw_error = (int16)(-(raw_center - (int16)TF_IMG_CENTER) * 2);
    raw_lookahead = (int16)(-(la_center - (int16)TF_IMG_CENTER) * 2);
    raw_trend = (int16)(raw_lookahead - raw_error);

    if (exit_bezier != 0u)
    {
        if (abs_i16(raw_error) > RA_RECOVER_ERROR_MAX)
            return 0u;
        if (abs_i16(raw_lookahead) > RA_RECOVER_LOOKAHEAD_MAX)
            return 0u;
        if (abs_i16(raw_trend) > RA_RECOVER_TREND_MAX)
            return 0u;
    }
    else
    {
        if (abs_i16(raw_error) > RA_CURVE_PID_LINE_ERR_MAX)
            return 0u;
        if (abs_i16(raw_lookahead) > RA_CURVE_PID_LINE_LA_MAX)
            return 0u;
        if (abs_i16(raw_trend) > RA_CURVE_PID_LINE_TREND_MAX)
            return 0u;
    }

    return 1u;
}

static void corner_fill_apply(int16 jrow, int16 end_row)
{
#if CORNER_FILL_ENABLE
    uint8 active_side = ra_exit_bezier_turn_dir();
    uint8 active_ip_row = ra_exit_bezier_ip_row();
    uint8 exit_bezier = (active_side != 0u) ? 1u : 0u;
    uint8 side;
    int16 ip_row;
    int16 entry_row;
    int16 exit_row;
    int16 p0_row;
    int16 p3_row;
    int16 p0_col;
    int16 p3_col;
    int16 dir_in = 0;
    int16 span;
    float l1;
    float l2;
    float p0;
    float p1;
    float p2;
    float p3;
    float dir_out;

    if (active_side == 0u)
    {
        active_side = ra_curve_turn_dir();
        active_ip_row = ra_curve_ip_row();
    }
    side = corner_fill_dir_hint();

    if (side == 0u)
        return;
    if (exit_bezier != 0u &&
        g_tf.valid_row_count < RA_EXIT_BEZIER_VALID_ROWS)
        return;
    if (corner_real_line_takeover_ready(jrow, end_row))
    {
        return;
    }

    if (active_side != 0u && active_ip_row != 0u)
        ip_row = (int16)active_ip_row;
    else if (s_first_miss_row >= (int16)INTER_MIN_MISS_ROW)
        ip_row = s_first_miss_row;
    else if (g_inter_result.left_ip.valid)
        ip_row = g_inter_result.left_ip.row;
    else if (g_inter_result.right_ip.valid)
        ip_row = g_inter_result.right_ip.row;
    else
        return;

    ip_row = clamp_i16_range(ip_row,
                             (int16)(end_row + CORNER_FILL_EXIT_MARGIN + 1),
                             (int16)(jrow - CORNER_FILL_ENTRY_MARGIN));
    entry_row = (int16)(ip_row + CORNER_FILL_ENTRY_MARGIN);
    exit_row = (int16)(ip_row - CORNER_FILL_EXIT_MARGIN);
    entry_row = clamp_i16_range(entry_row, end_row, jrow);
    exit_row = clamp_i16_range(exit_row, end_row, (int16)(entry_row - 1));

    p0_row = corner_find_entry_row(entry_row, jrow);
    if (p0_row < 0)
        return;

    p0_col = g_tf.center_line[p0_row];
    if (p0_col == TF_INVALID)
        return;

    if (exit_bezier != 0u)
    {
        p3_row = exit_row;
        p3_col = p0_col;
    }
    else if (!corner_find_branch_anchor(exit_row, p0_col, side, &p3_row, &p3_col))
    {
        if (active_side == 0u)
            return;
        p3_row = exit_row;
        p3_col = (side == 1u) ?
                 (int16)(TF_IMG_W - 1 - active_track_half_width()) :
                 active_track_half_width();
        p3_col = clamp_center_col(p3_col);
    }

    {
        int16 dir = (side == 1u) ? 1 : -1;
        int16 inner_shift = (int16)(active_track_half_width() * 2 + 6);
        (void)p3_col;
        inner_shift = clamp_i16_range(inner_shift,
                                      (int16)CORNER_FILL_INNER_SHIFT_MIN,
                                      (int16)CORNER_FILL_INNER_SHIFT_MAX);
        p3_col = clamp_center_col((int16)(p0_col + dir * inner_shift));
    }

    if (p3_row >= p0_row)
        return;

    span = (int16)(p0_row - p3_row);
    if (span < (int16)CORNER_FILL_MIN_SPAN)
        return;

    if (p0_row + 1 <= jrow &&
        g_tf.row_valid[p0_row + 1] &&
        g_tf.center_line[p0_row + 1] != TF_INVALID)
    {
        dir_in = (int16)(p0_col - g_tf.center_line[p0_row + 1]);
        dir_in = clamp_i16_range(dir_in,
                                 (int16)(-CORNER_FILL_DIR_MAX),
                                 (int16)CORNER_FILL_DIR_MAX);
    }

    l1 = (float)span * ((float)CORNER_FILL_L1_PCT * 0.01f);
    l2 = (float)span * ((float)CORNER_FILL_L2_PCT * 0.01f);
    p0 = (float)p0_col;
    p1 = p0 + (float)dir_in * l1;
    p3 = (float)p3_col;
    dir_out = (side == 1u) ? CORNER_FILL_OUT_SLOPE : -CORNER_FILL_OUT_SLOPE;
    p2 = p3 - dir_out * l2;

    for (int16 row = (int16)(p0_row - 1); row >= p3_row; row--)
    {
        float t = (float)(p0_row - row) / (float)span;
        int16 col = clamp_center_col((int16)(bezier_cubic_eval(p0, p1, p2, p3, t) + 0.5f));

        g_corner_fill_center[row] = col;
        g_corner_fill_valid[row] = 1u;
        g_corner_fill_active = 1u;
    }
#else
    (void)jrow;
    (void)end_row;
#endif
}

static uint8 symmetric_component_shape(int16 left_row, int16 right_row, int16 center_col)
{
    /* 行号无效检查，任一侧行号为负则不是对称分量  */
    if (left_row < 0 || right_row < 0)
        return 0u;

    /* 条件1：左右行差不能超INTER_SYM_ROW_DELTA(3)，否则不对称  */
    if (abs_i16(left_row - right_row) > INTER_SYM_ROW_DELTA)
        return 0u;

    /* 条件2：中心列不能偏离图像中心超过 INTER_SYM_CENTER_DELTA(8)  */
    if (abs_i16(center_col - (int16)TF_IMG_CENTER) > INTER_SYM_CENTER_DELTA)
        return 0u;

    /* 检查左侧外延白色段长度（从到中最小赛道宽度）  */
    uint8 left_streak = max_white_streak_on_row(
        left_row, 0, center_col - TF_MIN_TRACK_WIDTH);
    /* 检查右侧外延白色段长度（从中心+最小赛道宽度到末列 */
    uint8 right_streak = max_white_streak_on_row(
        right_row, center_col + TF_MIN_TRACK_WIDTH, TF_IMG_W - 1);

    /* 条件3：若任一侧外延白色段过长(>=18)，说明是真实道路分支，不是对称干 */
    if (left_streak >= INTER_SYM_BRANCH_STREAK ||
        right_streak >= INTER_SYM_BRANCH_STREAK)
    {
        return 0u;
    }

    /* 满足所有条件，判定为对称分量（三极管等干扰 */
    return 1u;
}

/**
 * @brief 检测指定行是否存在对称分量（左右两侧同时有白色像素且形状对称）
 *
 * 先检查左右探针列是否都有白色，再调用 symmetric_component_shape 进行形状验证
 * 两个条件都满足才判定为对称分量
 *
 * @param row        行号
 * @param center_col 中心列号（用于形状检测）
 * @return 1=存在对称分量=不存
  */
static uint8 symmetric_component_at_row(int16 row, int16 center_col)
{
    /* 左侧探针列必须检测到白色  */
    /* 右侧探针列也必须检测到白色  */
    /* 任一侧无白色则不是对称分 */
    if (!side_probe_has_white(row, ip_left_col) ||
        !side_probe_has_white(row, ip_right_col))
    {
        return 0u;
    }

    /* 左右都有白色，进一步检查形状是否对 */
    return symmetric_component_shape(row, row, center_col);
}

/**
 * @brief 从丢失区域向上扫描寻找拐
 *
 * lost_row 向上扫描，检查左右探针列（ip_left_col / ip_right_col
 * 是否有白色像素（表示该侧有道路延伸）
 *
 * 通过 found_side 返回哪侧有路
 *   - 1 = 右侧有路（右转弯场景
 *   - 2 = 左侧有路（左转弯场景
 *   - 3 = 两侧都有路（十字路口场景
 *
 * 找到拐点后还会进行对称分量检查，排除三极管干扰
 *
 * @param lost_row     丢失区域起始行号
 * @param last_center  最后有效中点列号（作为拐点列号参考）
 * @param ip           输出参数：拐点信息（valid/row/col
 * @param found_side   输出参数：哪侧有路（1=右，2=左，3=双侧
 * @return 1=找到有效拐点=未找
  */
static uint8 find_ip_from_lost_row(int16 lost_row, int16 last_center,
                                    InflectionPoint_t *ip, uint8 *found_side)
{
    /* 初始化拐点为无效状 */
    ip->valid = 0u; ip->row = 0; ip->col = 0;
    /* 初始化有路侧为无  */
    *found_side = 0u;

    /* 左侧首次检测到白色的行号，-1表示未检测到  */
    int16 left_white_row = -1;
    /* 右侧首次检测到白色的行号，-1表示未检测到  */
    int16 right_white_row = -1;

    /* 从丢失行向上扫描到图像顶部（行号递减 */
    for (int16 row = lost_row; row >= (int16)TF_SEARCH_END_ROW; row--)
    {
        /* 检查左侧探针列是否有白色（尚未找到时才检查）  */
        if (left_white_row < 0 &&
            (side_probe_has_white(row, ip_left_col) ||
             inter_side_branch_has_road(row, last_center, 2u)))
        {
            left_white_row = row;
        }

        /* 检查右侧探针列是否有白色（尚未找到时才检查）  */
        if (right_white_row < 0 &&
            (side_probe_has_white(row, ip_right_col) ||
             inter_side_branch_has_road(row, last_center, 1u)))
        {
            right_white_row = row;
        }

        /* 两侧都找到白色延伸，无需继续向上扫描  */
        if (left_white_row >= 0 && right_white_row >= 0)
            break;
    }

    /* 对称分量检查：若左右白色像素构成对称结构，判定为干扰，不作为拐 */
    if (symmetric_component_shape(left_white_row, right_white_row, last_center))
    {
        /* 标记拐点无效  */
        ip->valid = 0u;
        /* 无有路侧  */
        *found_side = 0u;
        /* 设置对称分量标志  */
        g_sym_component_flag = 1u;
        /* 对称分量干扰，返回未找到  */
        return 0u;
    }

    /* 拐点行号，取离相机更近的行（行号更大的）  */
    int16 ip_row = -1;
    /* 左右两侧都检测到白色延伸  */
    if (left_white_row >= 0 && right_white_row >= 0)
    {
        if (g_ra_pre_flag != 0u && (g_ra_pre_dir == 1u || g_ra_pre_dir == 2u))
        {
            *found_side = g_ra_pre_dir;
            if (abs_i16(left_white_row - right_white_row) <= INTER_BRANCH_PAIR_ROW_DELTA)
                ip_row = (int16)((left_white_row + right_white_row) / 2);
            else
                ip_row = (g_ra_pre_dir == 1u) ? right_white_row : left_white_row;
        }
        else
        {
            if (abs_i16(left_white_row - right_white_row) <= INTER_BRANCH_PAIR_ROW_DELTA)
            {
                ip_row = (int16)((left_white_row + right_white_row) / 2);
                *found_side = 3u;
            }
            /* 左侧行号更大 = 左侧道路更近 = 左转场景  */
            else if (left_white_row > right_white_row)
            {
                ip_row = left_white_row;
                *found_side = 2u;
            }
            /* 右侧行号更大 = 右侧道路更近 = 右转场景  */
            else if (right_white_row > left_white_row)
            {
                ip_row = right_white_row;
                *found_side = 1u;
            }
            /* 同行 = 两侧距离相同 = 十字路口场景  */
            else
            {
                ip_row = (int16)((left_white_row + right_white_row) / 2);
                *found_side = 3u;
            }
        }
    }
    /* 仅右侧检测到白色延伸  */
    else if (right_white_row >= 0)
    {
        /* 拐点行为右侧 */
        ip_row = right_white_row;
        /* 右侧有路 -> 右转  */
        *found_side = 1u;
    }
    /* 仅左侧检测到白色延伸  */
    else if (left_white_row >= 0)
    {
        /* 拐点行为左侧 */
        ip_row = left_white_row;
        /* 左侧有路 -> 左转  */
        *found_side = 2u;
    }

    /* 未找到任何白色延伸，返回未找 */
    if (ip_row < 0) return 0u;

    if (ip_row + INTER_IP_ROW_BIAS <= lost_row)
        ip_row = (int16)(ip_row + INTER_IP_ROW_BIAS);
    else
        ip_row = lost_row;

    /* 设置拐点有效标志  */
    ip->valid = 1u;
    /* 设置拐点行号  */
    ip->row = ip_row;
    /* 使用最后有效中点列号作为拐点列 */
    ip->col = last_center;
    /* 返回找到有效拐点  */
    return 1u;
}

static uint8 find_probe_trigger_ip(InflectionPoint_t *ip, uint8 *found_side)
{
    int16 left_row = -1;
    int16 right_row = -1;
    int16 start_row = s_jidian_row;

    ip->valid = 0u;
    ip->row = 0;
    ip->col = 0;
    *found_side = 0u;

    if (start_row >= TF_IMG_H)
        start_row = TF_IMG_H - 1;

    for (int16 row = start_row; row >= (int16)INTER_BOX_START_ROW; row--)
    {
        uint8 left_probe = side_probe_has_white(row, ip_left_col);
        uint8 right_probe = side_probe_has_white(row, ip_right_col);

        if (left_row < 0 &&
            left_probe &&
            (g_ra_pre_flag != 0u ||
             inter_side_branch_has_road(row, s_last_valid_center, 2u)))
        {
            left_row = row;
        }

        if (right_row < 0 &&
            right_probe &&
            (g_ra_pre_flag != 0u ||
             inter_side_branch_has_road(row, s_last_valid_center, 1u)))
        {
            right_row = row;
        }

        if (left_row >= 0 && right_row >= 0)
            break;
    }

    if (left_row < 0 && right_row < 0)
        return 0u;

    if (left_row >= 0 && right_row >= 0 &&
        symmetric_component_shape(left_row, right_row, s_last_valid_center))
    {
        g_sym_component_flag = 1u;
        return 0u;
    }

    if (left_row >= 0 && right_row >= 0)
    {
        if (abs_i16(left_row - right_row) <= INTER_BRANCH_PAIR_ROW_DELTA)
        {
            ip->row = (int16)((left_row + right_row) / 2);
            *found_side = 3u;
        }
        else if (left_row > right_row)
        {
            ip->row = left_row;
            *found_side = 2u;
        }
        else
        {
            ip->row = right_row;
            *found_side = 1u;
        }
    }
    else if (right_row >= 0)
    {
        ip->row = right_row;
        *found_side = 1u;
    }
    else
    {
        ip->row = left_row;
        *found_side = 2u;
    }

    if (ip->row + INTER_IP_ROW_BIAS < TF_IMG_H)
        ip->row = (int16)(ip->row + INTER_IP_ROW_BIAS);
    else
        ip->row = TF_IMG_H - 1;

    ip->valid = 1u;
    ip->col = s_last_valid_center;
    return 1u;
}

/**
 * @brief 检查水平带状区域是否有足够白色像素（用于路口分类）
 *
 * center_row 为中心、厚度为 INTER_BAND_THICKNESS(3) 行的水平带内
 * 检[col_start, col_end] 范围内白色像素数和最长连续白色段是否达标
 * 两个条件同时满足才判定有足够白色道路
 *
 * @param center_row  带中心行
 * @param col_start   起始列号
 * @param col_end     结束列号
 * @param min_white   最少白色像素数阈
 * @param min_streak  最长连续白色段阈
 * @return 1=有足够白色道路，0=
  */
static uint8 inter_horizontal_band_has_road(int16 center_row,
                                            int16 col_start,
                                            int16 col_end,
                                            uint8 min_white,
                                            uint8 min_streak)
{
    /* 带的半厚度（3/2=1 */
    int16 half = (int16)(INTER_BAND_THICKNESS / 2);
    /* 带的起始 */
    int16 row_start = center_row - half;
    /* 带的结束 */
    int16 row_end = center_row + half;
    /* 总白色像素计算  */
    uint16 white_count = 0u;
    /* 带内最长连续白色段长度  */
    uint8 max_streak = 0u;

    /* 行边界保护：不超出图像范 */
    if (row_start < 0) row_start = 0;
    if (row_end >= TF_IMG_H) row_end = TF_IMG_H - 1;
    /* 列边界保护：不超出图像范 */
    if (col_start < 0) col_start = 0;
    if (col_end >= TF_IMG_W) col_end = TF_IMG_W - 1;
    /* 范围有效性检 */
    if (row_start > row_end || col_start > col_end) return 0u;

    /* 遍历带内所有行，统计白色像素和最长连续白色段  */
    for (int16 r = row_start; r <= row_end; r++)
    {
        /* 当前行的连续白色计数  */
        uint8 streak = 0u;
        /* 遍历带内所有列  */
        for (int16 c = col_start; c <= col_end; c++)
        {
            /* 检查当前像素是否为白色  */
            if (is_white(r, c))
            {
                /* 白色像素，总数和连续计数递增  */
                white_count++;
                streak++;
                /* 更新最大连续长 */
                if (streak > max_streak) max_streak = streak;
            }
            else
            {
                /* 遇到黑色像素，重置连续计算  */
                streak = 0u;
            }
        }
    }

    /* 白色像素数和连续段长度都达标才返回有 */
    return (white_count >= min_white && max_streak >= min_streak) ? 1u : 0u;
}

static uint8 inter_top_window_has_road(int16 top,
                                       int16 bottom,
                                       int16 left,
                                       int16 right)
{
    int16 scan_end;

    if (top < 0) top = 0;
    if (bottom >= TF_IMG_H) bottom = TF_IMG_H - 1;
    scan_end = (int16)(top + INTER_TOP_SCAN_ROWS);
    if (scan_end > bottom) scan_end = bottom;

    for (int16 row = top; row <= scan_end; row++)
    {
        if (inter_horizontal_band_has_road(row, left, right, 4u, 3u))
            return 1u;
    }

    return 0u;
}

static uint8 inter_side_window_has_road(int16 top,
                                        int16 bottom,
                                        int16 left,
                                        int16 right,
                                        uint8 side)
{
    int16 col_start;
    int16 col_end;
    uint8 max_streak = 0u;
    uint8 hit_rows = 0u;
    uint16 white_sum = 0u;

    if (top < 0) top = 0;
    if (bottom >= TF_IMG_H) bottom = TF_IMG_H - 1;
    if (left < 0) left = 0;
    if (right >= TF_IMG_W) right = TF_IMG_W - 1;
    if (top > bottom || left > right) return 0u;

    if (side == 1u)
    {
        col_start = (int16)(right - INTER_SIDE_SCAN_COLS);
        col_end = right;
    }
    else
    {
        col_start = left;
        col_end = (int16)(left + INTER_SIDE_SCAN_COLS);
    }

    if (col_start < left) col_start = left;
    if (col_end > right) col_end = right;
    if (col_start < 0) col_start = 0;
    if (col_end >= TF_IMG_W) col_end = TF_IMG_W - 1;
    if (col_start > col_end) return 0u;

    for (int16 row = top; row <= bottom; row++)
    {
        uint8 streak = max_white_streak_on_row(row, col_start, col_end);
        if (streak > max_streak) max_streak = streak;
        if (streak >= 3u)
        {
            if (hit_rows < 255u) hit_rows++;
            white_sum = (uint16)(white_sum + streak);
        }
    }

    if (max_streak >= INTER_SIDE_CROSS_MIN_STREAK &&
        hit_rows >= INTER_SIDE_HIT_ROWS)
        return 1u;

    if (hit_rows >= INTER_SIDE_HIT_ROWS &&
        white_sum >= INTER_SIDE_HIT_WHITE)
    {
        return 1u;
    }

    return 0u;
}

static uint8 inter_side_crossing_has_road(int16 top,
                                          int16 bottom,
                                          int16 left,
                                          int16 right,
                                          uint8 side)
{
    int16 height;
    int16 row_start;
    int16 row_end;
    int16 center;
    uint8 hit_rows = 0u;

    if (top < 0) top = 0;
    if (bottom >= TF_IMG_H) bottom = TF_IMG_H - 1;
    if (left < 0) left = 0;
    if (right >= TF_IMG_W) right = TF_IMG_W - 1;
    if (top > bottom || left > right) return 0u;

    height = (int16)(bottom - top);
    row_start = (int16)(top + (height * INTER_SIDE_CROSS_TOP_PCT) / 100);
    row_end = (int16)(top + (height * INTER_SIDE_CROSS_BOTTOM_PCT) / 100);
    center = (int16)((left + right) / 2);

    if (row_start < top) row_start = top;
    if (row_end > bottom) row_end = bottom;
    if (row_start > row_end) return 0u;

    for (int16 row = row_start; row <= row_end; row++)
    {
        uint8 streak = 0u;

        if (side == 1u)
        {
            int16 edge_start = (int16)(right - INTER_SIDE_SCAN_COLS);
            int16 seed_col = -1;

            if (edge_start < center) edge_start = center;

            for (int16 col = right; col >= edge_start; col--)
            {
                if (is_white(row, col))
                {
                    seed_col = col;
                    break;
                }
            }

            if (seed_col >= 0)
            {
                for (int16 col = seed_col; col >= center; col--)
                {
                    if (!is_white(row, col))
                        break;
                    streak++;
                }
            }
        }
        else
        {
            int16 edge_end = (int16)(left + INTER_SIDE_SCAN_COLS);
            int16 seed_col = -1;

            if (edge_end > center) edge_end = center;

            for (int16 col = left; col <= edge_end; col++)
            {
                if (is_white(row, col))
                {
                    seed_col = col;
                    break;
                }
            }

            if (seed_col >= 0)
            {
                for (int16 col = seed_col; col <= center; col++)
                {
                    if (!is_white(row, col))
                        break;
                    streak++;
                }
            }
        }

        if (streak >= INTER_SIDE_CROSS_MIN_STREAK)
        {
            hit_rows++;
            if (hit_rows >= INTER_SIDE_CROSS_HIT_ROWS)
                return 1u;
        }
        else
        {
            hit_rows = 0u;
        }
    }

    return 0u;
}

/**
 * @brief 检查垂直带状区域是否有足够白色像素（用于路口分类）
 *
 * center_col 为中心、厚度为 INTER_BAND_THICKNESS(3) 列的垂直带内
 * 检[row_start, row_end] 范围内白色像素数和最长连续白色段是否达标
 * 两个条件同时满足才判定有足够白色道路
 *
 * @param center_col  带中心列
 * @param row_start   起始行号
 * @param row_end     结束行号
 * @param min_white   最少白色像素数阈
 * @param min_streak  最长连续白色段阈
 * @return 1=有足够白色道路，0=
  */
static uint8 inter_vertical_band_has_road(int16 center_col,
                                          int16 row_start,
                                          int16 row_end,
                                          uint8 min_white,
                                          uint8 min_streak)
{
    /* 带的半厚度（3/2=1 */
    int16 half = (int16)(INTER_BAND_THICKNESS / 2);
    /* 带的起始 */
    int16 col_start = center_col - half;
    /* 带的结束 */
    int16 col_end = center_col + half;
    /* 总白色像素计算  */
    uint16 white_count = 0u;
    /* 带内最长连续白色段长度  */
    uint8 max_streak = 0u;

    /* 行边界保护：不超出图像范 */
    if (row_start < 0) row_start = 0;
    if (row_end >= TF_IMG_H) row_end = TF_IMG_H - 1;
    /* 列边界保护：不超出图像范 */
    if (col_start < 0) col_start = 0;
    if (col_end >= TF_IMG_W) col_end = TF_IMG_W - 1;
    /* 范围有效性检 */
    if (row_start > row_end || col_start > col_end) return 0u;

    /* 遍历带内所有列，统计白色像素和最长连续白色段  */
    for (int16 c = col_start; c <= col_end; c++)
    {
        /* 当前列的连续白色计数  */
        uint8 streak = 0u;
        /* 遍历带内所有行  */
        for (int16 r = row_start; r <= row_end; r++)
        {
            /* 检查当前像素是否为白色  */
            if (is_white(r, c))
            {
                /* 白色像素，总数和连续计数递增  */
                white_count++;
                streak++;
                /* 更新最大连续长 */
                if (streak > max_streak) max_streak = streak;
            }
            else
            {
                /* 遇到黑色像素，重置连续计算  */
                streak = 0u;
            }
        }
    }

    /* 白色像素数和连续段长度都达标才返回有 */
    return (white_count >= min_white && max_streak >= min_streak) ? 1u : 0u;
}

/**
 * @brief 检查指定侧是否有足够长的白色分支道路（用于纯侧路检测）
 *
 * row 行为中心、厚度为 INTER_BAND_THICKNESS(3) 行的带内
 * 检center_col 以外（左侧或右侧）是否有长度 >= INTER_BRANCH_MIN_STREAK 的连续白色段
 * 用于检测纯侧路（flag 1/2）场景
 *
 * @param row        中心行号
 * @param center_col 中心列号（赛道中心）
 * @param side       1=检查右侧，2=检查左
 * @return 1=有足够长的分支道路，0=
  */
static uint8 inter_side_branch_has_road(int16 row,
                                        int16 center_col,
                                        uint8 side)
{
    /* 带的起始 */
    int16 row_start = row - (int16)INTER_BRANCH_AREA_HALF_ROWS;
    /* 带的结束 */
    int16 row_end = row + (int16)INTER_BRANCH_AREA_HALF_ROWS;
    /* 检测区域的起始列和结束 */
    int16 col_start;
    int16 col_end;
    /* 带内最大连续白色段长度  */
    uint8 max_streak = 0u;
    uint16 streak_sum = 0u;
    uint8 hit_rows = 0u;
    int16 branch_gap = active_track_half_width() + 2;

    /* 行边界保 */
    if (row_start < 0) row_start = 0;
    if (row_end >= TF_IMG_H) row_end = TF_IMG_H - 1;

    if (branch_gap < (int16)(TF_MIN_TRACK_WIDTH + 2))
        branch_gap = (int16)(TF_MIN_TRACK_WIDTH + 2);

    /* 根据检测侧确定列范 */
    if (side == 1u)
    {
        /* 右侧：从 center_col + 最小赛道宽图像右边 */
        col_start = center_col + branch_gap;
        col_end = TF_IMG_W - 1;
    }
    else
    {
        /* 左侧：从 图像左边center_col - 最小赛道宽 */
        col_start = 0;
        col_end = center_col - branch_gap;
    }

    /* 列边界保 */
    if (col_start < 0) col_start = 0;
    if (col_end >= TF_IMG_W) col_end = TF_IMG_W - 1;
    /* 列范围有效性检 */
    if (col_start > col_end) return 0u;

    /* 遍历带内所有行，取各行最大连续白色段的最大 */
    for (int16 r = row_start; r <= row_end; r++)
    {
        /* 计算当前行在指定列范围内的最长连续白色段  */
        uint8 streak = max_white_streak_on_row(r, col_start, col_end);
        /* 更新全局最大 */
        if (streak > max_streak)
            max_streak = streak;

        if (streak >= INTER_BAND_MIN_STREAK)
        {
            streak_sum = (uint16)(streak_sum + streak);
            hit_rows++;
        }
    }

    /* 连续白色段长度达到分支阈值才返回有分支道 */
    if (max_streak >= INTER_BRANCH_MIN_STREAK)
        return 1u;

    if (hit_rows >= INTER_BRANCH_AREA_MIN_ROWS &&
        streak_sum >= INTER_BRANCH_AREA_MIN_WHITE)
    {
        return 1u;
    }

    return 0u;
}

/* Strong side-branch evidence tuned from saved frames; components are shorter.  */
static uint8 inter_side_branch_strong_has_road(int16 row,
                                               int16 center_col,
                                               uint8 side)
{
    int16 row_start = row - (int16)INTER_BRANCH_AREA_HALF_ROWS;
    int16 row_end = row + (int16)INTER_BRANCH_AREA_HALF_ROWS;
    int16 col_start;
    int16 col_end;
    uint8 max_streak = 0u;
    uint16 streak_sum = 0u;
    uint8 hit_rows = 0u;
    int16 branch_gap = active_track_half_width() + 2;

    if (row_start < 0) row_start = 0;
    if (row_end >= TF_IMG_H) row_end = TF_IMG_H - 1;

    if (branch_gap < (int16)(TF_MIN_TRACK_WIDTH + 2))
        branch_gap = (int16)(TF_MIN_TRACK_WIDTH + 2);

    if (side == 1u)
    {
        col_start = center_col + branch_gap;
        col_end = TF_IMG_W - 1;
    }
    else
    {
        col_start = 0;
        col_end = center_col - branch_gap;
    }

    if (col_start < 0) col_start = 0;
    if (col_end >= TF_IMG_W) col_end = TF_IMG_W - 1;
    if (col_start > col_end) return 0u;

    for (int16 r = row_start; r <= row_end; r++)
    {
        uint8 streak = max_white_streak_on_row(r, col_start, col_end);
        if (streak > max_streak)
            max_streak = streak;

        if (streak >= INTER_BAND_MIN_STREAK)
        {
            streak_sum = (uint16)(streak_sum + streak);
            hit_rows++;
        }
    }

    if (max_streak >= INTER_BRANCH_STRONG_MIN_STREAK)
        return 1u;

    if (hit_rows >= INTER_BRANCH_STRONG_AREA_MIN_ROWS &&
        streak_sum >= INTER_BRANCH_STRONG_AREA_MIN_WHITE)
    {
        return 1u;
    }

    return 0u;
}

static uint8 inter_side_edge_reach_has_road(int16 row,
                                            int16 center_col,
                                            uint8 side)
{
    int16 row_start = row - (int16)INTER_BRANCH_AREA_HALF_ROWS;
    int16 row_end = row + (int16)INTER_BRANCH_AREA_HALF_ROWS;
    int16 gap = active_track_half_width() + 2;
    uint8 hit_rows = 0u;

    if (row_start < 0) row_start = 0;
    if (row_end >= TF_IMG_H) row_end = TF_IMG_H - 1;
    if (gap < (int16)(TF_MIN_TRACK_WIDTH + 2))
        gap = (int16)(TF_MIN_TRACK_WIDTH + 2);

    for (int16 r = row_start; r <= row_end; r++)
    {
        uint8 streak = 0u;
        int16 seed = -1;

        if (side == 1u)
        {
            int16 min_col = center_col + gap;
            int16 edge_limit = (int16)(TF_IMG_W - 1 - INTER_EDGE_TOUCH_MARGIN);

            if (min_col < 0) min_col = 0;
            if (edge_limit < min_col) edge_limit = min_col;

            for (int16 c = TF_IMG_W - 1; c >= edge_limit; c--)
            {
                if (is_white(r, c))
                {
                    seed = c;
                    break;
                }
            }

            if (seed >= 0)
            {
                for (int16 c = seed; c >= min_col; c--)
                {
                    if (!is_white(r, c))
                        break;
                    streak++;
                }
            }
        }
        else
        {
            int16 max_col = center_col - gap;
            int16 edge_limit = (int16)INTER_EDGE_TOUCH_MARGIN;

            if (max_col >= TF_IMG_W) max_col = TF_IMG_W - 1;
            if (edge_limit > max_col) edge_limit = max_col;

            for (int16 c = 0; c <= edge_limit; c++)
            {
                if (is_white(r, c))
                {
                    seed = c;
                    break;
                }
            }

            if (seed >= 0)
            {
                for (int16 c = seed; c <= max_col; c++)
                {
                    if (!is_white(r, c))
                        break;
                    streak++;
                }
            }
        }

        if (streak >= INTER_EDGE_CONNECT_MIN_STREAK)
        {
            hit_rows++;
            if (hit_rows >= INTER_EDGE_CONNECT_HIT_ROWS)
                return 1u;
        }
    }

    return 0u;
}

/* Pick a stable inflection-point column from the recent center buffer.  */
static void apply_ip_col_from_buffer(InflectionPoint_t *ip, uint8 found_side)
{
    /* 仅在拐点有效且缓冲区有数据时才从中点缓冲区选取  */
    if (ip->valid && s_center_buf_cnt > 0u)
    {
        /* 获取缓冲区偏移量（取倒数第几个）  */
        uint8 offset = (uint8)ip_col_offset;
        /* 获取缓冲区已存元素总数  */
        uint8 total = s_center_buf_cnt;
        /* 偏移量不超过已有数据量（取最早可用的 */
        if (offset >= total) offset = total - 1u;
        /* 计算环形缓冲区索引：取倒数(offset+1) 个有效中 */
        uint8 idx = (uint8)((total - 1u - offset) % IP_COL_BUF_SIZE);
        /* 用缓冲区中的稳定中点替换拐点列号  */
        ip->col = s_center_buf[idx];
    }

    /* 拐点无效则无需偏移，直接返 */
    if (!ip->valid)
        return;

    /* 根据检测到的有路侧，向对应方向偏移拐点列号  */
    if (found_side == 1u)
        /* 右侧有路：拐点列号向右偏移，帮助检测框罩住右侧岔路  */
        ip->col = clamp_center_col((int16)(ip->col + INTER_IP_SIDE_BIAS));
    else if (found_side == 2u)
        /* 左侧有路：拐点列号向左偏移，帮助检测框罩住左侧岔路  */
        ip->col = clamp_center_col((int16)(ip->col - INTER_IP_SIDE_BIAS));
}

/**
 * @brief int16 取绝对
 *
 * 用于对称分量检测中的距离计算等场景
 *
 * @param v 输入值（int16 类型
 * @return 输入值的绝对
  */
static int16 abs_i16(int16 v)
{
    if (v == (int16)(-32767 - 1))
        return 32767;
    return (v < 0) ? (int16)(-v) : v;
}

/**
 * @brief 重置检测框锁定状
 *
 * 清除候选框、锁定框、稳定计数和类型投票数据
 * 在路口处理完成或检测条件不满足时调用，将框状态恢复到初始值
  */
static void reset_box_lock(void)
{
    /* 清除候选框有效标志  */
    s_box_candidate_valid = 0u;
    /* 清除锁定框有效标 */
    s_box_lock_valid = 0u;
    /* 清除框位置稳定计算  */
    s_box_stable_cnt = 0u;
    /* 重置投票缓冲区写入索 */
    s_type_vote_idx = 0u;
    /* 清除投票缓冲区已存帧 */
    s_type_vote_cnt = 0u;

    /* 清零投票缓冲区中所有元 */
    for (uint8 i = 0u; i < INTER_TYPE_VOTE_FRAMES; i++)
        s_type_vote[i] = 0u;
}

/**
 * @brief 清除路口检测结果结构体
 *
 * g_inter_result 的所有字段清零，包括左右拐点、检测框边界和路口类型
 * 同时清除 g_ip_max_row（拐点最大行号）
  */
static void clear_inter_result(void)
{
    /* 左拐点有效标志清 */
    g_inter_result.left_ip.valid = 0u;
    /* 左拐点行号清 */
    g_inter_result.left_ip.row = 0;
    /* 左拐点列号清 */
    g_inter_result.left_ip.col = 0;
    /* 右拐点有效标志清 */
    g_inter_result.right_ip.valid = 0u;
    /* 右拐点行号清 */
    g_inter_result.right_ip.row = 0;
    /* 右拐点列号清 */
    g_inter_result.right_ip.col = 0;
    /* 检测框顶行清零  */
    g_inter_result.box_top = 0u;
    /* 检测框底行清零  */
    g_inter_result.box_bottom = 0u;
    /* 检测框左列清零  */
    g_inter_result.box_left = 0u;
    /* 检测框右列清零  */
    g_inter_result.box_right = 0u;
    /* 检测到的路口类型清 */
    g_inter_result.detected_type = 0u;
    g_inter_result.confidence = 0u;
    g_inter_result.evidence = 0u;
    /* 拐点最大行号清 */
    g_ip_max_row = 0u;
    g_ip_visual_row = 0u;
}

void track_intersection_suppress_after_turn(void)
{
    s_inter_post_turn_suppress_cnt = INTER_POST_TURN_SUPPRESS_FRAMES;
    s_inter_lock_cnt = 0u;
    s_inter_cooldown_cnt = 0u;
    g_ra_flag = 0u;
    g_ra_pre_flag = 0u;
    g_ra_pre_dir = 0u;
    g_ra_pre_slow_flag = 0u;
    reset_box_lock();
    clear_inter_result();
}

static uint8 valid_rows_in_range(int16 row_start, int16 row_end)
{
    uint8 count = 0u;

    if (row_start < 0) row_start = 0;
    if (row_end >= TF_IMG_H) row_end = TF_IMG_H - 1;
    if (row_start > row_end) return 0u;

    for (int16 row = row_start; row <= row_end; row++)
    {
        if (g_tf.row_valid[row])
            count++;
    }

    return count;
}

static uint8 inline_component_pre_guard(int16 far_center_span)
{
    if (g_tf.line_lost != 0u)
        return 0u;
    if (g_tf.valid_row_count < INTER_COMPONENT_PRE_VALID_ROWS)
        return 0u;
    if (valid_rows_in_range((int16)INTER_BOX_START_ROW,
                            (int16)RA_PRE_START_ROW) <
        INTER_COMPONENT_PRE_LOWER_ROWS)
        return 0u;
    if (valid_rows_in_range((int16)INTER_FAST_PASS_TOP_START_ROW,
                            (int16)INTER_FAST_PASS_TOP_END_ROW) <
        INTER_COMPONENT_PRE_TOP_ROWS)
        return 0u;
    if (abs_i16(g_tf.error) > INTER_COMPONENT_PRE_ERR_LIMIT)
        return 0u;
    if (abs_i16(g_tf.lookahead_error) > INTER_COMPONENT_PRE_LA_LIMIT)
        return 0u;
    if (abs_i16(g_tf.error_trend) > INTER_COMPONENT_PRE_TREND_LIMIT)
        return 0u;
    if (far_center_span > (int16)INTER_COMPONENT_PRE_CENTER_SPAN_MAX)
        return 0u;

    return 1u;
}

static uint8 inline_fast_pass_view(void)
{
    if (g_tf.line_lost != 0u)
        return 0u;
    if (g_tf.valid_row_count < INTER_FAST_PASS_VALID_ROWS)
        return 0u;
    if (abs_i16(g_tf.error) > INTER_FAST_PASS_ERR_LIMIT)
        return 0u;
    if (abs_i16(g_tf.lookahead_error) > INTER_FAST_PASS_LA_ERR_LIMIT)
        return 0u;
    if (abs_i16(g_tf.error_trend) > INTER_FAST_PASS_TREND_LIMIT)
        return 0u;

    if (valid_rows_in_range((int16)INTER_FAST_PASS_TOP_START_ROW,
                            (int16)INTER_FAST_PASS_TOP_END_ROW) <
        INTER_FAST_PASS_TOP_MIN_VALID)
        return 0u;

    for (int16 row = (int16)(RA_PRE_END_ROW + 1);
         row <= (int16)RA_PRE_START_ROW; row++)
    {
        if (row < 0 || row >= (int16)TF_IMG_H || !g_tf.row_valid[row])
            continue;
        if (g_tf.left_edge[row] <= (int16)INTER_FAST_PASS_EDGE_MARGIN)
            return 0u;
        if (g_tf.right_edge[row] >= (int16)(TF_IMG_W - INTER_FAST_PASS_EDGE_MARGIN))
            return 0u;
        if (inter_side_branch_has_road(row, g_tf.center_line[row], 1u) ||
            inter_side_branch_has_road(row, g_tf.center_line[row], 2u))
            return 0u;
    }

    return 1u;
}

static uint8 straight_inline_view(void)
{
    if (g_ra_pre_flag != 0u || g_tf.line_lost != 0u)
        return 0u;
    if (g_tf.valid_row_count < INTER_STRAIGHT_IGNORE_ROWS)
        return 0u;
    if (abs_i16(g_tf.error) > INTER_STRAIGHT_ERR_LIMIT)
        return 0u;
    if (abs_i16(g_tf.lookahead_error) > INTER_STRAIGHT_LA_ERR_LIMIT)
        return 0u;

    return 1u;
}

/**
 * @brief int16 钳位函数：将值限制在 [min_v, max_v] 范围
 *
 * 通用的数值钳位工具函数，用于检测框尺寸等参数的范围限制
 *
 * @param v     输入
 * @param min_v 下限
 * @param max_v 上限
 * @return 钳位后的
  */
int16 clamp_i16(int16 v, int16 min_v, int16 max_v)
{
    /* 低于下限，返回下 */
    if (v < min_v) return min_v;
    /* 高于上限，返回上 */
    if (v > max_v) return max_v;
    /* 在范围内，返回原 */
    return v;
}

/**
 * @brief 估算路口处的赛道宽度（像素）
 *
 * 估算策略（三级回退）：
 *   1. cy 行向上下各搜8 行，找到第一个有效边缘对，用其宽
 *   2. 若都不行，回退使用基点(left_jidian, right_jidian)边缘对宽
 *   3. 兜底使用 BOX_WIDTH/INTER_BOX_WIDTH_SCALE 默认
 *
 * 赛道宽度估算用于动态调整检测框的大小
 *
 * @param cy 中心行号（拐点行
 * @return 估算的赛道宽度（像素
  */
static int16 estimate_inter_track_width(int16 cy)
{
    (void)cy;
    return (int16)BOX_WIDTH;
#if 0

    /* 从偏开始，逐步扩大搜索范围  */
    for (int16 offset = 0; offset <= 8; offset++)
    {
        /* 向下偏移的行号（行号增大，更靠近相机 */
        int16 r1 = cy + offset;
        /* 向上偏移的行号（行号减小，更远离相机 */
        int16 r2 = cy - offset;

        /* 检查向下偏移的行是否有有效边缘 */
        if (r1 >= 0 && r1 < TF_IMG_H && g_tf.row_valid[r1] &&
            valid_pair(g_tf.left_edge[r1], g_tf.right_edge[r1]))
        {
            /* 返回该行的赛道宽度（右边- 左边缘）  */
            return g_tf.right_edge[r1] - g_tf.left_edge[r1];
        }

        /* 检查向上偏移的行（offset=0 时与 r1 相同，跳过避免重复检查）  */
        if (offset != 0 && r2 >= 0 && r2 < TF_IMG_H && g_tf.row_valid[r2] &&
            valid_pair(g_tf.left_edge[r2], g_tf.right_edge[r2]))
        {
            /* 返回该行的赛道宽 */
            return g_tf.right_edge[r2] - g_tf.left_edge[r2];
        }
    }

    /* 回退：使用基点边缘对宽度  */
    if (valid_pair((int16)g_tf.left_jidian, (int16)g_tf.right_jidian))
        return (int16)g_tf.right_jidian - (int16)g_tf.left_jidian;

    /* 兜底：使用默认赛道宽度 */
    return (int16)(BOX_WIDTH / INTER_BOX_WIDTH_SCALE);
#endif
}

/**
 * @brief 根据拐点位置构建路口检测框
 *
 * 检测框(cx, cy) 为锚点，更多面积放在拐点前方
 *   - 宽度 = 赛道宽度 * INTER_BOX_WIDTH_SCALE(3)，限制在 [28, 44] 像素
 *   - 高度 = 赛道宽度 * INTER_BOX_HEIGHT_SCALE(2)，限制在 [16, 24] 像素
 * 框边界不超过图像范围
 * 检测框用于后续的带状区域白色像素检测，以分类路口类型
 *
 * @param cx     框中心列
 * @param cy     框中心行
 * @param top    输出参数：框顶行
 * @param bottom 输出参数：框底行
 * @param left   输出参数：框左列
 * @param right  输出参数：框右列
  */
static void build_inter_box(int16 cx, int16 cy,
                            int16 *top, int16 *bottom,
                            int16 *left, int16 *right)
{
    /* 估算当前路口处的赛道宽度  */
    int16 track_w = estimate_inter_track_width(cy);
    /* 框宽= 赛道宽度 * 3，限制在 [28, 44]  */
    int16 box_w = track_w;
    /* 框高= 赛道宽度 * 2，限制在 [16, 24]  */
    int16 box_h = (int16)BOX_HEIGHT;
    int16 front_h = (int16)((box_h * INTER_BOX_FRONT_PCT) / 100);
    int16 b_top = cy - front_h;
    int16 b_bottom = b_top + box_h;
    /* 框左= 中心- 半宽 */
    int16 b_left = cx - box_w / 2;
    /* 框右= 中心+ 半宽 */
    int16 b_right = cx + box_w / 2;

    if (b_top < 0)
    {
        b_bottom = (int16)(b_bottom - b_top);
        b_top = 0;
    }
    if (b_bottom >= TF_IMG_H)
    {
        b_top = (int16)(b_top - (b_bottom - (TF_IMG_H - 1)));
        b_bottom = TF_IMG_H - 1;
        if (b_top < 0) b_top = 0;
    }
    if (b_left < 0)
    {
        b_right = (int16)(b_right - b_left);
        b_left = 0;
    }
    if (b_right >= TF_IMG_W)
    {
        b_left = (int16)(b_left - (b_right - (TF_IMG_W - 1)));
        b_right = TF_IMG_W - 1;
        if (b_left < 0) b_left = 0;
    }

    /* 输出计算后的框边 */
    *top = b_top;
    *bottom = b_bottom;
    *left = b_left;
    *right = b_right;
}

/**
 * @brief 更新检测框锁定状
 *
 * 使用帧间稳定性判断：候选框位置连续 INTER_BOX_STABLE_FRAMES 
 * INTER_BOX_STABLE_DELTA(3) 像素范围内不变，则锁定
 * 锁定后使用稳定位置进行路口分类，避免框随噪声跳动导致误分类
 *
 * @param cx 当前候选框中心列号
 * @param cy 当前候选框中心行号
 * @return 1=框已锁定可用=框未锁定
  */
static uint8 update_box_lock(int16 cx, int16 cy)
{
    /* 若当前无活跃候选框，初始化为第一个候选框  */
    if (!s_box_candidate_valid)
    {
        /* 标记候选框有效  */
        s_box_candidate_valid = 1u;
        /* 初始状态未锁定  */
        s_box_lock_valid = 0u;
        /* 稳定计数初始化为1  */
        s_box_stable_cnt = 1u;
        /* 记录候选框中心位置  */
        s_box_last_cx = cx;
        s_box_last_cy = cy;
        /* 锁定位置初始化为当前候选位 */
        s_box_lock_cx = cx;
        s_box_lock_cy = cy;
        /* 若只需1帧即锁定（INTER_BOX_STABLE_FRAMES=1），直接生效  */
        if (INTER_BOX_STABLE_FRAMES <= 1u)
        {
            /* 标记框已锁定  */
            s_box_lock_valid = 1u;
            /* 返回锁定可用  */
            return 1u;
        }
        /* 需要更多帧才能锁定，暂返回未锁 */
        return 0u;
    }

    /* 检查框位置是否在稳定阈3像素)内未移动  */
    if (abs_i16(cx - s_box_last_cx) <= INTER_BOX_STABLE_DELTA &&
        abs_i16(cy - s_box_last_cy) <= INTER_BOX_STABLE_DELTA)
    {
        /* 位置稳定，累加稳定计数（上限255防止溢出 */
        if (s_box_stable_cnt < 255u)
            s_box_stable_cnt++;
    }
    else
    {
        /* 位置移动过大，重置稳定状态和投票数据  */
        s_box_stable_cnt = 1u;
        /* 清除锁定状 */
        s_box_lock_valid = 0u;
        /* 重置投票索引  */
        s_type_vote_idx = 0u;
        /* 清除投票计数  */
        s_type_vote_cnt = 0u;
    }

    /* 更新上一帧候选框位置记录  */
    s_box_last_cx = cx;
    s_box_last_cy = cy;

    /* 稳定帧数达到阈值且尚未锁定，执行锁 */
    if (!s_box_lock_valid && s_box_stable_cnt >= INTER_BOX_STABLE_FRAMES)
    {
        /* 标记框已锁定  */
        s_box_lock_valid = 1u;
        /* 记录锁定位置  */
        s_box_lock_cx = cx;
        s_box_lock_cy = cy;
    }

    /* 返回当前锁定状 */
    return s_box_lock_valid;
}

/**
 * @brief 路口类型投票机制
 *
 * 在环形缓冲区中存储最INTER_TYPE_VOTE_FRAMES(2) 帧的检测结果，
 * 当缓冲区满后，统计各类型出现次数，达INTER_TYPE_VOTE_MIN 次的类型通过投票
 *
 * 投票机制可过滤单帧误检，提高检测可靠性
 * 当检测到类型0（无路口）时清空投票缓冲区，重新开始计数
 *
 * @param detected 当前帧检测到的路口类型（0=无，1~5=各类型）
 * @return 投票通过的路口类型，0=未通过投票
  */
static uint8 vote_inter_type(uint8 detected)
{
    /* 未检测到任何路口类型，清空投票缓冲区并返 */
    if (detected == 0u)
    {
        /* 重置投票写入索引  */
        s_type_vote_idx = 0u;
        /* 清除投票计数  */
        s_type_vote_cnt = 0u;
        /* 返回未通过  */
        return 0u;
    }

    /* 将当前帧检测结果写入环形缓冲区  */
    s_type_vote[s_type_vote_idx] = detected;
    /* 递增写入索引  */
    s_type_vote_idx++;
    /* 索引到达缓冲区末尾时回绕到开 */
    if (s_type_vote_idx >= INTER_TYPE_VOTE_FRAMES)
        s_type_vote_idx = 0u;

    /* 递增已存帧数（不超过缓冲区大小）  */
    if (s_type_vote_cnt < INTER_TYPE_VOTE_FRAMES)
        s_type_vote_cnt++;

    /* 缓冲区未满，暂不进行投票（需要足够样本）  */
    if (s_type_vote_cnt < INTER_TYPE_VOTE_FRAMES)
        return 0u;

    /* 遍历所有路口类1~5)，统计出现次 */
    for (uint8 type = 1u; type <= 5u; type++)
    {
        /* 当前类型的计算  */
        uint8 count = 0u;
        /* 遍历投票缓冲区，统计该类型出现次 */
        for (uint8 i = 0u; i < INTER_TYPE_VOTE_FRAMES; i++)
        {
            if (s_type_vote[i] == type)
                count++;
        }

        /* 超过最低票数阈1)，投票通过，返回该类型  */
        if (count >= INTER_TYPE_VOTE_MIN)
            return type;
    }

    /* 无类型获得足够票数，返回未通过  */
    return 0u;
}

/**
 * @brief 路口类型快速确认（跳过投票，立即确认）
 *
 * INTER_FAST_CONFIRM_ENABLE=1 时，强证据路口直接确认；
 * 元件误触发由直行元件保护和强侧向分支阈值提前过滤
 *
 * 快速确认可减少1帧延迟，在高速行驶时尤为重要
 *
 * @param detected 当前帧检测到的路口类
 * @return 确认的路口类型，0=未确认（需走投票流程）
  */
static uint8 fast_confirm_inter_type(uint8 detected, uint8 pure_ra_ok,
                                     uint8 type5_fast_ok,
                                     uint8 type34_fast_ok)
{
#if INTER_FAST_CONFIRM_ENABLE
    /* 类型 1/2：纯直角证据成立且不是稳定直线时快速确 */
    if ((detected == 1u || detected == 2u) &&
        pure_ra_ok &&
        !straight_inline_view_cached())
    {
        uint16 late_row = (ra_turn_row > 0) ? (uint16)ra_turn_row : 0u;
        late_row = (uint16)(late_row + (uint16)INTER_DIRECT_FAST_CONFIRM_ROW_MARGIN);

        if ((uint16)g_ip_max_row >= late_row)
            return detected;
    }

    if (detected == 5u && type5_fast_ok)
    {
        return 5u;
    }

    if ((detected == 3u || detected == 4u) && type34_fast_ok)
    {
        return detected;
    }

#else
    (void)detected;
    (void)pure_ra_ok;
    (void)type5_fast_ok;
    (void)type34_fast_ok;
#endif

    /* 快速确认未启用或类型不在快速确认范围内，返走投票流 */
    return 0u;
}

/**
 * @brief 路口检测主函数（在主循环中调用
 *
 * 检测流程（三阶段）
 *
 * 阶段1 - 冷却期处理：
 *   异常锁定结束后有 INTER_COOLDOWN_FRAMES 帧冷却期，正常RECOVER不额外冷
 *
 * 阶段2 - 锁定期处理：
 *   RA 执行期间持续更新拐点信息（用g_ip_max_row），超时则强制清
 *
 * 阶段3 - 正常检测流程：
 *   a. 检查是否满足检测条件（丢线/预检高行丢失
 *   b. 从丢失区域寻找拐
 *   c. 从中点缓冲区选取稳定拐点列号
 *   d. 更新检测框锁定状
 *   e. 对锁定框进行带状区域检测分
 *   f. 快速确认或投票确认路口类型
 *   g. 设置 g_ra_flag 触发 RA 状态机执行转弯
 *
 * 路口类型定义
 *   1 = 右侧有路（纯右转
 *   2 = 左侧有路（纯左转
 *   3 = 左（T 型左转）
 *   4 = 右（T 型右转）
 *   5 = 右（十字路口
  */
void detect_intersection(void)
{
    update_fast_ra_feature();

    uint8 post_turn_guard = 0u;

    if (s_inter_post_turn_suppress_cnt > 0u)
    {
        uint8 suppress_cnt = s_inter_post_turn_suppress_cnt;
        s_inter_post_turn_suppress_cnt--;
        if (suppress_cnt >
            (INTER_POST_TURN_SUPPRESS_FRAMES - INTER_POST_TURN_BLOCK_FRAMES))
        {
            g_ra_flag = 0u;
            g_ra_pre_flag = 0u;
            g_ra_pre_dir = 0u;
            g_ra_pre_slow_flag = 0u;
            reset_box_lock();
            clear_inter_result();
            return;
        }
        post_turn_guard = 1u;
    }

    /* ---- 阶段1：冷却期处理 ----  */
    /* 冷却期内（异常锁定结束后等待 INTER_COOLDOWN_FRAMES 帧），不进行新路口检 */
    if (s_inter_cooldown_cnt > 0u)
    {
        /* 递增冷却计数 */
        s_inter_cooldown_cnt++;
        /* 冷却期结束，重置计数 */
        if (s_inter_cooldown_cnt >= INTER_COOLDOWN_FRAMES) s_inter_cooldown_cnt = 0u;
        /* 清除路口检测结 */
        clear_inter_result();
        /* 冷却期内直接返回  */
        return;
    }

    /* ---- 阶段2：锁定期处理 ----  */
    /* 锁定期内（RA 正在执行转弯），持续更新拐点信息  */
    if (s_inter_lock_cnt > 0u)
    {
        /* 创建临时拐点结构 */
        InflectionPoint_t ip;
        /* 有路侧标 */
        uint8 found_side = 0u;
        /* 初始化拐点为无效  */
        ip.valid = 0u; ip.row = 0; ip.col = 0;

        /* 持续从丢失区域寻找拐点（用于更新 g_ip_max_row，供 RA 状态机使用 */
        if (s_first_miss_row >= (int16)INTER_MIN_MISS_ROW)
        {
            /* 从丢失行向上扫描寻找拐点  */
            find_ip_from_lost_row(s_first_miss_row, s_last_valid_center, &ip, &found_side);
            /* 从中点缓冲区选取稳定的拐点列 */
            apply_ip_col_from_buffer(&ip, found_side);
        }

        /* 若找到有效拐点，更新路口检测结 */
        if (ip.valid)
        {
            /* 更新左拐点信 */
            g_inter_result.left_ip = ip;
            /* 更新右拐点信息（锁定期左右拐点相同）  */
            g_inter_result.right_ip = ip;
            /* 拐点行号以兼PID 中的比较逻辑  */
            g_ip_max_row = (uint8)(ip.row * 2);
            g_ip_visual_row = g_ip_max_row;
            if (g_ip_visual_row > g_ip_ctrl_row)
            {
                g_ip_ctrl_row = g_ip_visual_row;
                g_ip_ctrl_score = 100u;
                g_ip_ctrl_reason = 0x01u;
            }
        }

        /* RA 已清除标志（通常进入RECOVER），结束锁定但不额外冷却  */
        if (g_ra_flag == 0u)
        {
            /* 清除锁定计数 */
            s_inter_lock_cnt = 0u;
            /* RA进入RECOVER后允许下一帧立即检测近距离下一个弯  */
            s_inter_cooldown_cnt = 0u;
            /* 重置框锁定状 */
            reset_box_lock();
            /* 清除路口检测结 */
            clear_inter_result();
        }
        else
        {
            /* RA 仍在执行，递增锁定计数 */
            s_inter_lock_cnt++;
            /* 锁定超时保护：最多锁INTER_MAX_LOCK_FRAMES(300) 帧（.3秒）  */
            if (s_inter_lock_cnt >= INTER_MAX_LOCK_FRAMES)
            {
                /* 强制清除 RA 标志  */
                g_ra_flag = 0u;
                /* 清除锁定计数 */
                s_inter_lock_cnt = 0u;
                /* 启动冷却 */
                s_inter_cooldown_cnt = 1u;
                /* 重置框锁定状 */
                reset_box_lock();
                /* 清除路口检测结 */
                clear_inter_result();
            }
        }
        /* 锁定期内直接返回，不进入正常检测流 */
        return;
    }

    /* ---- 阶段3：正常检测流----  */

    /* 创建拐点结构 */
    InflectionPoint_t ip;
    /* 初始化拐点为无效  */
    ip.valid = 0u; ip.row = 0; ip.col = 0;
    /* 有路侧标志：1=右侧有路, 2=左侧有路, 3=双侧有路  */
    uint8 found_side = 0u;
    InflectionPoint_t probe_ip;
    uint8 probe_side = 0u;
    uint8 probe_trigger = find_probe_trigger_ip(&probe_ip, &probe_side);
    uint8 route_single_edge_dir = single_edge_route_ra_dir();

    if (probe_trigger &&
        route_single_edge_dir == 0u &&
        s_first_miss_row < (int16)INTER_MIN_MISS_ROW &&
        inline_fast_pass_view_cached())
    {
        g_sym_component_flag = 1u;
        g_ra_pre_flag = 0u;
        g_ra_pre_dir = 0u;
        g_ra_pre_slow_flag = 0u;
        reset_box_lock();
        clear_inter_result();
        return;
    }

    /* 去掉straight_inline_view()拦截
     * 原因：直道上也可能遇到路口，此检查会导致路口漏检
     * 误报由分类阈值和对称分量过滤控制 */

    /* 不满足检测条件：无丢无预检无高行丢-> 清除并返 */
    if (!probe_trigger &&
        route_single_edge_dir == 0u)
    {
        /* 重置框锁定状 */
        reset_box_lock();
        /* 清除路口检测结 */
        clear_inter_result();
        /* 不满足检测条件，直接返回  */
        return;
    }

    /* 从丢失区域寻找拐点（仅当有高行丢失时 */
    if (s_first_miss_row >= (int16)INTER_MIN_MISS_ROW)
        find_ip_from_lost_row(s_first_miss_row, s_last_valid_center, &ip, &found_side);

    if (!ip.valid && probe_trigger)
    {
        ip = probe_ip;
        found_side = probe_side;
    }

    if (!ip.valid && route_single_edge_dir != 0u)
    {
        ip.valid = 1u;
        ip.row = s_single_edge_ra_row;
        if (ip.row < (int16)INTER_BOX_START_ROW)
            ip.row = (int16)INTER_BOX_START_ROW;
        ip.col = s_last_valid_center;
        found_side = route_single_edge_dir;
    }

    /* 用倒数第N个有效行的中点作为稳定的拐点列号  */
    apply_ip_col_from_buffer(&ip, found_side);

    if (route_single_edge_dir == 0u && inline_component_ip_guard(&ip))
    {
        g_sym_component_flag = 1u;
        g_ra_pre_flag = 0u;
        g_ra_pre_dir = 0u;
        g_ra_pre_slow_flag = 0u;
        reset_box_lock();
        clear_inter_result();
        return;
    }

    /* 更新路口检测结果中的左右拐 */
    g_inter_result.left_ip = ip;
    g_inter_result.right_ip = ip;

    /* 更新拐点最大行号（兼容 PID 比较），无效拐点则清 */
    g_ip_max_row = ip.valid ? (uint8)(ip.row * 2) : 0u;
    g_ip_visual_row = g_ip_max_row;
    if (g_ip_visual_row > g_ip_ctrl_row)
    {
        g_ip_ctrl_row = g_ip_visual_row;
        g_ip_ctrl_score = 100u;
        g_ip_ctrl_reason = 0x01u;
    }

    /* 拐点无效，清除并返回  */
    if (!ip.valid)
    {
        /* 重置框锁定状 */
        reset_box_lock();
        /* 清除路口检测结 */
        clear_inter_result();
        /* 拐点无效，返 */
        return;
    }

    /* 拐点行太远，跳过框绘 */
    if (ip.row < INTER_BOX_START_ROW && !probe_trigger)
    {
        /* 重置框锁定状 */
        reset_box_lock();
        /* 清除路口检测结 */
        clear_inter_result();
        /* 拐点太远，返 */
        return;
    }

    /* 更新框锁定状态（需要位置稳定才锁定，防止噪声导致框跳动 */
    if (!update_box_lock(ip.col, ip.row))
    {
        /* 框未锁定，仅绘制候选框用于调试显示，不进行路口分类  */
        int16 dbg_top, dbg_bottom, dbg_left, dbg_right;

        /* 构建候选框（用于TFT调试显示 */
        build_inter_box(ip.col, ip.row, &dbg_top, &dbg_bottom, &dbg_left, &dbg_right);
        /* 更新检测框边界（调试用 */
        g_inter_result.box_top = (uint8)dbg_top;
        g_inter_result.box_bottom = (uint8)dbg_bottom;
        g_inter_result.box_left = (uint8)dbg_left;
        g_inter_result.box_right = (uint8)dbg_right;
        /* 框未锁定，检测类型设（无路口 */
        g_inter_result.detected_type = 0u;
        /* 框未锁定，返回等待稳 */
        return;
    }

    /* ---- 框已锁定，使用稳定位置进行路口分----  */
    /* 当前帧检测到的路口类型（初始=无）  */
    uint8 detected = 0u;

    /* 检测框的四条边 */
    int16 b_top, b_bottom, b_left, b_right;

    /* 使用锁定后的稳定框位置构建检测框（避免使用移动中的候选位置）  */
    build_inter_box(s_box_lock_cx, s_box_lock_cy, &b_top, &b_bottom, &b_left, &b_right);

    /* 更新检测结果中的框边界  */
    g_inter_result.box_top = (uint8)b_top;
    g_inter_result.box_bottom = (uint8)b_bottom;
    g_inter_result.box_left = (uint8)b_left;
    g_inter_result.box_right = (uint8)b_right;

    /* ---- 检查框的四条边带和两侧分支是否有白色道----  */
    /* 检查框上边带是否有足够白色像素（上方有路）  */
    uint8 top_has = inter_horizontal_band_has_road(
        b_top, b_left, b_right, INTER_TOP_MIN_WHITE, INTER_BAND_MIN_STREAK);
    /* 检查框左边带是否有足够白色像素（左侧有路）  */
    uint8 left_has = inter_vertical_band_has_road(
        b_left, b_top, b_bottom, INTER_SIDE_MIN_WHITE, INTER_BAND_MIN_STREAK);
    /* 检查框右边带是否有足够白色像素（右侧有路）  */
    uint8 right_has = inter_vertical_band_has_road(
        b_right, b_top, b_bottom, INTER_SIDE_MIN_WHITE, INTER_BAND_MIN_STREAK);
    /* 检查左侧是否有足够长的分支道路（纯侧路检测）  */
    uint8 left_branch_has = inter_side_branch_has_road(ip.row, ip.col, 2u);
    /* 检查右侧是否有足够长的分支道路（纯侧路检测）  */
    uint8 right_branch_has = inter_side_branch_has_road(ip.row, ip.col, 1u);
    uint8 center_forward_has = inter_vertical_band_has_road(
        ip.col,
        b_top,
        (int16)(ip.row - 2),
        INTER_INLINE_CENTER_MIN_WHITE,
        INTER_INLINE_CENTER_MIN_STREAK);

    uint8 pure_ra_shape =
        ((found_side == 1u && right_branch_has && !left_branch_has) ||
         (found_side == 2u && left_branch_has && !right_branch_has)) ? 1u : 0u;
    uint8 single_edge_ra_dir = single_edge_route_ra_dir();
    uint8 cutoff_ready =
        (s_first_miss_row >= (int16)INTER_MIN_MISS_ROW ||
         g_tf.line_lost != 0u ||
         single_edge_ra_dir != 0u) ? 1u : 0u;
    uint8 pure_ra_ok =
        (g_tf.line_lost != 0u ||
         single_edge_ra_dir != 0u ||
         (cutoff_ready && pure_ra_shape)) ? 1u : 0u;
    uint8 ra_dir_hint = 0u;
    if (g_ra_pre_flag != 0u && (g_ra_pre_dir == 1u || g_ra_pre_dir == 2u))
        ra_dir_hint = g_ra_pre_dir;
    else if (g_ra_pre_flag != 0u && (found_side == 1u || found_side == 2u))
        ra_dir_hint = found_side;
    else if (single_edge_ra_dir != 0u)
        ra_dir_hint = single_edge_ra_dir;

    uint8 top_soft = inter_horizontal_band_has_road(
        b_top, b_left, b_right, 4u, 3u);
    uint8 top_road_has =
        (top_has || top_soft ||
         inter_top_window_has_road(b_top, b_bottom, b_left, b_right)) ? 1u : 0u;
    uint8 left_box_has =
        inter_side_window_has_road(b_top, b_bottom, b_left, b_right, 2u);
    uint8 right_box_has =
        inter_side_window_has_road(b_top, b_bottom, b_left, b_right, 1u);
    uint8 left_frame_has = (left_has || left_box_has) ? 1u : 0u;
    uint8 right_frame_has = (right_has || right_box_has) ? 1u : 0u;
    uint8 left_cross_has =
        inter_side_crossing_has_road(b_top, b_bottom, b_left, b_right, 2u);
    uint8 right_cross_has =
        inter_side_crossing_has_road(b_top, b_bottom, b_left, b_right, 1u);
    uint8 side_cross_ok = (left_cross_has && right_cross_has) ? 1u : 0u;
    uint8 left_box_edge_has = (left_has && left_box_has) ? 1u : 0u;
    uint8 right_box_edge_has = (right_has && right_box_has) ? 1u : 0u;
    uint8 left_box_road = (left_cross_has || left_box_edge_has) ? 1u : 0u;
    uint8 right_box_road = (right_cross_has || right_box_edge_has) ? 1u : 0u;
    uint8 left_edge_reach_has =
        (left_box_road || left_branch_has) ?
        inter_side_edge_reach_has_road(ip.row, ip.col, 2u) : 0u;
    uint8 right_edge_reach_has =
        (right_box_road || right_branch_has) ?
        inter_side_edge_reach_has_road(ip.row, ip.col, 1u) : 0u;
    uint8 left_continuous_side_has =
        (left_cross_has || left_edge_reach_has) ? 1u : 0u;
    uint8 right_continuous_side_has =
        (right_cross_has || right_edge_reach_has) ? 1u : 0u;
    uint8 left_strong_dir_has =
        (left_continuous_side_has ||
         (left_box_edge_has && left_branch_has && left_edge_reach_has)) ? 1u : 0u;
    uint8 right_strong_dir_has =
        (right_continuous_side_has ||
         (right_box_edge_has && right_branch_has && right_edge_reach_has)) ? 1u : 0u;
    uint16 direct_fast_row =
        (uint16)((ra_turn_row > 0) ? (uint16)ra_turn_row : 0u) +
        (uint16)INTER_DIRECT_FAST_CONFIRM_ROW_MARGIN;
    uint8 direct_fast_late =
        ((int32)motor_speed * 8 >= (int32)RA_FAST_SPEED_START &&
         (uint16)g_ip_max_row >= direct_fast_row) ? 1u : 0u;
    uint8 right_fast_dir_has =
        (right_strong_dir_has ||
         (direct_fast_late &&
          (ra_dir_hint == 1u || found_side == 1u || single_edge_ra_dir == 1u) &&
          right_box_edge_has &&
          right_branch_has)) ? 1u : 0u;
    uint8 left_fast_dir_has =
        (left_strong_dir_has ||
         (direct_fast_late &&
          (ra_dir_hint == 2u || found_side == 2u || single_edge_ra_dir == 2u) &&
           left_box_edge_has &&
           left_branch_has)) ? 1u : 0u;
    uint8 right_direct_evidence =
        (!top_road_has &&
         !center_forward_has &&
         pure_ra_ok &&
         (right_strong_dir_has ||
          (single_edge_ra_dir == 1u && right_fast_dir_has))) ? 1u : 0u;
    uint8 left_direct_evidence =
        (!top_road_has &&
         !center_forward_has &&
         pure_ra_ok &&
         (left_strong_dir_has ||
          (single_edge_ra_dir == 2u && left_fast_dir_has))) ? 1u : 0u;
    uint8 top_valid_rows = valid_rows_in_range(
        (int16)INTER_FAST_PASS_TOP_START_ROW,
        (int16)INTER_FAST_PASS_TOP_END_ROW);
    uint8 lower_valid_rows = valid_rows_in_range(
        (int16)INTER_BOX_START_ROW,
        (int16)RA_PRE_START_ROW);
    uint8 inline_component_candidate =
        (g_tf.line_lost == 0u &&
         g_tf.valid_row_count >= INTER_INLINE_COMPONENT_VALID_ROWS_MIN &&
         top_valid_rows >= INTER_INLINE_COMPONENT_TOP_VALID_MIN &&
         center_forward_has &&
         !left_box_road &&
         !right_box_road) ? 1u : 0u;
    uint8 tri_cross_candidate =
        (g_sym_component_flag != 0u &&
         left_box_road && right_box_road &&
         side_cross_ok &&
         !straight_inline_view_cached() &&
         (g_tf.valid_row_count <= INTER_TRI_CROSS_VALID_ROWS_MAX ||
          top_valid_rows <= INTER_TRI_CROSS_TOP_VALID_MAX)) ? 1u : 0u;
    uint8 type5_side_ok =
        (left_box_road && right_box_road && side_cross_ok) ? 1u : 0u;
    uint8 route_expected_type = route_next_expected_flag();
    uint8 route_expected_complex =
        (route_expected_type >= 3u && route_expected_type <= 5u) ? 1u : 0u;
    uint8 top_complex_evidence =
        (top_road_has && center_forward_has) ? 1u : 0u;
    uint8 left_complex_evidence =
        (top_complex_evidence && left_strong_dir_has) ? 1u : 0u;
    uint8 right_complex_evidence =
        (top_complex_evidence && right_strong_dir_has) ? 1u : 0u;
    uint8 cross_complex_evidence =
        (top_complex_evidence &&
         (side_cross_ok ||
          (left_strong_dir_has && right_strong_dir_has))) ? 1u : 0u;
    uint8 route_type5_soft_ok =
        (route_expected_complex != 0u &&
         route_expected_type == 5u &&
         g_tf.line_lost == 0u &&
         g_tf.valid_row_count >= INTER_MIN_VALID_ROWS &&
         top_complex_evidence &&
         !inline_component_candidate &&
         !straight_inline_view_cached() &&
         (left_complex_evidence ||
          right_complex_evidence ||
          cross_complex_evidence)) ? 1u : 0u;
    uint8 route_expected_side_evidence = 0u;
    uint8 route_expected_cutoff_evidence = 0u;
    uint8 route_expected_complex_evidence = 0u;
    uint8 route_expected_cutoff =
        (g_tf.line_lost != 0u ||
         g_tf.valid_row_count < RA_INTER_COMPLEX_ROUTE_VALID_ROWS ||
         g_ip_max_row >= RA_INTER_COMPLEX_LOST_IP_ROW) ? 1u : 0u;
    if (route_expected_complex != 0u)
    {
        if (route_expected_type == 3u)
        {
            route_expected_complex_evidence = left_complex_evidence;
            route_expected_side_evidence = left_strong_dir_has;
        }
        else if (route_expected_type == 4u)
        {
            route_expected_complex_evidence = right_complex_evidence;
            route_expected_side_evidence = right_strong_dir_has;
        }
        else if (route_expected_type == 5u)
        {
            route_expected_complex_evidence =
                (cross_complex_evidence || route_type5_soft_ok) ? 1u : 0u;
            route_expected_side_evidence =
                (left_strong_dir_has || right_strong_dir_has ||
                 cross_complex_evidence) ? 1u : 0u;
        }
    }
    if (route_expected_complex != 0u &&
        route_expected_cutoff != 0u &&
        route_expected_side_evidence != 0u)
    {
        route_expected_cutoff_evidence = 1u;
    }
    uint8 inline_straight_guard =
        (g_tf.line_lost == 0u &&
         single_edge_ra_dir == 0u &&
         g_ra_pre_flag == 0u &&
         center_forward_has &&
         abs_i16(g_tf.error) <= INTER_INLINE_STRAIGHT_ERR_LIMIT &&
         abs_i16(g_tf.lookahead_error) <= INTER_INLINE_STRAIGHT_LA_LIMIT &&
         abs_i16(g_tf.error_trend) <= INTER_INLINE_STRAIGHT_TREND_LIMIT &&
         ((g_tf.valid_row_count >= INTER_INLINE_STRAIGHT_VALID_ROWS &&
           top_valid_rows >= INTER_INLINE_STRAIGHT_TOP_VALID_MIN) ||
         (ip.row <= (int16)INTER_INLINE_STRAIGHT_IP_ROW_MAX &&
           lower_valid_rows >= INTER_INLINE_STRAIGHT_LOWER_VALID_MIN))) ? 1u : 0u;
    uint8 route_expected_early_evidence =
        (route_expected_complex != 0u &&
         g_tf.line_lost == 0u &&
         g_ip_max_row >= INTER_ROUTE_EARLY_IP_ROW &&
         g_tf.valid_row_count >= RA_INTER_COMPLEX_ROUTE_VALID_ROWS &&
         g_tf.valid_row_count <= INTER_ROUTE_EARLY_VALID_ROWS &&
         !inline_component_candidate &&
         !inline_straight_guard &&
         !straight_inline_view_cached() &&
         (abs_i16(g_tf.lookahead_error) >= INTER_ROUTE_EARLY_LA_MIN ||
          abs_i16(g_tf.error_trend) >= INTER_ROUTE_EARLY_TREND_MIN ||
          route_expected_side_evidence != 0u)) ? 1u : 0u;

    if (single_edge_ra_dir == 0u && inline_component_candidate)
    {
        g_sym_component_flag = 1u;
        g_ra_pre_flag = 0u;
        g_ra_pre_dir = 0u;
        g_ra_pre_slow_flag = 0u;
        reset_box_lock();
        clear_inter_result();
        return;
    }

    if (inline_straight_guard)
    {
        g_sym_component_flag = 1u;
        g_ra_pre_flag = 0u;
        g_ra_pre_dir = 0u;
        g_ra_pre_slow_flag = 0u;
        reset_box_lock();
        clear_inter_result();
        return;
    }

    if (single_edge_ra_dir == 0u &&
        g_ra_pre_flag == 0u &&
        left_branch_has && right_branch_has &&
        center_forward_has &&
        !left_has && !right_has &&
        !left_box_has && !right_box_has &&
        straight_inline_view_cached())
    {
        g_sym_component_flag = 1u;
        reset_box_lock();
        clear_inter_result();
        return;
    }

    /* ---- 路口类型分类逻辑 ----  */
    /* Final RA classification must use box-edge / crossing evidence.  */
    if (single_edge_ra_dir == 1u && right_direct_evidence &&
        !(abs_i16(g_tf.error) < 12 && abs_i16(g_tf.lookahead_error) < 12))
        detected = 1u;
    else if (single_edge_ra_dir == 2u && left_direct_evidence &&
        !(abs_i16(g_tf.error) < 12 && abs_i16(g_tf.lookahead_error) < 12))
        detected = 2u;
    else if ((type5_side_ok && cross_complex_evidence) ||
             (tri_cross_candidate && cross_complex_evidence) ||
             route_type5_soft_ok)
        detected = 5u;
    else if (left_complex_evidence)
        detected = 3u;
    else if (right_complex_evidence)
        detected = 4u;
    else if (ra_dir_hint == 1u && right_direct_evidence)
        detected = 1u;
    else if (ra_dir_hint == 2u && left_direct_evidence)
        detected = 2u;
    else if (found_side == 1u && right_direct_evidence)
        detected = 1u;
    else if (found_side == 2u && left_direct_evidence)
        detected = 2u;

    if (g_post_edge_side != EDGE_BOTH &&
        detected >= 3u && detected <= 5u &&
        !route_next_flag_is(detected))
    {
        g_ra_flag = 0u;
        g_ra_pre_flag = 0u;
        g_ra_pre_dir = 0u;
        g_ra_pre_slow_flag = 0u;
        reset_box_lock();
        clear_inter_result();
        return;
    }

#if INTER_COMPONENT_GUARD_ENABLE
    if (detected >= 3u && detected <= 5u)
    {
        uint8 real_complex_side =
            ((detected == 3u && left_complex_evidence) ||
             (detected == 4u && right_complex_evidence) ||
             (detected == 5u &&
              (cross_complex_evidence || route_type5_soft_ok))) ? 1u : 0u;
        uint8 route_match_complex = route_next_flag_is(detected);
        uint8 route_remap_window =
            (route_expected_complex != 0u &&
             (g_ip_max_row >= INTER_ROUTE_REMAP_IP_ROW ||
              g_tf.valid_row_count < INTER_ROUTE_REMAP_VALID_ROWS ||
              g_tf.line_lost != 0u)) ? 1u : 0u;
        uint8 strong_straight_component =
            (g_tf.valid_row_count >= INTER_COMPONENT_GUARD_STRONG_VALID_ROWS &&
             top_valid_rows >= INTER_COMPONENT_GUARD_TOP_ROWS &&
             abs_i16(g_tf.error) <= INTER_FAST_PASS_ERR_LIMIT &&
             abs_i16(g_tf.lookahead_error) <= INTER_FAST_PASS_LA_ERR_LIMIT &&
             abs_i16(g_tf.error_trend) <= INTER_FAST_PASS_TREND_LIMIT) ? 1u : 0u;
        uint8 stable_inline_component =
            (g_tf.line_lost == 0u &&
             center_forward_has &&
             lower_valid_rows >= INTER_COMPONENT_GUARD_LOWER_ROWS &&
             g_tf.valid_row_count >= INTER_COMPONENT_GUARD_VALID_ROWS &&
             abs_i16(g_tf.error) <= INTER_COMPONENT_GUARD_ERR_LIMIT &&
             abs_i16(g_tf.lookahead_error) <= INTER_COMPONENT_GUARD_LA_LIMIT &&
             abs_i16(g_tf.error_trend) <= INTER_COMPONENT_GUARD_TREND_LIMIT &&
             route_match_complex == 0u &&
             route_remap_window == 0u &&
             real_complex_side == 0u &&
             (g_sym_component_flag != 0u ||
              inline_component_candidate != 0u ||
              inline_straight_guard != 0u ||
              strong_straight_component != 0u)) ? 1u : 0u;

        if (stable_inline_component)
        {
            g_sym_component_flag = 1u;
            g_ra_pre_flag = 0u;
            g_ra_pre_dir = 0u;
            g_ra_pre_slow_flag = 0u;
            reset_box_lock();
            clear_inter_result();
            return;
        }
    }
#endif

    uint8 route_expected_fast_ok = 0u;
    if (route_expected_complex &&
        (g_ip_max_row >= INTER_ROUTE_REMAP_IP_ROW ||
         (g_ip_max_row >= RA_INTER_COMPLEX_LAST_CHANCE_ROW &&
          g_tf.valid_row_count < INTER_ROUTE_REMAP_VALID_ROWS) ||
         g_tf.line_lost != 0u ||
         route_expected_early_evidence != 0u) &&
        !inline_component_candidate &&
        (detected == 0u || !route_next_flag_is(detected) ||
         detected == route_expected_type) &&
        (route_expected_complex_evidence != 0u ||
         route_expected_cutoff_evidence != 0u ||
         route_expected_early_evidence != 0u))
    {
        detected = route_expected_type;
        route_expected_fast_ok = 1u;
    }

    /* ---- 确认路口类型：快速确-> 投票确认 ----  */
    /* 首先尝试快速确认（跳过投票，对高置信度类型直接确认 */
    uint8 type5_fast_ok =
        (detected == 5u &&
         left_frame_has && right_frame_has &&
         cross_complex_evidence &&
         !inline_component_candidate &&
         !straight_inline_view_cached()) ? 1u : 0u;
    uint8 type34_fast_ok =
        (!inline_component_candidate &&
         !straight_inline_view_cached() &&
         ((detected == 3u && left_complex_evidence) ||
          (detected == 4u && right_complex_evidence))) ? 1u : 0u;
    uint8 inter_evidence = 0u;
    uint8 inter_confidence = 0u;

    if (top_road_has)
        inter_evidence |= INTER_EVID_TOP;
    if (center_forward_has)
        inter_evidence |= INTER_EVID_CENTER;
    if (left_strong_dir_has)
        inter_evidence |= INTER_EVID_LEFT;
    if (right_strong_dir_has)
        inter_evidence |= INTER_EVID_RIGHT;
    if (left_edge_reach_has || right_edge_reach_has ||
        left_box_edge_has || right_box_edge_has)
        inter_evidence |= INTER_EVID_EDGE;
    if (route_expected_fast_ok)
        inter_evidence |= INTER_EVID_ROUTE;
    if (route_expected_cutoff_evidence)
        inter_evidence |= INTER_EVID_CUTOFF;
    if (type5_fast_ok || type34_fast_ok || direct_fast_late ||
        route_expected_early_evidence)
        inter_evidence |= INTER_EVID_FAST;

    if (detected == 1u || detected == 2u)
    {
        inter_confidence = pure_ra_ok ? 70u : 45u;
    }
    else if (detected == 3u)
    {
        inter_confidence = left_complex_evidence ? 75u : 45u;
    }
    else if (detected == 4u)
    {
        inter_confidence = right_complex_evidence ? 75u : 45u;
    }
    else if (detected == 5u)
    {
        inter_confidence = cross_complex_evidence ? 82u :
                           (route_type5_soft_ok ? 68u : 45u);
    }

    if (route_expected_fast_ok && detected == route_expected_type &&
        inter_confidence < 62u)
        inter_confidence = 62u;
    if (route_expected_cutoff_evidence && inter_confidence < 50u)
        inter_confidence = 50u;
    if ((inter_evidence & INTER_EVID_EDGE) != 0u &&
        inter_confidence < 48u)
        inter_confidence = 48u;

    /* 记录原始检测类型到结果结构 */
    g_inter_result.detected_type = detected;
    g_inter_result.confidence = inter_confidence;
    g_inter_result.evidence = inter_evidence;

    uint8 voted_type = fast_confirm_inter_type(detected, pure_ra_ok,
                                               type5_fast_ok,
                                               type34_fast_ok);
    /* 快速确认未通过，走投票确认流程  */
    if (voted_type == 0u && route_expected_fast_ok)
        voted_type = detected;
    if (voted_type == 0u)
        voted_type = vote_inter_type(detected);

    if (voted_type != 0u && !route_next_flag_is(voted_type))
    {
        uint8 complex_alias =
            (voted_type >= 3u && voted_type <= 5u) ? 1u : 0u;
        uint8 direct_to_cross_alias =
            (route_expected_complex != 0u &&
             (voted_type == 1u || voted_type == 2u)) ? 1u : 0u;
        if (route_expected_complex != 0u &&
            (complex_alias || direct_to_cross_alias) &&
            (g_ip_max_row >= INTER_ROUTE_REMAP_IP_ROW ||
             g_tf.valid_row_count < INTER_ROUTE_REMAP_VALID_ROWS) &&
            (route_expected_complex_evidence != 0u ||
             route_expected_cutoff_evidence != 0u))
        {
            voted_type = route_expected_type;
        }
    }

    if (voted_type >= 3u && voted_type <= 5u &&
        g_ip_max_row < (uint8)INTER_COMPLEX_ARM_IP_ROW)
    {
        g_ra_flag = 0u;
        g_ra_pre_flag = 0u;
        g_ra_pre_dir = 0u;
        g_ra_pre_slow_flag = 0u;
        reset_box_lock();
        clear_inter_result();
        return;
    }

    if (g_post_edge_side != EDGE_BOTH &&
        voted_type >= 3u && voted_type <= 5u &&
        !route_next_flag_is(voted_type))
    {
        g_ra_flag = 0u;
        g_ra_pre_flag = 0u;
        g_ra_pre_dir = 0u;
        g_ra_pre_slow_flag = 0u;
        reset_box_lock();
        clear_inter_result();
        return;
    }

    if (voted_type >= 3u && voted_type <= 5u &&
        (g_tf.line_lost != 0u ||
         g_tf.valid_row_count < RA_INTER_COMPLEX_ROUTE_VALID_ROWS) &&
        !(route_next_flag_is(voted_type) &&
          (g_ip_max_row >= RA_INTER_COMPLEX_LOST_IP_ROW ||
           g_tf.line_lost != 0u)))
    {
        g_ra_flag = 0u;
        g_ra_pre_flag = 0u;
        g_ra_pre_dir = 0u;
        g_ra_pre_slow_flag = 0u;
        reset_box_lock();
        clear_inter_result();
        return;
    }

    if (s_inter_post_turn_suppress_cnt >
        (INTER_POST_TURN_SUPPRESS_FRAMES - INTER_POST_TURN_BLOCK_FRAMES))
    {
        g_ra_flag = 0u;
        g_ra_pre_flag = 0u;
        g_ra_pre_dir = 0u;
        g_ra_pre_slow_flag = 0u;
        reset_box_lock();
        clear_inter_result();
        return;
    }

    if (post_turn_guard &&
        (voted_type == 0u || !route_next_flag_is(voted_type)))
    {
        g_ra_flag = 0u;
        g_ra_pre_flag = 0u;
        g_ra_pre_dir = 0u;
        g_ra_pre_slow_flag = 0u;
        reset_box_lock();
        clear_inter_result();
        return;
    }

    if (post_turn_guard && voted_type != 0u)
        s_inter_post_turn_suppress_cnt = 0u;

    /* 投票/快速确认通过，设RA 标志，触RA 状态机执行转弯  */
    if (voted_type != 0u)
    {
        /* 设置路口类型标志~5），Pid.c 中的 RA 状态机据此执行转弯  */
        g_ra_flag = voted_type;
        /* 更新检测结果中的类型为投票确认后的类型  */
        g_inter_result.detected_type = voted_type;
        if (route_expected_complex != 0u &&
            voted_type == route_expected_type &&
            inter_confidence < 62u)
        {
            inter_confidence = 62u;
            g_inter_result.confidence = inter_confidence;
            g_inter_result.evidence |= INTER_EVID_ROUTE;
        }
        /* 进入锁定期（计数器设 */
        s_inter_lock_cnt = 1u;
        /* 重置框锁定状态（为下次检测做准备 */
        reset_box_lock();
    }
}
