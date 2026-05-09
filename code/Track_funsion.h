#ifndef TRACK_FUSION_H
#define TRACK_FUSION_H

#include "zf_common_headfile.h"

extern int16 threshold_bias; // Otsu阈值偏移量（菜单可调）

/* ================================================================
 *  track_fusion.h / track_fusion.c - 赛道融合与循迹模块
 *
 *  使用方式：
 *    1. 初始化调用 track_fusion_init()
 *    2. 每帧调用 track_fusion_update()
 *    3. 获取 g_tf.error 用于PID控制
 *    4. bin_image[][] 可直接用于TFT显示或元素判断
 *       bin_image[i][j] = 255 表示赛道（白色，算法认为是"路"）
 *       bin_image[i][j] = 0   表示背景（黑色）
 * ================================================================ */

/* ==================== 图像尺寸定义 ==================== */
#define TF_IMG_H MT9V03X_H           // 图像高度 120
#define TF_IMG_W MT9V03X_W           // 图像宽度 188
#define TF_IMG_CENTER (TF_IMG_W / 2) // 图像中心列 94

/* ==================== 循迹算法参数 ==================== */
#define TF_OTSU_INTERVAL 2           // 每隔几帧计算一次阈值
#define TF_THRESHOLD_BIAS (-10)      // 阈值偏移量
#define TF_JIDIAN_ROW (TF_IMG_H - 4) // 底部极值检测行 = 116
#define TF_SEARCH_END_ROW 8          // 循迹搜索起始行
#define TF_LOCAL_RANGE 18            // 局部搜索范围
#define TF_LOOKAHEAD_START_ROW 70    // 前瞻起始行（中上部）
#define TF_LOOKAHEAD_END_ROW 40      // 前瞻终止行
#define TF_MAX_MISS_ROWS 5           // 最大允许丢线行数
#define TF_MIN_TRACK_WIDTH 4         // 有效赛道最小宽度
#define TF_MAX_TRACK_WIDTH 160       // 有效赛道最大宽度
#define TF_INVALID (-1)              // 无效值标记

/* ==================== 数据结构定义 ==================== */
typedef struct
{
    int16 left_edge[TF_IMG_H];    // 每行左边界列号（TF_INVALID=无效）
    int16 right_edge[TF_IMG_H];   // 每行右边界列号
    int16 center_line[TF_IMG_H];  // 每行中心线列号
    uint8 row_valid[TF_IMG_H];    // 该行是否有效

    int16 error;                  // 加权平均偏差（左=负偏 右=正偏）
    int16 lookahead_error;        // 前瞻行偏差（图像中上部，用于弯道预判）
    int16 error_trend;            // 偏差趋势 = lookahead_error - error（正=前方弯更急）
    uint16 valid_row_count;       // 有效行数
    uint8 line_lost;              // 丢线标志

    uint8 left_jidian;            // 底部左边界（用于起跑线检测）
    uint8 right_jidian;          // 底部右边界（用于起跑线检测）
    uint8 threshold;              // 当前阈值（二值化参数）
} TrackFusion_t;

extern TrackFusion_t g_tf;

/* ==================== 二值化图像 ==================== */
/* 基于原始灰度图 image[][] 转换而来
 * bin_image[i][j] = 255 表示赛道（白色，算法认为是"路"）
 * bin_image[i][j] = 0   表示背景（黑色）
 * 可直接传入 tft180_show_gray_image 显示
 */
extern uint8 bin_image[TF_IMG_H][TF_IMG_W];

/* ==================== 拐点法路口检测参数 ==================== */
#define INTER_JUMP_THRESH  30     // 相邻行边线跳变阈值（列数），超过此值认为拐点
#define INTER_IP_MIN_ROW   100    // 拐点触发最小行号（拐点必须在行100~116之间才会触发检测，防止远处提前拐弯）
#define INTER_BOX_EDGE_COUNT 5    // 矩形框边白像素数阈值（累计像素数）
#define INTER_MIN_BOX_SIZE 3      // 最小框宽/高（像素），小于此值跳过框边检查
#define INTER_CONFIRM_FRAMES 2    // 确认帧数，建议2~3帧
#define INTER_LOCK_FRAMES    30   // 最小锁定帧数（防止转弯中途误清零）
#define INTER_COOLDOWN_FRAMES 15  // 冷却帧数（完成后防立即重触发）
#define INTER_MAX_LOCK_FRAMES 120 // 最大锁定时长（超时保护）

/* ==================== 拐点数据结构 ==================== */
typedef struct
{
    uint8 valid;   // 该拐点是否有效
    int16 row;     // 拐点所在行号（跳变之前的行，即较低的边界）
    int16 col;     // 拐点所在列号（跳变前的边线位置）
} InflectionPoint_t;

/* ==================== 路口检测结果 ==================== */
typedef struct
{
    InflectionPoint_t left_ip;      // 左边界拐点
    InflectionPoint_t right_ip;     // 右边界拐点
    uint8 box_top;                  // 识别框上沿行号
    uint8 box_bottom;               // 识别框下沿行号
    uint8 box_left;                 // 识别框左沿列号
    uint8 box_right;                // 识别框右沿列号
    uint8 box_road_count;           // 矩形框4条边中检测到道路的边数 (0~4)
    uint8 box_road_mask;            // 道路边掩码: bit0=上沿 bit1=下沿 bit2=左边 bit3=右边
    uint8 detected_type;            // 本帧检测结果: 0=正常 1=右转 2=左转 3=十字
} IntersectionResult_t;

/* ==================== 路口检测结果 ==================== */
/*
 * g_ra_flag : 0=无直角  1=右直角弯  2=左直角弯  3=十字/T形路口
 *
 * 拐点法判断原理：
 *   从图像底部向上扫描，找到相邻行边线位置跳变 > INTER_JUMP_THRESH 的行
 *   左右拐点构建矩形框，统计4条框边的道路像素，≥3边有路=十字，2边=直角弯
 *   需 INTER_CONFIRM_FRAMES 帧连续确认，防止误触发
 *   detect_intersection() 负责发现入口（上升沿），Pid.c 负责清零
 */
extern uint8 g_ra_flag;
extern IntersectionResult_t g_inter_result;

/* ==================== 直角预判参数（远处边线丢线检测，用于提前减速）==================== */
#define RA_PRE_START_ROW 75    // 预判起始行（中上部）
#define RA_PRE_END_ROW 55      // 预判终止行
#define RA_PRE_LOST_THRESH 5   // 丢边行数阈值（比主检测的8更敏感）
#define RA_PRE_EDGE_MARGIN 5   // 边线贴近图像边缘的阈值

extern uint8 g_ra_pre_flag; // 1=远处看到直角，开始减速  0=正常

/* ==================== 函数接口 ==================== */
void track_fusion_init(void);
void track_fusion_update(void);

/* 每帧在 track_fusion_update() 之后调用
 * 拐点法路口检测，更新 g_ra_flag
 * 0=无直角  1=右直角弯  2=左直角弯  3=十字/T形 */
void detect_intersection(void);

/* detect_intersection() 之后调用，用边线丢线法在中上部提前发现直角
 * 更新 g_ra_pre_flag: 1=远处看到直角需减速  0=正常 */
void right_angle_pre_detect(void);
#endif /* TRACK_FUSION_H */
