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

/* ==================== 直角检测参数 ==================== */
#define RA_CHECK_ROWS 15            // 只检测底部多少行，越小对弯道越不敏感
#define RA_EDGE_MARGIN 3            // 边界接近图像边缘的阈值，越小越严格
#define RA_LOST_THRESH 8            // 底部丢线超过多少列判定为直角弯，越小越严格
#define RA_CONFIRM_FRAMES 2         // 需要几帧确认，建议2~3帧，1太快，4太慢
#define RA_TIMEOUT_FRAMES 50        // 标志最长消失帧数，50帧约1秒，超时强制归位

/* ==================== 直角检测结果 ==================== */
/*
 * g_ra_flag : 0=无直角  1=左直角弯  2=右直角弯  3=十字/T形路口
 *
 * 判断原理：
 *   只统计底部 RA_CHECK_ROWS 行，丢边方向即为弯道方向
 *   右侧边界接近图像右边缘 + 右侧丢线 = 左直角弯
 *   左侧边界接近图像左边缘 + 左侧丢线 = 右直角弯
 *   两侧同时丢线         + 赛道到达底部 = 十字/T形
 *   需 RA_CONFIRM_FRAMES 帧连续确认，防止误触发
 */
extern uint8 g_ra_flag; // 外部变量，判定是否有直角

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
 * 更新 g_ra_flag
 * 0=无直角  1=左直角弯  2=右直角弯  3=十字/T形 */
void right_angle_detect(void);

/* right_angle_detect() 之后调用，用边线丢线法在中上部提前发现直角
 * 更新 g_ra_pre_flag: 1=远处看到直角需减速  0=正常 */
void right_angle_pre_detect(void);
#endif /* TRACK_FUSION_H */