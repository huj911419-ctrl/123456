#ifndef TRACK_FUSION_H
#define TRACK_FUSION_H

#include "zf_common_headfile.h"

/* ================================================================
 *  track_fusion.h / track_fusion.c
 *
 *  使用方式：
 *    1. 上电调用 track_fusion_init()
 *    2. 每帧调用 track_fusion_update()
 *    3. 读取 g_tf.error 喂给PID
 *    4. bin_image[][] 可直接用于TFT显示和元素判断
 *       bin_image[i][j] = 255 → 赛道线（深色像素）
 *       bin_image[i][j] = 0   → 背景（亮色像素）
 * ================================================================ */

/* ==================== 图像参数 ==================== */
#define TF_IMG_H MT9V03X_H           // 120
#define TF_IMG_W MT9V03X_W           // 188
#define TF_IMG_CENTER (TF_IMG_W / 2) // 94

/* ==================== 循迹参数（可调）==================== */
#define TF_OTSU_INTERVAL 5           // 每几帧重算一次大津阈值
#define TF_THRESHOLD_BIAS (-10)      // 阈值偏移量
#define TF_JIDIAN_ROW (TF_IMG_H - 4) // 基点搜索行 = 116
#define TF_SEARCH_END_ROW 8          // 向上搜索终止行
#define TF_LOCAL_RANGE 18            // 本地搜索半宽
#define TF_MAX_MISS_ROWS 5           // 连续丢失行上限
#define TF_MIN_TRACK_WIDTH 4        // 有效赛道最小宽度
#define TF_MAX_TRACK_WIDTH 160       // 有效赛道最大宽度
#define TF_INVALID (-1)              // 无效值标记

/* ==================== 数据结构 ==================== */
typedef struct
{
    int16 left_edge[TF_IMG_H];   // 每行左边界列号（TF_INVALID=无效）
    int16 right_edge[TF_IMG_H];  // 每行右边界列号
    int16 center_line[TF_IMG_H]; // 每行中线列号
    uint8 row_valid[TF_IMG_H];   // 该行是否有效

    int16 error;            // 加权中线误差（负=偏左 正=偏右）
    uint16 valid_row_count; // 本帧有效行数
    uint8 line_lost;        // 丢线标志

    uint8 left_jidian;  // 基点行左边界（调试用）
    uint8 right_jidian; // 基点行右边界（调试用）
    uint8 threshold;    // 当前二值化阈值（调试用）
} TrackFusion_t;

extern TrackFusion_t g_tf;

/* ==================== 二值化图像 ==================== */
/* 和你原来的 image[][] 一样的用法
 * bin_image[i][j] = 255 → 赛道线（深色像素，算法认为是"白"）
 * bin_image[i][j] = 0   → 背景（亮色像素）
 * 可直接传给 tft180_show_gray_image 显示
 */
extern uint8 bin_image[TF_IMG_H][TF_IMG_W];

/* ==================== 对外接口 ==================== */
void track_fusion_init(void);
void track_fusion_update(void);

// 直角检测结果（供 line_pid.c 读取）
extern uint8 g_right_angle_flag;
extern int8 g_right_angle_dir;

void right_angle_detect(void);

#endif /* TRACK_FUSION_H */
