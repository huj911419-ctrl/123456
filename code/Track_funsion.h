#ifndef TRACK_FUSION_H
#define TRACK_FUSION_H

#include "zf_common_headfile.h"

extern int16 threshold_bias; // Otsu阈值偏移量（菜单可调）

/* ================================================================
 *  Otsu二值化循迹 + 拐点法路口检测
 *  图像压缩 188×120 → 94×60（2×2块平均）
 * ================================================================ */

/* ==================== 压缩图像尺寸定义 ==================== */
#define COMP_W          (MT9V03X_W / 2)   // 94
#define COMP_H          (MT9V03X_H / 2)   // 60
#define COMP_CENTER     (COMP_W / 2)      // 47

/* ==================== 图像尺寸定义 ==================== */
#define TF_IMG_H COMP_H                   // 压缩后高度 60
#define TF_IMG_W COMP_W                   // 压缩后宽度 94
#define TF_IMG_CENTER COMP_CENTER         // 中心列 47

/* ==================== 循迹算法参数 ==================== */
#define TF_OTSU_INTERVAL 2           // 每隔几帧计算一次阈值
#define TF_THRESHOLD_BIAS (-10)      // 阈值偏移量
#define TF_JIDIAN_ROW (TF_IMG_H - 4) // 底部极值检测行 = 56
#define TF_SEARCH_END_ROW 4          // 循迹搜索起始行（压缩后）
#define TF_LOCAL_RANGE 12            // 局部搜索范围（压缩后比例）
#define TF_ROW_STEP 1                // 扫描步长 1=每行
#define TF_LOOKAHEAD_START_ROW 35    // 前瞻起始行（压缩后=原70）
#define TF_LOOKAHEAD_END_ROW 20      // 前瞻终止行（压缩后=原40）
#define TF_MAX_MISS_ROWS 5           // 最大允许丢线行数
#define TF_MIN_TRACK_WIDTH 2         // 有效赛道最小宽度（压缩后=原4）
#define TF_MAX_TRACK_WIDTH 40        // 有效赛道最大宽度（压缩后=原160）
#define TF_INVALID (-1)              // 无效值标记

/* ==================== 数据结构定义 ==================== */
typedef struct
{
    int16 left_edge[TF_IMG_H];
    int16 right_edge[TF_IMG_H];
    int16 center_line[TF_IMG_H];
    uint8 row_valid[TF_IMG_H];

    int16 error;
    int16 lookahead_error;
    int16 error_trend;
    uint16 valid_row_count;
    uint8 line_lost;

    uint8 left_jidian;
    uint8 right_jidian;
    uint8 threshold;
} TrackFusion_t;

extern TrackFusion_t g_tf;

/* ==================== 二值化图像 ==================== */
extern uint8 bin_image[TF_IMG_H][TF_IMG_W];

/* ==================== 压缩灰度图像 ==================== */
extern uint8 compressed_gray[COMP_H][COMP_W];

/* ==================== 拐点法路口检测参数 ==================== */
#define INTER_JUMP_THRESH      10         // 压缩后跳变阈值（原20）
#define INTER_COOLDOWN_FRAMES  60
#define INTER_MAX_LOCK_FRAMES  300
#define INTER_BOX_START_ROW    25         // 压缩后，拐点行号>=此值才开始画框（原50）

/* 拐点增强检测参数 */
#define IP_STABLE_WINDOW       3          // 跳变前稳定检查行数
#define IP_CHECK_WINDOW        3          // 跳变后方向持续检查行数
#define IP_MIN_CONTINUE        2          // 跳变后最少持续有效行数
#define IP_STABLE_VAR_MAX      30         // 跳变前边缘方差上限（越小越严格）

#define BOX_HEIGHT         40             // 压缩后框高
#define BOX_WIDTH          60             // 压缩后框宽
#define INTER_BOX_MIN_STREAK 5            // 框边最少连续白点数（防单点误判）

/* ==================== 拐点数据结构 ==================== */
typedef struct
{
    uint8 valid;
    int16 row;
    int16 col;
} InflectionPoint_t;

/* ==================== 路口检测结果 ==================== */
typedef struct
{
    InflectionPoint_t left_ip;
    InflectionPoint_t right_ip;
    uint8 box_top;
    uint8 box_bottom;
    uint8 box_left;
    uint8 box_right;
    uint8 detected_type;            // 0=正常 1=右转 2=左转 3=十字
} IntersectionResult_t;

/* ==================== 导出变量 ==================== */
extern uint8 g_ra_flag;               // 0=无 1=右转 2=左转 3=十字
extern IntersectionResult_t g_inter_result;
extern uint8 g_ip_max_row;
extern uint8 g_debug_detected;

/* ==================== 函数接口 ==================== */
void track_fusion_init(void);
void track_fusion_update(void);
void detect_intersection(void);

#endif /* TRACK_FUSION_H */
