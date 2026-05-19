#ifndef TRACK_FUSION_H
#define TRACK_FUSION_H

#include "zf_common_headfile.h"

extern int16 threshold_bias;

/* ================================================================
 *  Otsu二值化循迹 + 迷宫搜线 + 拐点画框检测
 *  图像压缩 188x120 -> 94x60（最近邻插值）
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
#define TF_JIDIAN_ROW (TF_IMG_H - 4) // 底部极值检测行 = 56
#define TF_JIDIAN_MIN_ROW (TF_IMG_H - 15) // 底部找不到起点时，最多向上搜索到45行
#define TF_SEARCH_END_ROW 4          // 循迹搜索起始行（压缩后）
#define TF_LOCAL_RANGE 10            // 局部搜索范围（压缩后=原18/2）
#define TF_MAX_MISS_ROWS 5           // 最大允许丢线行数
#define TF_MIN_TRACK_WIDTH 4         // 有效赛道最小宽度（压缩后=原4/2）
#define TF_MAX_TRACK_WIDTH 30        // 有效赛道最大宽度（压缩后=原160/2）
#define TF_INVALID (-1)              // 无效值标记

/* ==================== 前瞻参数 ==================== */
#define TF_LOOKAHEAD_START_ROW 35    // 前瞻起始行（压缩后=原70）
#define TF_LOOKAHEAD_END_ROW 20      // 前瞻终止行（压缩后=原40）

/* ==================== 二值化常量 ==================== */
#define Image_WHITE 255u
#define Image_BLACK 0u

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
} TrackFusion_t;

extern TrackFusion_t g_tf;

/* ==================== 二值化图像 ==================== */
extern uint8 Image_Binarize[TF_IMG_H][TF_IMG_W];

/* ==================== 压缩灰度图像 ==================== */
extern uint8 image_0[COMP_H][COMP_W];

/* ==================== Otsu阈值 ==================== */
extern uint16 Image_Threshold;

/* ==================== 路口检测参数 ==================== */
#define INTER_COOLDOWN_FRAMES  60
#define INTER_MAX_LOCK_FRAMES  300
#define INTER_BOX_START_ROW    30    // 压缩后，拐点行号>=此值才开始画框
#define INTER_MIN_MISS_ROW     35    // miss必须发生在行35以下（靠近车头）才算
#define INTER_MIN_VALID_ROWS   25    // 有效行数>此值时，追踪质量好，不触发路口检测

#define BOX_HEIGHT         20        // 压缩后框高
#define BOX_WIDTH          40        // 压缩后框宽
#define INTER_BOX_MIN_STREAK 2       // 框边最少连续白点数

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
    uint8 detected_type;     // 0=正常 1=右转 2=左转 3=左右 4=十字 5=上右 6=上左右
} IntersectionResult_t;

/* ==================== 直角预判参数（压缩坐标） ==================== */
#define RA_PRE_START_ROW 38        // 预判起始行（压缩后=原75/2）
#define RA_PRE_END_ROW 28          // 预判终止行（压缩后=原55/2）
#define RA_PRE_LOST_THRESH 3       // 丢边行数阈值（压缩后=原5取整）
#define RA_PRE_EDGE_MARGIN 3       // 边线贴近图像边缘的阈值（压缩后=原5取整）

/* ==================== 导出变量 ==================== */
extern uint8 g_ra_flag;               // 0=无 1=右转 2=左转 3=左右 4=十字 5=上右 6=上左右
extern uint8 g_ra_pre_flag;           // 1=远处看到直角需减速 0=正常
extern IntersectionResult_t g_inter_result;
extern uint8 g_ip_max_row;
extern int16 ip_col_offset;
extern int16 ip_left_col;
extern int16 ip_right_col;

/* ==================== 函数接口 ==================== */
void track_fusion_init(void);
void track_fusion_update(void);
void right_angle_pre_detect(void);
void detect_intersection(void);

#endif /* TRACK_FUSION_H */
