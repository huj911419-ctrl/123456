/**
 * ========================================================================
 * Track_funsion.h - 赛道融合检测模块头文件
 * ========================================================================
 * 定义了视觉巡线系统的所有常量、数据结构和接口函数。
 *
 * 核心数据流：
 *   原始图像(188x120) -> 压缩(94x60) -> Otsu阈值 -> 二值化 -> 降噪
 *   -> 边缘扫描 -> 加权中线误差 -> PID控制
 *
 * 路口检测流：
 *   边缘丢失检测 -> 拐点扫描 -> 检测框构建 -> 类型分类(右转/左转/T型/十字)
 * ======================================================================== */
#ifndef TRACK_FUSION_H
#define TRACK_FUSION_H

#include "zf_common_headfile.h"
#include "App_Config.h"

extern int16 threshold_bias;    /* 阈值偏移量（菜单可调），正值=更高阈值（更少白色） */

/* ========================================================================
 * 图像尺寸常量
 * ======================================================================== */

#define COMP_W          (MT9V03X_W / 2)     /* 压缩后图像宽度：188/2 = 94像素 */
#define COMP_H          (MT9V03X_H / 2)     /* 压缩后图像高度：120/2 = 60像素 */
#define COMP_CENTER     (COMP_W / 2)        /* 压缩图像中心列：94/2 = 47 */

#define TF_IMG_H        COMP_H              /* 处理图像高度：60行 */
#define TF_IMG_W        COMP_W              /* 处理图像宽度：94列 */
#define TF_IMG_CENTER   COMP_CENTER         /* 图像中心列号：47 */

/* ========================================================================
 * Otsu自适应阈值参数
 * ======================================================================== */

#ifndef TF_OTSU_INTERVAL
#if RACE_MODE
#define TF_OTSU_INTERVAL    TF_OTSU_INTERVAL_RACE   /* 比赛模式：每8帧计算一次 */
#else
#define TF_OTSU_INTERVAL    TF_OTSU_INTERVAL_DEBUG  /* 调试模式：每6帧计算一次 */
#endif
#endif
#define TF_OTSU_MIN_THRESHOLD 60            /* Otsu最低阈值保护：防止低对比度场景阈值过低 */

/* ========================================================================
 * 基准行（基点）搜索参数
 * ========================================================================
 * 基准行是从底部开始搜索赛道的起始行，找到后从此行向上逐行扫描边缘。
 */

#define TF_JIDIAN_ROW       (TF_IMG_H - 4)  /* 基准行起始行号：56（从底部向上第4行） */
#define TF_JIDIAN_MIN_ROW   (TF_IMG_H - 15) /* 基准行最小行号：45（向上搜索12行） */
#define TF_SINGLE_EDGE_JIDIAN_MIN_ROW (TF_IMG_H - 32) /* 单边模式基准行最小行号：28 */
#define TF_SEARCH_END_ROW   4               /* 边缘扫描终止行号：4（接近图像顶部） */
#define TF_LOCAL_RANGE      10              /* 局部搜索范围：±10像素 */
#define TF_MAX_MISS_ROWS    5               /* 最大连续丢失行数：超过则停止向上扫描 */
#define TF_BRANCH_ROW_KEEP_SCAN 1           /* 分支行处理策略：1=跳过继续扫描（保住前瞻） */

/* ========================================================================
 * 赛道宽度参数
 * ======================================================================== */

#define TF_MIN_TRACK_WIDTH  4               /* 最小赛道宽度（像素）：低于此值视为噪声 */
#define TF_MAX_TRACK_WIDTH  30              /* 最大赛道宽度（像素）：高于此值视为背景 */
#define TRACK_HALF_WIDTH    8               /* 默认赛道半宽度（像素） */
#define TRACK_HALF_WIDTH_MIN 4              /* 动态半宽度最小值 */
#define TRACK_HALF_WIDTH_MAX 14             /* 动态半宽度最大值 */
#define TF_INVALID          (-1)            /* 无效值标记：-1表示边缘/中线未检测到 */

/* ========================================================================
 * 降噪参数
 * ======================================================================== */

#define TF_DENOISE_WHITE_MIN 4              /* 白色像素保留阈值：邻域白色>=4则保留 */
#define TF_DENOISE_BLACK_FILL 7             /* 黑色像素填充阈值：邻域白色>=7则变白 */

/* ========================================================================
 * 误差滤波参数
 * ======================================================================== */

#define TF_ERROR_FILTER_MAX_STEP 8          /* 误差滤波最大步长：每帧变化不超过8 */
#define TF_LOOKAHEAD_FILTER_MAX_STEP 10     /* 前瞻滤波最大步长：每帧变化不超过10 */

/* ========================================================================
 * 快速视觉模式参数
 * ========================================================================
 * 高速行驶时启用快速模式，降低降噪频率和辅助行扫描密度以节省CPU。
 */

#define TF_FAST_VISION_SPEED 1500           /* 快速模式速度阈值：base_speed>=1500时启用 */
#define TF_FAST_DENOISE_DIV 3u             /* 快速模式降噪分频：每3帧降噪1帧 */
#define TF_FAST_AUX_ROW_STEP 3u            /* 快速模式辅助行步长：每3行扫描1行 */

/* ========================================================================
 * 眩光防护参数
 * ========================================================================
 * 某些行可能因强光反射导致大面积白色（眩光），需要特殊处理。
 */

#define TF_GLARE_PRE_GUARD_ENABLE 1u        /* 眩光防护开关 */
#define TF_GLARE_PRE_ROW_WHITE_MIN 42u      /* 眩光判定：白色像素>=42 */
#define TF_GLARE_PRE_ROW_STREAK_MIN 30u     /* 眩光判定：最长连续白色>=30 */

/* ========================================================================
 * 前瞻误差参数
 * ========================================================================
 * 前瞻误差使用图像顶部的行计算，用于弯道预测和速度规划。
 */

#define TF_LOOKAHEAD_START_ROW 35           /* 前瞻起始行（行号较大，靠近底部） */
#define TF_LOOKAHEAD_END_ROW   20           /* 前瞻结束行（行号较小，靠近顶部） */

/* ========================================================================
 * 拐角填充（Corner Fill）参数
 * ========================================================================
 * 在RA退出阶段，当边缘丢失时用贝塞尔曲线填充缺失的中线，
 * 保证PID控制在转弯恢复期间仍有有效的误差输入。
 */

#define CORNER_FILL_ENABLE 1                /* 拐角填充开关 */
#define CORNER_FILL_ENTRY_MARGIN 5          /* 入口边距（行） */
#define CORNER_FILL_EXIT_MARGIN 5           /* 出口边距（行） */
#define CORNER_FILL_MIN_SPAN 6              /* 最小跨度（行） */
#define CORNER_FILL_BRANCH_GAP 4            /* 分支间隙（列） */
#define CORNER_FILL_EDGE_MARGIN 6           /* 边缘边距（列） */
#define CORNER_FILL_BRANCH_MIN_STREAK 5     /* 分支最小连续白色段长度 */
#define CORNER_FILL_EXIT_SCAN_ROWS 8        /* 出口扫描行数 */
#define CORNER_FILL_L1_PCT 14               /* 贝塞尔控制点1长度百分比 */
#define CORNER_FILL_L2_PCT 22               /* 贝塞尔控制点2长度百分比 */
#define CORNER_FILL_OUT_SLOPE 1.6f          /* 出口斜率 */
#define CORNER_FILL_DIR_MAX 3               /* 方向最大值 */
#define CORNER_FILL_EXIT_SHIFT_MIN 12       /* 出口偏移最小值 */
#define CORNER_FILL_EXIT_SHIFT_MAX 24       /* 出口偏移最大值 */
#define CORNER_FILL_INNER_SHIFT_MIN 8       /* 内侧偏移最小值 */
#define CORNER_FILL_INNER_SHIFT_MAX 16      /* 内侧偏移最大值 */
#define CORNER_FILL_TAKEOVER_LA_ROWS 6u     /* 接管前瞻行数 */

/* ========================================================================
 * 像素值定义
 * ======================================================================== */

#define Image_WHITE 255u                    /* 白色像素值（赛道区域） */
#define Image_BLACK 0u                      /* 黑色像素值（背景区域） */

/* ========================================================================
 * 核心数据结构
 * ======================================================================== */

/**
 * @brief 巡线融合主结构体
 *
 * 存储每帧视觉处理的完整结果，供PID控制和调试显示使用。
 * 通过 track_fusion_publish() 从中断安全地发布到全局变量 g_tf。
 */
typedef struct
{
    int16 left_edge[TF_IMG_H];      /* 每行的左边缘列号（TF_INVALID=无效） */
    int16 right_edge[TF_IMG_H];     /* 每行的右边缘列号（TF_INVALID=无效） */
    int16 center_line[TF_IMG_H];    /* 每行的中线列号（TF_INVALID=无效） */
    uint8 row_valid[TF_IMG_H];      /* 每行有效标志：0=无效，1=边缘对合法 */

    int16 error;                    /* 横向偏差（加权平均），负=偏左，正=偏右 */
    int16 lookahead_error;          /* 前瞻误差（顶部加权），用于弯道预测 */
    int16 error_trend;              /* 误差趋势 = 前瞻误差 - 横向误差 */
    uint16 valid_row_count;         /* 有效行计数，越多说明看到的赛道越远 */
    uint8 line_lost;                /* 丢线标志：0=正常，1=赛道丢失 */

    uint8 left_jidian;              /* 基准行左边缘列号 */
    uint8 right_jidian;             /* 基准行右边缘列号 */
} TrackFusion_t;

/* ========================================================================
 * 全局变量声明
 * ======================================================================== */

extern TrackFusion_t g_tf;                  /* 巡线融合主结构体（最新发布版本） */
extern uint8 g_corner_fill_active;          /* 拐角填充激活标志 */
extern int16 g_corner_fill_center[TF_IMG_H]; /* 拐角填充中线（贝塞尔曲线） */
extern uint8 g_corner_fill_valid[TF_IMG_H]; /* 拐角填充有效标志 */
extern uint8 Image_Binarize[TF_IMG_H][TF_IMG_W]; /* 二值化图像数据 */
extern uint8 image_0[COMP_H][COMP_W];      /* 压缩灰度图像数据 */
extern uint16 Image_Threshold;              /* 当前二值化阈值 */
extern volatile uint8 g_tf_image_ready;     /* 图像就绪标志 */
extern volatile uint32 g_tf_frame_count;    /* 帧计数器 */
extern volatile uint16 g_tf_white_count;    /* 白色像素计数 */

/* ========================================================================
 * 路口检测参数
 * ======================================================================== */

#define INTER_COOLDOWN_FRAMES  2            /* 路口冷却帧数 */
#define INTER_POST_TURN_SUPPRESS_FRAMES 8u  /* 转弯后抑制帧数 */
#define INTER_POST_TURN_BLOCK_FRAMES 2u     /* 转弯后阻断帧数 */
#define INTER_POST_TURN_FAR_BLOCK_FRAMES 1u /* 远场阻断帧数 */
#define INTER_MAX_LOCK_FRAMES  300          /* 最大锁定帧数 */
#define INTER_BOX_START_ROW    28           /* 检测框起始行 */
#define INTER_MIN_MISS_ROW     18           /* 最小丢失行 */
#define INTER_MIN_VALID_ROWS   25           /* 最小有效行数 */
#define BOX_HEIGHT             36           /* 默认框高度 */
#define BOX_WIDTH              60           /* 默认框宽度 */
#define INTER_BOX_WIDTH_MIN    56            /* 框宽度最小值 */
#define INTER_BOX_WIDTH_MAX    78            /* 框宽度最大值 */
#define INTER_BOX_WIDTH_SCALE  4             /* 框宽度缩放 */
#define INTER_BOX_HEIGHT_MIN   32            /* 框高度最小值 */
#define INTER_BOX_HEIGHT_MAX   46            /* 框高度最大值 */
#define INTER_BOX_HEIGHT_SCALE 3             /* 框高度缩放 */
#define INTER_BOX_FRONT_PCT    80            /* 框前百分比 */
#define INTER_BAND_THICKNESS   3             /* 带厚度 */
#define INTER_TOP_MIN_WHITE    8             /* 顶部最小白色 */
#define INTER_TOP_SCAN_ROWS    6             /* 顶部扫描行数 */
#define INTER_SIDE_MIN_WHITE   5             /* 侧面最小白色 */
#define INTER_SIDE_SCAN_COLS   10            /* 侧面扫描列数 */
#define INTER_SIDE_HIT_ROWS    2             /* 侧面命中行数 */
#define INTER_SIDE_HIT_WHITE   8             /* 侧面命中白色 */
#define INTER_SIDE_CROSS_TOP_PCT 15          /* 侧面交叉顶部百分比 */
#define INTER_SIDE_CROSS_BOTTOM_PCT 75       /* 侧面交叉底部百分比 */
#define INTER_SIDE_CROSS_MIN_STREAK 8        /* 侧面交叉最小连续 */
#define INTER_SIDE_CROSS_HIT_ROWS 2          /* 侧面交叉命中行数 */
#define INTER_BAND_MIN_STREAK  5             /* 带最小连续 */
#define INTER_BRANCH_MIN_STREAK 12           /* 分支最小连续 */
#define INTER_BRANCH_AREA_HALF_ROWS 4        /* 分支区域半行数 */
#define INTER_BRANCH_AREA_MIN_ROWS 3         /* 分支区域最小行数 */
#define INTER_BRANCH_AREA_MIN_WHITE 28       /* 分支区域最小白色 */
#define INTER_BRANCH_STRONG_MIN_STREAK 16    /* 分支强最小连续 */
#define INTER_BRANCH_STRONG_AREA_MIN_ROWS 3  /* 分支强区域最小行数 */
#define INTER_BRANCH_STRONG_AREA_MIN_WHITE 36 /* 分支强区域最小白色 */
#define INTER_BRANCH_PAIR_ROW_DELTA 8        /* 分支行对差值 */
#define INTER_BRANCH_NEAR_GAP 6              /* 分支近间隙 */
#define INTER_BRANCH_MIN_REACH 22            /* 分支最小到达 */
#define INTER_BRANCH_REAL_ROWS 2             /* 分支真实行数 */
#define INTER_EDGE_TOUCH_MARGIN 5u           /* 边缘接触边距 */
#define INTER_EDGE_CONNECT_MIN_STREAK 14u    /* 边缘连接最小连续 */
#define INTER_EDGE_CONNECT_HIT_ROWS 2u      /* 边缘连接命中行数 */
#define INTER_BOX_STABLE_DELTA 3             /* 框稳定偏差 */
#define INTER_BOX_STABLE_FRAMES 1            /* 框稳定帧数 */
#define INTER_TYPE_VOTE_FRAMES 2             /* 类型投票帧数 */
#define INTER_TYPE_VOTE_MIN    2             /* 类型投票最小值 */
#define INTER_FAST_CONFIRM_ENABLE 1          /* 快速确认开关 */
#define INTER_DIRECT_FAST_CONFIRM_ROW_MARGIN 0u /* 直接快速确认行边距 */
#define INTER_IP_SIDE_BIAS     8             /* 拐点侧面偏差 */
#define INTER_IP_ROW_BIAS      2             /* 拐点行偏差 */
#define INTER_SYM_ROW_DELTA    3             /* 对称行差值 */
#define INTER_SYM_CENTER_DELTA 8             /* 对称中心偏差 */
#define INTER_SYM_BRANCH_STREAK 18           /* 对称分支连续 */
#define INTER_SYM_PRE_ROWS     2             /* 对称预行数 */
#define INTER_INLINE_AHEAD_ROWS 18           /* 内联前行数 */
#define INTER_INLINE_CENTER_MIN_WHITE 4      /* 内联中心最小白色 */
#define INTER_INLINE_CENTER_MIN_STREAK 2     /* 内联中心最小连续 */
#define INTER_INLINE_CENTER_DELTA 16         /* 内联中心偏差 */
#define INTER_STRAIGHT_IGNORE_ROWS 42        /* 直行忽略行数 */
#define INTER_STRAIGHT_ERR_LIMIT 18          /* 直行误差限制 */
#define INTER_STRAIGHT_LA_ERR_LIMIT 24       /* 直行前瞻误差限制 */
#define INTER_FAST_PASS_VALID_ROWS 45u       /* 快速通过有效行数 */
#define INTER_FAST_PASS_TOP_START_ROW 4      /* 快速通过顶部起始行 */
#define INTER_FAST_PASS_TOP_END_ROW 16       /* 快速通过顶部结束行 */
#define INTER_FAST_PASS_TOP_MIN_VALID 8u     /* 快速通过顶部最小有效 */
#define INTER_FAST_PASS_ERR_LIMIT 32         /* 快速通过误差限制 */
#define INTER_FAST_PASS_LA_ERR_LIMIT 36      /* 快速通过前瞻误差限制 */
#define INTER_FAST_PASS_TREND_LIMIT 36       /* 快速通过趋势限制 */
#define INTER_FAST_PASS_EDGE_MARGIN 6        /* 快速通过边缘边距 */
#define INTER_TRI_CROSS_VALID_ROWS_MAX 38u   /* 三角交叉最大有效行数 */
#define INTER_TRI_CROSS_TOP_VALID_MAX 3u     /* 三角交叉顶部最大有效 */
#define INTER_INLINE_COMPONENT_VALID_ROWS_MIN 44u /* 内联组件最小有效行数 */
#define INTER_INLINE_COMPONENT_TOP_VALID_MIN 8u   /* 内联组件顶部最小有效 */
#define INTER_INLINE_COMPONENT_BOTTOM_ROW_MIN 51  /* 内联组件底部最小行 */
#define INTER_INLINE_COMPONENT_BOTTOM_ROW_MAX 53  /* 内联组件底部最大行 */
#define INTER_INLINE_COMPONENT_BOTTOM_VALID_MIN 47u /* 内联组件底部最小有效 */
#define INTER_INLINE_COMPONENT_BOTTOM_TOP_MIN 10u  /* 内联组件底部顶部最小 */
#define INTER_INLINE_COMPONENT_WHITE_MIN 280u     /* 内联组件最小白色 */
#define INTER_INLINE_STRAIGHT_VALID_ROWS 40u      /* 内联直行有效行数 */
#define INTER_INLINE_STRAIGHT_TOP_VALID_MIN 4u    /* 内联直行顶部最小有效 */
#define INTER_INLINE_STRAIGHT_LOWER_VALID_MIN 20u /* 内联直行下部最小有效 */
#define INTER_INLINE_STRAIGHT_IP_ROW_MAX 32       /* 内联直行最大拐点行 */
#define INTER_INLINE_STRAIGHT_ERR_LIMIT 6         /* 内联直行误差限制 */
#define INTER_INLINE_STRAIGHT_LA_LIMIT 6          /* 内联直行前瞻限制 */
#define INTER_INLINE_STRAIGHT_TREND_LIMIT 6       /* 内联直行趋势限制 */
#define INTER_COMPONENT_GUARD_ENABLE 1u           /* 组件防护开关 */
#define INTER_COMPONENT_GUARD_VALID_ROWS 32u      /* 组件防护有效行数 */
#define INTER_COMPONENT_GUARD_LOWER_ROWS 16u      /* 组件防护下部行数 */
#define INTER_COMPONENT_GUARD_STRONG_VALID_ROWS 44u /* 组件防护强有效行数 */
#define INTER_COMPONENT_GUARD_TOP_ROWS 4u         /* 组件防护顶部行数 */
#define INTER_COMPONENT_GUARD_ERR_LIMIT 76        /* 组件防护误差限制 */
#define INTER_COMPONENT_GUARD_LA_LIMIT 76         /* 组件防护前瞻限制 */
#define INTER_COMPONENT_GUARD_TREND_LIMIT 76      /* 组件防护趋势限制 */
#define INTER_COMPONENT_PRE_GUARD_ENABLE 1u       /* 组件预防护开关 */
#define INTER_COMPONENT_PRE_VALID_ROWS 40u        /* 组件预防护有效行数 */
#define INTER_COMPONENT_PRE_LOWER_ROWS 24u        /* 组件预防护下部行数 */
#define INTER_COMPONENT_PRE_TOP_ROWS 2u           /* 组件预防护顶部行数 */
#define INTER_COMPONENT_PRE_CENTER_SPAN_MAX 14    /* 组件预防护中心跨度最大值 */
#define INTER_COMPONENT_PRE_ERR_LIMIT 32          /* 组件预防护误差限制 */
#define INTER_COMPONENT_PRE_LA_LIMIT 36           /* 组件预防护前瞻限制 */
#define INTER_COMPONENT_PRE_TREND_LIMIT 36        /* 组件预防护趋势限制 */
#define INTER_ROUTE_REMAP_IP_ROW 70u              /* 路线重映射拐点行 */
#define INTER_ROUTE_REMAP_VALID_ROWS 30u          /* 路线重映射有效行数 */
#define INTER_ROUTE_EARLY_IP_ROW 72u              /* 路线早期复杂拐点行 */
#define INTER_ROUTE_EARLY_VALID_ROWS 34u          /* 路线早期有效行上限 */
#define INTER_ROUTE_EARLY_LA_MIN 20               /* 路线早期前瞻最小变化 */
#define INTER_ROUTE_EARLY_TREND_MIN 20            /* 路线早期趋势最小变化 */
#define INTER_COMPLEX_ARM_IP_ROW 68u              /* 复杂臂拐点行 */
#define INTER_SINGLE_EDGE_COMPONENT_ARM_IP_ROW 88u /* 单边组件臂拐点行 */

/* ========================================================================
 * 证据标志位（用于调试和分类）
 * ======================================================================== */

#define INTER_EVID_TOP     0x01u            /* 顶部证据 */
#define INTER_EVID_CENTER  0x02u            /* 中心证据 */
#define INTER_EVID_LEFT    0x04u            /* 左侧证据 */
#define INTER_EVID_RIGHT   0x08u            /* 右侧证据 */
#define INTER_EVID_EDGE    0x10u            /* 边缘证据 */
#define INTER_EVID_ROUTE   0x20u            /* 路线证据 */
#define INTER_EVID_CUTOFF  0x40u            /* 截断证据 */
#define INTER_EVID_FAST    0x80u            /* 快速证据 */

/* ========================================================================
 * 路口检测数据结构
 * ======================================================================== */

/**
 * @brief 拐点信息结构体
 *
 * 拐点是赛道边缘发生突变的位置（如路口分支点）。
 * 通过从丢失行向上扫描白色像素来定位。
 */
typedef struct
{
    uint8 valid;    /* 拐点有效标志：0=未检测到，1=有效 */
    int16 row;      /* 拐点行号（图像坐标） */
    int16 col;      /* 拐点列号（图像坐标） */
} InflectionPoint_t;

/**
 * @brief 路口检测结果结构体
 *
 * 存储路口检测的完整结果，包括左右拐点坐标、检测框边界和分类结果。
 */
typedef struct
{
    InflectionPoint_t left_ip;  /* 左拐点信息 */
    InflectionPoint_t right_ip; /* 右拐点信息 */
    uint8 box_top;              /* 检测框上边界（行号） */
    uint8 box_bottom;           /* 检测框下边界（行号） */
    uint8 box_left;             /* 检测框左边界（列号） */
    uint8 box_right;            /* 检测框右边界（列号） */
    uint8 detected_type;        /* 检测到的路口类型：1=右转，2=左转，3=T左，4=T右，5=十字 */
    uint8 confidence;           /* 检测置信度（0-100） */
    uint8 evidence;             /* 证据标志位组合（INTER_EVID_*） */
} IntersectionResult_t;

/* ========================================================================
 * 直角预检测参数
 * ======================================================================== */

#define RA_PRE_START_ROW    (TF_IMG_H - 5)  /* 预检测起始行：55 */
#define RA_PRE_END_ROW      8               /* 预检测结束行：8 */
#define RA_PRE_LOST_THRESH  1               /* 预检测丢失阈值 */
#define RA_PRE_EDGE_MARGIN  20              /* 预检测边缘边距 */
#define RA_PRE_FAR_START_ROW 28             /* 远场预检测起始行 */
#define RA_PRE_FAR_END_ROW   0              /* 远场预检测结束行 */
#define RA_PRE_FAR_WIDTH_MIN 10             /* 远场最小宽度 */
#define RA_PRE_FAR_LEFT_COL  36             /* 远场左列阈值 */
#define RA_PRE_FAR_RIGHT_COL 64             /* 远场右列阈值 */
#define RA_PRE_FAR_OPEN_ROWS 2              /* 远场打开行数 */
#define RA_PRE_VALID_ROWS_LOW 26            /* 低有效行数阈值 */
#define RA_PRE_CENTER_SPAN_MIN 14           /* 中心跨度最小值 */
#define RA_PRE_COMPONENT_OPEN_ROWS 5        /* 组件打开行数 */
#define RA_PRE_COMPONENT_CENTER_SPAN 30     /* 组件中心跨度 */
#define RA_PRE_COMPONENT_VALID_ROWS 36      /* 组件有效行数 */
#define RA_PRE_ROUTE_VALID_ROWS 42u         /* 路线有效行数 */
#define RA_PRE_ROUTE_START_ROWS 36u         /* 路线起始行数 */
#define RA_PRE_ROUTE_VALID_ROWS_MIN 8u      /* 路线最小有效行数 */
#define RA_PRE_ROUTE_ERR_MAX 54             /* 路线最大误差 */
#define RA_PRE_ROUTE_LA_MAX 64              /* 路线最大前瞻误差 */
#define RA_PRE_ROUTE_STRAIGHT_ERR_MAX 16    /* 路线直行最大误差 */
#define RA_PRE_ROUTE_STRAIGHT_LA_MAX 20     /* 路线直行最大前瞻误差 */
#define RA_PRE_ROUTE_IP_ROW 30u             /* 路线拐点行 */
#define RA_PRE_CONFIRM_FRAMES 1             /* 预检测确认帧数 */
#define RA_PRE_SLOW_OFF_FRAMES 4            /* 预减速关闭帧数 */

/* ========================================================================
 * 全局标志变量声明
 * ======================================================================== */

extern uint8 g_ra_flag;                     /* 路口/直角标志：0=无，1=右转，2=左转，3~5=普通路口 */
extern uint8 g_ra_pre_flag;                 /* 直角预检测标志：1=检测到远场边缘丢失 */
extern uint8 g_ra_pre_dir;                  /* 预检测方向：1=右，2=左 */
extern uint8 g_ra_pre_slow_flag;            /* 预减速标志：1=需要提前减速 */
extern uint8 g_sym_component_flag;          /* 对称分量标志：1=检测到三极管等干扰 */
extern IntersectionResult_t g_inter_result; /* 路口检测结果结构体 */
extern uint8 g_ip_max_row;                  /* 最大拐点行号（像素/2），用于RA阶段转换判断 */
extern int16 ip_col_offset;                 /* 中点缓冲区偏移量 */
extern int16 ip_left_col;                   /* 左侧探针列号 */
extern int16 ip_right_col;                  /* 右侧探针列号 */

/* ========================================================================
 * 接口函数声明
 * ======================================================================== */

void track_fusion_init(void);               /* 巡线融合模块初始化 */
void track_fusion_update(void);             /* 每帧视觉处理主函数 */
void track_fusion_publish(void);            /* 发布结果到全局变量（中断安全） */
void track_intersection_suppress_after_turn(void); /* 转弯后抑制路口检测 */
void right_angle_pre_detect(void);          /* 直角/弯道预检测 */
void detect_intersection(void);             /* 路口检测（拐点+框+分类） */

#endif /* TRACK_FUSION_H */
