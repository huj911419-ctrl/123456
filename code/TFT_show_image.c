#include "TFT_show_image.h"    /* 包含TFT显示模块头文件，提供TFT180驱动函数声明 */
#include "Track_funsion.h"     /* 包含视觉融合模块头文件，提供g_tf、g_inter_result等全局数据 */
#include "Function.h"          /* 包含功能函数头文件，提供CLAMP宏定义 */
#include "IMU.h"               /* 包含IMU模块头文件，提供yaw_angle等航向角数据 */
#include "Menu.h"              /* 包含菜单模块头文件，提供motor_enable等运行参数 */
#include "Pid.h"               /* 包含PID控制模块头文件，提供电机速度、调试变量等 */
#include "Battery.h"           /* 包含电池电压模块头文件，提供battery_get_voltage_x10 */

/* 外部性能计时变量，用于测量 track_fusion 和路口检测的执行耗时（微秒） */
extern volatile uint32 prof_tf_us;      /* track_fusion_update() 的执行耗时（微秒） */
extern volatile uint32 prof_inter_us;   /* right_angle_pre_detect()+detect_intersection() 的执行耗时（微秒） */


/* ================================================================
 *  显示逻辑：
 *    第一步：先把 Image_Binarize 显示到TFT上（黑白二值化的图像）
 *    第二步：在地图上画边界线和中线
 *    第三步：在右上角显示一些数值
 *
 *  效果：可以直接看到算法"看到"的情况（白色=赛道线，黑色=背景
 *        蓝色线=左边界  绿色线=右边界  红色线=中线
 * ================================================================ */

/* ---- 图像缩放参数 ---- */
/*
 * TFT 屏幕为 128x160 像素。
 * 压缩后图像尺寸为 94x60（COMP_W x COMP_H）。
 * 为了在 TFT 上显示，需要进行缩放：
 *   列方向：94 * 1.15 = 108 像素（留出右侧空间给状态文字）
 *   行方向：60 * 1.33 = 80 像素（占屏幕上半部分）
 */
#define TFT_COL_SCALE_C   1.15f   /* 列方向缩放系数：94 -> 108，留出右侧状态文字区域 */
#define TFT_ROW_SCALE_C   1.33f   /* 行方向缩放系数：60 -> 80，给底部文字区留出间距 */
#define TFT_IMG_DISP_W    108u    /* 缩放后图像显示宽度（像素），94*1.15=108 */
#define TFT_IMG_DISP_H    80u     /* 缩放后图像显示高度（像素），不占用底部调试区 */

/* ---- 颜色定义（RGB565 格式，16位色） ---- */
#ifndef RGB565_YELLOW
#define RGB565_YELLOW   (0xFFE0u)   /* 黄色，RGB565编码，用于标记左拐点 */
#endif
#ifndef RGB565_MAGENTA
#define RGB565_MAGENTA  (0xF81Fu)   /* 洋红色，RGB565编码，用于标记右拐点 */
#endif
#ifndef RGB565_CYAN
#define RGB565_CYAN     (0x07FFu)   /* 青色，RGB565编码，用于绘制路口检测框 */
#endif

/* 拐点 X 标记的半尺寸（标记为一个 X 形，half_size=3 表示 X 跨度为 7 像素） */
#define IP_MARK_SIZE      3u        /* X形标记的半尺寸，从中心向外延伸3像素 */

/**
 * tft_map_col() - 将图像列坐标映射到 TFT 屏幕 x 坐标
 *
 * 将压缩图像的列号（0~93）乘以缩放系数，得到 TFT 屏幕上的 x 像素位置。
 * 结果被限制在 [0, TFT_IMG_DISP_W-1] 范围内，防止越界。
 *
 * @param col  压缩图像的列坐标（0 ~ COMP_W-1）
 * @return     TFT 屏幕上的 x 坐标（0 ~ TFT_IMG_DISP_W-1）
 */
static uint8 tft_map_col(int16 col)
{
    int16 x = (int16)(TFT_COL_SCALE_C * (float)col + 0.5f);  /* 将列坐标乘以缩放系数并四舍五入 */
    return (uint8)CLAMP(0, x, (int16)TFT_IMG_DISP_W - 1);    /* 限幅到有效显示范围[0, 107] */
}

/**
 * tft_map_row() - 将图像行坐标映射到 TFT 屏幕 y 坐标
 *
 * 将压缩图像的行号（0~59）乘以缩放系数，得到 TFT 屏幕上的 y 像素位置。
 * 结果被限制在 [0, TFT_IMG_DISP_H-1] 范围内，防止越界。
 *
 * @param row  压缩图像的行坐标（0 ~ COMP_H-1）
 * @return     TFT 屏幕上的 y 坐标（0 ~ TFT_IMG_DISP_H-1）
 */
static uint8 tft_map_row(int16 row)
{
    int16 y = (int16)(TFT_ROW_SCALE_C * (float)row + 0.5f);  /* 将行坐标乘以缩放系数并四舍五入 */
    return (uint8)CLAMP(0, y, (int16)TFT_IMG_DISP_H - 1);    /* 限幅到有效显示范围[0, 80] */
}

/**
 * tft_draw_x_mark() - 在 TFT 屏幕上绘制一个 X 形标记
 *
 * 从中心点 (x, y) 向四周扩展 half_size 个像素，绘制两条对角线组成的 X。
 * 用于在图像上标记拐点位置（左拐点用黄色，右拐点用洋红色）。
 * 每个像素绘制前都会检查是否在有效显示区域内。
 *
 * @param x         标记中心的 TFT x 坐标
 * @param y         标记中心的 TFT y 坐标
 * @param half_size 标记的半尺寸（X 从中心向外延伸的像素数）
 * @param color     标记颜色（RGB565 格式）
 */
static void tft_draw_x_mark(uint8 x, uint8 y, uint8 half_size, uint16 color)
{
    int16 i;  /* 循环变量，用于遍历对角线上的每个像素偏移 */
    /* 遍历从 -half_size 到 +half_size，绘制两条对角线上的所有像素 */
    for (i = -(int16)half_size; i <= (int16)half_size; i++)  /* i为相对于中心的偏移量 */
    {
        int16 px1 = (int16)x + i;       /* 当前像素的x坐标 = 中心x + 偏移i */
        int16 py1 = (int16)y + i;       /* 主对角线y偏移（左上->右下方向），y随x同步偏移 */
        int16 py2 = (int16)y - i;       /* 副对角线y偏移（右上->左下方向），y随x反向偏移 */

        /* 检查 x 坐标是否在有效显示范围内 */
        if (px1 >= 0 && px1 < (int16)TFT_IMG_DISP_W)  /* x在[0, 107]范围内 */
        {
            /* 绘制主对角线上的点（检查y范围） */
            if (py1 >= 0 && py1 < (int16)TFT_IMG_DISP_H)  /* y在[0, 80]范围内 */
                tft180_draw_point((uint8)px1, (uint8)py1, color);  /* 绘制主对角线像素 */
            /* 绘制副对角线上的点（检查y范围） */
            if (py2 >= 0 && py2 < (int16)TFT_IMG_DISP_H)  /* y在[0, 80]范围内 */
                tft180_draw_point((uint8)px1, (uint8)py2, color);  /* 绘制副对角线像素 */
        }
    }
}

/**
 * tft_draw_rect_outline() - 在 TFT 屏幕上绘制矩形边框（仅四条边，不填充）
 *
 * 用于绘制路口检测框的外轮廓。先画上下两条水平边，再画左右两条垂直边。
 * 如果 left > right 或 top > bottom（无效矩形），则直接返回。
 *
 * @param left    矩形左边 x 坐标
 * @param top     矩形上边 y 坐标
 * @param right   矩形右边 x 坐标
 * @param bottom  矩形下边 y 坐标
 * @param color   边框颜色（RGB565 格式）
 */
static void tft_draw_rect_outline(uint8 left, uint8 top, uint8 right, uint8 bottom, uint16 color)
{
    uint8 c, r;  /* c为列循环变量，r为行循环变量 */

    /* 参数合法性检查：左上角坐标必须小于右下角坐标，否则为无效矩形 */
    if (left > right || top > bottom)  /* 无效矩形参数 */
        return;                        /* 直接返回，不绘制 */

    /* 绘制上下两条水平边（从 left 到 right 逐像素画线） */
    for (c = left; c <= right; c++)  /* 遍历水平方向每个像素 */
    {
        tft180_draw_point(c, top, color);     /* 绘制上边水平线上的像素 */
        tft180_draw_point(c, bottom, color);  /* 绘制下边水平线上的像素 */
    }
    /* 绘制左右两条垂直边（从 top 到 bottom 逐像素画线） */
    for (r = top; r <= bottom; r++)  /* 遍历垂直方向每个像素 */
    {
        tft180_draw_point(left, r, color);    /* 绘制左边垂直线上的像素 */
        tft180_draw_point(right, r, color);   /* 绘制右边垂直线上的像素 */
    }
}

/**
 * tft_draw_line() - 在 TFT 屏幕上绘制任意直线（Bresenham 画线算法）
 *
 * 使用经典的 Bresenham 直线算法，在两点之间画一条直线。
 * 该算法只使用整数运算，适合嵌入式环境。
 * 画线前会裁剪到图像显示区域 [0, TFT_IMG_DISP_W) x [0, TFT_IMG_DISP_H) 内。
 *
 * 用于连接相邻行的边界点和中线点，形成连续的线段。
 *
 * @param x0    起点 x 坐标
 * @param y0    起点 y 坐标
 * @param x1    终点 x 坐标
 * @param y1    终点 y 坐标
 * @param color 线段颜色（RGB565 格式）
 */
static void tft_draw_line(uint8 x0, uint8 y0, uint8 x1, uint8 y1, uint16 color)
{
    int16 x = (int16)x0;    /* 当前绘制点的x坐标，从起点开始 */
    int16 y = (int16)y0;    /* 当前绘制点的y坐标，从起点开始 */
    int16 tx = (int16)x1;   /* 目标点的x坐标（终点） */
    int16 ty = (int16)y1;   /* 目标点的y坐标（终点） */
    int16 dx = (tx >= x) ? (tx - x) : (x - tx);  /* x方向的绝对距离（|x1-x0|） */
    int16 dy = (ty >= y) ? (ty - y) : (y - ty);  /* y方向的绝对距离（|y1-y0|） */
    int16 sx = (x < tx) ? 1 : -1;  /* x方向步进方向：终点在右则+1，在左则-1 */
    int16 sy = (y < ty) ? 1 : -1;  /* y方向步进方向：终点在下则+1，在上则-1 */
    int16 err = dx - dy;           /* Bresenham误差项初始值 */

    while (1)  /* 循环绘制每个像素，直到到达终点 */
    {
        /* 仅在显示区域内绘制像素（裁剪到有效范围） */
        if (x >= 0 && x < (int16)TFT_IMG_DISP_W &&    /* x在[0, 107]范围内 */
            y >= 0 && y < (int16)TFT_IMG_DISP_H)       /* y在[0, 80]范围内 */
        {
            tft180_draw_point((uint8)x, (uint8)y, color);  /* 在当前点绘制一个像素 */
        }

        /* 到达终点，退出循环 */
        if (x == tx && y == ty)  /* 当前坐标与目标坐标完全重合 */
            break;               /* 绘制完成，退出循环 */

        {
            int16 e2 = (int16)(err * 2);  /* 误差项乘2，用于判断下一步移动方向 */
            /* 如果 e2 > -dy，说明需要在 x 方向前进一步（修正垂直方向的偏移） */
            if (e2 > -dy)
            {
                err -= dy;   /* 更新误差项：减去y方向距离 */
                x += sx;     /* x方向前进一步 */
            }
            /* 如果 e2 < dx，说明需要在 y 方向前进一步（修正水平方向的偏移） */
            if (e2 < dx)
            {
                err += dx;   /* 更新误差项：加上x方向距离 */
                y += sy;     /* y方向前进一步 */
            }
        }
    }
}

/**
 * draw_inflection_overlay() - 绘制拐点标记和路口检测框的叠加层
 *
 * 在二值化图像上叠加以下调试信息：
 *   - 左拐点：黄色 X 标记（仅在 RA 标志有效且左拐点有效时显示）
 *   - 右拐点：洋红色 X 标记（仅在 RA 标志有效且右拐点有效时显示）
 *   - 路口检测框：青色矩形边框（仅在 RA 标志有效且至少一个拐点有效、
 *     且框尺寸合法时显示）
 *
 * g_ra_flag: 路口/直角标志（0=无，1~5 表示不同类型路口）
 * g_inter_result: 路口检测结果结构体，包含左右拐点和检测框信息
 */
static void draw_inflection_overlay(void)
{
    uint8 has_box = 0u;  /* 标记是否需要绘制路口检测框，1=需要绘制 */
    uint8 has_ip = (g_inter_result.left_ip.valid || g_inter_result.right_ip.valid) ? 1u : 0u;
    uint8 same_ip = 0u;

    /* 左拐点标记：RA 标志有效（非0）且左拐点有效时，在图像上画黄色 X 形标记 */
    if (g_inter_result.left_ip.valid)  /* 路口已检测到且左拐点有效 */
    {
        tft_draw_x_mark(tft_map_col(g_inter_result.left_ip.col),   /* 将拐点列坐标映射到TFT屏幕x */
                        tft_map_row(g_inter_result.left_ip.row),   /* 将拐点行坐标映射到TFT屏幕y */
                        IP_MARK_SIZE, RGB565_YELLOW);              /* 半尺寸3，颜色为黄色 */
    }

    if (g_inter_result.left_ip.valid && g_inter_result.right_ip.valid &&
        g_inter_result.left_ip.row == g_inter_result.right_ip.row &&
        g_inter_result.left_ip.col == g_inter_result.right_ip.col)
    {
        same_ip = 1u;
    }

    /* 右拐点标记：RA 标志有效且右拐点有效时，在图像上画洋红色 X 形标记 */
    if (g_inter_result.right_ip.valid && same_ip == 0u)  /* 路口已检测到且右拐点有效 */
    {
        tft_draw_x_mark(tft_map_col(g_inter_result.right_ip.col),  /* 将拐点列坐标映射到TFT屏幕x */
                        tft_map_row(g_inter_result.right_ip.row),  /* 将拐点行坐标映射到TFT屏幕y */
                        IP_MARK_SIZE, RGB565_MAGENTA);             /* 半尺寸3，颜色为洋红色 */
    }

    /* 判断是否需要绘制路口检测框：
     * 条件1：RA 标志有效（g_ra_flag != 0）
     * 条件2：至少一个拐点有效（左或右）
     * 条件3：检测框的 bottom > top 且 right > left（框尺寸合法） */
    if (has_ip != 0u)  /* 条件2：至少一个拐点有效 */
    {
        if (g_inter_result.box_bottom > g_inter_result.box_top &&      /* 条件3a：框高度>0 */
            g_inter_result.box_right > g_inter_result.box_left)         /* 条件3b：框宽度>0 */
        {
            has_box = 1u;  /* 所有条件满足，标记需要绘制检测框 */
        }
    }

    /* 绘制青色路口检测框（仅在has_box为1时执行） */
    if (has_box)  /* 需要绘制检测框 */
    {
        tft_draw_rect_outline(tft_map_col((int16)g_inter_result.box_left),    /* 框左边界映射到TFT x坐标 */
                              tft_map_row((int16)g_inter_result.box_top),     /* 框上边界映射到TFT y坐标 */
                              tft_map_col((int16)g_inter_result.box_right),   /* 框右边界映射到TFT x坐标 */
                              tft_map_row((int16)g_inter_result.box_bottom),  /* 框下边界映射到TFT y坐标 */
                              RGB565_CYAN);                                   /* 边框颜色为青色 */
    }
}

/**
 * draw_line() - TFT 调试显示主函数 -- 显示二值化图像、边界线和关键数值
 *
 * 本函数在主循环中调用，负责将视觉算法的处理结果实时显示在 TFT180 屏幕上。
 * 屏幕布局分为三个区域：
 *   1. 左上区域（x=0~107, y=0~79）：灰度图像 + 叠加的边界线、中线、拐点标记
 *   2. 右侧区域（x=112~159, y=0~79）：电机/路口/耗时关键值
 *   3. 底部区域（y=84~115）：误差、速度规划、阈值等辅助调试值
 *
 * 图像颜色编码：
 *   - 白色像素 = 赛道线（二值化后为 255）
 *   - 黑色像素 = 背景（二值化后为 0）
 *   - 蓝色线 = 左边界
 *   - 绿色线 = 右边界
 *   - 红色线 = 中线（中心线）
 *   - 黄色 X = 左拐点
 *   - 洋红色 X = 右拐点
 *   - 青色框 = 路口检测区域
 *
 * 必须在 track_fusion_update() 之后调用，以确保数据是最新的。
 */
void draw_line(void)
{
    static uint8 s_first_clear = 1u;

    if (s_first_clear != 0u)
    {
        tft180_full(RGB565_WHITE);  /* 只在首次进入时清屏，避免每帧整屏刷白造成闪烁 */
        s_first_clear = 0u;
    }

    /* ============================================================
     * 第一步：显示压缩后的灰度图像
     * 这里不用二值图做底图，避免 Otsu 阈值和降噪导致黑白面积整块跳变，
     * TFT 上看起来会比二值图更接近连续视频。
     * ============================================================ */
    tft180_show_gray_image(0, 0,                    /* 图像在TFT屏幕左上角(0,0)开始显示 */
        Image_Binarize[0],                           /* 二值化图像数据指针（一维数组形式访问二维数组首地址） */
        COMP_W, COMP_H,                              /* 原始压缩图像尺寸：宽94 x 高60 */
        TFT_IMG_DISP_W, TFT_IMG_DISP_H,             /* TFT 显示目标尺寸：宽108 x 高80 */
        128);                                        /* 阈值=128：按黑白二值显示 */

    /* 丢线时在左上角显示 "LOST" 提示，但仍叠加拐点与框供调试观察 */
    if (g_tf.line_lost)  /* 视觉模块判定赛道线丢失 */
    {
        tft180_show_string(0, 0, "LOST");  /* 在图像左上角显示"LOST"提示文字 */
    }

    /* ============================================================
     * 第二步：在图像上画边界线和中线
     * 从底部行（TF_JIDIAN_ROW）向上扫描到顶部（TF_SEARCH_END_ROW），
     * 将每一行的有效边界点和中线点连接成连续线段。
     * ============================================================ */
    if (!g_tf.line_lost)  /* 仅在赛道线未丢失时绘制边界和中线 */
    {
        uint8 have_prev = 0u;       /* 是否有上一个有效行的数据（用于连线），0=无，1=有 */
        uint8 prev_left = 0u;       /* 上一行左边界在TFT屏幕上的x坐标 */
        uint8 prev_right = 0u;      /* 上一行右边界在TFT屏幕上的x坐标 */
        uint8 prev_mid = 0u;        /* 上一行中线在TFT屏幕上的x坐标 */
        uint8 prev_row = 0u;        /* 上一行在TFT屏幕上的y坐标 */
        uint8 prev_src_row = 0u;    /* 上一行在原始图像中的行号（用于判断行间距是否过大） */

        /* 遍历从基点行到搜索结束行的每一行（从下往上扫描，行号递减） */
        for (uint8 i = TF_JIDIAN_ROW; i > TF_SEARCH_END_ROW; i--)  /* 从底部基点行向上遍历到顶部 */
        {
            /* 跳过无效行（该行未检测到有效边界数据） */
            if (!g_tf.row_valid[i])  /* 第i行的边界数据无效 */
                continue;            /* 跳过该行，继续处理下一行 */

            /* 将当前行的边界和中线坐标从图像坐标映射到 TFT 屏幕坐标 */
            uint8 tft_left  = tft_map_col(g_tf.left_edge[i]);    /* 左边界：图像列号 -> TFT屏幕x坐标 */
            uint8 tft_right = tft_map_col(g_tf.right_edge[i]);   /* 右边界：图像列号 -> TFT屏幕x坐标 */
            uint8 tft_mid   = tft_map_col(g_tf.center_line[i]);  /* 中线：图像列号 -> TFT屏幕x坐标 */
            uint8 tft_row   = tft_map_row((int16)i);             /* 当前行：图像行号 -> TFT屏幕y坐标 */

            /* 如果有上一行数据，且行间距不超过最大允许跳行数，则画连线连接相邻行 */
            if (have_prev && (uint8)(prev_src_row - i) <= (uint8)TF_MAX_MISS_ROWS)  /* 行间距在允许范围内 */
            {
                /* 用线段连接相邻行的左边界（蓝色）、右边界（绿色）、中线（红色） */
                tft_draw_line(prev_left,  prev_row, tft_left,  tft_row, RGB565_BLUE);   /* 左边界蓝色线段 */
                tft_draw_line(prev_right, prev_row, tft_right, tft_row, RGB565_GREEN);  /* 右边界绿色线段 */
                tft_draw_line(prev_mid,   prev_row, tft_mid,   tft_row, RGB565_RED);    /* 中线红色线段 */
            }
            else
            {
                /* 行间距过大或首行：只画单点，不连线（避免跳行时画出跨越空白的长线） */
                tft180_draw_point(tft_left,  tft_row, RGB565_BLUE);   /* 左边界蓝色单点 */
                tft180_draw_point(tft_right, tft_row, RGB565_GREEN);  /* 右边界绿色单点 */
                tft180_draw_point(tft_mid,   tft_row, RGB565_RED);    /* 中线红色单点 */
            }

            /* 保存当前行数据，供下一次迭代时作为"上一行"用于连线 */
            have_prev = 1u;        /* 标记已有上一行数据 */
            prev_left = tft_left;  /* 保存左边界x坐标 */
            prev_right = tft_right;/* 保存右边界x坐标 */
            prev_mid = tft_mid;    /* 保存中线x坐标 */
            prev_row = tft_row;    /* 保存当前行y坐标 */
            prev_src_row = i;      /* 保存原始图像行号 */
        }
    }

    /* 拐点标记与路口检测框叠加层 */
    /* 黄色 X = 左拐点，洋红色 X = 右拐点，青色矩形 = 路口检测框 */
    draw_inflection_overlay();  /* 绘制拐点标记和路口检测框叠加层 */

    /* ============================================================
     * 第三步：在屏幕下方和右侧显示关键调试数值
     *
     * 底部区域（y=84~115）：
     *   ERR  = 转向误差（g_tf.error），正值偏右，负值偏左
     *   ROW  = 有效行数（g_tf.valid_row_count），越多说明看到的赛道越远
     *   THR  = 二值化阈值（Image_Threshold），由 Otsu 算法自适应计算
     *
     * 右侧区域（x=112~159）：
     *   左列速度 = receive_left_speed_data（左侧电机实际速度反馈）
     *   右列速度 = receive_right_speed_data（右侧电机实际速度反馈）
     *   RA = g_ra_flag（路口/直角标志，0=无，1~5=不同类型）
     *   PR = 预减速标志（g_ra_pre_flag 或 g_ra_pre_slow_flag）
     *   ST = ra_dbg_state（RA 状态机当前状态，用于调试）
     *   BS = base_speed（基础速度，速度规划的输出）
     *   PH = ra_dbg_phase（RA 阶段：WAIT/SLOW/APPROACH/HARD/RECOVER）
     *   SO = speed_dbg_out（速度 PI 输出值）
     *   RW = speed_dbg_raw（速度 PI 原始误差）
     *   TG = speed_dbg_plan（速度规划目标值）
     *   RS = speed_dbg_reason（速度降低原因代码）
     *   TF = track_fusion_update() 耗时，单位us
     *   IN = 预检测+路口检测耗时，单位us
     * ============================================================ */

    /* 右侧状态区：160x128横屏下，x=124显示5位数刚好不越界 */
    tft180_show_string(112,  0, "L");                                  /* 左轮速度 */
    tft180_show_int(124, 0, motor_value.receive_left_speed_data, 4);
    tft180_show_string(112,  8, "R");                                  /* 右轮速度 */
    tft180_show_int(124, 8, motor_value.receive_right_speed_data, 4);
    tft180_show_string(112, 16, "RA");                                 /* 路口/直角标志 */
    tft180_show_int(124, 16, (int32)g_ra_flag, 2);
    tft180_show_string(112, 24, "PR");                                 /* 预检测标志 */
    tft180_show_int(124, 24,
                    (int32)((g_ra_pre_flag || g_ra_pre_slow_flag) ? 1u : 0u),
                    2);
    tft180_show_string(112, 32, "IP");                                 /* 最大拐点行 */
    tft180_show_int(124, 32, (int32)g_ip_max_row, 3);
    tft180_show_string(112, 40, "TF");                                 /* track_fusion耗时us */
    tft180_show_int(124, 40, (int32)((prof_tf_us > 99999u) ? 99999u : prof_tf_us), 5);
    tft180_show_string(112, 48, "IN");                                 /* 预检测+路口检测耗时us */
    tft180_show_int(124, 48, (int32)((prof_inter_us > 99999u) ? 99999u : prof_inter_us), 5);
    tft180_show_string(112, 56, "YA");                                 /* 航向角（度） */
    tft180_show_int(124, 56, (int32)yaw_angle, 4);
    tft180_show_string(112, 64, "B");                                  /* 电池电压x10 */
    tft180_show_int(124, 64, (int32)battery_get_voltage_x10(), 3);
    tft180_show_string(112, 72, "AD");
    tft180_show_int(124, 72, (int32)battery_get_adc(), 4);

    /* 底部调试区：三列固定布局，每行8像素，避免重叠 */
    tft180_show_string(0,   84, "ER");                                 /* 转向误差 */
    tft180_show_int(18, 84, (int32)g_tf.error, 4);
    tft180_show_string(54,  84, "RO");                                 /* 有效行数 */
    tft180_show_int(72, 84, (int32)g_tf.valid_row_count, 3);
    tft180_show_string(108, 84, "TH");                                 /* 二值化阈值 */
    tft180_show_int(126, 84, (int32)Image_Threshold, 3);

    tft180_show_string(0,   92, "RS");                                 /* 速度原因 */
    tft180_show_int(18, 92, (int32)speed_dbg_reason, 2);
    tft180_show_string(54,  92, "DT");                                 /* 框分类 */
    tft180_show_int(72, 92, (int32)g_inter_result.detected_type, 2);
    tft180_show_string(102, 92, " ");
    tft180_show_string(108, 92, "RC");                                 /* 比赛状态 */
    tft180_show_int(126, 92, (int32)race_state, 1);

    tft180_show_string(0,   100, "WC");                                /* 白点计数 */
    tft180_show_int(18, 100, (int32)g_tf_white_count, 4);
    tft180_show_string(54, 100, "K");                                  /* 原始按键/拨码状态 */
    tft180_show_int(66, 100, (int32)ui_raw_input_state(), 2);
    tft180_show_string(102, 100, (menu_cursor == 0u) ? ">" : " ");
    tft180_show_string(108, 100, "E");                                 /* 单边巡线 */
    tft180_show_int(126, 100, (int32)g_post_edge_side, 1);

    tft180_show_string(102, 108, (menu_cursor == 1u) ? ">" : " ");
    tft180_show_string(108, 108, "S");                                 /* 目标速度 */
    tft180_show_int(126, 108, (int32)motor_speed, 3);
}
