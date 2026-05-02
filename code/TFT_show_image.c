#include "TFT_show_image.h"


/* ================================================================
 *  显示逻辑：
 *    第一步：先把 bin_image 显示到TFT（黑白二值化底图）
 *    第二步：在底图上叠加边界线和中线
 *    第三步：左上角显示调试数值
 *
 *  效果：你能直接看到算法"眼中"的赛道，白色=赛道线，黑色=背景
 *        蓝色点=左边界  绿色点=右边界  红色点=中线
 * ================================================================ */

#define TFT_COL_SCALE   0.68f   // 列缩放：188→128

void draw_line(void)
{
    /* 第一步：显示二值化底图
     * bin_image[i][j]=255 显示为白（赛道线）
     * bin_image[i][j]=0   显示为黑（背景）
     * 阈值传1，让255显示白、0显示黑 */
    tft180_show_gray_image(0, 0,
        bin_image[0],
        MT9V03X_W, MT9V03X_H,
        128, 81,
        1);

    /* 丢线时显示LOST */
    if (g_tf.line_lost)
    {
        tft180_show_string(0, 0, "LOST");
        return;
    }

    /* 第二步：在底图上叠加边界线和中线 */
     for (uint8 i = TF_JIDIAN_ROW; i > TF_SEARCH_END_ROW; i--)
    {
        if (!g_tf.row_valid[i]) continue;

        uint8 tft_left  = (uint8)(TFT_COL_SCALE * (float)Limit_int16(1, g_tf.left_edge[i],   MT9V03X_W - 2));
        uint8 tft_right = (uint8)(TFT_COL_SCALE * (float)Limit_int16(1, g_tf.right_edge[i],  MT9V03X_W - 2));
        uint8 tft_mid   = (uint8)(TFT_COL_SCALE * (float)Limit_int16(1, g_tf.center_line[i], MT9V03X_W - 2));
        uint8 tft_row   = (uint8)(0.68f * (float)i);

        tft180_draw_point(tft_left,  tft_row, RGB565_BLUE);
        tft180_draw_point(tft_right, tft_row, RGB565_GREEN);
        tft180_draw_point(tft_mid,   tft_row, RGB565_RED);
    }

    /* 第三步：调试数值（显示在底图下方，不遮挡图像）*/
     tft180_show_string(0,  82, "ERR:");
    tft180_show_int   (36, 82, (int32)g_tf.error,          4);// 位置误差  (负=偏左 正=偏右)
    tft180_show_string(0,  98, "ROW:");
    tft180_show_int   (36, 98, (int32)g_tf.valid_row_count, 3);// 有效行数
    tft180_show_string(0, 114, "THR:");
    tft180_show_int(36, 114, (int32)g_tf.threshold, 3); // 当前二值化阈值
    tft180_show_int(130,0, motor_value.receive_left_speed_data,3);
    tft180_show_int(130,10,motor_value.receive_right_speed_data,3);
    tft180_show_int(130, 20, g_ra_flag, 3);
}
