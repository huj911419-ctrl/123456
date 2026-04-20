#include "TFT_show_image.h"

extern uint8 mt9v03x_image[MT9V03X_H][MT9V03X_W];

static uint8 s_frame_cnt = 0;

void track_display_init(void)
{
    s_frame_cnt = 0;
}

void track_display_update(DispMode_t mode)
{
    s_frame_cnt++;
    if(s_frame_cnt < TFT_REFRESH_DIV)
    {
        return;
    }
    s_frame_cnt = 0;

    if(DISP_BINARY == mode)
    {
        tft180_show_gray_image(0, 0, bin_image[0], MT9V03X_W, MT9V03X_H, MT9V03X_W / 2, MT9V03X_H / 2, 1);
    }
    else
    {
        tft180_show_gray_image(0, 0, mt9v03x_image[0], MT9V03X_W, MT9V03X_H, MT9V03X_W / 2, MT9V03X_H / 2, 0);
    }

    if(g_tf.line_lost)
    {
        tft180_show_string(0, 0, "LOST");
    }
    else
    {
        for(uint8 i = TF_JIDIAN_ROW; i > TF_SEARCH_END_ROW; i--)
        {
            if(!g_tf.row_valid[i])
                continue;

            int16 tft_left = (int16)(0.5f * (float)g_tf.left_edge[i]);
            int16 tft_right = (int16)(0.5f * (float)g_tf.right_edge[i]);
            int16 tft_mid = (int16)(0.5f * (float)g_tf.center_line[i]);
            uint8 tft_y = i / 2;

            tft180_draw_point(tft_left, tft_y, RGB565_BLUE);
            tft180_draw_point(tft_right, tft_y, RGB565_GREEN);
            tft180_draw_point(tft_mid, tft_y, RGB565_RED);
        }
    }

    tft180_show_string(60, 0, "E:");
    tft180_show_int(76, 0, (int32)g_tf.error, 4);
    tft180_show_string(60, 16, "R:");
    tft180_show_int(76, 16, (int32)g_tf.valid_row_count, 3);
    tft180_show_string(60, 32, "T:");
    tft180_show_int(76, 32, (int32)g_tf.threshold, 3);
}