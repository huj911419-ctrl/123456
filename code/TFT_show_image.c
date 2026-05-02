#include "TFT_show_image.h"
#include "IMU.h"


/* ================================================================
 *  ��ʾ�߼���
 *    ��һ�����Ȱ� bin_image ��ʾ��TFT���ڰ׶�ֵ����ͼ��
 *    �ڶ������ڵ�ͼ�ϵ��ӱ߽��ߺ�����
 *    �����������Ͻ���ʾ������ֵ
 *
 *  Ч��������ֱ�ӿ����㷨"����"����������ɫ=�����ߣ���ɫ=����
 *        ��ɫ��=��߽�  ��ɫ��=�ұ߽�  ��ɫ��=����
 * ================================================================ */

#define TFT_COL_SCALE   0.68f   // �����ţ�188��128

void draw_line(void)
{
    /* ��һ������ʾ��ֵ����ͼ
     * bin_image[i][j]=255 ��ʾΪ�ף������ߣ�
     * bin_image[i][j]=0   ��ʾΪ�ڣ�������
     * ��ֵ��1����255��ʾ�ס�0��ʾ�� */
    tft180_show_gray_image(0, 0,
        bin_image[0],
        MT9V03X_W, MT9V03X_H,
        128, 81,
        1);

    /* ����ʱ��ʾLOST */
    if (g_tf.line_lost)
    {
        tft180_show_string(0, 0, "LOST");
        return;
    }

    /* �ڶ������ڵ�ͼ�ϵ��ӱ߽��ߺ����� */
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

    /* ��������������ֵ����ʾ�ڵ�ͼ�·������ڵ�ͼ��*/
     tft180_show_string(0,  82, "ERR:");
    tft180_show_int   (36, 82, (int32)g_tf.error,          4);// λ�����  (��=ƫ�� ��=ƫ��)
    tft180_show_string(0,  98, "ROW:");
    tft180_show_int   (36, 98, (int32)g_tf.valid_row_count, 3);// ��Ч����
    tft180_show_string(0, 114, "THR:");
    tft180_show_int(36, 114, (int32)g_tf.threshold, 3); // ��ǰ��ֵ����ֵ
    tft180_show_int(130,0, motor_value.receive_left_speed_data,3);
    tft180_show_int(130,10,motor_value.receive_right_speed_data,3);
    tft180_show_int(130, 20, g_ra_flag, 3);
    // Yaw 信息（显示为 yaw*10 的整数，方便看一位小数）
    tft180_show_int(130, 30, (int32)(yaw_angle * 10), 4);   // yaw角
    tft180_show_int(130, 40, (int32)yaw_rate, 4);           // yaw角速度
}
