#include "Image.h"

extern uint8 mt9v03x_image[MT9V03X_H][MT9V03X_W];

TrackFusion_t g_tf;
uint8 bin_image[TF_IMG_H][TF_IMG_W];

static uint8 s_otsu_cnt = 0;
static uint16 s_hist[256];
static float s_P[256];
static float s_PK[256];
static float s_Mk[256];

static uint8 calc_otsu(void)
{
    float imgsize = (float)((uint32)TF_IMG_H * TF_IMG_W);

    for (uint16 i = 0; i < 256; i++)
        s_hist[i] = 0;

    for (uint8 i = 0; i < TF_IMG_H; i++)
        for (uint8 j = 0; j < TF_IMG_W; j++)
            s_hist[mt9v03x_image[i][j]]++;

    for (uint16 i = 0; i < 256; i++)
        s_P[i] = s_hist[i] / imgsize;

    s_PK[0] = s_P[0];
    s_Mk[0] = 0.0f;
    for (uint16 i = 1; i < 256; i++)
    {
        s_PK[i] = s_PK[i - 1] + s_P[i];
        s_Mk[i] = s_Mk[i - 1] + (float)i * s_P[i];
    }

    float max_sigma = 0.0f;
    uint8 best = 128u;
    float Mg = s_Mk[255];

    for (uint16 i = 1; i < 255; i++)
    {
        float pk = s_PK[i];
        if (pk < 1e-6f || pk > 1.0f - 1e-6f)
            continue;
        float mk = s_Mk[i];
        float sigma = (mk - Mg * pk) * (mk - Mg * pk) / (pk * (1.0f - pk));
        if (sigma > max_sigma)
        {
            max_sigma = sigma;
            best = (uint8)i;
        }
    }
    return best;
}

static void binarize_image(void)
{
    for (uint16 i = 0; i < TF_IMG_H; i++)
        for (uint16 j = 0; j < TF_IMG_W; j++)
            bin_image[i][j] = (mt9v03x_image[i][j] < g_tf.threshold) ? 255u : 0u;
}

static inline uint8 is_white(uint8 row, int16 col)
{
    return (bin_image[row][(uint8)col] == 255u) ? 1u : 0u;
}

static inline uint8 is_left_edge(uint8 row, int16 col)
{
    if (col < 1 || col >= TF_IMG_W - 1)
        return 0u;
    return (!is_white(row, col - 1) && is_white(row, col) && is_white(row, col + 1)) ? 1u : 0u;
}

static inline uint8 is_right_edge(uint8 row, int16 col)
{
    if (col < 1 || col >= TF_IMG_W - 1)
        return 0u;
    return (is_white(row, col - 1) && is_white(row, col) && !is_white(row, col + 1)) ? 1u : 0u;
}

static int16 scan_left_edge_right(uint8 row, int16 start, int16 end)
{
    if (start < 1)
        start = 1;
    if (end >= TF_IMG_W - 1)
        end = TF_IMG_W - 2;
    if (start > end)
        return TF_INVALID;
    for (int16 c = start; c <= end; c++)
        if (is_left_edge(row, c))
            return c;
    return TF_INVALID;
}

static int16 scan_left_edge_left(uint8 row, int16 start, int16 end)
{
    if (start >= TF_IMG_W - 1)
        start = TF_IMG_W - 2;
    if (end < 1)
        end = 1;
    if (start < end)
        return TF_INVALID;
    for (int16 c = start; c >= end; c--)
        if (is_left_edge(row, c))
            return c;
    return TF_INVALID;
}

static int16 scan_right_edge_left(uint8 row, int16 start, int16 end)
{
    if (start >= TF_IMG_W - 1)
        start = TF_IMG_W - 2;
    if (end < 1)
        end = 1;
    if (start < end)
        return TF_INVALID;
    for (int16 c = start; c >= end; c--)
        if (is_right_edge(row, c))
            return c;
    return TF_INVALID;
}

static int16 scan_right_edge_right(uint8 row, int16 start, int16 end)
{
    if (start < 1)
        start = 1;
    if (end >= TF_IMG_W - 1)
        end = TF_IMG_W - 2;
    if (start > end)
        return TF_INVALID;
    for (int16 c = start; c <= end; c++)
        if (is_right_edge(row, c))
            return c;
    return TF_INVALID;
}

static inline uint8 valid_pair(int16 lb, int16 rb)
{
    if (lb == TF_INVALID || rb == TF_INVALID)
        return 0u;
    int16 w = rb - lb;
    return (w >= TF_MIN_TRACK_WIDTH && w <= TF_MAX_TRACK_WIDTH) ? 1u : 0u;
}

static uint8 find_jidian1(void)
{
    const uint8 row = TF_JIDIAN_ROW;
    const int16 mid = TF_IMG_CENTER;
    const int16 qtr = TF_IMG_W / 4;
    const int16 tqtr = TF_IMG_W * 3 / 4;
    int16 lb = TF_INVALID, rb = TF_INVALID;

    if (is_white(row, mid - 1) && is_white(row, mid) && is_white(row, mid + 1))
    {
        lb = scan_left_edge_left(row, mid, 1);
        rb = scan_right_edge_right(row, mid, TF_IMG_W - 2);
    }
    else if (is_white(row, qtr - 1) && is_white(row, qtr) && is_white(row, qtr + 1))
    {
        lb = scan_left_edge_left(row, qtr, 1);
        rb = scan_right_edge_right(row, qtr, TF_IMG_W - 2);
    }
    else if (is_white(row, tqtr - 1) && is_white(row, tqtr) && is_white(row, tqtr + 1))
    {
        lb = scan_left_edge_left(row, tqtr, 1);
        rb = scan_right_edge_right(row, tqtr, TF_IMG_W - 2);
    }
    else
    {
        lb = scan_left_edge_right(row, 1, TF_IMG_W - 2);
        if (lb != TF_INVALID)
            rb = scan_right_edge_left(row, TF_IMG_W - 2, lb + TF_MIN_TRACK_WIDTH);
    }

    if (!valid_pair(lb, rb))
        return 0u;

    g_tf.left_jidian = (uint8)lb;
    g_tf.right_jidian = (uint8)rb;
    return 1u;
}

static uint8 search_row_edges(uint8 row, int16 prev_lb, int16 prev_rb,
                              int16 *out_lb, int16 *out_rb)
{
    int16 lb = TF_INVALID, rb = TF_INVALID;
    const int16 R = TF_LOCAL_RANGE;
    const int16 MID = TF_IMG_CENTER;

    lb = scan_left_edge_right(row, prev_lb - R, prev_lb + R);
    if (lb == TF_INVALID)
        lb = scan_left_edge_left(row, prev_lb + R, prev_lb - R);
    if (lb == TF_INVALID)
        lb = scan_left_edge_left(row, MID, 1);

    rb = scan_right_edge_left(row, prev_rb + R, prev_rb - R);
    if (rb == TF_INVALID)
        rb = scan_right_edge_right(row, prev_rb - R, prev_rb + R);
    if (rb == TF_INVALID)
        rb = scan_right_edge_right(row, MID, TF_IMG_W - 2);

    *out_lb = lb;
    *out_rb = rb;
    return valid_pair(lb, rb);
}

void track_fusion_init(void)
{
    for (uint8 i = 0; i < TF_IMG_H; i++)
    {
        g_tf.left_edge[i] = TF_INVALID;
        g_tf.right_edge[i] = TF_INVALID;
        g_tf.center_line[i] = TF_INVALID;
        g_tf.row_valid[i] = 0u;
    }
    g_tf.error = 0;
    g_tf.valid_row_count = 0u;
    g_tf.line_lost = 0u;
    g_tf.left_jidian = (uint8)TF_IMG_CENTER;
    g_tf.right_jidian = (uint8)TF_IMG_CENTER;
    g_tf.threshold = 128u;
    s_otsu_cnt = (uint8)TF_OTSU_INTERVAL;

    for (uint16 i = 0; i < TF_IMG_H; i++)
        for (uint16 j = 0; j < TF_IMG_W; j++)
            bin_image[i][j] = 0u;
}

void track_fusion_update(void)
{
    for (uint8 i = 0; i < TF_IMG_H; i++)
    {
        g_tf.left_edge[i] = TF_INVALID;
        g_tf.right_edge[i] = TF_INVALID;
        g_tf.center_line[i] = TF_INVALID;
        g_tf.row_valid[i] = 0u;
    }
    g_tf.error = 0;
    g_tf.valid_row_count = 0u;
    g_tf.line_lost = 0u;

    s_otsu_cnt++;
    if (s_otsu_cnt >= TF_OTSU_INTERVAL)
    {
        s_otsu_cnt = 0u;
        int16 raw = (int16)calc_otsu() + TF_THRESHOLD_BIAS;
        if (raw < 20)
            raw = 20;
        if (raw > 240)
            raw = 240;
        g_tf.threshold = (uint8)raw;
    }

    binarize_image();

    if (!find_jidian1())
    {
        g_tf.line_lost = 1u;
        return;
    }

    const uint8 jrow = TF_JIDIAN_ROW;
    int16 prev_lb = (int16)g_tf.left_jidian;
    int16 prev_rb = (int16)g_tf.right_jidian;

    g_tf.left_edge[jrow] = prev_lb;
    g_tf.right_edge[jrow] = prev_rb;
    g_tf.center_line[jrow] = (prev_lb + prev_rb) / 2;
    g_tf.row_valid[jrow] = 1u;
    g_tf.valid_row_count = 1u;

    uint8 miss_streak = 0u;
    int16 end_row = (int16)TF_SEARCH_END_ROW;

    for (int16 row = (int16)jrow - 1; row >= end_row; row--)
    {
        int16 lb, rb;
        if (search_row_edges((uint8)row, prev_lb, prev_rb, &lb, &rb))
        {
            g_tf.left_edge[row] = lb;
            g_tf.right_edge[row] = rb;
            g_tf.center_line[row] = (lb + rb) / 2;
            g_tf.row_valid[row] = 1u;
            g_tf.valid_row_count++;
            prev_lb = lb;
            prev_rb = rb;
            miss_streak = 0u;
        }
        else
        {
            g_tf.row_valid[row] = 0u;
            if (++miss_streak >= TF_MAX_MISS_ROWS)
                break;
        }
    }

    if (g_tf.valid_row_count == 0u)
    {
        g_tf.line_lost = 1u;
        return;
    }

    int32 weighted_sum = 0, weight_total = 0;
    for (int16 row = (int16)jrow; row >= end_row; row--)
    {
        if (g_tf.row_valid[row])
        {
            int32 w = (int32)(row - end_row + 1);
            weighted_sum += (int32)g_tf.center_line[row] * w;
            weight_total += w;
        }
    }

    int16 avg_center = (weight_total > 0)
                           ? (int16)(weighted_sum / weight_total)
                           : (int16)TF_IMG_CENTER;

    g_tf.error = avg_center - (int16)TF_IMG_CENTER;
    g_tf.line_lost = 0u;
}