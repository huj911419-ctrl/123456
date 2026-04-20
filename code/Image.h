#ifndef CODE_IMAGE_H_
#define CODE_IMAGE_H_

#include "zf_common_headfile.h"

#define MT9V03X_W (188)
#define MT9V03X_H (120)

#define TF_IMG_H MT9V03X_H
#define TF_IMG_W MT9V03X_W
#define TF_IMG_CENTER (TF_IMG_W / 2)

#define TF_OTSU_INTERVAL 5
#define TF_THRESHOLD_BIAS (-10)
#define TF_JIDIAN_ROW (TF_IMG_H - 4)
#define TF_SEARCH_END_ROW 8
#define TF_LOCAL_RANGE 18
#define TF_MAX_MISS_ROWS 5
#define TF_MIN_TRACK_WIDTH 25
#define TF_MAX_TRACK_WIDTH 160
#define TF_INVALID (-1)

typedef struct
{
    int16 left_edge[TF_IMG_H];
    int16 right_edge[TF_IMG_H];
    int16 center_line[TF_IMG_H];
    uint8 row_valid[TF_IMG_H];
    int16 error;
    uint16 valid_row_count;
    uint8 line_lost;
    uint8 left_jidian;
    uint8 right_jidian;
    uint8 threshold;
} TrackFusion_t;

extern uint8 mt9v03x_image[MT9V03X_H][MT9V03X_W];
extern TrackFusion_t g_tf;
extern uint8 bin_image[TF_IMG_H][TF_IMG_W];

void track_fusion_init(void);
void track_fusion_update(void);

#endif