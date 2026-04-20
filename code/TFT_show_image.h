#ifndef _TFT_SHOW_IMAGE_H_
#define _TFT_SHOW_IMAGE_H_

#include "zf_common_headfile.h"
#include "Image.h"

void draw_line1(void);
void draw_line2(void);

typedef enum
{
    DISP_GRAY = 0,
    DISP_BINARY = 1,
} DispMode_t;

#define TFT_REFRESH_DIV 4

void track_display_init(void);
void track_display_update(DispMode_t mode);

#endif