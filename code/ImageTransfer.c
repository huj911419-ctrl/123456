/**
 * ImageTransfer.c - 通过UART0发送二值化图像、边线坐标、TFT参数到电脑
 *
 * ==================== 数据包格式说明 ====================
 *
 * 每帧发送三个包，通过类型字节区分：
 *
 * 包1 - 图像数据包（类型 0x00）：
 *   [0xAA] [0x55] [0x00] [长度H] [长度L] [705字节压缩图像]
 *
 * 包2 - 边线坐标数据包（类型 0x01）：
 *   [0xAA] [0x55] [0x01] [长度H] [长度L] [360字节边线坐标]
 *   每行6字节：[左边线H] [左边线L] [右边线H] [右边线L] [中线H] [中线L]
 *
 * 包3 - TFT参数数据包（类型 0x02）：
 *   [0xAA] [0x55] [0x02] [长度H] [长度L] [41字节参数]
 *   字节0-1:  error (int16, 转向误差)
 *   字节2-3:  valid_rows (uint16, 有效行数)
 *   字节4-5:  threshold (uint16, 二值化阈值)
 *   字节6:    motor_enable (uint8, 电机使能)
 *   字节7-8:  white_count (uint16, 白点数量)
 *   字节9:    detected_type (uint8, 框分类结果)
 *   字节10-11: left_speed (int16, 左电机速度)
 *   字节12-13: right_speed (int16, 右电机速度)
 *   字节14:   ra_flag (uint8, RA标志)
 *   字节15:   ra_pre_flag (uint8, 预减速标志)
 *   字节16:   ra_state (uint8, RA状态)
 *   字节17-18: base_speed (int16, 基础速度)
 *   字节19:   ra_phase (uint8, RA阶段)
 *   字节20-21: speed_out (int16, 速度PI输出)
 *   字节22-23: speed_raw (int16, 速度原始值)
 *   字节24-25: speed_plan (int16, 速度规划值)
 *   字节26:   speed_reason (uint8, 速度原因)
 *   字节27-28: yaw_angle (int16, yaw角度×10)
 *   字节29:   ip_max_row (uint8, 最大拐点行)
 *   字节30-33: prof_tf (uint32, 视觉处理耗时us)
 *   字节34-37: prof_in (uint32, 路口检测耗时us)
 *   字节38-39: ra_dbg_yaw10 (int16, yaw进度×10)
 *   字节40:   route_dbg_step (uint8, 路线步骤)
 *
 * ==========================================================
 */

#include "zf_common_headfile.h"
#include "Track_funsion.h"
#include "Pid.h"
#include "IMU.h"

/* 外部变量声明 */
extern volatile uint32 prof_tf_us;
extern volatile uint32 prof_inter_us;

/* ==================== 帧头和类型定义 ==================== */
#define FRAME_HEADER_1    0xAA
#define FRAME_HEADER_2    0x55
#define FRAME_TYPE_IMAGE  0x00
#define FRAME_TYPE_EDGES  0x01
#define FRAME_TYPE_PARAMS 0x02

/* ==================== 图像尺寸定义 ==================== */
#define IMG_W   94
#define IMG_H   60

/* ==================== 数据长度定义 ==================== */
#define PACKED_SIZE    ((IMG_W * IMG_H + 7) / 8)
#define EDGE_DATA_SIZE (IMG_H * 6)
#define PARAM_DATA_SIZE 41

/* ==================== 无效边线标记 ==================== */
#define EDGE_INVALID   0xFFFF

/**
 * send_uart_packet - 发送一个数据包
 */
static void send_uart_packet(uint8 type, const uint8 *data, uint16 len)
{
    uint8 header[5];
    header[0] = FRAME_HEADER_1;
    header[1] = FRAME_HEADER_2;
    header[2] = type;
    header[3] = (uint8)((len >> 8) & 0xFF);
    header[4] = (uint8)(len & 0xFF);
    uart_write_buffer(UART_0, header, 5);
    uart_write_buffer(UART_0, data, len);
}

/**
 * send_image_uart0 - 发送压缩图像 + 边线坐标 + TFT参数到电脑
 */
void send_image_uart0(void)
{
    uint8 packed[PACKED_SIZE];
    uint8 edge_data[EDGE_DATA_SIZE];
    uint8 param_data[PARAM_DATA_SIZE];
    uint16 idx = 0;
    uint8 bit_cnt = 0;
    uint8 byte_val = 0;

    /* ===== 包1：压缩图像 ===== */
    for (uint8 row = 0; row < IMG_H; row++)
    {
        for (uint8 col = 0; col < IMG_W; col++)
        {
            if (Image_Binarize[row][col] > 0)
                byte_val |= (1 << (7 - bit_cnt));
            bit_cnt++;
            if (bit_cnt == 8)
            {
                packed[idx++] = byte_val;
                byte_val = 0;
                bit_cnt = 0;
            }
        }
    }
    if (bit_cnt > 0)
        packed[idx++] = byte_val;

    send_uart_packet(FRAME_TYPE_IMAGE, packed, PACKED_SIZE);

    /* ===== 包2：边线坐标 ===== */
    idx = 0;
    for (uint8 row = 0; row < IMG_H; row++)
    {
        int16 lb = (g_tf.row_valid[row]) ? g_tf.left_edge[row]   : (int16)EDGE_INVALID;
        int16 rb = (g_tf.row_valid[row]) ? g_tf.right_edge[row]  : (int16)EDGE_INVALID;
        int16 cc = (g_tf.row_valid[row]) ? g_tf.center_line[row] : (int16)EDGE_INVALID;

        edge_data[idx++] = (uint8)((lb >> 8) & 0xFF);
        edge_data[idx++] = (uint8)(lb & 0xFF);
        edge_data[idx++] = (uint8)((rb >> 8) & 0xFF);
        edge_data[idx++] = (uint8)(rb & 0xFF);
        edge_data[idx++] = (uint8)((cc >> 8) & 0xFF);
        edge_data[idx++] = (uint8)(cc & 0xFF);
    }

    send_uart_packet(FRAME_TYPE_EDGES, edge_data, EDGE_DATA_SIZE);

    /* ===== 包3：TFT参数 ===== */
    idx = 0;

    /* error (int16) */
    param_data[idx++] = (uint8)((g_tf.error >> 8) & 0xFF);
    param_data[idx++] = (uint8)(g_tf.error & 0xFF);
    /* valid_rows (uint16) */
    param_data[idx++] = (uint8)((g_tf.valid_row_count >> 8) & 0xFF);
    param_data[idx++] = (uint8)(g_tf.valid_row_count & 0xFF);
    /* threshold (uint16) */
    param_data[idx++] = (uint8)((Image_Threshold >> 8) & 0xFF);
    param_data[idx++] = (uint8)(Image_Threshold & 0xFF);
    /* motor_enable (uint8) */
    param_data[idx++] = (uint8)motor_enable;
    /* white_count (uint16) */
    param_data[idx++] = (uint8)((g_tf_white_count >> 8) & 0xFF);
    param_data[idx++] = (uint8)(g_tf_white_count & 0xFF);
    /* detected_type (uint8) */
    param_data[idx++] = g_inter_result.detected_type;
    /* left_speed (int16) */
    param_data[idx++] = (uint8)((motor_value.receive_left_speed_data >> 8) & 0xFF);
    param_data[idx++] = (uint8)(motor_value.receive_left_speed_data & 0xFF);
    /* right_speed (int16) */
    param_data[idx++] = (uint8)((motor_value.receive_right_speed_data >> 8) & 0xFF);
    param_data[idx++] = (uint8)(motor_value.receive_right_speed_data & 0xFF);
    /* ra_flag (uint8) */
    param_data[idx++] = g_ra_flag;
    /* ra_pre_flag (uint8) */
    param_data[idx++] = (uint8)((g_ra_pre_flag || g_ra_pre_slow_flag) ? 1u : 0u);
    /* ra_state (uint8) */
    param_data[idx++] = ra_dbg_state;
    /* base_speed (int16) */
    param_data[idx++] = (uint8)((base_speed >> 8) & 0xFF);
    param_data[idx++] = (uint8)(base_speed & 0xFF);
    /* ra_phase (uint8) */
    param_data[idx++] = ra_dbg_phase;
    /* speed_out (int16) */
    param_data[idx++] = (uint8)((speed_dbg_out >> 8) & 0xFF);
    param_data[idx++] = (uint8)(speed_dbg_out & 0xFF);
    /* speed_raw (int16) */
    param_data[idx++] = (uint8)((speed_dbg_raw >> 8) & 0xFF);
    param_data[idx++] = (uint8)(speed_dbg_raw & 0xFF);
    /* speed_plan (int16) */
    param_data[idx++] = (uint8)((speed_dbg_plan >> 8) & 0xFF);
    param_data[idx++] = (uint8)(speed_dbg_plan & 0xFF);
    /* speed_reason (uint8) */
    param_data[idx++] = speed_dbg_reason;
    /* yaw_angle (int16, ×10) */
    {
        int16 yaw10 = (int16)(yaw_angle * 10.0f);
        param_data[idx++] = (uint8)((yaw10 >> 8) & 0xFF);
        param_data[idx++] = (uint8)(yaw10 & 0xFF);
    }
    /* ip_max_row (uint8) */
    param_data[idx++] = g_ip_max_row;
    /* prof_tf (uint32) */
    param_data[idx++] = (uint8)((prof_tf_us >> 24) & 0xFF);
    param_data[idx++] = (uint8)((prof_tf_us >> 16) & 0xFF);
    param_data[idx++] = (uint8)((prof_tf_us >> 8) & 0xFF);
    param_data[idx++] = (uint8)(prof_tf_us & 0xFF);
    /* prof_in (uint32) */
    param_data[idx++] = (uint8)((prof_inter_us >> 24) & 0xFF);
    param_data[idx++] = (uint8)((prof_inter_us >> 16) & 0xFF);
    param_data[idx++] = (uint8)((prof_inter_us >> 8) & 0xFF);
    param_data[idx++] = (uint8)(prof_inter_us & 0xFF);
    /* ra_dbg_yaw10 (int16) */
    param_data[idx++] = (uint8)((ra_dbg_yaw10 >> 8) & 0xFF);
    param_data[idx++] = (uint8)(ra_dbg_yaw10 & 0xFF);
    /* route_dbg_step (uint8) */
    param_data[idx++] = route_dbg_step;

    send_uart_packet(FRAME_TYPE_PARAMS, param_data, PARAM_DATA_SIZE);
}
