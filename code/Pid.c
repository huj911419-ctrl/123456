#include "Pid.h"
// ==================== 内部状态变量 ====================

// 转向 PD
static float s_steer_last_pos_err = 0.0f; // 上次位置误差（像素）
static uint8 s_steer_d_reset_flag = 0u;   // 微分复位标志（置1时下次D项清零）

// 速度 PI
static float s_speed_integral_l = 0.0f;// 左轮速度积分项
static float s_speed_integral_r = 0.0f;// 右轮速度积分项

// ==================== 内部函数 ====================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     转向 PD 控制
// 参数说明     pos_err     位置误差（像素，正：偏右，负：偏左）
// 返回参数     float       转向叠加量（正：向右修正）
// 备注信息     含死区处理 + 微分手动复位
//-------------------------------------------------------------------------------------------------------------------
static float steer_pd_calc(int16 pos_err)
{
    // --- 死区处理：误差绝对值过小时直接归零，消除抖动 ---
    if (pos_err > -STEER_DEADZONE && pos_err < STEER_DEADZONE)
    {
        s_steer_last_pos_err = 0.0f; // 进入死区同步清除上次误差
        return 0.0f;
    }

    float err = (float)pos_err;

    // --- P 项 ---
    float p_out = STEER_KP * err;

    // --- D 项（微分手动复位：丢线恢复或初始化后，D项跳变清零）---
    float d_out = 0.0f;
    if (s_steer_d_reset_flag == 0u)
    {
        d_out = STEER_KD * (err - s_steer_last_pos_err);
    }
    else
    {
        s_steer_d_reset_flag = 0u; // 消耗复位标志，本次D项为0
    }

    s_steer_last_pos_err = err;

    float output = p_out + d_out;

    if (output > STEER_MAX)
        output = STEER_MAX;
    else if (output < -STEER_MAX)
        output = -STEER_MAX;

    return output;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     速度 PI 控制（含积分分离）
// 参数说明     target      目标速度（编码器计数/周期）
// 参数说明     actual      实际速度
// 参数说明     integral    积分项指针
// 参数说明     pos_err_abs 位置误差绝对值（用于积分分离判断）
// 返回参数     float       速度 PI 输出
// 备注信息     积分分离：位置误差过大（急弯）时暂停积分累加，防止超调
//-------------------------------------------------------------------------------------------------------------------
static float speed_pi_calc(float target, float actual, float *integral, int16 pos_err_abs)
{
    float speed_err = target - actual;

    // --- 积分分离：位置误差在阈值内才允许积分累加 ---
    if (pos_err_abs < SPEED_I_SEPARATION)
    {
        *integral += speed_err;

        if (*integral > SPEED_I_MAX)
            *integral = SPEED_I_MAX;
        else if (*integral < -SPEED_I_MAX)
            *integral = -SPEED_I_MAX;
    }
    // 超出阈值时积分保持上次值，不清零也不累加（区别于完全清零的方案）

    float p_out = SPEED_KP * speed_err;
    float i_out = SPEED_KI * (*integral);

    return p_out + i_out;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     输出占空比限幅
//-------------------------------------------------------------------------------------------------------------------
static int16 clamp_duty(float val)
{
    if (val > MAX_DUTY)
        val = MAX_DUTY;
    else if (val < -MAX_DUTY)
        val = -MAX_DUTY;
    return (int16)val;
}

// ==================== 对外接口 ====================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     PID 模块初始化
// 使用示例     line_pid_init();
//-------------------------------------------------------------------------------------------------------------------
void line_pid_init(void)
{
    s_steer_last_pos_err = 0.0f;// 转向PD上次误差清零
    s_steer_d_reset_flag = 1u; // 初始化时触发一次复位，避免启动D项跳变
    s_speed_integral_l = 0.0f;// 左轮速度积分清零
    s_speed_integral_r = 0.0f;// 右轮速度积分清零
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     微分手动复位（外部主动调用）
// 使用示例     line_pid_reset_derivative();
// 备注信息     在特殊状态切换时（如丢线恢复、急停后重启）调用，防止D项突变
//-------------------------------------------------------------------------------------------------------------------
void line_pid_reset_derivative(void)
{
    s_steer_d_reset_flag = 1u;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     并行PID循迹主控（建议 10ms 定时器周期调用）
// 使用示例     line_pid_control();
// 备注信息     转向路：摄像头位置误差 → PD
//              速度路：磁编码器速度误差 → PI（含积分分离）
//              合并  ：左轮 = 速度PI + 转向PD，右轮 = 速度PI - 转向PD
//-------------------------------------------------------------------------------------------------------------------
void line_pid_control(void)
{
    // -------- 丢线处理 --------
  /* if (g_tf.line_lost)
    {
        small_driver_set_duty(0, 0);
        s_speed_integral_l = 0.0f; // 丢线时清除速度积分，防止恢复时冲速
        s_speed_integral_r = 0.0f;
        line_pid_reset_derivative(); // 丢线恢复后D项不产生跳变
        return;
    }*/

    int16 pos_err = g_tf.error; // 位置误差（像素）
    int16 pos_err_abs = pos_err >= 0 ? pos_err : -pos_err;

    // -------- 1. 转向 PD（位置误差 → 转向量）--------
    float steer = 0;//steer_pd_calc(pos_err);

    // -------- 2. 弯道自适应目标速度 --------
    float target_speed = BASE_SPEED - (float)pos_err_abs * SPEED_REDUCE_K;
    if (target_speed < MIN_SPEED)
        target_speed = MIN_SPEED;

    // -------- 3. 速度 PI（编码器速度误差 → 基础占空比，含积分分离）--------
    float actual_l = (float)motor_value.receive_left_speed_data;
    float actual_r = (float)motor_value.receive_right_speed_data;

    float speed_out_l = speed_pi_calc(target_speed, actual_l, &s_speed_integral_l, pos_err_abs);
    float speed_out_r = speed_pi_calc(target_speed, actual_r, &s_speed_integral_r, pos_err_abs);

    // -------- 4. 并行叠加 --------
    // pos_err > 0（偏右）→ 左轮加速，右轮减速
    float out_l = speed_out_l + steer;
    float out_r = speed_out_r - steer;

    // -------- 5. 限幅输出 --------
    small_driver_set_duty(clamp_duty(out_l), clamp_duty(out_r));
}
