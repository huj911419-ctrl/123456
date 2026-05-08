#include "Pid.h"
#include "Menu.h"
#include "IMU.h"

int16 base_speed = 0; // 锟斤拷锟斤拷锟劫讹拷

// ================= 全锟街撅拷态锟斤拷锟斤拷 =================
static float s_steer_last_pos_err = 0.0f; // 锟斤拷一锟斤拷位锟斤拷锟斤拷睿拷锟斤拷锟紻锟斤拷锟斤拷悖�
static uint8 s_steer_d_reset_flag = 0u;   // 微锟街革拷位锟斤拷志锟斤拷1时锟铰达拷D锟筋不锟斤拷锟诫）

static float s_speed_integral = 0.0f;

// 锟皆讹拷停止 + 直锟斤拷锟斤拷状态
static uint32 s_motor_run_counter = 0;
static uint8 s_prev_ra_flag = 0; // 锟斤拷一锟斤拷直锟斤拷锟斤拷锟街�

// ================================================================
// 锟节诧拷锟斤拷锟斤拷锟斤拷转锟斤拷 PD 锟斤拷锟斤拷
// ================================================================
static float steer_pd_calc(int16 pos_err)
{
    // 涓�闃朵綆閫氭护娉紝鎶戝埗鎽勫儚澶村抚闂村櫔澹板紩璧风殑楂橀鎶栧姩
    static float s_filtered_err = 0.0f;
    s_filtered_err = s_filtered_err * ERROR_FILTER_ALPHA + (float)pos_err * (1.0f - ERROR_FILTER_ALPHA);
    float err = s_filtered_err;

    // 姝诲尯锛氬亸宸皬涓嶈緭鍑猴紝浣嗕繚鎸佹护娉㈢姸鎬佸拰 last_err锛堜慨澶嶉��鍑烘鍖烘椂 D 椤圭獊璺筹級
    if (err > -STEER_DEADZONE && err < STEER_DEADZONE)
    {
        s_steer_last_pos_err = err;
        return 0.0f;
    }

    float p_out = STEER_KP * err;

    float d_out = 0.0f;
    if (s_steer_d_reset_flag == 0u)
    {
        d_out = STEER_KD * (err - s_steer_last_pos_err);
    }
    else
    {
        s_steer_d_reset_flag = 0u;
    }

    s_steer_last_pos_err = err;

    float output = p_out + d_out;

    if (output > STEER_MAX)
        output = STEER_MAX;
    else if (output < -STEER_MAX)
        output = -STEER_MAX;

    return output;
}

// ================================================================
// 锟节诧拷锟斤拷锟斤拷锟斤拷锟劫讹拷 PI 锟斤拷锟斤拷锟斤拷
// ================================================================
static float speed_pi_calc(float target, float actual, float *integral, int16 pos_err_abs)
{
    float speed_err = target - actual;

    if (pos_err_abs < SPEED_I_SEPARATION)
    {
        *integral += speed_err;

        if (*integral > SPEED_I_MAX)
            *integral = SPEED_I_MAX;
        else if (*integral < -SPEED_I_MAX)
            *integral = -SPEED_I_MAX;
    }

    float p_out = SPEED_KP * speed_err;
    float i_out = SPEED_KI * (*integral);

    return p_out + i_out;
}

// ================================================================
// 锟节诧拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷薹锟�
// ================================================================
static int16 clamp_duty(float val)
{
    if (val > MAX_DUTY)
        val = MAX_DUTY;
    else if (val < -MAX_DUTY)
        val = -MAX_DUTY;
    return (int16)val;
}

// ================================================================
// PID锟斤拷始锟斤拷
// ================================================================
void line_pid_init(void)
{
    s_steer_last_pos_err = 0.0f;
    s_steer_d_reset_flag = 1u;
    s_speed_integral = 0.0f;
    s_prev_ra_flag = 0;
    imu_reset_yaw();
}

// ================================================================
// 锟斤拷位D锟斤拷
// ================================================================
void line_pid_reset_derivative(void)
{
    s_steer_d_reset_flag = 1u;
}

// ================================================================
// PID锟斤拷锟斤拷锟狡猴拷锟斤拷锟斤拷30ms锟斤拷锟节ｏ拷
// ================================================================
void line_pid_control(void)
{
    if (motor_enable == 0)
    {
        small_driver_set_duty(0, 0);
        s_speed_integral = 0.0f;
        s_motor_run_counter = 0;
        return;
    }

    s_motor_run_counter++;
    if (s_motor_run_counter >= (uint32)motor_run_time * 1000 / 11)
    {
        motor_enable = 0;
        small_driver_set_duty(0, 0);
        s_speed_integral = 0.0f;
        s_motor_run_counter = 0;
        return;
    }

    int16 pos_err = g_tf.error;
    int16 pos_err_abs = pos_err >= 0 ? pos_err : -pos_err;
    base_speed = (int16)motor_speed * 8;

    // 鐩磋棰勫垽鍑忛�� + 閿佸瓨闃查棯鐑� + 瓒呮椂闃茶瑙﹀彂
    {
        static uint8 s_pre_lock = 0;
        static uint8 s_pre_timeout = 0;

        if (g_ra_pre_flag && g_ra_flag == 0)
        {
            s_pre_lock = 1;
            s_pre_timeout = 0;
        }

        if (g_ra_flag != 0)
        {
            s_pre_lock = 0;
        }

        if (s_pre_lock)
        {
            base_speed = base_speed / 7;
            if (!g_ra_pre_flag)
            {
                s_pre_timeout++;
                if (s_pre_timeout > 30) // ~330ms 棰勫垽娑堝け浠嶄笉鎭㈠ 鈫� 瑙ｉ攣
                {
                    s_pre_lock = 0;
                    s_pre_timeout = 0;
                }
            }
        }
    }

    // 鐩磋寮杞拷锟斤拷浯︼拷锟�
    if (g_ra_flag == 1)
    {
        s_prev_ra_flag = 1;
        small_driver_set_duty(0, base_speed);
        turn_right_led_on();
        return;
    }
    else if (g_ra_flag == 2)
    {
        s_prev_ra_flag = 2;
        small_driver_set_duty(base_speed, 0);
        turn_right_led_off();
        return;
    }

    if (s_prev_ra_flag != 0)
    {
        imu_reset_yaw();
        s_prev_ra_flag = 0;
        turn_right_led_off();
    }

    float steer = steer_pd_calc(pos_err);

    // Yaw锟斤拷锟斤拷
    {
        float yaw_kp_val = (float)yaw_kp / 10.0f;
        float yaw_comp = 0.0f;
        float yaw_abs = yaw_angle >= 0 ? yaw_angle : -yaw_angle;
        if (yaw_abs > YAW_DEADZONE)
            yaw_comp = yaw_kp_val * yaw_angle;
        steer += yaw_comp;
    }

    float target_speed = base_speed;

    float actual_l = (float)motor_value.receive_left_speed_data;
    float actual_r = (float)motor_value.receive_right_speed_data;
    float avg_actual = (actual_l + actual_r) * 0.5f;

    // 鍗曢�熷害鐜細涓よ疆骞冲潎閫熷害闂幆锛岄伩鍏嶅乏鍙砅I浜掔浉瀵规姉
    float speed_out = speed_pi_calc(target_speed, avg_actual, &s_speed_integral, pos_err_abs);

    // 閫熷害瓒婇珮闇�瑕佽秺澶ц浆鍚戝姏搴�
    float speed_factor = 1.0f + (float)base_speed * 0.002f;
    steer *= speed_factor;

    float out_l = speed_out + steer;
    float out_r = speed_out - steer;

    small_driver_set_duty(clamp_duty(out_l), clamp_duty(out_r));
}
