/*
 * 文件名称：Car_Steer.c
 * 模块功能：虚拟转向舵机模型。移植自 HFK_STC32G144K 的 setServoDuty()：
 *           转角命令先乘增益、再限幅，然后按舵机速率慢慢逼近目标。
 *           与原实现的区别是这里不直接写 PWM，而是把实际转角交给
 *           Car_Pursuit 换算成横摆角速度和左右轮转速，适配差速底盘。
 * 运行位置：由 Car_Pursuit 在控制周期内调用，纯算法，不访问外设。
 */

#include "Car_Steer.h"

#include <math.h>
#include <string.h>

/* 判断浮点数是否为有限数。 */
static uint8_t finite(float x)
{
    return (uint8_t)(isfinite(x) ? 1U : 0U);
}

/* 把浮点数限制到 [-lim, lim]。 */
static float clamp_sym(float x, float lim)
{
    if (x > lim)
    {
        return lim;
    }

    if (x < -lim)
    {
        return -lim;
    }

    return x;
}

/*
 * 函数名称：CarSteer_Init
 * 函数功能：清零对象并保存舵机模型参数，参数非法时锁存故障。
 * 参数说明：见头文件。
 * 返回参数：1 成功；0 参数非法。
 */
uint8_t CarSteer_Init(CarSteer *s, float gain, float limit_rad,
                      float rate_limit_rad_s, float pwm_center_us,
                      float pwm_per_rad_us)
{
    if (s == NULL)
    {
        return 0;
    }

    memset(s, 0, sizeof(*s));

    if (!finite(gain) || gain < 0.0f || gain > 10.0f ||
        !finite(limit_rad) || limit_rad <= 0.0f || limit_rad > 1.57f ||
        !finite(rate_limit_rad_s) || rate_limit_rad_s <= 0.0f ||
        rate_limit_rad_s > 100.0f ||
        !finite(pwm_center_us) || pwm_center_us < 500.0f ||
        pwm_center_us > 2500.0f ||
        !finite(pwm_per_rad_us) || pwm_per_rad_us < 0.0f ||
        pwm_per_rad_us > 2000.0f)
    {
        s->fault = 1;
        return 0;
    }

    s->gain = gain;
    s->limit_rad = limit_rad;
    s->rate_limit_rad_s = rate_limit_rad_s;
    s->pwm_center_us = pwm_center_us;
    s->pwm_per_rad_us = pwm_per_rad_us;
    return 1;
}

/*
 * 函数名称：CarSteer_Reset
 * 函数功能：清空实际/目标转角和故障，保留模型参数。
 * 参数说明：s —— 舵机对象。
 * 返回参数：无。
 */
void CarSteer_Reset(CarSteer *s)
{
    if (s == NULL)
    {
        return;
    }

    s->angle_rad = 0.0f;
    s->target_rad = 0.0f;
    s->fault = 0;
}

/*
 * 函数名称：CarSteer_SetAngle
 * 函数功能：把命令转角乘增益并限幅，写入目标转角。
 * 参数说明：s —— 舵机对象；angle_rad —— 命令转角（rad，左转为正）。
 * 返回参数：1 成功；0 故障或输入非法。
 */
uint8_t CarSteer_SetAngle(CarSteer *s, float angle_rad)
{
    if (s == NULL || s->fault || !finite(angle_rad))
    {
        if (s != NULL)
        {
            s->fault = 1;
        }
        return 0;
    }

    s->target_rad = clamp_sym(angle_rad * s->gain, s->limit_rad);
    return 1;
}

/*
 * 函数名称：CarSteer_Update
 * 函数功能：按周期限速逼近目标转角，模拟舵机响应。
 * 参数说明：s —— 舵机对象；dt_s —— 周期（秒）。
 * 返回参数：实际转角（rad）。
 */
float CarSteer_Update(CarSteer *s, float dt_s)
{
    float step;

    if (s == NULL)
    {
        return 0.0f;
    }

    if (!finite(dt_s) || dt_s <= 0.0f || dt_s > 0.1f || s->fault)
    {
        return s->angle_rad;
    }

    step = s->rate_limit_rad_s * dt_s;

    if (s->target_rad > s->angle_rad + step)
    {
        s->angle_rad += step;
    }
    else if (s->target_rad < s->angle_rad - step)
    {
        s->angle_rad -= step;
    }
    else
    {
        s->angle_rad = s->target_rad;
    }

    return s->angle_rad;
}

/*
 * 函数名称：CarSteer_PwmUs
 * 函数功能：把实际转角换算成舵机脉宽，供未来阿克曼底盘直接使用。
 * 参数说明：s —— 舵机对象。
 * 返回参数：脉宽（微秒）。
 */
float CarSteer_PwmUs(const CarSteer *s)
{
    if (s == NULL)
    {
        return 0.0f;
    }

    return s->pwm_center_us + s->angle_rad * s->pwm_per_rad_us;
}

/*
 * 函数名称：CarSteer_YawRate
 * 函数功能：由舵机转角和前进速度求横摆角速度，采用自行车模型。
 * 参数说明：s —— 舵机对象；speed_mps —— 前进速度；wheelbase_m —— 轴距。
 * 返回参数：横摆角速度（rad/s，左转为正）；参数非法或转角极小时为 0。
 */
float CarSteer_YawRate(const CarSteer *s, float speed_mps, float wheelbase_m)
{
    if (s == NULL || !finite(speed_mps) || speed_mps < 0.0f ||
        !finite(wheelbase_m) || wheelbase_m <= 0.0f)
    {
        return 0.0f;
    }

    if (fabsf(s->angle_rad) < 1e-4f)
    {
        return 0.0f;
    }

    return speed_mps * tanf(s->angle_rad) / wheelbase_m;
}
