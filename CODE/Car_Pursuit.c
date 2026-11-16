/*
 * 文件名称：Car_Pursuit.c
 * 模块功能：纯跟踪路径跟随与差速换算。算法来自 HFK_STC32G144K 的
 *           alongLine()/setServoDuty()，但输入改为车辆坐标系的路径点，
 *           并加入随速度变化的动态预瞄；舵机转角经 Car_Steer 限幅/限速后，
 *           再用自行车模型换算成横摆角速度，最后分解为左右轮 rpm。
 * 运行位置：比赛状态机 Race_Step 在 RACE_TRACK 相位调用，纯算法。
 */

#include "Car_Pursuit.h"

#include <math.h>
#include <string.h>

#define CAR_PURSUIT_EPS 1e-6f

/* 判断浮点数是否为有限数。 */
static uint8_t finite(float x)
{
    return (uint8_t)(isfinite(x) ? 1U : 0U);
}

/* 把浮点数限制到 [lo, hi]。 */
static float clampf(float x, float lo, float hi)
{
    if (x < lo)
    {
        return lo;
    }

    if (x > hi)
    {
        return hi;
    }

    return x;
}

/*
 * 函数名称：config_ok
 * 函数功能：检查纯跟踪配置的有限性与物理范围。
 * 参数说明：c —— 配置。
 * 返回参数：1 合法；0 非法。
 */
static uint8_t config_ok(const CarPursuitConfig *c)
{
    return (uint8_t)(
        finite(c->wheelbase_m) && c->wheelbase_m > 0.0f && c->wheelbase_m <= 2.0f &&
        finite(c->track_m) && c->track_m > 0.0f && c->track_m <= 2.0f &&
        finite(c->wheel_radius_m) && c->wheel_radius_m > 0.0f &&
        c->wheel_radius_m <= 0.5f &&
        finite(c->lookahead_min_m) && c->lookahead_min_m > 0.0f &&
        c->lookahead_min_m <= 5.0f &&
        finite(c->lookahead_per_mps) && c->lookahead_per_mps >= 0.0f &&
        c->lookahead_per_mps <= 10.0f &&
        finite(c->lookahead_max_m) && c->lookahead_max_m >= c->lookahead_min_m &&
        c->lookahead_max_m <= 5.0f &&
        finite(c->steer_limit_rad) && c->steer_limit_rad > 0.0f &&
        c->steer_limit_rad <= 1.57f &&
        finite(c->steer_gain) && c->steer_gain >= 0.0f && c->steer_gain <= 10.0f &&
        finite(c->steer_rate_rad_s) && c->steer_rate_rad_s > 0.0f &&
        c->steer_rate_rad_s <= 100.0f &&
        finite(c->lateral_accel_limit_mps2) && c->lateral_accel_limit_mps2 > 0.0f &&
        c->lateral_accel_limit_mps2 <= 100.0f &&
        finite(c->speed_limit_mps) && c->speed_limit_mps > 0.0f &&
        c->speed_limit_mps <= 20.0f);
}

/*
 * 函数名称：CarPursuit_Init
 * 函数功能：保存配置并初始化虚拟舵机，配置失败时锁存故障。
 * 参数说明：p —— 跟踪对象；cfg —— 配置。
 * 返回参数：1 成功；0 参数非法。
 */
uint8_t CarPursuit_Init(CarPursuit *p, const CarPursuitConfig *cfg)
{
    if (p == NULL)
    {
        return 0;
    }

    memset(p, 0, sizeof(*p));

    if (cfg == NULL || !config_ok(cfg))
    {
        p->fault = 1;
        return 0;
    }

    p->config = *cfg;

    if (!CarSteer_Init(&p->steer, cfg->steer_gain, cfg->steer_limit_rad,
                       cfg->steer_rate_rad_s, cfg->pwm_center_us,
                       cfg->pwm_per_rad_us))
    {
        p->fault = 1;
        return 0;
    }

    return 1;
}

/*
 * 函数名称：CarPursuit_Reset
 * 函数功能：清空跟踪结果与舵机动态状态，保留配置。
 * 参数说明：p —— 跟踪对象。
 * 返回参数：无。
 */
void CarPursuit_Reset(CarPursuit *p)
{
    if (p == NULL)
    {
        return;
    }

    CarSteer_Reset(&p->steer);
    memset(&p->result, 0, sizeof(p->result));
    p->fault = 0;
}

/*
 * 函数名称：CarPursuit_Update
 * 函数功能：在路径折线上按动态预瞄距离找目标点，计算纯跟踪曲率、
 *           舵机转角和曲率限速。
 * 参数说明：见头文件。
 * 返回参数：1 结果有效；0 输入非法或目标点不在前方。
 */
uint8_t CarPursuit_Update(CarPursuit *p, const float *points_xy,
                          unsigned count, float speed_mps, float dt_s)
{
    float lookahead;
    float travelled = 0.0f;
    float px = 0.0f;
    float py = 0.0f;
    float tx = 0.0f;
    float ty = 0.0f;
    float lateral;
    float forward;
    float curvature;
    float target_angle;
    float max_speed;
    unsigned i;
    uint8_t found = 0;

    if (p == NULL || points_xy == NULL || count < 2U || p->fault ||
        !finite(speed_mps) || speed_mps < 0.0f ||
        !finite(dt_s) || dt_s <= 0.0f || dt_s > 0.1f)
    {
        if (p != NULL)
        {
            p->result.valid = 0;
        }
        return 0;
    }

    if (!(points_xy[0] >= -100.0f && points_xy[0] <= 100.0f &&
          points_xy[1] >= -100.0f && points_xy[1] <= 100.0f))
    {
        p->result.valid = 0;
        return 0;
    }

    /*
     * 动态预瞄：对应 HFK 的 DTarget(speed) 分段前瞻。
     * 速度越快，看得越远，减少高速时的转向抖动。
     */
    lookahead = p->config.lookahead_min_m +
                p->config.lookahead_per_mps * speed_mps;
    lookahead = clampf(lookahead, p->config.lookahead_min_m,
                       p->config.lookahead_max_m);

    /*
     * 把车辆当前位置 (0,0) 作为折线起点，沿路径累积弧长，
     * 找到第一个达到预瞄距离的位置并线性插值。
     */
    for (i = 0; i < count; i++)
    {
        const float nx = points_xy[2U * i];
        const float ny = points_xy[2U * i + 1U];
        const float dx = nx - px;
        const float dy = ny - py;
        const float len = sqrtf(dx * dx + dy * dy);

        if (!finite(nx) || !finite(ny) || !finite(len))
        {
            p->result.valid = 0;
            return 0;
        }

        if (len < CAR_PURSUIT_EPS)
        {
            continue;
        }

        if (travelled + len >= lookahead)
        {
            const float ratio = (lookahead - travelled) / len;

            tx = px + dx * ratio;
            ty = py + dy * ratio;
            found = 1;
            break;
        }

        travelled += len;
        px = nx;
        py = ny;
    }

    if (!found)
    {
        /* 路径总长不足预瞄距离：退回最后一个点。 */
        tx = points_xy[2U * (count - 1U)];
        ty = points_xy[2U * (count - 1U) + 1U];
    }

    lateral = tx;               /* 车辆坐标系：x 向右为正 */
    forward = ty;               /* 车辆坐标系：y 向前为正 */

    if (!finite(lateral) || !finite(forward) || forward <= 0.05f)
    {
        p->result.valid = 0;
        return 0;
    }

    /*
     * 纯跟踪几何：κ = 2y / (x² + y²)。
     * 视觉坐标 x 向右为正，而本工程曲率约定左转为正，
     * 因此取负号。
     */
    curvature = -2.0f * lateral / (lateral * lateral + forward * forward);

    /* 舵机目标转角由轴距和曲率反解，再经过限幅和速率限制。 */
    target_angle = atanf(p->config.wheelbase_m * curvature);
    (void)CarSteer_SetAngle(&p->steer, target_angle);

    p->result.valid = 1;
    p->result.lookahead_m = lookahead;
    p->result.lateral_m = lateral;
    p->result.forward_m = forward;
    p->result.curvature_inv_m = curvature;
    p->result.steering_rad = CarSteer_Update(&p->steer, dt_s);
    p->result.target_x_m = tx;
    p->result.target_y_m = ty;

    /*
     * 曲率限速：v = sqrt(a_lat / |κ|)，对应 HFK 的 sqrt(r*c4)。
     * 直道时退回配置的绝对速度上限。
     */
    max_speed = sqrtf(p->config.lateral_accel_limit_mps2 /
                      (fabsf(curvature) + 1e-3f));
    p->result.speed_limit_mps = fminf(max_speed, p->config.speed_limit_mps);

    return 1;
}

/*
 * 函数名称：CarPursuit_Wheels
 * 函数功能：用实际舵机转角和前进速度求横摆角速度，再分解成左右轮 rpm。
 * 参数说明：p —— 跟踪对象；speed_mps —— 前进速度；rpm —— 输出双轮 rpm。
 * 返回参数：无。
 */
void CarPursuit_Wheels(const CarPursuit *p, float speed_mps, float rpm[2])
{
    float half;
    float v_left;
    float v_right;
    float scale;

    if (rpm == NULL)
    {
        return;
    }

    rpm[0] = 0.0f;
    rpm[1] = 0.0f;

    if (p == NULL || !p->result.valid || !finite(speed_mps) || speed_mps <= 0.0f)
    {
        return;
    }

    /* 自行车模型：ω = v * tan(δ) / L；正转角为左转。 */
    half = 0.5f * p->config.track_m *
           CarSteer_YawRate(&p->steer, speed_mps, p->config.wheelbase_m);

    v_left = speed_mps - half;
    v_right = speed_mps + half;

    /* 线速度 m/s -> 输出轴 rpm。 */
    scale = 60.0f / (6.2831853f * p->config.wheel_radius_m);
    rpm[0] = v_left * scale;
    rpm[1] = v_right * scale;
}
