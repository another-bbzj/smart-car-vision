#include "Car_Pursuit.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

/* 纯跟踪/虚拟舵机回归测试：直道、左右弯、动态预瞄、限速和非法输入。 */

static CarPursuitConfig config(void) {
    CarPursuitConfig c;
    c.wheelbase_m = 0.152f;
    c.track_m = 0.173f;
    c.wheel_radius_m = 0.0325f;
    c.lookahead_min_m = 0.20f;
    c.lookahead_per_mps = 0.35f;
    c.lookahead_max_m = 1.20f;
    c.steer_limit_rad = 0.52f;
    c.steer_gain = 1.0f;
    c.steer_rate_rad_s = 6.0f;
    c.lateral_accel_limit_mps2 = 0.55f;
    c.speed_limit_mps = 1.20f;
    c.pwm_center_us = 1500.0f;
    c.pwm_per_rad_us = 500.0f;
    return c;
}

static void straight(void) {
    CarPursuit p;
    CarPursuitConfig c = config();
    const float path[] = {0.0f, 0.40f, 0.0f, 0.80f, 0.0f, 1.20f};
    float rpm[2];
    assert(CarPursuit_Init(&p, &c));
    assert(CarPursuit_Update(&p, path, 3, 0.5f, 0.005f));
    assert(p.result.valid);
    assert(fabsf(p.result.curvature_inv_m) < 1e-5f);
    assert(fabsf(p.result.steering_rad) < 1e-4f);
    CarPursuit_Wheels(&p, 0.5f, rpm);
    assert(fabsf(rpm[0] - rpm[1]) < 1e-3f);
    assert(rpm[0] > 0.0f);
}

static void right_turn(void) {
    CarPursuit p;
    CarPursuitConfig c = config();
    /* 路径持续向右偏：x 向右为正。 */
    const float path[] = {0.10f, 0.40f, 0.25f, 0.80f, 0.40f, 1.20f};
    float rpm[2];
    assert(CarPursuit_Init(&p, &c));
    assert(CarPursuit_Update(&p, path, 3, 0.5f, 0.005f));
    assert(p.result.valid);
    assert(p.result.curvature_inv_m < 0.0f);   /* 右转为负曲率 */
    assert(p.result.steering_rad < 0.0f);      /* 右转为负转角 */
    CarPursuit_Wheels(&p, 0.5f, rpm);
    assert(rpm[0] > rpm[1]);                   /* 右转时左轮更快 */
}

static void left_turn(void) {
    CarPursuit p;
    CarPursuitConfig c = config();
    const float path[] = {-0.10f, 0.40f, -0.25f, 0.80f, -0.40f, 1.20f};
    float rpm[2];
    assert(CarPursuit_Init(&p, &c));
    assert(CarPursuit_Update(&p, path, 3, 0.5f, 0.005f));
    assert(p.result.valid);
    assert(p.result.curvature_inv_m > 0.0f);
    assert(p.result.steering_rad > 0.0f);
    CarPursuit_Wheels(&p, 0.5f, rpm);
    assert(rpm[0] < rpm[1]);
}

static void dynamic_lookahead(void) {
    CarPursuit p;
    CarPursuitConfig c = config();
    const float path[] = {0.0f, 0.40f, 0.0f, 0.80f, 0.0f, 1.20f, 0.0f, 1.60f};
    float low, high;
    assert(CarPursuit_Init(&p, &c));
    assert(CarPursuit_Update(&p, path, 4, 0.1f, 0.005f));
    low = p.result.lookahead_m;
    assert(CarPursuit_Update(&p, path, 4, 1.0f, 0.005f));
    high = p.result.lookahead_m;
    assert(high > low);
    assert(high <= c.lookahead_max_m + 1e-5f);
}

static void curved_speed_limit(void) {
    CarPursuit p;
    CarPursuitConfig c = config();
    const float path[] = {0.20f, 0.40f, 0.50f, 0.60f, 0.80f, 0.70f};
    assert(CarPursuit_Init(&p, &c));
    assert(CarPursuit_Update(&p, path, 3, 0.8f, 0.005f));
    assert(p.result.speed_limit_mps < c.speed_limit_mps);
    assert(p.result.speed_limit_mps > 0.0f);
}

static void invalid_inputs(void) {
    CarPursuit p;
    CarPursuitConfig c = config();
    const float behind[] = {0.0f, -0.40f, 0.0f, -0.80f};
    const float path[] = {0.0f, 0.50f, 0.0f, 1.00f};
    assert(CarPursuit_Init(&p, &c));
    assert(!CarPursuit_Update(&p, path, 1, 0.5f, 0.005f));
    assert(!CarPursuit_Update(&p, NULL, 2, 0.5f, 0.005f));
    assert(!CarPursuit_Update(&p, behind, 2, 0.5f, 0.005f));
    assert(!CarPursuit_Update(&p, path, 2, 0.5f, 0.2f));

    c.track_m = -1.0f;
    assert(!CarPursuit_Init(&p, &c));
    assert(p.fault);
}

static void steer_rate_limit(void) {
    CarSteer s;
    float a;
    assert(CarSteer_Init(&s, 1.0f, 0.52f, 6.0f, 1500.0f, 500.0f));
    assert(CarSteer_SetAngle(&s, 0.50f));
    a = CarSteer_Update(&s, 0.005f);
    assert(fabsf(a - 0.03f) < 1e-5f);       /* 6 rad/s × 5 ms */
    assert(CarSteer_PwmUs(&s) == 1500.0f + a * 500.0f);
    assert(CarSteer_SetAngle(&s, NAN) == 0);
    assert(s.fault);
}

int main(void) {
    straight();
    right_turn();
    left_turn();
    dynamic_lookahead();
    curved_speed_limit();
    invalid_inputs();
    steer_rate_limit();
    puts("car pursuit tests passed");
    return 0;
}
