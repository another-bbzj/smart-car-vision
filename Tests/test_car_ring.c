#include "Car_Ring.h"
#include "Car_Cross.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

/* 环岛/十字状态机回归测试：确认帧、方向偏置、里程退出与超时故障。 */

static CarVisionObservation obs(uint16_t seq, uint8_t element) {
    CarVisionObservation o;
    memset(&o, 0, sizeof(o));
    o.sequence = seq;
    o.element = element;
    o.confidence = 0.9f;
    o.lateral_m = 0.0f;
    return o;
}

/* 环岛偏置方向：左环岛曲率为正。 */
static int last_bias_positive(const CarRingOutput *out) {
    return out->curvature_bias > 0.0f;
}

static void ring_left(void) {
    CarRing r;
    CarVisionObservation o;
    CarRingOutput out;
    CarRing_Reset(&r);

    o = obs(1, VISION_ELEMENT_RING);
    out = CarRing_Step(&r, &o, 0.0f, 1000U, 1);
    assert(!out.valid && r.phase == CAR_RING_IDLE);
    o = obs(2, VISION_ELEMENT_RING);
    out = CarRing_Step(&r, &o, 0.0f, 1000U, 1);
    assert(r.phase == CAR_RING_ENTER);
    assert(r.side == 1);

    o = obs(3, VISION_ELEMENT_RING);
    out = CarRing_Step(&r, &o, 0.31f, 1200U, 1);
    assert(out.valid && out.curvature_bias > 0.0f);
    assert(r.phase == CAR_RING_INSIDE);

    o = obs(4, VISION_ELEMENT_RING);
    out = CarRing_Step(&r, &o, 1.55f, 3000U, 1);
    assert(out.valid && last_bias_positive(&out));
    assert(r.phase == CAR_RING_EXIT);

    o = obs(5, VISION_ELEMENT_RING);
    out = CarRing_Step(&r, &o, 2.00f, 3500U, 1);
    assert(!out.valid && r.phase == CAR_RING_IDLE);
}

static void ring_right_and_timeout(void) {
    CarRing r;
    CarVisionObservation o;
    CarRingOutput out;
    CarRing_Reset(&r);

    o = obs(1, VISION_ELEMENT_RING);
    (void)CarRing_Step(&r, &o, 0.0f, 0U, -1);
    o = obs(2, VISION_ELEMENT_RING);
    (void)CarRing_Step(&r, &o, 0.0f, 0U, -1);
    assert(r.phase == CAR_RING_ENTER && r.side == -1);
    o = obs(3, VISION_ELEMENT_RING);
    out = CarRing_Step(&r, &o, 0.35f, 100U, -1);
    assert(out.curvature_bias < 0.0f);

    o = obs(4, VISION_ELEMENT_RING);
    out = CarRing_Step(&r, &o, 0.40f, 20000U, -1);
    assert(out.fault && r.phase == CAR_RING_FAULT);
}

static void ring_side_inference(void) {
    CarRing r;
    CarVisionObservation o;
    CarRing_Reset(&r);
    o = obs(1, VISION_ELEMENT_RING);
    o.lateral_m = -0.2f; /* 横向偏差左正，负值推断为右环岛 */
    (void)CarRing_Step(&r, &o, 0.0f, 0U, 0);
    o = obs(2, VISION_ELEMENT_RING);
    o.lateral_m = -0.2f;
    (void)CarRing_Step(&r, &o, 0.0f, 0U, 0);
    assert(r.side == -1);
}

static void cross_straight(void) {
    CarCross c;
    CarVisionObservation o;
    CarCrossOutput out;
    CarCross_Reset(&c);

    o = obs(1, VISION_ELEMENT_CROSS);
    out = CarCross_Step(&c, &o, 0.0f, 0U);
    assert(!out.active);
    o = obs(2, VISION_ELEMENT_CROSS);
    (void)CarCross_Step(&c, &o, 0.0f, 0U);
    assert(c.phase == CAR_CROSS_INSIDE);

    o = obs(3, VISION_ELEMENT_CROSS);
    out = CarCross_Step(&c, &o, 0.20f, 100U);
    assert(out.active && out.straight);
    assert(out.speed_scale < 1.0f);

    o = obs(4, VISION_ELEMENT_NONE);
    out = CarCross_Step(&c, &o, 0.65f, 500U);
    assert(!out.active && c.phase == CAR_CROSS_IDLE);
}

static void cross_timeout(void) {
    CarCross c;
    CarVisionObservation o;
    CarCrossOutput out;
    CarCross_Reset(&c);
    o = obs(1, VISION_ELEMENT_CROSS);
    (void)CarCross_Step(&c, &o, 0.0f, 0U);
    o = obs(2, VISION_ELEMENT_CROSS);
    (void)CarCross_Step(&c, &o, 0.0f, 0U);
    o = obs(3, VISION_ELEMENT_CROSS);
    out = CarCross_Step(&c, &o, 0.10f, 6000U);
    assert(out.fault && c.phase == CAR_CROSS_FAULT);
}

int main(void) {
    ring_left();
    ring_right_and_timeout();
    ring_side_inference();
    cross_straight();
    cross_timeout();
    puts("car ring/cross tests passed");
    return 0;
}
