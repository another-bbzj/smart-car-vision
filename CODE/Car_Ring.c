/*
 * 文件名称：Car_Ring.c
 * 模块功能：环岛四相位状态机。相位划分沿用 HFK_STC32G144K 的
 *           ROUND_START / ROUND_IN / ROUND_END / ROUND_EXIT，
 *           触发条件改为 W 车的视觉元素 + 累计里程，输出曲率偏置。
 * 运行位置：Car_Race.c 的 RACE_TRACK 分支每个控制周期调用。
 */

#include "Car_Ring.h"
#include "Car_Config.h"

#include <string.h>

/* 判断浮点数是否为有限数。 */
static uint8_t finite(float x)
{
    return (uint8_t)(x == x && x > -1.0e30f && x < 1.0e30f);
}

/* 环岛转向偏置：左环岛为 +1/R，右环岛为 -1/R。 */
static float bias_for(int8_t side)
{
    return (side > 0 ? 1.0f : -1.0f) / CAR_RING_RADIUS_M;
}

/*
 * 函数名称：CarRing_Reset
 * 函数功能：清空相位、去重序号与计时，回到 IDLE。
 * 参数说明：r —— 环岛对象。
 * 返回参数：无。
 */
void CarRing_Reset(CarRing *r)
{
    if (r != NULL)
    {
        memset(r, 0, sizeof(*r));
        r->phase = CAR_RING_IDLE;
    }
}

/*
 * 函数名称：CarRing_Step
 * 函数功能：按视觉元素和里程推进环岛相位，输出曲率偏置。
 * 参数说明：见头文件。
 * 返回参数：本周期环岛输出。
 */
CarRingOutput CarRing_Step(CarRing *r, const CarVisionObservation *o,
                           float distance_m, uint32_t now_ms,
                           int8_t side_hint)
{
    CarRingOutput out;
    uint8_t new_frame = 0;
    float s;
    uint32_t elapsed;

    memset(&out, 0, sizeof(out));
    out.speed_scale = 1.0f;

    if (r == NULL)
    {
        out.fault = 1;
        return out;
    }

    if (r->phase == CAR_RING_FAULT)
    {
        out.fault = 1;
        return out;
    }

    if (o == NULL || !finite(distance_m))
    {
        return out;
    }

    new_frame = (uint8_t)(!r->have_sequence || o->sequence != r->last_sequence);
    r->last_sequence = o->sequence;
    r->have_sequence = 1;
    elapsed = now_ms - r->entered_ms;

    if (r->phase == CAR_RING_IDLE)
    {
        /*
         * 连续两帧（去重后）识别到环岛且置信度足够才进入，
         * 避免单帧误检直接改变转向。
         */
        if (o->element == VISION_ELEMENT_RING &&
            o->confidence >= CAR_RING_MIN_CONFIDENCE)
        {
            if (new_frame)
            {
                if (r->confirm < 255U)
                {
                    r->confirm++;
                }
            }

            if (r->confirm >= CAR_RING_CONFIRM_FRAMES)
            {
                /*
                 * 方向优先采用视觉提示；没有提示时按横向偏差判断：
                 * W 视觉横向偏差以左为正，左环岛时车在环岛右侧。
                 */
                r->side = side_hint;
                if (r->side == 0)
                {
                    r->side = (o->lateral_m >= 0.0f) ? 1 : -1;
                }

                r->phase = CAR_RING_ENTER;
                r->entry_distance_m = distance_m;
                r->entered_ms = now_ms;
                r->confirm = 0;
            }
        }
        else
        {
            r->confirm = 0;
        }

        return out;
    }

    s = distance_m - r->entry_distance_m;
    if (!finite(s))
    {
        return out;
    }

    /* 环岛内长时间未走完，视为定位或视觉失效。 */
    if (elapsed > CAR_RING_TIMEOUT_MS)
    {
        r->phase = CAR_RING_FAULT;
        out.fault = 1;
        return out;
    }

    out.valid = 1;
    out.speed_scale = CAR_RING_SPEED_SCALE;

    if (r->phase == CAR_RING_ENTER)
    {
        out.curvature_bias = 0.5f * bias_for(r->side);

        if (s >= CAR_RING_ENTER_M || elapsed > 1500U)
        {
            r->phase = CAR_RING_INSIDE;
        }
    }
    else if (r->phase == CAR_RING_INSIDE)
    {
        out.curvature_bias = bias_for(r->side);

        if (s >= CAR_RING_ENTER_M + CAR_RING_INSIDE_M)
        {
            r->phase = CAR_RING_EXIT;
        }
    }
    else /* CAR_RING_EXIT */
    {
        const float travelled = s - (CAR_RING_ENTER_M + CAR_RING_INSIDE_M);

        if (travelled >= CAR_RING_EXIT_M)
        {
            r->phase = CAR_RING_IDLE;
            out.valid = 0;
            out.curvature_bias = 0.0f;
            out.speed_scale = 1.0f;
        }
        else
        {
            const float remain = 1.0f - travelled / CAR_RING_EXIT_M;

            out.curvature_bias = bias_for(r->side) * remain;
        }
    }

    return out;
}
