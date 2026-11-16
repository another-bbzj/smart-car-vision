/*
 * 文件名称：Car_Cross.c
 * 模块功能：十字路口两相位状态机。沿用 HFK_STC32G144K 的
 *           CROSS_READY / CROSS_IN 划分，触发改为视觉元素 + 里程，
 *           路口内强制直行并限制速度，走过固定里程后回到巡线。
 * 运行位置：Car_Race.c 的 RACE_TRACK 分支每个控制周期调用。
 */

#include "Car_Cross.h"
#include "Car_Config.h"

#include <string.h>

/* 判断浮点数是否为有限数。 */
static uint8_t finite(float x)
{
    return (uint8_t)(x == x && x > -1.0e30f && x < 1.0e30f);
}

/*
 * 函数名称：CarCross_Reset
 * 函数功能：清空相位、去重序号与计时，回到 IDLE。
 * 参数说明：c —— 十字对象。
 * 返回参数：无。
 */
void CarCross_Reset(CarCross *c)
{
    if (c != NULL)
    {
        memset(c, 0, sizeof(*c));
        c->phase = CAR_CROSS_IDLE;
    }
}

/*
 * 函数名称：CarCross_Step
 * 函数功能：按视觉元素和里程推进十字相位，输出直行标志。
 * 参数说明：见头文件。
 * 返回参数：本周期十字输出。
 */
CarCrossOutput CarCross_Step(CarCross *c, const CarVisionObservation *o,
                             float distance_m, uint32_t now_ms)
{
    CarCrossOutput out;
    uint8_t new_frame;
    float s;
    uint32_t elapsed;

    memset(&out, 0, sizeof(out));
    out.speed_scale = 1.0f;

    if (c == NULL)
    {
        out.fault = 1;
        return out;
    }

    if (c->phase == CAR_CROSS_FAULT)
    {
        out.fault = 1;
        return out;
    }

    if (o == NULL || !finite(distance_m))
    {
        return out;
    }

    new_frame = (uint8_t)(!c->have_sequence || o->sequence != c->last_sequence);
    c->last_sequence = o->sequence;
    c->have_sequence = 1;
    elapsed = now_ms - c->entered_ms;

    if (c->phase == CAR_CROSS_IDLE)
    {
        /* 连续两帧（去重后）识别到十字且置信度足够才进入。 */
        if (o->element == VISION_ELEMENT_CROSS &&
            o->confidence >= CAR_CROSS_MIN_CONFIDENCE)
        {
            if (new_frame && c->confirm < 255U)
            {
                c->confirm++;
            }

            if (c->confirm >= CAR_CROSS_CONFIRM_FRAMES)
            {
                c->phase = CAR_CROSS_INSIDE;
                c->entry_distance_m = distance_m;
                c->entered_ms = now_ms;
                c->confirm = 0;
            }
        }
        else
        {
            c->confirm = 0;
        }

        return out;
    }

    /* CAR_CROSS_INSIDE：强制直行，走过设定里程或超时后退出。 */
    s = distance_m - c->entry_distance_m;
    if (!finite(s))
    {
        return out;
    }

    out.active = 1;
    out.straight = 1;
    out.speed_scale = CAR_CROSS_SPEED_SCALE;

    if (s >= CAR_CROSS_HOLD_M)
    {
        c->phase = CAR_CROSS_IDLE;
        out.active = 0;
        out.straight = 0;
        out.speed_scale = 1.0f;
    }
    else if (elapsed > CAR_CROSS_TIMEOUT_MS)
    {
        c->phase = CAR_CROSS_FAULT;
        out.fault = 1;
        out.active = 0;
        out.straight = 0;
        out.speed_scale = 1.0f;
    }

    return out;
}
