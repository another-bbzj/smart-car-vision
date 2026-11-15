/*
 * 文件名称：Car_Race.c
 * 模块功能：比赛巡线状态机。每个控制周期接收视觉观测（曲率、元素、距离、置信度）、
 *           左右轮编码器增量、陀螺仪角速度和静止标志，积分出里程与航向，速度按曲率和
 *           置信度限制；分相位处理：巡线中识别蓝底缺口、黄色绕障（进入时记录障碍距离、
 *           按偏移跟随绕过后回到巡线）、第二圈红牌停车（静止满 5s 后继续）和完赛，
 *           每个周期输出左右轮目标转速。
 * 运行位置：在 PIT 5ms 中断中由 race_tick() 调用；视觉丢失、周期异常或相位超时直接进入
 *           RACE_FAULT，进入后不再输出，只能人工停车清除。
 */

#include "Car_Race.h"
#include "Car_Config.h"
#include <math.h>
#include <string.h>

/* 函数名称：limit
 * 函数功能：把浮点量限制到 [-bound, +bound]，用于横摆角速度和转向修正的限幅。
 * 参数说明：x —— 待限幅数值；bound —— 正数上限。
 * 返回参数：限幅后的数值。
 */
static float limit(float x, float bound) {
    return fmaxf(-bound, fminf(bound, x));
}

/* 函数名称：Race_Start
 * 函数功能：比赛开始前清零赛况：圈数置 1，里程、航向、相位、缺口/绕障去重标记全部归零，避免沿用上一场残留状态。
 * 参数说明：r —— 赛况对象。
 * 返回参数：无。
 */
void Race_Start(CarRace *r) {
    memset(r, 0, sizeof(*r));
    r->lap = 1;
    r->last_gap = r->last_action = -100.0f;
}

/* 函数名称：drive
 * 函数功能：把巡线层给出的前进速度与横摆角速度换算成左右轮目标 rpm，并按 CAR_RPM_LIMIT 等比例限幅。
 * 参数说明：v —— 前进速度（m/s）；omega —— 横摆角速度（rad/s）。
 * 返回参数：RaceDrive 双轮 rpm。
 */
static RaceDrive drive(float v, float omega) {
    RaceDrive out;
    float factor = 60.0f / (6.2831853f * RACE_RADIUS_M);
    float half = 0.5f * RACE_TRACK_M * omega;
    float peak;
    out.rpm[0] = (v - half) * factor;
    out.rpm[1] = (v + half) * factor;
    peak = fmaxf(fabsf(out.rpm[0]), fabsf(out.rpm[1]));
    if (peak > CAR_RPM_LIMIT) {
        out.rpm[0] *= CAR_RPM_LIMIT / peak;
        out.rpm[1] *= CAR_RPM_LIMIT / peak;
    }
    return out;
}

/* 函数名称：enter
 * 函数功能：切换比赛相位时记录进入点：里程、航向、绕障偏移、进入时刻和视觉清帧计数，供各相位判断距离与超时。
 * 参数说明：r —— 赛况对象；phase —— 目标相位；now —— 当前毫秒时基。
 * 返回参数：无。
 */
static void enter(CarRace *r, RacePhase phase, uint32_t now) {
    r->phase = phase;
    r->entry = r->distance;
    r->entry_yaw = r->yaw;
    r->offset = 0;
    r->entered_ms = now;
    r->clear_frames = 0;
}

/* 函数名称：Race_Step
 * 函数功能：比赛状态机主步进：用编码器增量与陀螺仪融合更新里程/航向，识别蓝底缺口、黄块绕障、第二圈红牌停车和完赛，输出双轮目标转速；视觉丢失或周期异常直接进入 RACE_FAULT。
 * 参数说明：r —— 赛况对象；o —— 视觉观测；dl/dr —— 左右轮增量距离；dt —— 控制周期（s）；gyro —— 陀螺仪 Z 轴角速度；gyro_ok —— 陀螺仪有效；stationary —— 车辆静止；now —— 毫秒时基。
 * 返回参数：RaceDrive 双轮 rpm；故障、完赛或红牌等待时返回 0。
 * 备注信息：每个控制周期调用一次；进入故障后不再输出，必须人工停车清除。
 */
RaceDrive Race_Step(CarRace *r, const CarVisionObservation *o, float dl,
                    float dr, float dt, float gyro, uint8_t gyro_ok,
                    uint8_t stationary, uint32_t now) {
    RaceDrive stop = {{0,0}};
    float ds = 0.5f * (dl + dr);
    float dyaw = (dr - dl) / RACE_TRACK_M;
    float s, curve, speed, heading, target, error, along;
    int road, new_frame, in_window;
    if (r->phase == RACE_FINISHED || r->phase == RACE_FAULT) return stop;
    if (!o || !(dt >= 0.001f && dt <= 0.0151f)) {
        r->phase = RACE_FAULT;
        return stop;
    }
    if (gyro_ok && isfinite(gyro)) dyaw = 0.85f * gyro * dt + 0.15f * dyaw;
    r->yaw += dyaw;
    r->distance += ds;
    r->lap = r->distance >= RACE_LAP_M ? 2 : 1;
    if (r->distance >= RACE_FINISH_M) {
        r->phase = RACE_FINISHED;
        return stop;
    }
    new_frame = !r->have_sequence || o->sequence != r->last_sequence;
    r->last_sequence = o->sequence;
    r->have_sequence = 1;
    road = (o->valid_mask & VISION_CURVATURE_VALID) && o->confidence >= 0.35f;
    if (new_frame && o->element != r->held_element &&
        r->distance - r->last_action > 0.8f) r->held_element = 0;
    if (r->phase == RACE_TRACK) {
        along = r->distance - (r->lap - 1) * RACE_LAP_M;
        in_window = RACE_GAP_ENTRY_M <= 0 || fabsf(along - RACE_GAP_ENTRY_M) < 1.0f;
        if (!road) {
            /* Only one crossing per circuit. Vision supplies the blue mouth
             * candidate, recent straight travel rejects ordinary corner losses. */
            if (o->element == VISION_ELEMENT_GAP && in_window &&
                r->distance - r->last_gap > RACE_LAP_M * 0.75f &&
                (RACE_GAP_ENTRY_M > 0 || r->straight_m >= 0.10f)) {
                enter(r, RACE_GAP, now);
                r->last_gap = r->distance;
            } else {
                r->phase = RACE_FAULT;
                return stop;
            }
        } else if ((o->valid_mask & VISION_DISTANCE_VALID) &&
                   o->element != r->held_element && o->confidence >= 0.50f) {
            if (o->element == VISION_ELEMENT_YELLOW && o->distance_m <= 0.80f) {
                enter(r, RACE_BYPASS, now);
                r->obstacle_m = fmaxf(0.08f, o->distance_m);
                r->goal_m = r->obstacle_m + 0.42f;
            } else if (o->element == VISION_ELEMENT_RED && r->lap == 2 &&
                       o->distance_m <= 0.20f) {
                enter(r, RACE_RED, now);
                r->timing_stop = 0;
            }
            if (r->phase != RACE_TRACK) {
                r->held_element = o->element;
                r->last_action = r->distance;
            }
        }
    }
    s = r->distance - r->entry;
    if (r->phase == RACE_RED) {
        if (!stationary) r->timing_stop = 0;
        else if (!r->timing_stop) { r->timing_stop = 1; r->still_ms = now; }
        else if (now - r->still_ms >= 5000U) r->phase = RACE_TRACK;
        if (now - r->entered_ms > 12000U) r->phase = RACE_FAULT;
        return stop;
    }
    heading = r->yaw - r->entry_yaw;
    if (r->phase == RACE_GAP) {
        if (s > 1.95f || now - r->entered_ms > 12500U) {
            r->phase = RACE_FAULT;
            return stop;
        }
        if (new_frame) {
            if (s >= 1.30f && road) ++r->clear_frames;
            else r->clear_frames = 0;
        }
        if (r->clear_frames < 3) return drive(0.22f, limit(-3.0f * heading, 1.2f));
        r->phase = RACE_TRACK;
        r->straight_m = 0;
    }
    if (r->phase == RACE_BYPASS) {
        /* Follow an offset in the entry frame. Integrate measured forward
         * and side displacement; hold clearance across the obstacle itself. */
        r->offset += ds * sinf(heading);
        r->goal_m -= ds * cosf(heading);
        along = r->obstacle_m + 0.42f - r->goal_m;
        if (now - r->entered_ms > 12000U || fabsf(heading) > 1.55f || r->goal_m < -0.25f) {
            r->phase = RACE_FAULT;
            return stop;
        }
        target = RACE_BYPASS_SIDE * RACE_BYPASS_OFFSET_M;
        if (along > r->obstacle_m + 0.10f) target *= fmaxf(0, r->goal_m / 0.32f);
        if (r->goal_m <= 0.02f && road && fabsf(r->offset) < 0.075f) {
            r->phase = RACE_TRACK;
        } else {
            error = atanf(5.5f * (target - r->offset));
            return drive(0.18f, limit(4.5f * (error - heading), 2.5f));
        }
    }
    if (!road) { r->phase = RACE_FAULT; return stop; }
    curve = limit(o->curvature_inv_m, 15.0f);
    r->straight_m = fabsf(curve) < 1.75f ? r->straight_m + fmaxf(0,ds) : 0;
    /* Bound lateral acceleration v²|k|, reduce speed on uncertain geometry. */
    speed = fminf(r->lap == 1 ? 0.55f : 0.32f, sqrtf(0.55f / (fabsf(curve) + 0.01f)));
    if (o->confidence < 0.65f || o->element == VISION_ELEMENT_RING ||
        o->element == VISION_ELEMENT_CROSS) speed = fminf(speed, 0.24f);
    if (o->element == VISION_ELEMENT_RED && r->lap == 2) speed = fminf(speed, 0.20f);
    speed = fminf(speed, sqrtf(fmaxf(0.0025f, 0.8f * (RACE_FINISH_M - r->distance))));
    return drive(speed, speed * curve);
}
