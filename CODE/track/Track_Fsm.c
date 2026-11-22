/*
 * ============================================================================
 * 文件名称：Track_Fsm.c
 * ============================================================================
 * 模块功能
 * ----------------------------------------------------------------------------
 * 巡线状态机：视觉曲率 -> 左右轮目标转速，并管理赛段切换。
 *
 *   赛段（TrackPhase）
 *     TRACK_SEGMENT_TRACK    正常巡线：曲率决定角速度，丢线时沿用上次曲率并降速
 *     TRACK_SEGMENT_GAP      断路：盲走固定低速，靠航向保持直线，看到路再回 TRACK
 *     TRACK_SEGMENT_BYPASS   绕障：在进入坐标系里跟踪横向偏移，越过障碍后归位
 *     TRACK_SEGMENT_RED      红灯停车：静止停留 5 s（可配置），超时判故障
 *     TRACK_SEGMENT_FINISHED 跑完两圈：停车并触发数独收尾
 *     TRACK_SEGMENT_FAULT    赛段异常：由任务层锁存 CAR_RACE_FAULT
 *
 *   关键计算
 *     Track_Drive(v, omega)：自行车模型 -> 左右轮 rpm，并限制内轮不反转，
 *       最后按 BOARD_RPM_LIMIT 等比缩放到限幅内。
 *     Track_Step()          ：里程/航向积分（轮速 + 陀螺融合）-> 赛段判定 ->
 *       速度规划（曲率越大越慢，丢线降到 TRACK_HOLD_SPEED_MPS）。
 *
 * 运行视角
 *   控制中断每拍：Track_Step(&track, 视觉快照, 左轮米数, 右轮米数, dt,
 *   陀螺角速度, 陀螺有效, 是否静止, 毫秒时间) -> TrackDrive -> Car_RaceWheels。
 *
 * 函数列表
 *   static float     limit(float x, float bound)
 *   void             Track_Start(TrackState *r)
 *   static TrackDrive Track_Drive(float v, float omega)
 *   static void      enter(TrackState *r, TrackPhase phase, uint32_t now)
 *   TrackDrive       Track_Step(...)
 * ============================================================================
 */
#include "Track_Fsm.h"
#include "Board_Config.h"
#include <math.h>
#include <string.h>

static float limit(float x, float bound) {
    return fmaxf(-bound, fminf(bound, x));
}

void Track_Start(TrackState *r) {
    memset(r, 0, sizeof(*r));
    r->lap = 1;
    r->last_gap = r->last_action = -100.0f;
}

static TrackDrive drive(float v, float omega) {
    TrackDrive out;
    float factor = 60.0f / (6.2831853f * TRACK_RADIUS_M);
    float half = 0.5f * TRACK_WIDTH_M * omega;
    float peak;
    /* Curvature spikes can ask the inner wheel to reverse while the chassis is
     * still rolling forward. A reversed inner wheel is dragged backwards by the
     * ground: it stops tracking its target, and the stall or direction guard
     * then stops the whole car. Keep the differential within |half| <= |v| so
     * the inner wheel may slow to zero but never crosses zero. */
    half = limit(half, fabsf(v));
    out.rpm[0] = (v - half) * factor;
    out.rpm[1] = (v + half) * factor;
    peak = fmaxf(fabsf(out.rpm[0]), fabsf(out.rpm[1]));
    if (peak > BOARD_RPM_LIMIT) {
        out.rpm[0] *= BOARD_RPM_LIMIT / peak;
        out.rpm[1] *= BOARD_RPM_LIMIT / peak;
    }
    return out;
}

static void enter(TrackState *r, TrackPhase phase, uint32_t now) {
    r->phase = phase;
    r->entry = r->distance;
    r->entry_yaw = r->yaw;
    r->offset = 0;
    r->entered_ms = now;
    r->last_road_ms = now;   /* hold timer restarts for every blind phase */
    r->clear_frames = 0;
}

TrackDrive Track_Step(TrackState *r, const CarVisionObservation *o, float dl,
                    float dr, float dt, float gyro, uint8_t gyro_ok,
                    uint8_t stationary, uint32_t now) {
    TrackDrive stop = {{0,0}};
    float ds = 0.5f * (dl + dr);
    float dyaw = (dr - dl) / TRACK_WIDTH_M;
    float s, curve, speed, heading, target, error, along;
    int road, new_frame, in_window;
    if (r->phase == TRACK_SEGMENT_FINISHED || r->phase == TRACK_SEGMENT_FAULT) return stop;
    /* A bad control period still faults: it means the 5 ms loop itself failed. */
    if (!(dt >= 0.001f && dt <= 0.0151f)) {
        r->phase = TRACK_SEGMENT_FAULT;
        return stop;
    }
    if (gyro_ok && isfinite(gyro)) dyaw = 0.85f * gyro * dt + 0.15f * dyaw;
    r->yaw += dyaw;
    r->distance += ds;
    r->lap = r->distance >= TRACK_LAP_M ? 2 : 1;
    if (r->distance >= TRACK_FINISH_M) {
        r->phase = TRACK_SEGMENT_FINISHED;
        return stop;
    }
    /* A missing or stale observation no longer faults the race. Keep the most
     * recent accepted curvature so the car can bridge short vision dropouts;
     * only a nonzero TRACK_VISION_LOST_STOP_MS turns prolonged loss into a stop. */
    new_frame = 0;
    road = 0;
    if (o) {
        new_frame = !r->have_sequence || o->sequence != r->last_sequence;
        r->last_sequence = o->sequence;
        r->have_sequence = 1;
        road = (o->valid_mask & VISION_CURVATURE_VALID) && o->confidence >= 0.35f;
        if (road) {
            r->last_curve = limit(o->curvature_inv_m, 15.0f);
            r->last_road_ms = now;
            r->have_curve = 1;
        }
        if (new_frame && o->element != r->held_element &&
            r->distance - r->last_action > 0.8f) r->held_element = 0;
    }
    r->degraded = 0;   /* warning applies only to ordinary line following */
    if (r->phase == TRACK_SEGMENT_TRACK) {
        along = r->distance - (r->lap - 1) * TRACK_LAP_M;
        in_window = TRACK_GAP_ENTRY_M <= 0 || fabsf(along - TRACK_GAP_ENTRY_M) < 1.0f;
        if (!road) {
            /* Only one crossing per circuit. Vision supplies the blue mouth
             * candidate, recent straight travel rejects ordinary corner losses.
             * Anything else stays in TRACK_SEGMENT_TRACK and coasts on last_curve. */
            if (o && o->element == VISION_ELEMENT_GAP && in_window &&
                r->distance - r->last_gap > TRACK_LAP_M * 0.75f &&
                (TRACK_GAP_ENTRY_M > 0 || r->straight_m >= 0.10f)) {
                enter(r, TRACK_SEGMENT_GAP, now);
                r->last_gap = r->distance;
            }
        } else if ((o->valid_mask & VISION_DISTANCE_VALID) &&
                   o->element != r->held_element && o->confidence >= 0.50f) {
            if (o->element == VISION_ELEMENT_YELLOW && o->distance_m <= 0.80f) {
                enter(r, TRACK_SEGMENT_BYPASS, now);
                r->obstacle_m = fmaxf(0.08f, o->distance_m);
                r->goal_m = r->obstacle_m + 0.42f;
            } else if (o->element == VISION_ELEMENT_RED && r->lap == 2 &&
                       o->distance_m <= 0.20f) {
                enter(r, TRACK_SEGMENT_RED, now);
                r->timing_stop = 0;
            }
            if (r->phase != TRACK_SEGMENT_TRACK) {
                r->held_element = o->element;
                r->last_action = r->distance;
            }
        }
    }
    s = r->distance - r->entry;
    if (r->phase == TRACK_SEGMENT_RED) {
        if (!stationary) r->timing_stop = 0;
        else if (!r->timing_stop) { r->timing_stop = 1; r->still_ms = now; }
        else if (now - r->still_ms >= 5000U) {
            r->phase = TRACK_SEGMENT_TRACK;
            r->last_road_ms = now;
        }
        if (now - r->entered_ms > 12000U) r->phase = TRACK_SEGMENT_FAULT;
        return stop;
    }
    heading = r->yaw - r->entry_yaw;
    if (r->phase == TRACK_SEGMENT_GAP) {
        if (s > 1.95f || now - r->entered_ms > 12500U) {
            r->phase = TRACK_SEGMENT_FAULT;
            return stop;
        }
        if (new_frame) {
            if (s >= 1.30f && road) ++r->clear_frames;
            else r->clear_frames = 0;
        }
        if (r->clear_frames < 3) return drive(0.22f, limit(-3.0f * heading, 1.2f));
        r->phase = TRACK_SEGMENT_TRACK;
        r->straight_m = 0;
        r->last_road_ms = now;
    }
    if (r->phase == TRACK_SEGMENT_BYPASS) {
        /* Follow an offset in the entry frame. Integrate measured forward
         * and side displacement; hold clearance across the obstacle itself. */
        r->offset += ds * sinf(heading);
        r->goal_m -= ds * cosf(heading);
        along = r->obstacle_m + 0.42f - r->goal_m;
        if (now - r->entered_ms > 12000U || fabsf(heading) > 1.55f || r->goal_m < -0.25f) {
            r->phase = TRACK_SEGMENT_FAULT;
            return stop;
        }
        target = TRACK_BYPASS_SIDE * TRACK_BYPASS_OFFSET_M;
        if (along > r->obstacle_m + 0.10f) target *= fmaxf(0, r->goal_m / 0.32f);
        if (r->goal_m <= 0.02f && road && fabsf(r->offset) < 0.075f) {
            r->phase = TRACK_SEGMENT_TRACK;
            r->last_road_ms = now;
        } else {
            error = atanf(5.5f * (target - r->offset));
            return drive(0.18f, limit(4.5f * (error - heading), 2.5f));
        }
    }
    /* Line lost while following the track: keep the last accepted curvature
     * and slow down. The LCD warning is raised through r->degraded. */
    r->degraded = r->phase == TRACK_SEGMENT_TRACK && !road;
#if TRACK_VISION_LOST_STOP_MS > 0
    if (r->degraded && r->have_curve &&
        now - r->last_road_ms >= TRACK_VISION_LOST_STOP_MS) {
        r->phase = TRACK_SEGMENT_FAULT;
        return stop;
    }
#endif
    curve = r->last_curve;
    r->straight_m = fabsf(curve) < 1.75f ? r->straight_m + fmaxf(0,ds) : 0;
    /* Bound lateral acceleration v²|k|, reduce speed on uncertain geometry. */
    speed = fminf(r->lap == 1 ? 0.55f : 0.32f, sqrtf(0.55f / (fabsf(curve) + 0.01f)));
    if (!road) speed = fminf(speed, TRACK_HOLD_SPEED_MPS);
    /* o is NULL when no fresh observation exists; the hold speed above already
     * bounds the command, so element-specific limits apply only to live frames. */
    if (o && (o->confidence < 0.65f || o->element == VISION_ELEMENT_RING ||
              o->element == VISION_ELEMENT_CROSS)) speed = fminf(speed, 0.24f);
    if (o && o->element == VISION_ELEMENT_RED && r->lap == 2) speed = fminf(speed, 0.20f);
    speed = fminf(speed, sqrtf(fmaxf(0.0025f, 0.8f * (TRACK_FINISH_M - r->distance))));
    return drive(speed, speed * curve);
}
