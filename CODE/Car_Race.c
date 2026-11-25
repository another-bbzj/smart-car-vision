#include "Car_Race.h"
#include "Car_Config.h"
#include <math.h>
#include <string.h>

static float limit(float x, float bound) {
    return fmaxf(-bound, fminf(bound, x));
}

float Race_ClampRpm(float target_rpm) {
    float hi = fminf((float)RACE_MAX_RPM, CAR_RPM_LIMIT);
    if (!isfinite(target_rpm)) return (float)RACE_DEFAULT_RPM;
    if (target_rpm < (float)RACE_MIN_RPM) return (float)RACE_MIN_RPM;
    if (target_rpm > hi) return hi;
    return target_rpm;
}

void Race_Start(CarRace *r) {
    memset(r, 0, sizeof(*r));
    r->lap = 1;
    r->target_rpm = RACE_MPS_TO_RPM(0.55f);
    r->last_gap = r->last_action = -100.0f;
}

void Race_StartWithRpm(CarRace *r, float target_rpm) {
    Race_Start(r);
    r->target_rpm = Race_ClampRpm(target_rpm);
}

static RaceDrive drive(float v, float omega) {
    RaceDrive out;
    float factor = 60.0f / (6.2831853f * RACE_RADIUS_M);
    float half = 0.5f * RACE_TRACK_M * omega;
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
    if (peak > CAR_RPM_LIMIT) {
        out.rpm[0] *= CAR_RPM_LIMIT / peak;
        out.rpm[1] *= CAR_RPM_LIMIT / peak;
    }
    return out;
}

static void enter(CarRace *r, RacePhase phase, uint32_t now) {
    r->phase = phase;
    r->entry = r->distance;
    r->entry_yaw = r->yaw;
    r->offset = 0;
    r->entered_ms = now;
    r->last_road_ms = now;   /* hold timer restarts for every blind phase */
    r->clear_frames = 0;
}

RaceDrive Race_Step(CarRace *r, const CarVisionObservation *o, float dl,
                    float dr, float dt, float gyro, uint8_t gyro_ok,
                    uint8_t stationary, uint32_t now) {
    RaceDrive stop = {{0,0}};
    float ds = 0.5f * (dl + dr);
    float dyaw = (dr - dl) / RACE_TRACK_M;
    float s, curve, speed, heading, target, error, along, v_target;
    int road, measured_road, new_frame, in_window;
    if (r->phase == RACE_FINISHED || r->phase == RACE_FAULT) return stop;
    v_target = (isfinite(r->target_rpm) && r->target_rpm > 0.0f)
             ? RACE_RPM_TO_MPS(Race_ClampRpm(r->target_rpm)) : 0.55f;
    /* A bad control period still faults: it means the 5 ms loop itself failed. */
    if (!(dt >= 0.001f && dt <= 0.0151f)) {
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
    if (!r->have_step_time) {
        r->have_step_time = 1;
        r->last_road_ms = now;
    }
    new_frame = 0;
    road = 0;
    measured_road = 0;
    if (o) {
        new_frame = !r->have_sequence || o->sequence != r->last_sequence;
        r->last_sequence = o->sequence;
        r->have_sequence = 1;
        road = (o->valid_mask & VISION_CURVATURE_VALID) && o->confidence >= 0.35f;
        measured_road = road && !(o->valid_mask & VISION_CURVATURE_ESTIMATED);
        if (measured_road || (road && r->phase == RACE_TRACK)) {
            r->last_curve = limit(o->curvature_inv_m, 15.0f);
        }
        if (measured_road) r->last_measured_curve = r->last_curve;
        if (measured_road && new_frame) r->last_road_ms = now;
        if (new_frame && o->element != r->held_element &&
            r->distance - r->last_action > 0.8f) r->held_element = 0;
    }
    r->degraded = 0;   /* warning applies only to ordinary line following */
    if (r->phase == RACE_TRACK) {
        along = r->distance - (r->lap - 1) * RACE_LAP_M;
        in_window = RACE_GAP_ENTRY_M <= 0 || fabsf(along - RACE_GAP_ENTRY_M) < 1.0f;
        if (!road) {
            /* Only one crossing per circuit. Vision supplies the blue mouth
             * candidate, recent straight travel rejects ordinary corner losses.
             * Anything else stays in RACE_TRACK and coasts on last_curve. */
            if (o && o->element == VISION_ELEMENT_GAP && in_window &&
                r->distance - r->last_gap > RACE_LAP_M * 0.75f &&
                (RACE_GAP_ENTRY_M > 0 || r->straight_m >= 0.10f)) {
                enter(r, RACE_GAP, now);
                r->last_gap = r->distance;
            }
        } else if (measured_road && (o->valid_mask & VISION_DISTANCE_VALID) &&
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
        else if (now - r->still_ms >= 5000U) {
            r->phase = RACE_TRACK;
            r->last_road_ms = now;
        }
        if (now - r->entered_ms > 12000U) r->phase = RACE_FAULT;
        return stop;
    }
    heading = r->yaw - r->entry_yaw;
    if (r->phase == RACE_GAP) {
        float v_gap = fminf(0.22f, v_target);
        uint32_t gap_timeout_ms = (uint32_t)fmaxf(12500.0f, 2200.0f / fmaxf(v_gap, 0.05f));
        if (s > 1.95f || now - r->entered_ms > gap_timeout_ms) {
            r->phase = RACE_FAULT;
            return stop;
        }
        if (new_frame) {
            if (s >= 1.30f && measured_road) ++r->clear_frames;
            else r->clear_frames = 0;
        }
        if (r->clear_frames < 3) return drive(v_gap, limit(-3.0f * heading, 1.2f));
        r->phase = RACE_TRACK;
        r->straight_m = 0;
        r->last_road_ms = now;
    }
    if (r->phase == RACE_BYPASS) {
        float v_bypass = fminf(0.18f, v_target);
        uint32_t bypass_timeout_ms = (uint32_t)fmaxf(12000.0f, 1500.0f / fmaxf(v_bypass, 0.05f));
        /* Follow an offset in the entry frame. Integrate measured forward
         * and side displacement; hold clearance across the obstacle itself. */
        r->offset += ds * sinf(heading);
        r->goal_m -= ds * cosf(heading);
        along = r->obstacle_m + 0.42f - r->goal_m;
        if (now - r->entered_ms > bypass_timeout_ms || fabsf(heading) > 1.55f || r->goal_m < -0.25f) {
            r->phase = RACE_FAULT;
            return stop;
        }
        target = RACE_BYPASS_SIDE * RACE_BYPASS_OFFSET_M;
        if (along > r->obstacle_m + 0.10f) target *= fmaxf(0, r->goal_m / 0.32f);
        if (r->goal_m <= 0.02f && measured_road && fabsf(r->offset) < 0.075f) {
            r->phase = RACE_TRACK;
            r->last_road_ms = now;
        } else {
            error = atanf(5.5f * (target - r->offset));
            return drive(v_bypass, limit(4.5f * (error - heading), 2.5f));
        }
    }
    /* Line lost while following the track: keep the last accepted curvature
     * and slow down. The LCD warning is raised through r->degraded. */
    r->degraded = r->phase == RACE_TRACK && !measured_road;
#if RACE_VISION_LOST_STOP_MS > 0
    if (r->degraded && now - r->last_road_ms >= RACE_VISION_LOST_STOP_MS) {
        r->phase = RACE_FAULT;
        return stop;
    }
#endif
    curve = r->last_curve;
    r->straight_m = fabsf(r->last_measured_curve) < 1.75f ? r->straight_m + fmaxf(0,ds) : 0;
    /* Bound lateral acceleration v²|k|, reduce speed on uncertain geometry. */
    {
        float lap_v = (r->lap == 1)
                    ? v_target
                    : fminf(v_target, fmaxf(0.32f, v_target * (0.32f / 0.55f)));
        float a_lat = fmaxf(0.55f, 0.30f * v_target * v_target);
        speed = fminf(lap_v, sqrtf(a_lat / (fabsf(curve) + 0.01f)));
    }
    if (r->degraded) speed = fminf(speed, RACE_VISION_HOLD_SPEED_MPS);
    /* o is NULL when no fresh observation exists; the hold speed above already
     * bounds the command, so element-specific limits apply only to live frames. */
    if (o && (o->confidence < 0.65f || o->element == VISION_ELEMENT_RING ||
              o->element == VISION_ELEMENT_CROSS)) speed = fminf(speed, 0.24f);
    if (o && o->element == VISION_ELEMENT_RED && r->lap == 2) speed = fminf(speed, 0.20f);
    speed = fminf(speed, sqrtf(fmaxf(0.0025f, 0.8f * (RACE_FINISH_M - r->distance))));
    return drive(speed, speed * curve);
}
