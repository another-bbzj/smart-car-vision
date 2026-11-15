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
    /*
     * 已完赛或已故障：直接返回双零转速，不再改动赛况；
     * 上层看到 RACE_FINISHED 会停车结算，看到 RACE_FAULT 会锁存故障。
     */
    if (r->phase == RACE_FINISHED || r->phase == RACE_FAULT) return stop;
    /*
     * 观测或周期非法：没有视觉帧，或 dt 不在 1~15.1 ms 内时立即置 RACE_FAULT；
     * 返回零转速，避免用坏数据继续积分里程和航向。
     */
    if (!o || !(dt >= 0.001f && dt <= 0.0151f)) {
        r->phase = RACE_FAULT;
        return stop;
    }
    /*
     * 航向融合：陀螺仪有效时按 85% 陀螺 + 15% 差速更新，否则退回纯差速；
     * ds 是双轮平均前进距离（m），累加进 distance，圈数和元素判断都依赖它。
     */
    if (gyro_ok && isfinite(gyro)) dyaw = 0.85f * gyro * dt + 0.15f * dyaw;
    r->yaw += dyaw;
    r->distance += ds;
    /*
     * 两圈赛制：超过单圈里程后 lap=2；红牌只在第二圈生效，
     * 巡线限速也从第一圈的 0.55 m/s 降到第二圈的 0.32 m/s。
     */
    r->lap = r->distance >= RACE_LAP_M ? 2 : 1;
    /*
     * 跑满两圈立即置 RACE_FINISHED 并返回零转速；
     * 上层 race_tick 随后停车，并开始收集四块数独瓦片。
     */
    if (r->distance >= RACE_FINISH_M) {
        r->phase = RACE_FINISHED;
        return stop;
    }
    /*
     * 帧去重：只有视觉帧序号变化才算新帧；
     * 后续元素识别和缺口清帧都按新帧计数，避免同一帧在多个 5 ms 周期里重复触发。
     */
    new_frame = !r->have_sequence || o->sequence != r->last_sequence;
    r->last_sequence = o->sequence;
    r->have_sequence = 1;
    /*
     * 道路可用：曲率位有效且置信度不低于 0.35 才算可用；
     * 不可用时巡线只允许走缺口分支，其余情况直接故障，不按旧曲率盲走。
     */
    road = (o->valid_mask & VISION_CURVATURE_VALID) && o->confidence >= 0.35f;
    /*
     * 元素解锁：出现新元素、且已离开上次动作点超过 0.8 m 时清除去重标记，
     * 允许同一元素在下一个位置或下一圈被重新识别。
     */
    if (new_frame && o->element != r->held_element &&
        r->distance - r->last_action > 0.8f) r->held_element = 0;
    /*
     * 巡线相位：根据视觉元素决定进入缺口、绕障或红牌；
     * along 是本圈已行驶里程，用于缺口进入窗口判断。
     */
    if (r->phase == RACE_TRACK) {
        /*
         * 缺口窗口：RACE_GAP_ENTRY_M 为 0 时不看里程，只按视觉元素判定；
         * 大于 0 时要求 along 落在窗口 ±1 m 内，才允许进入缺口相位。
         */
        along = r->distance - (r->lap - 1) * RACE_LAP_M;
        in_window = RACE_GAP_ENTRY_M <= 0 || fabsf(along - RACE_GAP_ENTRY_M) < 1.0f;
        /*
         * 视觉判定无路：仅当元素为蓝底缺口、在进入窗口内、距上次缺口超过 0.75 圈，
         * 且最近有 0.1 m 直行（或未启用里程窗口）时进入缺口相位；
         * 否则锁存 RACE_FAULT，防止丢线后继续按旧曲率行驶。
         */
        if (!road) {
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
            /*
             * 元素动作：必须同时具备距离量、元素与上次处理的不同、置信度不低于 0.5，
             * 避免远处或模糊元素提前触发动作。
             * 黄色绕障：障碍距离不超过 0.8 m 时进入绕障相位，
             * 记录障碍位置并设定 0.42 m 的绕行目标长度。
             */
            if (o->element == VISION_ELEMENT_YELLOW && o->distance_m <= 0.80f) {
                enter(r, RACE_BYPASS, now);
                r->obstacle_m = fmaxf(0.08f, o->distance_m);
                r->goal_m = r->obstacle_m + 0.42f;
            } else if (o->element == VISION_ELEMENT_RED && r->lap == 2 &&
                       o->distance_m <= 0.20f) {
                /*
                 * 第二圈红灯：距离不超过 0.2 m 时进入红牌相位，清零静止计时准备停车等待。
                 */
                enter(r, RACE_RED, now);
                r->timing_stop = 0;
            }
            /*
             * 记住已处理的元素和动作位置：0.8 m 内不再响应同一元素，
             * 防止元素在连续多帧中被反复触发。
             */
            if (r->phase != RACE_TRACK) {
                r->held_element = o->element;
                r->last_action = r->distance;
            }
        }
    }
    /*
     * s 为进入当前相位后的位移（m），红牌、缺口和绕障都用它判断是否超限。
     */
    s = r->distance - r->entry;
    /*
     * 红牌相位：非静止就清零计时；静止满 5 s 回到巡线；
     * 进入超过 12 s 仍无法继续则判故障。期间始终返回零转速，车辆保持停止。
     */
    if (r->phase == RACE_RED) {
        if (!stationary) r->timing_stop = 0;
        else if (!r->timing_stop) { r->timing_stop = 1; r->still_ms = now; }
        else if (now - r->still_ms >= 5000U) r->phase = RACE_TRACK;
        if (now - r->entered_ms > 12000U) r->phase = RACE_FAULT;
        return stop;
    }
    /*
     * 相位内航向误差（rad）：相对进入相位时的航向，
     * 缺口回正和绕障横向修正都以它为基准。
     */
    heading = r->yaw - r->entry_yaw;
    /*
     * 缺口相位：位移超过 1.95 m 或 12.5 s 未脱困即判故障；
     * 新帧且位移≥1.3 m 时看到有效道路才累加清帧，连续 3 帧后回到巡线；
     * 未满足前以 0.22 m/s 直行，并按 -3×航向误差（限 1.2 rad/s）回正。
     */
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
    /*
     * 绕障相位：用 ds 在进入坐标系中积分横向偏移 offset 和剩余里程 goal_m；
     * 超时 12 s、航向偏差>1.55 rad 或目标越过 0.25 m 即判故障。
     * 目标横移 ±0.18 m，接近障碍尾部时按剩余里程收敛回中心线；
     * 剩余≤0.02 m、道路有效且 |offset|<0.075 m 时回到巡线，
     * 否则以 0.18 m/s、4.5 倍航向误差（限 2.5 rad/s）绕行。
     */
    if (r->phase == RACE_BYPASS) {
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
    /*
     * 缺口/绕障都已结束仍无有效道路：视觉持续失效，锁存故障并停车。
     */
    if (!road) { r->phase = RACE_FAULT; return stop; }
    /*
     * 曲率限幅到 ±15 1/m，过滤视觉异常值；
     * |曲率|<1.75 时累计直行里程，供缺口入口判定使用。
     */
    curve = limit(o->curvature_inv_m, 15.0f);
    r->straight_m = fabsf(curve) < 1.75f ? r->straight_m + fmaxf(0,ds) : 0;
    /*
     * 速度上限：第一圈 0.55 m/s、第二圈 0.32 m/s；
     * 同时按 v²·|k|≈0.55 m/s² 限制横向加速度，弯道自动减速。
     */
    speed = fminf(r->lap == 1 ? 0.55f : 0.32f, sqrtf(0.55f / (fabsf(curve) + 0.01f)));
    /*
     * 置信度低于 0.65，或遇到环岛/十字等几何不确定元素时，再降到 0.24 m/s。
     */
    if (o->confidence < 0.65f || o->element == VISION_ELEMENT_RING ||
        o->element == VISION_ELEMENT_CROSS) speed = fminf(speed, 0.24f);
    /*
     * 第二圈看到红灯再降到 0.20 m/s，保证能停进 0.2 m 的停车判定区。
     */
    if (o->element == VISION_ELEMENT_RED && r->lap == 2) speed = fminf(speed, 0.20f);
    /*
     * 终点收敛：按剩余里程的平方根限速（减速度约 0.8 m/s²，最低 0.05 m/s），
     * 避免冲过终点线后无路可走。
     */
    speed = fminf(speed, sqrtf(fmaxf(0.0025f, 0.8f * (RACE_FINISH_M - r->distance))));
    /*
     * 输出双轮目标转速：横摆角速度 = 前进速度 × 曲率；
     * drive() 内部再按 CAR_RPM_LIMIT 等比例限幅。
     */
    return drive(speed, speed * curve);
}
