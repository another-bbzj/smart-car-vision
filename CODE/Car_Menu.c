#include "Car_Menu.h"
#include "My_Lcd.h"
#include "Car_Config.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

/* Navigation follows task2: circular lists, per-page cursor, OK applies,
 * BACK discards. The third level holds the actual action or live data. */
static const char *const groups[] = {"比赛控制", "传感器监控", "左轮内环PID", "右轮内环PID"};
static const char *const race_items[] = {"发车启动", "比赛状态", "数独结果", "比赛转速设置"};
static const char *const sensor_items[] = {"左右轮转速", "陀螺仪 MPU6050", "视觉数据"};
static const char *const gain_names[] = {"比例 Kp", "积分 Ki", "微分 Kd", "保存参数到Flash"};
static const char *const conditions[] = {
    "条件就绪,可发车",
    "请先停车或清故障",
    "车轮仍在转动",
    "正在求解数独结果",
    "等待视觉功能开启",
    "等待识别有效赛道",
    "请重启视觉(--hello)"
};
static const char *const phases[] = {"巡线", "断路", "绕障", "红障停", "完赛", "故障"};
enum { FB_NONE, FB_APPLIED, FB_STOP, FB_INVALID };

static int vision_compiled(void)
{
#if CAR_VISION_ENABLE
    return 1;
#else
    return 0;
#endif
}

static unsigned group_item_count(unsigned group)
{
    return group == MENU_SENSORS ? 3U : 4U;
}

static uint16_t clamp_rpm(unsigned rpm)
{
    unsigned hi = (unsigned)CAR_RPM_LIMIT;
    if (hi > RACE_MAX_RPM) hi = RACE_MAX_RPM;
    if (rpm < RACE_MIN_RPM) return (uint16_t)RACE_MIN_RPM;
    if (rpm > hi) return (uint16_t)hi;
    return (uint16_t)rpm;
}

static uint16_t step_rpm(unsigned current, uint8_t key_id, uint8_t key_type)
{
    unsigned hi = (unsigned)CAR_RPM_LIMIT;
    unsigned step = key_type == KEY_EVENT_REPEAT ? RACE_RPM_STEP_COARSE : RACE_RPM_STEP_FINE;
    if (hi > RACE_MAX_RPM) hi = RACE_MAX_RPM;
    current = clamp_rpm(current);
    if (key_id == KEY_UP) {
        return (uint16_t)(current + step > hi ? hi : current + step);
    }
    return (uint16_t)(current < RACE_MIN_RPM + step ? RACE_MIN_RPM : current - step);
}

void CarMenu_Init(CarMenu *m)
{
    if (!m) return;
    memset(m, 0, sizeof(*m));
    m->race_rpm = RACE_DEFAULT_RPM;
}

uint16_t CarMenu_GetRaceRpm(const CarMenu *m)
{
    if (!m || m->race_rpm == 0U) return clamp_rpm(RACE_DEFAULT_RPM);
    return clamp_rpm(m->race_rpm);
}

void CarMenu_Disarm(CarMenu *m)
{
    if (m) m->race_armed = 0U;
}

StartCondition CarMenu_StartCondition(const Car_Core *car, uint8_t still, uint8_t pending,
                                      const CarVisionObservation *road, uint32_t previous_session)
{
    if (car->state != CAR_STOP) return START_STOP_FIRST;
    if (!still) return START_MOVING;
    if (pending) return START_RESULT;
    if (!vision_compiled()) return START_NO_VISION;
    if (!road || road->session == 0U) return START_NO_VISION;
    if (road->session == previous_session) return START_NEW_SESSION;
    if (!(road->valid_mask & VISION_CURVATURE_VALID) ||
        (road->valid_mask & VISION_CURVATURE_ESTIMATED) || !(road->confidence >= 0.35f))
        return START_NO_ROAD;
    return START_READY;
}

MenuAction CarMenu_PollArmed(CarMenu *m, StartCondition start)
{
    if (!m || !m->race_armed) return MENU_IDLE;
    if (!vision_compiled() || m->depth != 2U || m->group != MENU_RACE || m->item[MENU_RACE] != 0U ||
        start == START_STOP_FIRST || start == START_MOVING || start == START_RESULT) {
        m->race_armed = 0U;
        return MENU_IDLE;
    }
    if (start == START_READY) {
        m->race_rpm = CarMenu_GetRaceRpm(m);
        m->race_armed = 0U;
        m->item[MENU_RACE] = 1U;
        return MENU_LAUNCH;
    }
    return MENU_IDLE;
}

static const char *const *items(unsigned group)
{
    if (group == MENU_RACE) return race_items;
    if (group == MENU_SENSORS) return sensor_items;
    return gain_names;
}

MenuAction CarMenu_Key(CarMenu *m, Key_Event_Struct key, Car_Core *car, StartCondition start)
{
    uint8_t *cursor = m->depth == 0 ? &m->group : &m->item[m->group];
    unsigned count = m->depth == 0 ? MENU_GROUPS : group_item_count(m->group);
    unsigned item = m->item[m->group];
    int start_page = m->group == MENU_RACE && item == 0U;
    int rpm_page = m->group == MENU_RACE && item == 3U;
    int pid = m->group >= MENU_LEFT_PID;
    if (m->params_status == PARAMS_BUSY) return MENU_IDLE;
    m->feedback = FB_NONE;
    if (car->state != CAR_STOP || start == START_STOP_FIRST ||
        start == START_MOVING || start == START_RESULT) {
        m->race_armed = 0U;
    }
    if (key.id == KEY_BACK && key.type == KEY_EVENT_PRESS) {
        if (m->depth == 2U && start_page && m->race_armed) {
            m->race_armed = 0U;
        } else if (m->depth) {
            m->race_armed = 0U;
            --m->depth;
        }
    } else if (key.id == KEY_UP || key.id == KEY_DOWN) {
        if (m->depth < 2U) {
            *cursor = (uint8_t)((*cursor + (key.id == KEY_DOWN ? 1U : count - 1U)) % count);
        } else if (start_page) {
            if (m->race_armed) {
                if (key.id == KEY_UP) m->race_armed = 0U;
            } else if (car->state == CAR_STOP) {
                m->race_rpm = step_rpm(CarMenu_GetRaceRpm(m), key.id, key.type);
            }
        } else if (rpm_page) {
            unsigned base = m->candidate ? m->candidate : CarMenu_GetRaceRpm(m);
            m->candidate = step_rpm(base, key.id, key.type);
        } else if (pid && item < 3U) {
            /* Fine taps (Ki/Kd), coarse auto-repeat for larger changes. */
            unsigned step = key.type == KEY_EVENT_REPEAT ? 10U : 1U;
            if (key.id == KEY_UP)
                m->candidate = m->candidate > 65535U - step ? 65535U : m->candidate + step;
            else
                m->candidate = m->candidate < step ? 0U : m->candidate - step;
        }
    } else if (key.id == KEY_OK && key.type == KEY_EVENT_PRESS) {
        if (m->depth < 2U) {
            ++m->depth;
            if (m->depth == 2U && rpm_page) {
                m->candidate = CarMenu_GetRaceRpm(m);
            }
            if (m->depth == 2U && pid && item < 3U) {
                Car_PidInc_Struct *p = &car->pid_speed[m->group - MENU_LEFT_PID];
                float gains[] = {p->kp, p->ki, p->kd};
                m->candidate = (unsigned)(gains[item] * 100.0f + 0.5f);
            }
        } else if (pid && item == 3U) {
            if (car->state != CAR_STOP || !(fabsf(car->speed[0]) < 2 && fabsf(car->speed[1]) < 2)
                || start == START_RESULT) {
                m->params_status = PARAMS_STOP_FIRST;
            } else {
                m->params_status = PARAMS_BUSY;
                return MENU_SAVE_PARAMS;
            }
        } else if (pid) {
            unsigned wheel = m->group - MENU_LEFT_PID;
            Car_PidInc_Struct *p = &car->pid_speed[wheel];
            float gains[] = {p->kp, p->ki, p->kd};
            uint8_t result;
            gains[item] = m->candidate / 100.0f;
            result = Car_SetPid(car, wheel, gains[0], gains[1], gains[2]);
            m->feedback = result == 0 ? FB_APPLIED : (result == 1 ? FB_STOP : FB_INVALID);
        } else if (rpm_page) {
            if (car->state != CAR_STOP || m->race_armed) {
                m->feedback = FB_STOP;
            } else {
                m->race_rpm = clamp_rpm(m->candidate ? m->candidate : CarMenu_GetRaceRpm(m));
                m->candidate = m->race_rpm;
                m->feedback = FB_APPLIED;
            }
        } else if (start_page) {
            m->race_rpm = CarMenu_GetRaceRpm(m);
            if (!vision_compiled()) {
                m->race_armed = 0U;
                m->feedback = FB_INVALID;
            } else if (start == START_READY) {
                m->race_armed = 0U;
                m->item[MENU_RACE] = 1U;
                return MENU_LAUNCH;
            } else if (start == START_NO_VISION || start == START_NO_ROAD || start == START_NEW_SESSION) {
                m->race_armed = (uint8_t)!m->race_armed;
            } else {
                m->race_armed = 0U;
                m->feedback = FB_STOP;
            }
        }
    }
    return MENU_IDLE;
}

void CarMenu_ShowResult(CarMenu *m)
{
    m->depth = 2U;
    m->group = MENU_RACE;
    m->item[MENU_RACE] = 2U;
    m->race_armed = 0U;
    m->feedback = FB_NONE;
}

static float clamp_display_speed(float rpm)
{
    if (!isfinite(rpm)) return 0.0f;
    if (rpm > 999.0f) return 999.0f;
    if (rpm < -999.0f) return -999.0f;
    return rpm;
}

void CarMenu_DrawRow(const MenuView *v, unsigned row, const uint8_t board[36], uint8_t result)
{
    const CarMenu *m = &v->menu;
    unsigned group = m->group, item = m->item[group];
    const char *const *list = items(group);
    unsigned rpm = v->race_rpm ? clamp_rpm(v->race_rpm) : CarMenu_GetRaceRpm(m);
    int vision_en = vision_compiled() && !v->vision_disabled;
    UI_Style_Enum style = UI_STYLE_NORMAL;
    char s[UI_LINE_BYTES] = "";
    if (row == 0U) {
        style = UI_STYLE_TITLE;
        snprintf(s, sizeof s, "%s", m->depth == 0U ? "智能车菜单" : groups[group]);
    } else if (row == 9U) {
        style = UI_STYLE_HINT;
        if (v->state != CAR_STOP) {
            snprintf(s, sizeof s, "PE10: 急停/清故障");
        } else if (m->race_armed) {
            snprintf(s, sizeof s, "OK/返回/PE10: 取消等待");
        } else if (m->depth < 2U) {
            snprintf(s, sizeof s, "上下移动  OK进入  返回上级");
        } else if (group == MENU_RACE && item == 0U) {
            snprintf(s, sizeof s, "上下调RPM  OK发车  返回");
        } else if ((group == MENU_RACE && item == 3U) || (group >= MENU_LEFT_PID && item < 3U)) {
            snprintf(s, sizeof s, "上下调节  OK应用  返回放弃");
        } else if (group >= MENU_LEFT_PID && item == 3U) {
            snprintf(s, sizeof s, "OK写入Flash  PE9返回");
        } else {
            snprintf(s, sizeof s, "PE9: 返回");
        }
    } else if (row == 8U) {
        if (v->state == CAR_STOP && m->race_armed) {
            style = UI_STYLE_WARN;
            snprintf(s, sizeof s, "待视觉开启 %3uRPM 故障%u", rpm, (unsigned)v->fault);
        } else {
            style = (v->state == CAR_FAULT || v->degraded) ? UI_STYLE_WARN :
                    (v->state == CAR_RUN && v->autonomous ? UI_STYLE_GOOD : UI_STYLE_HINT);
            snprintf(s, sizeof s, "%s%s  故障 %u", v->state == CAR_STOP ? "停车" :
                     (v->state == CAR_FAULT ? "故障" : (v->autonomous ? "比赛" : "手动")),
                     v->degraded ? " 视觉丢失" : "", (unsigned)v->fault);
        }
    } else if (m->depth == 0U && row >= 5U) {
        const char *device;
        const char *status;
        int connected;
        int vision_online = vision_en && (v->camera_status != VISION_CAMERA_UNKNOWN || v->fresh);
        if (row == 5U) {
            device = "摄像头";
            connected = vision_en && (v->camera_status == VISION_CAMERA_STREAMING ||
                        (v->camera_status == VISION_CAMERA_UNKNOWN && v->fresh));
            status = !vision_en ? "已关闭" : connected ? "已连接" :
                     vision_online ? "无画面" : "无法确认";
        } else if (row == 6U) {
            device = "香橙派";
            connected = vision_online;
            status = !vision_en ? "已关闭" : connected ? "已连接" : "未连接";
        } else {
            device = "蓝牙上位机";
            connected = v->bt_connected;
            status = connected ? "已连接" : "未连接";
        }
        style = connected ? UI_STYLE_GOOD : UI_STYLE_WARN;
        snprintf(s, sizeof s, "%s: %s", device, status);
    } else if (m->depth < 2U) {
        unsigned n = row - 1U;
        if (n < (m->depth ? group_item_count(group) : (unsigned)MENU_GROUPS)) {
            if (n == (m->depth ? item : group)) style = UI_STYLE_SELECT;
            snprintf(s, sizeof s, "%s %s", style == UI_STYLE_SELECT ? ">" : " ",
                     m->depth ? list[n] : groups[n]);
        }
    } else if (row == 1U) {
        snprintf(s, sizeof s, "%s", list[item]);
    } else if (group == MENU_RACE && item == 0U) {
        if (row == 2U) {
            style = (!m->race_armed && v->state == CAR_STOP) ? UI_STYLE_EDIT : UI_STYLE_NORMAL;
            snprintf(s, sizeof s, "目标 %3u rpm (%.2f m/s)", rpm, (double)RACE_RPM_TO_MPS(rpm));
        } else if (row == 3U) {
            if (!vision_en) {
                style = UI_STYLE_WARN;
                snprintf(s, sizeof s, "视觉: 配置已关闭");
            } else if (!v->fresh) {
                style = UI_STYLE_WARN;
                snprintf(s, sizeof s, "%s",
                         v->session_valid ? "视觉: 数据过期(无新帧)" : "视觉: 未开启(等--hello)");
            } else if (v->start == START_NEW_SESSION) {
                style = UI_STYLE_WARN;
                snprintf(s, sizeof s, "视觉: 旧会话(需--hello)");
            } else if (v->start == START_NO_ROAD) {
                style = UI_STYLE_WARN;
                snprintf(s, sizeof s, "视觉: 已开启(等识别赛道)");
            } else {
                float conf = isfinite(v->obs.confidence) ? v->obs.confidence : 0.0f;
                if (conf < 0.0f) conf = 0.0f;
                if (conf > 1.0f) conf = 1.0f;
                style = UI_STYLE_GOOD;
                snprintf(s, sizeof s, "视觉: 已开启 置信度 %.2f", (double)conf);
            }
        } else if (row == 4U) {
            if (m->race_armed) {
                style = UI_STYLE_WARN;
                snprintf(s, sizeof s, "已预备: 视觉就绪自动开跑");
            } else {
                style = v->start == START_READY ? UI_STYLE_GOOD : UI_STYLE_NORMAL;
                snprintf(s, sizeof s, "%s", conditions[v->start]);
            }
        } else if (row == 5U) {
            if (m->race_armed) {
                style = UI_STYLE_EDIT;
                snprintf(s, sizeof s, "OK/返回: 取消等待发车");
            } else if (vision_en && v->start == START_READY) {
                style = UI_STYLE_EDIT;
                snprintf(s, sizeof s, "OK: 立即发车 (%u rpm)", rpm);
            } else if (vision_en && (v->start == START_NO_VISION ||
                       v->start == START_NO_ROAD || v->start == START_NEW_SESSION)) {
                style = UI_STYLE_EDIT;
                snprintf(s, sizeof s, "OK: 预备(等视觉开启开跑)");
            }
        } else if (row == 6U) {
            if (m->feedback == FB_STOP) {
                style = UI_STYLE_WARN;
                snprintf(s, sizeof s, "请先停车并保持车轮静止");
            } else if (m->feedback == FB_INVALID) {
                style = UI_STYLE_WARN;
                snprintf(s, sizeof s, "请开启 CAR_VISION_ENABLE=1");
            } else if (m->race_armed) {
                style = UI_STYLE_HINT;
                snprintf(s, sizeof s, "电机已锁停,视觉就绪即跑");
            } else if (v->state == CAR_STOP) {
                style = UI_STYLE_HINT;
                snprintf(s, sizeof s, "上下键: +-5 rpm(长按+-20)");
            }
        }
    } else if (group == MENU_RACE && item == 1U) {
        if (row == 2U) snprintf(s, sizeof s, "第%u圈  %s", (unsigned)v->lap, v->lap ? phases[v->phase] : "空闲");
        if (row == 3U) snprintf(s, sizeof s, "里程 %.2f m", v->distance);
        if (row == 4U) snprintf(s, sizeof s, "航向 %.1f deg", v->yaw_deg);
        if (row == 5U) snprintf(s, sizeof s, "数独子块 %u/4",
            (unsigned)((v->tile_mask & 1) + ((v->tile_mask >> 1) & 1) +
                       ((v->tile_mask >> 2) & 1) + ((v->tile_mask >> 3) & 1)));
        if (row == 6U && v->degraded) {
            style = UI_STYLE_WARN;
            snprintf(s, sizeof s, "视觉丢失: 保持上次曲率");
        }
        if (row == 7U) {
            float spd0 = clamp_display_speed(v->speed[0]);
            float spd1 = clamp_display_speed(v->speed[1]);
            snprintf(s, sizeof s, "设%3u L%4.0f R%4.0f rpm", rpm, (double)spd0, (double)spd1);
        }
    } else if (group == MENU_RACE && item == 3U) {
        unsigned edit_rpm = clamp_rpm(m->candidate ? m->candidate : rpm);
        if (row == 2U) snprintf(s, sizeof s, "当前 %3u rpm (%.2f m/s)", rpm, (double)RACE_RPM_TO_MPS(rpm));
        if (row == 3U) {
            style = UI_STYLE_EDIT;
            snprintf(s, sizeof s, "编辑 %3u rpm (%.2f m/s)", edit_rpm, (double)RACE_RPM_TO_MPS(edit_rpm));
        }
        if (row == 4U) snprintf(s, sizeof s, "范围 %u..%u rpm", (unsigned)RACE_MIN_RPM, (unsigned)RACE_MAX_RPM);
        if (row == 5U) snprintf(s, sizeof s, "上下+-5(连发+-20)  OK确认");
        if (row == 6U) snprintf(s, sizeof s, "%s", m->feedback == FB_APPLIED ? "已生效" :
                                (v->state != CAR_STOP || m->feedback == FB_STOP ? "请先停车再应用" :
                                "按返回键放弃修改"));
        if (row == 7U) snprintf(s, sizeof s, "发车页也可直接上下调速");
    } else if (group == MENU_RACE) {
        if (row == 2U && result != SUDOKU_SOLVED) {
            const char *status = "采集中: 完赛后自动求解";
            switch (result) {
            case SUDOKU_WAITING: status = "已停车: 等待末尾子块"; break;
            case SUDOKU_MISSING: status = "缺少子块,无法求解"; break;
            case SUDOKU_INVALID: status = "题面冲突或无解"; break;
            case SUDOKU_AMBIGUOUS: status = "存在多解,非唯一解"; break;
            case SUDOKU_LIMIT: status = "求解超出搜索上限"; break;
            default: break;
            }
            snprintf(s, sizeof s, "%s", status);
        }
        if (result != SUDOKU_SOLVED && row >= 3U && row <= 6U) {
            const char *const names[] = {"左上(Tag0/1)", "右上(Tag2/3)", "左下(Tag4/5)", "右下(Tag6/7)"};
            unsigned t = row - 3U;
            snprintf(s, sizeof s, "%s  %s", names[t], (v->tile_mask & (1U << t)) ? "已收" : "缺失");
        }
        if (result == SUDOKU_SOLVED && row >= 2U && row <= 7U) {
            const uint8_t *b = board + 6 * (row - 2U);
            snprintf(s, sizeof s, "  %u  %u  %u | %u  %u  %u", b[0], b[1], b[2], b[3], b[4], b[5]);
        }
    } else if (group >= MENU_LEFT_PID && item == 3U) {
        static const char *const messages[] = {
            "当前使用默认参数", "已从Flash加载参数", "已保存,重启自动恢复", "参数无变化,无需重写",
            "读写失败,请重试", "PID参数无效", "保存中...请勿断电", "请先停车并完成结果求解"
        };
        if (row == 2U) snprintf(s, sizeof s, "同时保存左右轮共6项PID");
        if (row == 3U) {
            style = m->params_status == PARAMS_ERROR || m->params_status == PARAMS_INVALID ? UI_STYLE_WARN : UI_STYLE_NORMAL;
            snprintf(s, sizeof s, "%s", messages[m->params_status]);
        }
        if (row == 4U) snprintf(s, sizeof s, "%s", !v->params_saved ? "Flash无已存参数" :
                                (v->params_dirty ? "有未保存的修改" : "内存与已存参数一致"));
        if (row == 5U && m->params_status != PARAMS_BUSY) {
            int ready = v->state == CAR_STOP && fabsf(v->speed[0]) < 2 && fabsf(v->speed[1]) < 2
                        && v->start != START_RESULT;
            style = ready ? UI_STYLE_EDIT : UI_STYLE_NORMAL;
            snprintf(s, sizeof s, "%s", ready ? "OK: 保存参数到Flash" :
                     (v->start == START_RESULT ? "请等待数独求解完成" : "请停车并保持轮静止"));
        }
        if (row == 6U) snprintf(s, sizeof s, "仅保存已按OK生效的参数");
    } else if (group >= MENU_LEFT_PID) {
        if (row == 2U) snprintf(s, sizeof s, "当前 %.2f", v->gains[item]);
        if (row == 3U) { style = UI_STYLE_EDIT; snprintf(s, sizeof s, "编辑 %.2f", m->candidate / 100.0); }
        if (row == 4U) snprintf(s, sizeof s, "上下+-  OK应用到内存");
        if (row == 6U) snprintf(s, sizeof s, "%s", m->feedback == FB_APPLIED ? "已生效" :
                                (v->state != CAR_STOP || m->feedback == FB_STOP ? "请先停车再应用" :
                                (m->feedback == FB_INVALID ? "参数无效" : "按返回键放弃修改")));
        if (row == 7U) snprintf(s, sizeof s, "调好后请进保存页写Flash");
    } else if (item == 0U) {
        if (row == 2U) snprintf(s, sizeof s, "     目标   实测   PWM%%");
        if (row == 3U || row == 4U) {
            unsigned w = row - 3U;
            snprintf(s, sizeof s, "%s %6.1f %6.1f %6.1f", w ? "右" : "左", v->reference[w], v->speed[w], v->pwm[w]);
        }
    } else if (item == 1U) {
        if (row == 2U) snprintf(s, sizeof s, "%s", !v->imu_present ? "MPU6050不可用" :
                                (v->samples < 100 ? "校零中: 请保持静止" : (v->gyro_ok ? "陀螺仪数据正常" : "陀螺仪过期/不可用")));
        if (row == 3U) snprintf(s, sizeof s, "零偏采样 %u/100", (unsigned)v->samples);
        if (row == 4U && v->samples == 100U) snprintf(s, sizeof s, "Z轴零偏 %+.2f deg/s", v->bias_dps);
        if (row == 5U && v->gyro_ok) snprintf(s, sizeof s, "Z轴速率 %+.2f deg/s", v->gyro_dps);
    } else if (!v->fresh) {
        if (row == 2U) snprintf(s, sizeof s, "无新鲜视觉帧");
    } else {
        if (row == 2U) snprintf(s, sizeof s, "会话 %lu", (unsigned long)v->obs.session);
        if (row == 3U) snprintf(s, sizeof s, "帧号 %u  置信度 %.2f", (unsigned)v->obs.sequence, v->obs.confidence);
        if (row == 4U && (v->obs.valid_mask & VISION_CURVATURE_VALID))
            snprintf(s, sizeof s, "曲率 %+.2f /m", v->obs.curvature_inv_m);
        if (row == 5U) snprintf(s, sizeof s, "元素 %u", (unsigned)v->obs.element);
    }
    /* 接收统计与是否新鲜无关：bad 表示 CRC/长度错，rejected 表示 CRC 对但
     * 会话、序号、年龄或字段越界被业务层拒绝。两者持续增长说明协议不匹配。 */
    if ((group == MENU_SENSORS && item == 2U) && m->depth >= 2U) {
        if (row == 6U) snprintf(s, sizeof s, "正常 %lu  错帧 %lu",
            (unsigned long)v->vision_ok, (unsigned long)v->vision_bad);
        if (row == 7U) snprintf(s, sizeof s, "拒绝 %lu", (unsigned long)v->vision_rejected);
    }
    My_Lcd_print((uint8_t)row, style, "%s", s);
}
