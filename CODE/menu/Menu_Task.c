/*
 * ============================================================================
 * 文件名称：Menu_Task.c
 * ============================================================================
 * 模块功能
 * ----------------------------------------------------------------------------
 * 板端菜单的全部逻辑，界面只依赖 MenuView 快照，不直接读控制对象：
 *   1. Menu_StartCondition()：能否发车的 6 种条件；
 *   2. Menu_Key()           ：三级导航 + 增益编辑 + 动作返回；
 *   3. Menu_ShowResult()    ：跳转到数独结果页；
 *   4. Menu_DrawRow()       ：按行渲染，每行只算自己需要的字段。
 *
 * 设计要点
 *   - 控制中断只拿 Menu_Key() 的返回值，不在这里发车或写 Flash；
 *   - 增益编辑用“放大 100 倍的整数”做候选值，避免浮点累加误差；
 *   - 发车要求 START_READY：停车、静止、没有待处理数独、视觉路况有效、
 *     且视觉会话与上一场不同（防止一发车就吃上一场的旧帧）。
 *
 * 函数列表
 *   static const char *const *items(unsigned group)
 *   StartCondition Menu_StartCondition(...)
 *   MenuAction     Menu_Key(...)
 *   void           Menu_ShowResult(MenuState *m)
 *   void           Menu_DrawRow(const MenuView *v, unsigned row, const uint8_t board[36], uint8_t result)
 * ============================================================================
 */
#include "Menu_Task.h"
#include "Drv_Lcd.h"
#include <stdio.h>
#include <math.h>

/* Navigation follows task2: circular lists, per-page cursor, OK applies,
 * BACK discards. The third level holds the actual action or live data. */
static const char *const groups[] = {"Race", "Sensors", "Left wheel inner PID", "Right wheel inner PID"};
static const char *const race_items[] = {"Start race", "Race status", "Sudoku result"};
static const char *const sensor_items[] = {"Wheel speeds", "IMU / MPU6050", "Vision"};
static const char *const gain_names[] = {"Kp", "Ki", "Kd", "Save parameters"};
static const char *const conditions[] = {"Ready", "Stop / clear fault first", "Wheels moving",
    "Preparing result", "Waiting for valid road", "Restart vision with --hello"};
static const char *const phases[] = {"Track","Gap","Bypass","Red stop","Finish","Fault"};
enum { FB_NONE, FB_APPLIED, FB_STOP, FB_INVALID };

StartCondition Menu_StartCondition(const Car_Core *car, uint8_t still, uint8_t pending,
                                      const CarVisionObservation *road, uint32_t previous_session)
{
    if (car->state != CAR_STOP) return START_STOP_FIRST;
    if (!still) return START_MOVING;
    if (pending) return START_RESULT;
    if (!road || !(road->valid_mask & VISION_CURVATURE_VALID) || !(road->confidence >= 0.35f))
        return START_NO_ROAD;
    if (road->session == previous_session) return START_NEW_SESSION;
    return START_READY;
}

static const char *const *items(unsigned group)
{
    if (group == MENU_RACE) return race_items;
    if (group == MENU_SENSORS) return sensor_items;
    return gain_names;
}

MenuAction Menu_Key(MenuState *m, Key_Event_Struct key, Car_Core *car, StartCondition start)
{
    uint8_t *cursor = m->depth == 0 ? &m->group : &m->item[m->group];
    unsigned count = m->depth == 0 ? MENU_GROUPS : (m->group >= MENU_LEFT_PID ? 4U : 3U);
    unsigned item = m->item[m->group];
    int pid = m->group >= MENU_LEFT_PID;
    if (m->params_status == PARAMS_BUSY) return MENU_IDLE;
    m->feedback = FB_NONE;
    if (key.id == KEY_BACK && key.type == KEY_EVENT_PRESS) {
        if (m->depth) --m->depth;
    } else if (key.id == KEY_UP || key.id == KEY_DOWN) {
        if (m->depth < 2) {
            *cursor = (uint8_t)((*cursor + (key.id == KEY_DOWN ? 1U : count-1U)) % count);
        } else if (pid && item < 3) {
            /* Fine taps (Ki/Kd), coarse auto-repeat for larger changes. */
            unsigned step = key.type == KEY_EVENT_REPEAT ? 10U : 1U;
            if (key.id == KEY_UP)
                m->candidate = m->candidate > 65535U-step ? 65535U : m->candidate+step;
            else
                m->candidate = m->candidate < step ? 0U : m->candidate-step;
        }
    } else if (key.id == KEY_OK && key.type == KEY_EVENT_PRESS) {
        if (m->depth < 2) {
            ++m->depth;
            if (m->depth == 2 && pid && item < 3) {
                Car_PidInc_Struct *p = &car->pid_speed[m->group - MENU_LEFT_PID];
                float gains[] = {p->kp, p->ki, p->kd};
                m->candidate = (unsigned)(gains[item]*100.0f+0.5f);
            }
        } else if (pid && item == 3) {
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
            float gains[] = {p->kp,p->ki,p->kd};
            uint8_t result;
            gains[item] = m->candidate / 100.0f;
            result = Car_SetPid(car,wheel,gains[0],gains[1],gains[2]);
            m->feedback = result == 0 ? FB_APPLIED : (result == 1 ? FB_STOP : FB_INVALID);
        } else if (m->group == MENU_RACE && item == 0 && start == START_READY) {
            m->item[MENU_RACE] = 1;
            return MENU_LAUNCH;
        }
    }
    return MENU_IDLE;
}

void Menu_ShowResult(MenuState *m)
{
    m->depth = 2;
    m->group = MENU_RACE;
    m->item[MENU_RACE] = 2;
    m->feedback = FB_NONE;
}

void Menu_DrawRow(const MenuView *v, unsigned row, const uint8_t board[36], uint8_t result)
{
    const MenuState *m = &v->menu;
    unsigned group = m->group, item = m->item[group];
    const char *const *list = items(group);
    UI_Style_Enum style = UI_STYLE_NORMAL;
    char s[LCD_COLS+1] = "";
    if (row == 0) {
        style = UI_STYLE_TITLE;
        snprintf(s,sizeof s,"%s",m->depth == 0 ? "Smart car" : groups[group]);
    } else if (row == 9) {
        style = UI_STYLE_HINT;
        snprintf(s,sizeof s,"%s", v->state != CAR_STOP ? "PE10: STOP / clear fault" :
                 (m->depth < 2 ? "UP/DN move  OK enter  BACK exit" : "PE9: Back"));
    } else if (row == 8) {
        style = (v->state == CAR_FAULT || v->degraded) ? UI_STYLE_WARN : UI_STYLE_HINT;
        snprintf(s,sizeof s,"%s%s  Fault %u",v->state == CAR_STOP ? "STOP" :
                 (v->state == CAR_FAULT ? "FAULT" : (v->motion == CAR_MOTION_RACE ? "RACE" : "MANUAL")),
                 v->degraded ? " VISION LOST" : "", (unsigned)v->fault);
    } else if (m->depth < 2) {
        unsigned n = row-1;
        if (n < (m->depth ? (group >= MENU_LEFT_PID ? 4U : 3U) : (unsigned)MENU_GROUPS)) {
            if (n == (m->depth ? item : group)) style = UI_STYLE_SELECT;
            snprintf(s,sizeof s,"%s %s",style == UI_STYLE_SELECT ? ">" : " ",m->depth ? list[n] : groups[n]);
        }
    } else if (row == 1) {
        snprintf(s,sizeof s,"%s",list[item]);
    } else if (group == MENU_RACE && item == 0) {
        if (row == 3) snprintf(s,sizeof s,"%s",conditions[v->start]);
        if (row == 5 && v->start == START_READY) {
            style = UI_STYLE_EDIT;
            snprintf(s,sizeof s,"OK: start race");
        }
    } else if (group == MENU_RACE && item == 1) {
        if (row == 2) snprintf(s,sizeof s,"Lap %u   %s",(unsigned)v->lap,v->lap ? phases[v->phase] : "Idle");
        if (row == 3) snprintf(s,sizeof s,"Distance %.2f m",v->distance);
        if (row == 4) snprintf(s,sizeof s,"Race yaw %.1f deg",v->yaw_deg);
        if (row == 5) snprintf(s,sizeof s,"Tiles %u/4",
            (unsigned)((v->tile_mask&1)+((v->tile_mask>>1)&1)+((v->tile_mask>>2)&1)+((v->tile_mask>>3)&1)));
        if (row == 6 && v->degraded) {
            style = UI_STYLE_WARN;
            snprintf(s,sizeof s,"Vision lost: last curve");
        }
    } else if (group == MENU_RACE) {
        if (row == 2 && result != SUDOKU_SOLVED) {
            const char *status = "Collecting: solve after finish";
            switch (result) {
            case SUDOKU_WAITING: status = "Stopped: receiving last tiles"; break;
            case SUDOKU_MISSING: status = "Missing tiles"; break;
            case SUDOKU_INVALID: status = "Conflicting / unsolvable cells"; break;
            case SUDOKU_AMBIGUOUS: status = "Multiple solutions"; break;
            case SUDOKU_LIMIT: status = "Solver search limit"; break;
            default: break;
            }
            snprintf(s,sizeof s,"%s",status);
        }
        if (result != SUDOKU_SOLVED && row >= 3 && row <= 6) {
            const char *const names[] = {"TL (tags 0/1)","TR (tags 2/3)","BL (tags 4/5)","BR (tags 6/7)"};
            unsigned t = row-3;
            snprintf(s,sizeof s,"%s  %s",names[t],(v->tile_mask & (1U<<t)) ? "received" : "missing");
        }
        if (result == SUDOKU_SOLVED && row >= 2 && row <= 7) {
            const uint8_t *b = board+6*(row-2);
            snprintf(s,sizeof s,"  %u  %u  %u | %u  %u  %u", b[0],b[1],b[2],b[3],b[4],b[5]);
        }
    } else if (group >= MENU_LEFT_PID && item == 3) {
        static const char *const messages[] = {
            "Using defaults", "Loaded from Flash", "Saved / reboot ready", "Already saved",
            "Save/read failed: retry", "Invalid PID values", "Saving... keep power on", "Stop / finish result first"
        };
        if (row == 2) snprintf(s,sizeof s,"Saves BOTH wheels (6 gains)");
        if (row == 3) {
            style = m->params_status == PARAMS_ERROR || m->params_status == PARAMS_INVALID ? UI_STYLE_WARN : UI_STYLE_NORMAL;
            snprintf(s,sizeof s,"%s",messages[m->params_status]);
        }
        if (row == 4) snprintf(s,sizeof s,"%s",!v->params_saved ? "No saved parameters" :
                              (v->params_dirty ? "Unsaved changes" : "RAM matches saved parameters"));
        if (row == 5 && m->params_status != PARAMS_BUSY) {
            int ready = v->state == CAR_STOP && fabsf(v->speed[0]) < 2 && fabsf(v->speed[1]) < 2
                        && v->start != START_RESULT;
            style = ready ? UI_STYLE_EDIT : UI_STYLE_NORMAL;
            snprintf(s,sizeof s,"%s",ready ? "OK: save parameters" :
                     (v->start == START_RESULT ? "Wait for result processing" : "Stop and keep wheels still"));
        }
        if (row == 6) snprintf(s,sizeof s,"Only applied gains are saved");
    } else if (group >= MENU_LEFT_PID) {
        if (row == 2) snprintf(s,sizeof s,"Active %.2f",v->gains[item]);
        if (row == 3) { style = UI_STYLE_EDIT; snprintf(s,sizeof s,"Edit   %.2f",m->candidate/100.0); }
        if (row == 4) snprintf(s,sizeof s,"UP/DN +/-  OK apply (RAM)");
        if (row == 6) snprintf(s,sizeof s,"%s",m->feedback == FB_APPLIED ? "Applied" :
                               (v->state != CAR_STOP || m->feedback == FB_STOP ? "Stop to apply" :
                               (m->feedback == FB_INVALID ? "Invalid gain" : "BACK discards edits")));
        if (row == 7) snprintf(s,sizeof s,"Then use Save parameters");
    } else if (item == 0) {
        if (row == 2) snprintf(s,sizeof s,"       Ref    RPM    PWM%%");
        if (row == 3 || row == 4) {
            unsigned w = row-3;
            snprintf(s,sizeof s,"%c   %6.1f %6.1f %6.1f", w ? 'R' : 'L',v->reference[w],v->speed[w],v->pwm[w]);
        }
    } else if (item == 1) {
        if (row == 2) snprintf(s,sizeof s,"%s",!v->imu_present ? "MPU unavailable" :
                               (v->samples < 100 ? "Calibrating: keep still" : (v->gyro_ok ? "Gyro fresh" : "Gyro stale / unavailable")));
        if (row == 3) snprintf(s,sizeof s,"Zero samples %u/100",(unsigned)v->samples);
        if (row == 4 && v->samples == 100) snprintf(s,sizeof s,"Z bias %+.2f deg/s",v->bias_dps);
        if (row == 5 && v->gyro_ok) snprintf(s,sizeof s,"Z rate %+.2f deg/s",v->gyro_dps);
    } else if (!v->fresh) {
        if (row == 2) snprintf(s,sizeof s,"No fresh observation");
    } else {
        if (row == 2) snprintf(s,sizeof s,"Session %lu",(unsigned long)v->obs.session);
        if (row == 3) snprintf(s,sizeof s,"Frame %u  confidence %.2f",(unsigned)v->obs.sequence,v->obs.confidence);
        if (row == 4 && (v->obs.valid_mask & VISION_CURVATURE_VALID))
            snprintf(s,sizeof s,"Curvature %+.2f /m",v->obs.curvature_inv_m);
        if (row == 5) snprintf(s,sizeof s,"Element %u",(unsigned)v->obs.element);
    }
    /* 接收统计与是否新鲜无关：bad 表示 CRC/长度错，rejected 表示 CRC 对但
     * 会话、序号、年龄或字段越界被业务层拒绝。两者持续增长说明协议不匹配。 */
    if ((group == MENU_SENSORS && item == 2) && m->depth >= 2) {
        if (row == 6) snprintf(s,sizeof s,"OK %lu  bad %lu",
            (unsigned long)v->vision_ok,(unsigned long)v->vision_bad);
        if (row == 7) snprintf(s,sizeof s,"Rejected %lu",(unsigned long)v->vision_rejected);
    }
    Lcd_Print((uint8_t)row,style,"%s",s);
}
