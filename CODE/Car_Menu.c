/*
 * 文件名称：Car_Menu.c
 * 模块功能：按键菜单的状态机和界面文案。接收按键事件以及车辆/PID/视觉/IMU 快照，按
 *           “组 -> 条目 -> 功能页”三级深度维护光标和 PID 候选值，返回“启动比赛”
 *           “保存参数”等动作。比赛页显示启动条件、赛况和数独结果；PID 页显示当前增益、
 *           候选值和保存状态；传感器页显示轮速、IMU 和视觉观测。绘制时把当前页面逐行
 *           交给 My_Lcd 显示。
 * 运行位置：按键处理在 PIT 中断的 race_tick() 中，刷屏在主循环 poll_menu() 中。
 * 关键约束：参数保存忙时忽略按键；应用 PID 只在停车状态生效；保存只在车轮静止且无
 *           待处理结果时发起。
 */

#include "Car_Menu.h"
#include "My_Lcd.h"
#include <stdio.h>
#include <math.h>

/* Navigation follows task2: circular lists, per-page cursor, OK applies,
 * BACK discards. The third level holds the actual action or live data. */
static const char *const groups[] = {"Race", "Sensors", "Left wheel PID", "Right wheel PID"};
static const char *const race_items[] = {"Start race", "Race status", "Sudoku result"};
static const char *const sensor_items[] = {"Wheel speeds", "IMU / MPU6050", "Vision"};
static const char *const gain_names[] = {"Kp", "Ki", "Kd", "Save parameters"};
static const char *const conditions[] = {"Ready", "Stop / clear fault first", "Wheels moving",
    "Preparing result", "Waiting for valid road", "Restart vision with --hello"};
static const char *const phases[] = {"Track","Gap","Bypass","Red stop","Finish","Fault"};
/* PID 应用结果反馈，显示在编辑页底部提示行。 */
enum {
    FB_NONE,    /* 本次没有尝试应用 */
    FB_APPLIED, /* 增益已写入该轮 PID */
    FB_STOP,    /* 车辆未停车，拒绝应用 */
    FB_INVALID  /* 增益非法，Car_SetPid 返回 2 */
};

/* 函数名称：CarMenu_StartCondition
 * 函数功能：判断“开始比赛”此刻是否可用：要求 STOP、车轮静止、没有待处理结果、视觉观测新鲜且曲率有效、置信度不低于 0.35，并区分是否为新的视觉会话。
 * 参数说明：car —— 控制对象；still —— 车轮静止标志；pending —— 有待处理完赛结果；road —— 最新视觉观测；previous_session —— 上一场使用的会话号。
 * 返回参数：启动条件枚举（READY/STOP_FIRST/MOVING/RESULT/NO_ROAD/NEW_SESSION）。
 * 备注信息：结果只用于菜单提示，真正启动仍由 CarMenu_Key 与 Car_StartRace 双重把关。
 */
StartCondition CarMenu_StartCondition(const Car_Core *car, uint8_t still, uint8_t pending,
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

/* 函数名称：items
 * 函数功能：按菜单组返回对应的条目名称表，供菜单逻辑与绘制共用同一份文案。
 * 参数说明：group —— 菜单组（比赛/传感器/左轮 PID/右轮 PID）。
 * 返回参数：条目名称数组指针。
 */
static const char *const *items(unsigned group)
{
    if (group == MENU_RACE) return race_items;
    if (group == MENU_SENSORS) return sensor_items;
    return gain_names;
}

/* 函数名称：CarMenu_Key
 * 函数功能：消费一个按键事件推进菜单状态机：移动/进入/返回光标；PID 页用 UP/DN 修改候选值；OK 应用 PID、请求保存参数或在比赛页返回启动动作。
 * 参数说明：m —— 菜单对象；key —— 按键事件；car —— 控制对象；start —— 本次启动条件。
 * 返回参数：MENU_IDLE / MENU_SAVE_PARAMS / MENU_LAUNCH。
 * 备注信息：参数保存忙时忽略按键；应用 PID 需停车并通过 Car_SetPid 校验；保存只在车轮静止且无待处理结果时发起。
 */
MenuAction CarMenu_Key(CarMenu *m, Key_Event_Struct key, Car_Core *car, StartCondition start)
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
                Car_Pid_Struct *p = &car->pid[m->group - MENU_LEFT_PID];
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
            Car_Pid_Struct *p = &car->pid[wheel];
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

/* 函数名称：CarMenu_ShowResult
 * 函数功能：把菜单直接跳到“比赛 → 数独结果”页，用于完赛或结果更新时强制切到结果视图。
 * 参数说明：m —— 菜单对象。
 * 返回参数：无。
 */
void CarMenu_ShowResult(CarMenu *m)
{
    m->depth = 2;
    m->group = MENU_RACE;
    m->item[MENU_RACE] = 2;
    m->feedback = FB_NONE;
}

/* 函数名称：CarMenu_DrawRow
 * 函数功能：按菜单深度/分组/选中项和车辆快照生成第 row 行文本，交给 My_Lcd_print 显示；标题、选中、告警、编辑等样式由当前状态决定。
 * 参数说明：v —— 视图快照；row —— 行号（0..UI_ROWS-1）；board —— 数独答案 36 格；result —— 求解状态。
 * 返回参数：无。
 * 备注信息：只读快照，不修改控制状态。
 */
void CarMenu_DrawRow(const MenuView *v, unsigned row, const uint8_t board[36], uint8_t result)
{
    const CarMenu *m = &v->menu;
    unsigned group = m->group, item = m->item[group];
    const char *const *list = items(group);
    UI_Style_Enum style = UI_STYLE_NORMAL;
    char s[UI_COLS+1] = "";
    if (row == 0) {
        style = UI_STYLE_TITLE;
        snprintf(s,sizeof s,"%s",m->depth == 0 ? "Smart car" : groups[group]);
    } else if (row == 9) {
        style = UI_STYLE_HINT;
        snprintf(s,sizeof s,"%s", v->state != CAR_STOP ? "PE10: STOP / clear fault" :
                 (m->depth < 2 ? "UP/DN move  OK enter  BACK exit" : "PE9: Back"));
    } else if (row == 8) {
        style = v->state == CAR_FAULT ? UI_STYLE_WARN : UI_STYLE_HINT;
        snprintf(s,sizeof s,"%s  Fault %u",v->state == CAR_STOP ? "STOP" :
                 (v->state == CAR_FAULT ? "FAULT" : (v->autonomous ? "RACE" : "MANUAL")), (unsigned)v->fault);
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
    My_Lcd_print((uint8_t)row,style,"%s",s);
}
