#include "Car_Menu.h"
#include "My_Lcd.h"
#include "My_Lcd_FontZh.h"
#include <assert.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static char rows[UI_ROWS][UI_LINE_BYTES];

static int has_zh_glyph(uint16_t unicode)
{
    unsigned lo = 0U, hi = MY_LCD_ZH_COUNT;
    while (lo < hi) {
        unsigned mid = lo + ((hi - lo) >> 1);
        uint16_t key = s_my_lcd_zh_font[mid].unicode;
        if (key == unicode) return 1;
        if (key < unicode) lo = mid + 1U;
        else hi = mid;
    }
    return 0;
}

static unsigned utf8_display_cols_and_verify(const char *s)
{
    const uint8_t *p = (const uint8_t *)s;
    unsigned cols = 0U;
    while (*p != '\0') {
        if (*p < 0x80U) {
            assert(*p >= 32U && *p <= 126U);
            cols += 1U;
            p += 1;
        } else if ((p[0] & 0xE0U) == 0xC0U) {
            uint16_t cp;
            assert((p[1] & 0xC0U) == 0x80U);
            cp = (uint16_t)(((uint16_t)(p[0] & 0x1FU) << 6) | (uint16_t)(p[1] & 0x3FU));
            assert(has_zh_glyph(cp));
            cols += 2U;
            p += 2;
        } else if ((p[0] & 0xF0U) == 0xE0U) {
            uint16_t cp;
            assert((p[1] & 0xC0U) == 0x80U && (p[2] & 0xC0U) == 0x80U);
            cp = (uint16_t)(((uint16_t)(p[0] & 0x0FU) << 12) |
                            ((uint16_t)(p[1] & 0x3FU) << 6) |
                            (uint16_t)(p[2] & 0x3FU));
            assert(has_zh_glyph(cp));
            cols += 2U;
            p += 3;
        } else {
            assert(0 && "invalid UTF-8 byte sequence");
        }
    }
    return cols;
}

void My_Lcd_print(uint8_t row, UI_Style_Enum style, const char *fmt, ...)
{
    va_list args;
    int n;
    (void)style;
    assert(row < UI_ROWS);
    va_start(args,fmt);
    n = vsnprintf(rows[row],sizeof rows[row],fmt,args);
    va_end(args);
    assert(n >= 0 && n < (int)sizeof rows[row]);
    assert(utf8_display_cols_and_verify(rows[row]) <= UI_COLS);
}

static MenuAction press(CarMenu *m, Car_Core *c, unsigned key, StartCondition ready)
{
    Key_Event_Struct event = {(uint8_t)key,KEY_EVENT_PRESS};
    return CarMenu_Key(m,event,c,ready);
}

static MenuAction repeat_key(CarMenu *m, Car_Core *c, unsigned key, StartCondition ready)
{
    Key_Event_Struct event = {(uint8_t)key,KEY_EVENT_REPEAT};
    return CarMenu_Key(m,event,c,ready);
}

int main(void)
{
    Car_Core car;
    CarMenu menu = {0};
    CarMenuRoadConfirm confirm = {0};
    CarVisionObservation road = {0};
    MenuView view = {0};
    uint8_t board[36] = {0};
    unsigned row, group, item;
    float before;
    CarMenu_DrawRow(&view, 0, board, 0);
    assert(strstr(rows[0], "T0000000s"));
    view.uptime_s = 1U;
    view.menu.depth = 1U;
    view.menu.group = MENU_SENSORS;
    CarMenu_DrawRow(&view, 0, board, 0);
    assert(strstr(rows[0], "T0000001s"));
    view.menu.depth = 0U;
    view.menu.group = MENU_RACE;
    view.state = CAR_FAULT;
    view.fault = CAR_TIMING_FAULT;
    view.timing_fault_us = 20000U;
    CarMenu_DrawRow(&view, 8, board, 0);
    assert(strstr(rows[8], "dt=20000us"));
    view.fault = CAR_RACE_FAULT;
    for (int why = RACE_FAULT_PERIOD; why <= RACE_FAULT_RED; ++why) {
        view.race_fault_reason = (RaceFaultReason)why;
        CarMenu_DrawRow(&view, 8, board, 0);
        assert(strstr(rows[8], "故障 7"));
    }
    view.race_fault_reason = RACE_FAULT_VISION;
    CarMenu_DrawRow(&view, 8, board, 0);
    assert(strstr(rows[8], "视觉丢失"));
    view.state = CAR_STOP;
    view.fault = 0U;

    /* Verify font table is strictly sorted ascending for binary search. */
    assert(MY_LCD_ZH_COUNT > 0U);
    for (item = 1U; item < MY_LCD_ZH_COUNT; ++item) {
        assert(s_my_lcd_zh_font[item - 1U].unicode < s_my_lcd_zh_font[item].unicode);
    }

    Car_Init(&car);
    /* Zero-initialized CarMenu still normalizes to RACE_DEFAULT_RPM. */
    assert(CarMenu_GetRaceRpm(&menu) == RACE_DEFAULT_RPM);
    CarMenu_Init(&menu);
    assert(CarMenu_GetRaceRpm(&menu) == RACE_DEFAULT_RPM);

    /* PID submenu must offer a fourth entry for explicit persistent saving. */
    menu.depth = 1;
    menu.group = MENU_LEFT_PID;
    press(&menu,&car,KEY_UP,START_READY);
    assert(menu.item[MENU_LEFT_PID] == 3);
    assert(press(&menu,&car,KEY_OK,START_READY) == MENU_IDLE);
    assert(menu.depth == 2);
    assert(repeat_key(&menu,&car,KEY_OK,START_READY)==MENU_IDLE);
    car.speed[0]=3;
    assert(press(&menu,&car,KEY_OK,START_READY)==MENU_IDLE);
    car.speed[0]=0;
    car.state=CAR_RUN;
    assert(press(&menu,&car,KEY_OK,START_READY)==MENU_IDLE);
    car.state=CAR_STOP;
    assert(press(&menu,&car,KEY_OK,START_RESULT)==MENU_IDLE);
    assert(press(&menu,&car,KEY_OK,START_READY)==MENU_SAVE_PARAMS);
    assert(menu.params_status==PARAMS_BUSY);
    assert(press(&menu,&car,KEY_OK,START_READY)==MENU_IDLE);

    /* StartCondition: distinguish vision off, old session, no road, ready. */
    memset(&menu,0,sizeof menu);
    road.session = 12;
    road.valid_mask = VISION_CURVATURE_VALID;
    road.confidence = 0.9f;
#if CAR_VISION_ENABLE
    road.sequence = 65534U;
    road.received_ms = 10U;
    assert(CarMenu_StartCondition(&confirm,&car,1,0,&road,11) == START_NO_ROAD);
    for (unsigned tick = 0; tick < 4; ++tick)
        assert(CarMenu_StartCondition(&confirm,&car,1,0,&road,11) == START_NO_ROAD);
    road.sequence = 0U; /* Wrap and a non-road business sequence between observations. */
    road.received_ms = 20U;
    assert(CarMenu_StartCondition(&confirm,&car,1,0,&road,11) == START_NO_ROAD);
    road.sequence = 1U;
    road.received_ms = 30U;
    road.element = VISION_ELEMENT_CROSS;
    assert(CarMenu_StartCondition(&confirm,&car,1,0,&road,11) == START_NO_ROAD);
    road.element = VISION_ELEMENT_NONE;
    for (unsigned i = 1; i <= 3; ++i) {
        ++road.sequence;
        road.received_ms += 40U;
        assert(CarMenu_StartCondition(&confirm,&car,1,0,&road,11) ==
               (i == 3 ? START_READY : START_NO_ROAD));
    }
    assert(CarMenu_StartCondition(&confirm,&car,1,0,&road,11) == START_READY);
    ++road.sequence;
    road.received_ms += CAR_VISION_MAX_AGE_MS + 1U;
    assert(CarMenu_StartCondition(&confirm,&car,1,0,&road,11) == START_NO_ROAD);
    ++road.sequence;
    road.received_ms += CAR_VISION_MAX_AGE_MS;
    assert(CarMenu_StartCondition(&confirm,&car,1,0,&road,11) == START_NO_ROAD);
    ++road.sequence;
    road.received_ms += 10U;
    assert(CarMenu_StartCondition(&confirm,&car,1,0,&road,11) == START_READY);
    road.session = 13U;
    assert(CarMenu_StartCondition(&confirm,&car,1,0,&road,11) == START_NO_ROAD);
    road.session = 12U;
    road.valid_mask |= VISION_CURVATURE_ESTIMATED;
    assert(CarMenu_StartCondition(&confirm,&car,1,0,&road,11) == START_NO_ROAD);
    road.valid_mask = VISION_CURVATURE_VALID;
    assert(CarMenu_StartCondition(&confirm,&car,0,0,&road,11) == START_MOVING);
    assert(CarMenu_StartCondition(&confirm,&car,1,1,&road,11) == START_RESULT);
    assert(CarMenu_StartCondition(&confirm,&car,1,0,NULL,11) == START_NO_VISION);
    road.session = 0;
    assert(CarMenu_StartCondition(&confirm,&car,1,0,&road,11) == START_NO_VISION);
    road.session = 12;
    road.confidence = NAN;
    assert(CarMenu_StartCondition(&confirm,&car,1,0,&road,11) == START_NO_ROAD);
    road.valid_mask = 0;
    assert(CarMenu_StartCondition(&confirm,&car,1,0,&road,12) == START_NEW_SESSION);
    road.valid_mask = VISION_CURVATURE_VALID;
    road.confidence = 0.9f;
    assert(CarMenu_StartCondition(&confirm,&car,1,0,&road,12) == START_NEW_SESSION);
#else
    assert(CarMenu_StartCondition(&confirm,&car,1,0,&road,11) == START_NO_VISION);
    assert(CarMenu_StartCondition(&confirm,&car,0,0,&road,11) == START_MOVING);
    assert(CarMenu_StartCondition(&confirm,&car,1,1,&road,11) == START_RESULT);
#endif

    /* Race -> Race RPM (4th item in MENU_RACE): fine/coarse edit, BACK discards, OK applies. */
    CarMenu_Init(&menu);
    assert(press(&menu,&car,KEY_OK,START_READY) == MENU_IDLE && menu.depth == 1);
    press(&menu,&car,KEY_UP,START_READY);
    assert(menu.item[MENU_RACE] == 3);
    assert(press(&menu,&car,KEY_OK,START_READY) == MENU_IDLE && menu.depth == 2);
    assert(menu.candidate == RACE_DEFAULT_RPM);
    press(&menu,&car,KEY_UP,START_READY);
    assert(menu.candidate == RACE_DEFAULT_RPM + RACE_RPM_STEP_FINE);
    repeat_key(&menu,&car,KEY_UP,START_READY);
    assert(menu.candidate == RACE_DEFAULT_RPM + RACE_RPM_STEP_FINE + RACE_RPM_STEP_COARSE);
    press(&menu,&car,KEY_BACK,START_READY);
    assert(menu.depth == 1 && CarMenu_GetRaceRpm(&menu) == RACE_DEFAULT_RPM); /* BACK discards */
    press(&menu,&car,KEY_OK,START_READY);
    repeat_key(&menu,&car,KEY_DOWN,START_READY);
    press(&menu,&car,KEY_OK,START_READY);
    assert(CarMenu_GetRaceRpm(&menu) == RACE_DEFAULT_RPM - RACE_RPM_STEP_COARSE);
    /* Cannot apply Race RPM while running. */
    assert(Car_StartRace(&car));
    press(&menu,&car,KEY_UP,START_STOP_FIRST);
    press(&menu,&car,KEY_OK,START_STOP_FIRST);
    assert(CarMenu_GetRaceRpm(&menu) == RACE_DEFAULT_RPM - RACE_RPM_STEP_COARSE);
    Car_LocalStop(&car);

    /* Entering Start race must never start a motor. Long OK cannot launch. */
    CarMenu_Init(&menu);
    assert(press(&menu,&car,KEY_OK,START_READY) == MENU_IDLE && menu.depth == 1);
    assert(press(&menu,&car,KEY_OK,START_READY) == MENU_IDLE && menu.depth == 2);
    assert(repeat_key(&menu,&car,KEY_OK,START_READY) == MENU_IDLE);
    /* Direct RPM editing on Start race (fine +/-5, coarse +/-20, clamped 20..900). */
    press(&menu,&car,KEY_DOWN,START_NO_VISION);
    assert(CarMenu_GetRaceRpm(&menu) == RACE_DEFAULT_RPM - RACE_RPM_STEP_FINE);
    repeat_key(&menu,&car,KEY_UP,START_NO_VISION);
    assert(CarMenu_GetRaceRpm(&menu) == RACE_DEFAULT_RPM - RACE_RPM_STEP_FINE + RACE_RPM_STEP_COARSE);
    menu.race_rpm = RACE_MIN_RPM;
    press(&menu,&car,KEY_DOWN,START_NO_VISION);
    assert(CarMenu_GetRaceRpm(&menu) == RACE_MIN_RPM);
    menu.race_rpm = RACE_MAX_RPM;
    press(&menu,&car,KEY_UP,START_NO_VISION);
    assert(CarMenu_GetRaceRpm(&menu) == RACE_MAX_RPM);
    menu.race_rpm = 120U;

#if CAR_VISION_ENABLE
    /* Vision-gated auto-start: arm while vision off, stays stopped until START_READY. */
    assert(press(&menu,&car,KEY_OK,START_NO_VISION) == MENU_IDLE && car.state == CAR_STOP);
    assert(menu.race_armed == 1U);
    assert(CarMenu_PollArmed(&menu, START_NO_VISION) == MENU_IDLE && menu.race_armed == 1U);
    assert(CarMenu_PollArmed(&menu, START_NEW_SESSION) == MENU_IDLE && menu.race_armed == 1U);
    assert(CarMenu_PollArmed(&menu, START_NO_ROAD) == MENU_IDLE && menu.race_armed == 1U);
    /* Cancel via OK while still waiting, re-arm, cancel via BACK, re-arm, cancel via UP. */
    assert(press(&menu,&car,KEY_OK,START_NO_VISION) == MENU_IDLE && menu.race_armed == 0U);
    assert(press(&menu,&car,KEY_OK,START_NO_ROAD) == MENU_IDLE && menu.race_armed == 1U);
    assert(press(&menu,&car,KEY_BACK,START_NO_ROAD) == MENU_IDLE && menu.race_armed == 0U && menu.depth == 2U);
    assert(press(&menu,&car,KEY_OK,START_NEW_SESSION) == MENU_IDLE && menu.race_armed == 1U);
    assert(press(&menu,&car,KEY_UP,START_NEW_SESSION) == MENU_IDLE && menu.race_armed == 0U);
    /* Auto-disarm if car moves or faults while armed. */
    assert(press(&menu,&car,KEY_OK,START_NO_VISION) == MENU_IDLE && menu.race_armed == 1U);
    assert(CarMenu_PollArmed(&menu, START_MOVING) == MENU_IDLE && menu.race_armed == 0U);
    assert(CarMenu_PollArmed(&menu, START_READY) == MENU_IDLE);
    /* Re-arm and let three distinct normal-road observations trigger auto-launch. */
    assert(press(&menu,&car,KEY_OK,START_NO_VISION) == MENU_IDLE && menu.race_armed == 1U);
    for (unsigned i = 1; i <= 3; ++i) {
        ++road.sequence;
        road.received_ms += 40U;
        StartCondition start = CarMenu_StartCondition(&confirm,&car,1,0,&road,11);
        assert(CarMenu_PollArmed(&menu, start) == (i == 3 ? MENU_LAUNCH : MENU_IDLE));
    }
    assert(menu.race_armed == 0U && menu.item[MENU_RACE] == 1U && CarMenu_GetRaceRpm(&menu) == 120U);

    /* Direct launch when Vision is already ON & ready. */
    menu.item[MENU_RACE] = 0U;
    assert(press(&menu,&car,KEY_OK,START_READY) == MENU_LAUNCH);
    assert(menu.item[MENU_RACE] == 1U);
    assert(press(&menu,&car,KEY_OK,START_READY) == MENU_IDLE);
    assert(Car_StartRace(&car));
    assert(CarMenu_StartCondition(&confirm,&car,1,0,&road,11) == START_STOP_FIRST);
    Car_LocalStop(&car);
#else
    /* Build with CAR_VISION_ENABLE=0: Start race must never arm or launch. */
    assert(press(&menu,&car,KEY_OK,START_NO_VISION) == MENU_IDLE && menu.race_armed == 0U);
    assert(press(&menu,&car,KEY_OK,START_READY) == MENU_IDLE && menu.race_armed == 0U);
#endif

    memset(&menu,0,sizeof menu);
    press(&menu,&car,KEY_UP,START_READY);
    assert(menu.group == MENU_RIGHT_PID);
    press(&menu,&car,KEY_UP,START_READY);
    assert(menu.group == MENU_LEFT_PID);
    press(&menu,&car,KEY_OK,START_READY);
    press(&menu,&car,KEY_OK,START_READY);
    before = car.pid_speed[0].kp;
    press(&menu,&car,KEY_UP,START_READY);
    press(&menu,&car,KEY_BACK,START_READY);
    assert(car.pid_speed[0].kp == before); /* cancellation preserves gains */
    press(&menu,&car,KEY_OK,START_READY);
    press(&menu,&car,KEY_UP,START_READY);
    press(&menu,&car,KEY_OK,START_READY);
    assert(fabsf(car.pid_speed[0].kp-before-0.01f) < 0.0001f);
    before = car.pid_speed[0].kp;
    assert(Car_StartRace(&car));
    press(&menu,&car,KEY_UP,START_STOP_FIRST);
    press(&menu,&car,KEY_OK,START_STOP_FIRST);
    assert(car.pid_speed[0].kp == before);
    Car_LocalStop(&car);
    menu.candidate = 65535;
    press(&menu,&car,KEY_UP,START_READY);
    assert(menu.candidate == 65535);
    menu.candidate = 0;
    press(&menu,&car,KEY_DOWN,START_READY);
    assert(menu.candidate == 0);
    assert(Car_SetPid(&car,2,1,1,1) == 2);
    assert(Car_SetPid(&car,0,NAN,1,1) == 2);
    assert(car.pid_speed[0].kp == before);
    menu.race_armed = 1U;
    CarMenu_ShowResult(&menu);
    assert(menu.depth == 2 && menu.group == MENU_RACE && menu.item[MENU_RACE] == 2 && menu.race_armed == 0U);

    /* Render depth 0, depth 1, and every detail page verifying UTF-8, glyph coverage & <= 32 cols. */
    view.menu.depth = 0;
    for (row = 0; row < UI_ROWS; ++row) CarMenu_DrawRow(&view, row, board, 0);
    assert(strstr(rows[5], "摄像头") && strstr(rows[6], "香橙派") &&
           strstr(rows[7], "蓝牙上位机") && strstr(rows[7], "未连接"));
    view.camera_status = VISION_CAMERA_NO_FRAME;
    view.bt_connected = 1;
    for (row = 5; row <= 7; ++row) CarMenu_DrawRow(&view, row, board, 0);
#if CAR_VISION_ENABLE
    assert(strstr(rows[5], "无画面") && strstr(rows[6], "已连接"));
#endif
    assert(strstr(rows[7], "已连接"));
    view.menu.depth = 1;
    for (group = 0; group < MENU_GROUPS; ++group) {
        view.menu.group = (uint8_t)group;
        for (row = 0; row < UI_ROWS; ++row) CarMenu_DrawRow(&view, row, board, 0);
    }
    view.menu.depth = 2;
    view.samples = 100;
    view.imu_present = 1;
    view.gyro_ok = 1;
    view.race_rpm = 900;
    view.speed[0] = -1200.0f;
    view.speed[1] = 1200.0f;
    view.degraded = 1;
    view.lap = 1;
    view.fresh = 1;
    view.obs.valid_mask = VISION_CURVATURE_VALID;
    view.obs.confidence = 0.90f;
    for (group = 0; group < MENU_GROUPS; ++group) {
        view.menu.group = (uint8_t)group;
        for (item = 0; item < (group == MENU_SENSORS ? 3U : 4U); ++item) {
            view.menu.item[group] = (uint8_t)item;
            for (row = 0; row < UI_ROWS; ++row) CarMenu_DrawRow(&view, row, board, 2);
        }
    }
    /* Sweep all StartCondition states, armed/unarmed, and feedback lines on Start race. */
    view.menu.group = MENU_RACE;
    view.menu.item[MENU_RACE] = 0;
    view.degraded = 0;
    for (item = START_READY; item <= START_NEW_SESSION; ++item) {
        unsigned armed;
        view.start = (StartCondition)item;
        view.fresh = (item != START_NO_VISION);
        view.session_valid = (item != START_NO_VISION);
        for (armed = 0; armed <= 1; ++armed) {
            view.menu.race_armed = (uint8_t)armed;
            for (row = 0; row < UI_ROWS; ++row) CarMenu_DrawRow(&view, row, board, 0);
        }
    }
    view.menu.race_armed = 0;
    view.fresh = 0;
    view.session_valid = 1;
    for (row = 0; row < UI_ROWS; ++row) CarMenu_DrawRow(&view, row, board, 0);
    view.vision_disabled = 1;
    for (row = 0; row < UI_ROWS; ++row) CarMenu_DrawRow(&view, row, board, 0);
    assert(strstr(rows[3], "已关闭"));
    view.vision_disabled = 0;
#if CAR_VISION_ENABLE
    view.fresh = 1;
    view.start = START_NO_ROAD;
    view.obs.valid_mask = 0;
    CarMenu_DrawRow(&view,3,board,0);
    assert(strstr(rows[3],"无有效曲率"));
    view.obs.valid_mask = VISION_CURVATURE_VALID | VISION_CURVATURE_ESTIMATED;
    CarMenu_DrawRow(&view,3,board,0);
    assert(strstr(rows[3],"等待实测曲率"));
    view.obs.valid_mask = VISION_CURVATURE_VALID;
    view.obs.confidence = 0.2f;
    CarMenu_DrawRow(&view,3,board,0);
    assert(strstr(rows[3],"0.200 < 0.350"));
    view.obs.confidence = 0.9f;
    view.obs.element = VISION_ELEMENT_YELLOW;
    CarMenu_DrawRow(&view,3,board,0);
    assert(strstr(rows[3],"元素 5"));
    view.obs.element = VISION_ELEMENT_NONE;
    view.launch_road_count = 2;
    CarMenu_DrawRow(&view,3,board,0);
    assert(strstr(rows[3],"2/3"));
#endif

    /* Sweep Race status across all phases and states. */
    view.menu.group = MENU_RACE;
    view.menu.item[MENU_RACE] = 1;
    for (item = 0; item <= RACE_FAULT; ++item) {
        view.phase = (uint8_t)item;
        for (row = 0; row < UI_ROWS; ++row) CarMenu_DrawRow(&view, row, board, 0);
    }
    view.lap = 0;
    for (row = 0; row < UI_ROWS; ++row) CarMenu_DrawRow(&view, row, board, 0);

    /* Sweep Save parameters & PID feedback messages. */
    view.menu.group = MENU_RIGHT_PID;
    view.menu.item[MENU_RIGHT_PID] = 3;
    for (item = PARAMS_DEFAULTS; item <= PARAMS_STOP_FIRST; item++) {
        view.menu.params_status = (CarParamsStatus)item;
        for (row = 0; row < UI_ROWS; row++) CarMenu_DrawRow(&view, row, board, 0);
    }
    view.menu.params_status = PARAMS_SAVED;
    view.params_saved = 1; view.params_dirty = 1;
    CarMenu_DrawRow(&view, 4, board, 0);
    assert(strstr(rows[4], "未保存"));
    view.start = START_RESULT;
    CarMenu_DrawRow(&view, 5, board, 0);
    assert(strstr(rows[5], "数独求解"));
    view.start = START_READY;
    view.menu.group = MENU_SENSORS;
    view.menu.item[MENU_SENSORS] = 1;
    view.gyro_ok = 0;
    CarMenu_DrawRow(&view, 5, board, 0);
    assert(!rows[5][0]); /* stale gyro values are hidden */
    view.menu.group = MENU_RACE;
    view.menu.item[MENU_RACE] = 2;
    view.tile_mask = 5;
    CarMenu_DrawRow(&view, 2, board, SUDOKU_AMBIGUOUS);
    assert(strstr(rows[2], "多解"));
    CarMenu_DrawRow(&view, 3, board, SUDOKU_MISSING);
    assert(strstr(rows[3], "已收"));
    CarMenu_DrawRow(&view, 4, board, SUDOKU_MISSING);
    assert(strstr(rows[4], "缺失"));
    CarMenu_DrawRow(&view, 2, board, SUDOKU_INVALID);
    assert(strstr(rows[2], "冲突"));
    CarMenu_DrawRow(&view, 2, board, SUDOKU_WAITING);
    CarMenu_DrawRow(&view, 2, board, SUDOKU_LIMIT);
    CarMenu_DrawRow(&view, 2, board, SUDOKU_COLLECTING);
    /* Also verify Car_Hardware's flash saving message has all glyphs covered. */
    assert(utf8_display_cols_and_verify("保存中...请勿断电") <= UI_COLS);
    puts("PASS Chinese three-level menu, 20x20 glyph coverage, RPM config, vision-gated auto-start and rendering");
    return 0;
}
