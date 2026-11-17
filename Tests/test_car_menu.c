#include "Car_Menu.h"
#include "My_Lcd.h"
#include <assert.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static char rows[UI_ROWS][UI_COLS+1];
void My_Lcd_print(uint8_t row, UI_Style_Enum style, const char *fmt, ...)
{
    va_list args;
    (void)style;
    assert(row < UI_ROWS);
    va_start(args,fmt);
    assert(vsnprintf(rows[row],sizeof rows[row],fmt,args) <= UI_COLS);
    va_end(args);
}

static MenuAction press(CarMenu *m, Car_Core *c, unsigned key, StartCondition ready)
{
    Key_Event_Struct event = {(uint8_t)key,KEY_EVENT_PRESS};
    return CarMenu_Key(m,event,c,ready);
}

int main(void)
{
    Car_Core car;
    CarMenu menu = {0};
    CarVisionObservation road = {0};
    MenuView view = {0};
    uint8_t board[36] = {0};
    unsigned row, group, item;
    float before;
    Car_Init(&car);
    /* PID submenu must offer a fourth entry for explicit persistent saving. */
    menu.depth = 1;
    menu.group = MENU_LEFT_PID;
    press(&menu,&car,KEY_UP,START_READY);
    assert(menu.item[MENU_LEFT_PID] == 3);
    assert(press(&menu,&car,KEY_OK,START_READY) == MENU_IDLE);
    assert(menu.depth == 2);
    {
        Key_Event_Struct repeat = {KEY_OK,KEY_EVENT_REPEAT};
        assert(CarMenu_Key(&menu,repeat,&car,START_READY)==MENU_IDLE);
    }
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
    memset(&menu,0,sizeof menu);
    road.session = 12;
    road.valid_mask = VISION_CURVATURE_VALID;
    road.confidence = 0.9f;
    assert(CarMenu_StartCondition(&car,1,0,&road,11) == START_READY);
    assert(CarMenu_StartCondition(&car,0,0,&road,11) == START_MOVING);
    assert(CarMenu_StartCondition(&car,1,1,&road,11) == START_RESULT);
    assert(CarMenu_StartCondition(&car,1,0,NULL,11) == START_NO_ROAD);
    road.confidence = NAN;
    assert(CarMenu_StartCondition(&car,1,0,&road,11) == START_NO_ROAD);
    road.confidence = 0.9f;
    assert(CarMenu_StartCondition(&car,1,0,&road,12) == START_NEW_SESSION);

    /* Entering either menu level must never start a motor. Long OK cannot launch. */
    assert(press(&menu,&car,KEY_OK,START_READY) == MENU_IDLE && menu.depth == 1);
    assert(press(&menu,&car,KEY_OK,START_READY) == MENU_IDLE && menu.depth == 2);
    {
        Key_Event_Struct repeat = {KEY_OK,KEY_EVENT_REPEAT};
        assert(CarMenu_Key(&menu,repeat,&car,START_READY) == MENU_IDLE);
    }
    assert(press(&menu,&car,KEY_OK,START_NO_ROAD) == MENU_IDLE && car.state == CAR_STOP);
    assert(press(&menu,&car,KEY_OK,START_READY) == MENU_LAUNCH);
    assert(menu.item[MENU_RACE] == 1);
    assert(press(&menu,&car,KEY_OK,START_READY) == MENU_IDLE);
    assert(Car_StartRace(&car));
    assert(CarMenu_StartCondition(&car,1,0,&road,11) == START_STOP_FIRST);
    Car_LocalStop(&car);

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
    CarMenu_ShowResult(&menu);
    assert(menu.depth == 2 && menu.group == MENU_RACE && menu.item[MENU_RACE] == 2);

    /* Render every detail page through the actual formatter and fixed row width. */
    view.menu.depth = 2;
    view.samples = 100;
    view.imu_present = 1;
    for (group=0; group<MENU_GROUPS; ++group) {
        view.menu.group = (uint8_t)group;
        for (item=0; item<(group>=MENU_LEFT_PID?4U:3U); ++item) {
            view.menu.item[group] = (uint8_t)item;
            for (row=0; row<UI_ROWS; ++row) CarMenu_DrawRow(&view,row,board,2);
        }
    }
    view.menu.group=MENU_RIGHT_PID;
    view.menu.item[MENU_RIGHT_PID]=3;
    for(item=PARAMS_DEFAULTS;item<=PARAMS_STOP_FIRST;item++) {
        view.menu.params_status=(CarParamsStatus)item;
        for(row=0;row<UI_ROWS;row++) CarMenu_DrawRow(&view,row,board,0);
    }
    view.menu.params_status=PARAMS_SAVED;
    view.params_saved=1; view.params_dirty=1;
    CarMenu_DrawRow(&view,4,board,0);
    assert(strstr(rows[4],"Unsaved"));
    view.start=START_RESULT;
    CarMenu_DrawRow(&view,5,board,0);
    assert(strstr(rows[5],"Wait for result"));
    view.start=START_READY;
    view.menu.group = MENU_SENSORS;
    view.menu.item[MENU_SENSORS] = 1;
    view.gyro_ok = 0;
    CarMenu_DrawRow(&view,5,board,0);
    assert(!rows[5][0]); /* stale gyro values are hidden */
    view.menu.group = MENU_RACE;
    view.menu.item[MENU_RACE] = 2;
    view.tile_mask = 5;
    CarMenu_DrawRow(&view,2,board,SUDOKU_AMBIGUOUS);
    assert(strstr(rows[2],"Multiple"));
    CarMenu_DrawRow(&view,3,board,SUDOKU_MISSING);
    assert(strstr(rows[3],"received"));
    CarMenu_DrawRow(&view,4,board,SUDOKU_MISSING);
    assert(strstr(rows[4],"missing"));
    CarMenu_DrawRow(&view,2,board,SUDOKU_INVALID);
    assert(strstr(rows[2],"Conflicting"));
    puts("PASS three-level menu, launch gates, PID transaction, cancellation and rendering");
    return 0;
}
