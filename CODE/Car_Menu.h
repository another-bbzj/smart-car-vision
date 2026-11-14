#ifndef CAR_MENU_H
#define CAR_MENU_H
#include "My_Key.h"
#include "Car_Core.h"
#include "Car_Race.h"

typedef enum { MENU_RACE, MENU_SENSORS, MENU_LEFT_PID, MENU_RIGHT_PID, MENU_GROUPS } MenuGroup;
typedef enum { START_READY, START_STOP_FIRST, START_MOVING, START_RESULT,
               START_NO_ROAD, START_NEW_SESSION } StartCondition;
typedef enum { MENU_IDLE, MENU_LAUNCH } MenuAction;

typedef struct {
    uint8_t depth, group, item[MENU_GROUPS], feedback;
    unsigned candidate; /* selected gain in hundredths */
} CarMenu;

typedef struct {
    CarMenu menu;
    StartCondition start;
    uint8_t state, fault, autonomous, lap, phase, fresh;
    uint8_t imu_present, gyro_ok, samples;
    uint8_t tile_mask;
    float distance, yaw_deg, gyro_dps, bias_dps;
    float speed[2], reference[2], pwm[2], gains[3];
    CarVisionObservation obs;
} MenuView;

StartCondition CarMenu_StartCondition(const Car_Core *car, uint8_t still, uint8_t pending,
                                      const CarVisionObservation *road, uint32_t previous_session);
MenuAction CarMenu_Key(CarMenu *menu, Key_Event_Struct key, Car_Core *car, StartCondition start);
void CarMenu_ShowResult(CarMenu *menu);
/* Draw one LCD row at a time from one consistent foreground snapshot. */
void CarMenu_DrawRow(const MenuView *v, unsigned row, const uint8_t board[36], uint8_t result);
#endif
