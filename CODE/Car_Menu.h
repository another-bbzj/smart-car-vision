#ifndef CAR_MENU_H
#define CAR_MENU_H
#include "My_Key.h"
#include "Car_Core.h"
#include "Car_Race.h"
#include "Car_Params.h"

typedef enum { MENU_RACE, MENU_SENSORS, MENU_LEFT_PID, MENU_RIGHT_PID, MENU_GROUPS } MenuGroup;
typedef enum { START_READY, START_STOP_FIRST, START_MOVING, START_RESULT,
               START_NO_VISION, START_NO_ROAD, START_NEW_SESSION } StartCondition;
typedef enum { MENU_IDLE, MENU_LAUNCH, MENU_SAVE_PARAMS } MenuAction;

typedef struct {
    uint32_t session, received_ms;
    uint16_t sequence;
    uint8_t count;
} CarMenuRoadConfirm;

typedef struct {
    uint8_t depth, group, item[MENU_GROUPS], feedback;
    uint8_t race_armed;  /* 1 = armed on Start race, auto-launch once vision is ON & ready */
    uint16_t race_rpm;   /* configured target race RPM (0 normalizes to RACE_DEFAULT_RPM) */
    unsigned candidate;  /* selected gain in hundredths, or candidate race RPM */
    CarParamsStatus params_status;
} CarMenu;

typedef struct {
    CarMenu menu;
    StartCondition start;
    uint8_t state, fault, autonomous, lap, phase, fresh, degraded;
    uint8_t launch_road_count;
    uint8_t session_valid, vision_disabled;
    CarVisionCameraStatus camera_status;
    uint8_t bt_connected;
    uint8_t imu_present, gyro_ok, samples;
    uint8_t tile_mask;
    uint8_t params_dirty, params_saved;
    uint16_t race_rpm;
    /* 视觉接收统计：联调时区分"没收到帧"和"收到但校验/业务被拒"。 */
    uint32_t vision_ok, vision_bad, vision_rejected;
    uint32_t uptime_s;
    uint32_t timing_fault_us;
    RaceFaultReason race_fault_reason;
    float distance, yaw_deg, gyro_dps, bias_dps;
    float speed[2], reference[2], pwm[2], gains[3];
    CarVisionObservation obs;
} MenuView;

void CarMenu_Init(CarMenu *menu);
uint16_t CarMenu_GetRaceRpm(const CarMenu *menu);
void CarMenu_Disarm(CarMenu *menu);
/* Pass only a fresh road observation. Initialize confirmation state to zero. */
StartCondition CarMenu_StartCondition(CarMenuRoadConfirm *confirm, const Car_Core *car,
                                      uint8_t still, uint8_t pending,
                                      const CarVisionObservation *road, uint32_t previous_session);
MenuAction CarMenu_Key(CarMenu *menu, Key_Event_Struct key, Car_Core *car, StartCondition start);
MenuAction CarMenu_PollArmed(CarMenu *menu, StartCondition start);
void CarMenu_ShowResult(CarMenu *menu);
/* Draw one LCD row at a time from one consistent foreground snapshot. */
void CarMenu_DrawRow(const MenuView *v, unsigned row, const uint8_t board[36], uint8_t result);
#endif
