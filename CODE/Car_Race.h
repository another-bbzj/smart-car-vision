#ifndef CAR_RACE_H
#define CAR_RACE_H
#include "Car_Vision.h"

/* Measure these on the assembled car. Route positions are centreline metres
 * from the actual start, increasing in the prescribed driving direction. */
#define RACE_RADIUS_M        0.0325f /* 65 mm diameter; verify loaded rollout */
#define RACE_TRACK_M         0.173f  /* (200 mm outside + 146 mm inside) / 2; verify by rotation */
#define RACE_DEFAULT_RPM     300U    /* ~1.02 m/s nominal cruise speed */
#define RACE_MIN_RPM         20U     /* Safe low-speed test floor */
#define RACE_MAX_RPM         900U    /* Matches CAR_RPM_LIMIT (90% of 1000 rpm) */
#define RACE_RPM_STEP_FINE   5U      /* Single-press UP/DOWN step in menu */
#define RACE_RPM_STEP_COARSE 20U     /* Auto-repeat UP/DOWN step in menu */
#define RACE_RPM_TO_MPS(rpm) ((float)(rpm) * (6.2831853f * RACE_RADIUS_M / 60.0f))
#define RACE_MPS_TO_RPM(mps) ((float)(mps) * (60.0f / (6.2831853f * RACE_RADIUS_M)))
#define RACE_LAP_M           55.061f
#define RACE_GAP_ENTRY_M     0.0f   /* 0 = visually qualified entry, no course survey */
#define RACE_FINISH_M        (2.0f * RACE_LAP_M)
#define RACE_BYPASS_SIDE     1.0f   /* +1 left, -1 right */
#define RACE_BYPASS_OFFSET_M 0.18f
#define RACE_YELLOW_CONFIRM_FRAMES 3U
#define RACE_GYRO_SIGN       1.0f   /* chassis CCW is positive */

/* Slow down during a short dropout or single-edge estimate. Only a measured
 * road observation refreshes the ordinary tracking timeout. Gap is exempt. */
#define RACE_VISION_HOLD_SPEED_MPS 0.18f
#define RACE_VISION_LOST_STOP_MS   700U

typedef enum { RACE_TRACK, RACE_GAP, RACE_BYPASS, RACE_RED,
               RACE_FINISHED, RACE_FAULT } RacePhase;
typedef enum { RACE_FAULT_NONE, RACE_FAULT_PERIOD, RACE_FAULT_VISION,
               RACE_FAULT_GAP, RACE_FAULT_BYPASS, RACE_FAULT_RED } RaceFaultReason;
typedef struct {
    RacePhase phase;
    RaceFaultReason fault_reason;
    float distance, yaw, entry, entry_yaw, offset, obstacle_m, goal_m;
    float last_gap, last_action, straight_m;
    float last_curve;            /* last accepted curvature, 1/m */
    float last_measured_curve;   /* keep gap-entry geometry independent of single-edge estimates */
    float target_rpm;            /* configured cruise wheel speed, rpm */
    uint32_t entered_ms, still_ms, last_road_ms, last_yellow_ms;
    uint16_t last_sequence;
    uint8_t lap, clear_frames, held_element, have_sequence, timing_stop;
    uint8_t degraded, have_step_time, yellow_frames;
} CarRace;
typedef struct { float rpm[2]; } RaceDrive;
void Race_Start(CarRace *r);
void Race_StartWithRpm(CarRace *r, float target_rpm);
float Race_ClampRpm(float target_rpm);
RaceDrive Race_Step(CarRace *r, const CarVisionObservation *obs, float left_m,
                    float right_m, float dt, float gyro_rad_s, uint8_t gyro_ok,
                    uint8_t stationary, uint32_t now_ms);
/* Four tile indices in reading order. Accept only a unique 2x3-box solution. */
typedef enum {
    SUDOKU_COLLECTING = 0, SUDOKU_MISSING = 1, SUDOKU_SOLVED = 2,
    SUDOKU_INVALID = 3, SUDOKU_AMBIGUOUS = 4, SUDOKU_LIMIT = 5,
    SUDOKU_WAITING = 6
} RaceSudokuStatus;
/* Failure leaves result unchanged. Search is bounded to keep the UI responsive. */
RaceSudokuStatus Race_SolveDetailed(const uint8_t tiles[4][9], uint8_t result[36]);
int Race_Solve(const uint8_t tiles[4][9], uint8_t result[36]);
#endif
