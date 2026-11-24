#ifndef CAR_RACE_H
#define CAR_RACE_H
#include "Car_Vision.h"

/* Measure these on the assembled car. Route positions are centreline metres
 * from the actual start, increasing in the prescribed driving direction. */
#define RACE_RADIUS_M        0.0325f /* 65 mm diameter; verify loaded rollout */
#define RACE_TRACK_M         0.173f  /* (200 mm outside + 146 mm inside) / 2; verify by rotation */
#define RACE_DEFAULT_RPM     160U    /* ~0.545 m/s nominal cruise RPM */
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
#define RACE_GYRO_SIGN       1.0f   /* chassis CCW is positive */

/*
 * Vision-loss policy: a fresh frame without valid curvature, or no fresh frame
 * at all, no longer faults the race. The car keeps the last accepted curvature,
 * drives at the reduced speed below and raises only a warning on the LCD.
 * RACE_VISION_LOST_STOP_MS can restore a hard failsafe: set it to a nonzero
 * duration (milliseconds) to stop after that long without a valid road frame
 * while in RACE_TRACK. The gap phase is exempt because it is blind by design.
 */
#define RACE_VISION_HOLD_SPEED_MPS 0.18f
#define RACE_VISION_LOST_STOP_MS   0U

typedef enum { RACE_TRACK, RACE_GAP, RACE_BYPASS, RACE_RED,
               RACE_FINISHED, RACE_FAULT } RacePhase;
typedef struct {
    RacePhase phase;
    float distance, yaw, entry, entry_yaw, offset, obstacle_m, goal_m;
    float last_gap, last_action, straight_m;
    float last_curve;            /* last accepted curvature, 1/m */
    float target_rpm;            /* configured cruise wheel speed, rpm */
    uint32_t entered_ms, still_ms, last_road_ms;
    uint16_t last_sequence;
    uint8_t lap, clear_frames, held_element, have_sequence, timing_stop;
    uint8_t degraded, have_curve;
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
