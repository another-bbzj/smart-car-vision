#ifndef CAR_RACE_H
#define CAR_RACE_H
#include "Car_Vision.h"

#define RACE_RADIUS_M        0.0325f             /* 车轮半径（65 mm 直径），里程积分用；以带载滚动半径为准；须实车实测核对 */
#define RACE_TRACK_M         0.173f              /* 左右轮中心距，差速换算和航向积分用 */
#define RACE_LAP_M           55.061f             /* 单圈中心线长度，用于圈数判断和缺口进入窗口 */
#define RACE_GAP_ENTRY_M     0.0f                /* 缺口进入的里程窗口；0 表示不按里程，只按视觉元素判定 */
#define RACE_FINISH_M        (2.0f * RACE_LAP_M) /* 完赛里程：两圈 */
#define RACE_BYPASS_SIDE     1.0f                /* 绕障方向：+1 向左、-1 向右 */
#define RACE_BYPASS_OFFSET_M 0.18f               /* 绕障时相对中心线的横向偏移，单位米 */
#define RACE_GYRO_SIGN       1.0f                /* 陀螺仪符号：底盘逆时针为正；装反时改为 -1 */

/* 比赛相位，决定 Race_Step 的分支和界面显示。 */
typedef enum {
    RACE_TRACK,    /* 正常巡线：按视觉曲率控速控向 */
    RACE_GAP,      /* 蓝底缺口：直行通过并等待视觉恢复 */
    RACE_BYPASS,   /* 黄色绕障：按记录的偏移绕过障碍后回到巡线 */
    RACE_RED,      /* 第二圈红牌：原地等待静止满 5 s */
    RACE_FINISHED, /* 已完成两圈，停车并等待数独结算 */
    RACE_FAULT     /* 观测丢失或相位越界，已锁存故障，需人工停车 */
} RacePhase;

/* 一场比赛的运行状态，由 Race_Step 每周期更新。 */
typedef struct {
    RacePhase phase;        /* 当前相位 */
    float distance;         /* 累计里程，单位米，两圈连续累加 */
    float yaw;              /* 累计航向，单位弧度，陀螺仪与差速融合 */
    float entry;            /* 进入当前相位时的里程，用于计算相位内位移 */
    float entry_yaw;        /* 进入当前相位时的航向，用于相对航向控制 */
    float offset;           /* 绕障相位内相对进入方向的横向偏移，单位米 */
    float obstacle_m;       /* 进入绕障时记录的障碍距离，单位米 */
    float goal_m;           /* 绕障剩余里程，减到 0 且回正后回到巡线 */
    float last_gap;         /* 上次进入缺口的里程，限制每圈只触发一次 */
    float last_action;      /* 上次相位切换的里程，用于元素去抖 */
    float straight_m;       /* 最近一段直行的里程，缺口判定需要 */
    uint32_t entered_ms;    /* 进入当前相位的时刻，用于相位超时 */
    uint32_t still_ms;      /* 红牌相位中开始静止的时刻 */
    uint16_t last_sequence; /* 上一个已处理视觉帧的序号，用于去重 */
    uint8_t lap;            /* 当前圈数 1/2 */
    uint8_t clear_frames;   /* 缺口后连续看到有效道路的帧数，满 3 帧退出 */
    uint8_t held_element;   /* 已处理过的元素，防止同一元素重复触发 */
    uint8_t have_sequence;  /* 1 表示 last_sequence 有效 */
    uint8_t timing_stop;    /* 红牌相位中是否已开始计静止时间 */
} CarRace;

/* Race_Step 的输出：左右轮目标转速，由 Car_RaceWheels 下发。 */
typedef struct { float rpm[2]; } RaceDrive;

void Race_Start(CarRace *r);
RaceDrive Race_Step(CarRace *r, const CarVisionObservation *obs, float left_m,
                    float right_m, float dt, float gyro_rad_s, uint8_t gyro_ok,
                    uint8_t stationary, uint32_t now_ms);
/* 四块瓦片按阅读顺序排列；只接受 2x3 宫格下唯一解。 */
typedef enum {
    SUDOKU_COLLECTING = 0, /* 比赛进行中，持续收集瓦片 */
    SUDOKU_MISSING = 1,    /* 完赛后仍有瓦片缺失 */
    SUDOKU_SOLVED = 2,     /* 唯一解，答案已写入结果 */
    SUDOKU_INVALID = 3,    /* 给定数字冲突或无解 */
    SUDOKU_AMBIGUOUS = 4,  /* 多解，无法确定答案 */
    SUDOKU_LIMIT = 5,      /* 搜索节点超限，放弃求解 */
    SUDOKU_WAITING = 6     /* 已完赛停车，正在等最后一帧瓦片 */
} RaceSudokuStatus;
/* 求解失败时不改写 result；搜索有上限，保证界面不被长时间阻塞。 */
RaceSudokuStatus Race_SolveDetailed(const uint8_t tiles[4][9], uint8_t result[36]);
int Race_Solve(const uint8_t tiles[4][9], uint8_t result[36]);
#endif
