#ifndef CAR_MENU_H
#define CAR_MENU_H
#include "My_Key.h"
#include "Car_Core.h"
#include "Car_Race.h"
#include "Car_Params.h"

/* 顶级菜单分组，同时决定显示顺序；MENU_GROUPS 只作计数和越界检查。 */
typedef enum {
    MENU_RACE,      /* 比赛页：启动比赛、赛况、数独结果 */
    MENU_SENSORS,   /* 传感器页：轮速、IMU、视觉观测 */
    MENU_LEFT_PID,  /* 左轮 PID 页：Kp/Ki/Kd 与保存参数 */
    MENU_RIGHT_PID, /* 右轮 PID 页：同上 */
    MENU_GROUPS     /* 分组个数，不显示 */
} MenuGroup;

/* “开始比赛”条件：由 CarMenu_StartCondition 判定，既用于页面提示，也决定 OK 能否启动。 */
typedef enum {
    START_READY,      /* 停车、静止、视觉有效且是新会话，可以启动 */
    START_STOP_FIRST, /* 当前不在 STOP，需先停车清故障 */
    START_MOVING,     /* 车轮仍在转，需等完全静止 */
    START_RESULT,     /* 上一场数独结果尚未处理完 */
    START_NO_ROAD,    /* 视觉观测缺失、曲率无效或置信度低于 0.35 */
    START_NEW_SESSION /* 视觉仍是上一场的会话，需香橙派重启视觉会话 */
} StartCondition;

/* 菜单处理完一个按键后返回给硬件层的动作。 */
typedef enum {
    MENU_IDLE,       /* 无动作，光标或候选值已在菜单对象内更新 */
    MENU_LAUNCH,     /* 请求启动比赛，调用者再执行 Car_StartRace 与 Race_Start */
    MENU_SAVE_PARAMS /* 请求保存参数，调用者置忙并执行 Flash 写入 */
} MenuAction;

/* 菜单光标与编辑状态；PIT 中断里更新，主循环取快照绘制。 */
typedef struct {
    uint8_t depth;                 /* 当前层级：0 组、1 条目、2 功能页 */
    uint8_t group;                 /* 当前组，见 MenuGroup */
    uint8_t item[MENU_GROUPS];     /* 每组各自的光标位置，切组后不丢失 */
    uint8_t feedback;              /* 最近一次 PID 应用结果：无/已应用/需停车/参数非法 */
    unsigned candidate;            /* 正在编辑的增益，放大 100 倍存成整数 */
    CarParamsStatus params_status; /* 参数保存流程状态，显示在保存页 */
} CarMenu;

/* 一帧界面快照：在关中断区内整体拷贝，绘制时不再读实时对象。 */
typedef struct {
    CarMenu menu;             /* 菜单光标、候选值和保存状态 */
    StartCondition start;     /* 启动比赛条件，显示在比赛页 */
    uint8_t state;            /* 整车状态 CAR_STOP/CAR_RUN/CAR_FAULT */
    uint8_t fault;            /* 故障码，FAULT 时显示在状态行 */
    uint8_t autonomous;       /* 1 表示当前由比赛状态机驱动 */
    uint8_t lap;              /* 当前圈数 1/2 */
    uint8_t phase;            /* 比赛相位，见 RacePhase */
    uint8_t fresh;            /* 1 表示 obs 是新鲜且通过校验的观测 */
    uint8_t imu_present;      /* 1 表示上电时检测到 MPU6050 */
    uint8_t gyro_ok;          /* 1 表示陀螺仪数据在 40 ms 内更新过 */
    uint8_t samples;          /* 零偏标定进度，满 100 后才输出角速度 */
    uint8_t tile_mask;        /* 四块数独瓦片接收位图，bit0..3 对应左上/右上/左下/右下 */
    uint8_t params_dirty;     /* 1 表示 RAM 增益与 Flash 保存值不同 */
    uint8_t params_saved;     /* 1 表示 Flash 中存在有效参数 */
    float distance;           /* 本场已行驶里程，单位米 */
    float yaw_deg;            /* 比赛累计航向，单位度 */
    float gyro_dps;           /* 当前 Z 轴角速度，单位 deg/s */
    float bias_dps;           /* 陀螺仪零偏估计，单位 deg/s */
    float speed[2];           /* 左右轮实测转速，单位 rpm */
    float reference[2];       /* 左右轮参考转速，单位 rpm */
    float pwm[2];             /* 左右轮输出占空比，单位 %（PWM 数值 / 100） */
    float gains[3];           /* 当前显示组的 Kp/Ki/Kd */
    CarVisionObservation obs; /* 最近一帧道路观测，用于视觉页 */
} MenuView;

StartCondition CarMenu_StartCondition(const Car_Core *car, uint8_t still, uint8_t pending,
                                      const CarVisionObservation *road, uint32_t previous_session);
MenuAction CarMenu_Key(CarMenu *menu, Key_Event_Struct key, Car_Core *car, StartCondition start);
void CarMenu_ShowResult(CarMenu *menu);
/* 使用同一份前台快照逐行绘制 LCD，避免一行内状态不一致。 */
void CarMenu_DrawRow(const MenuView *v, unsigned row, const uint8_t board[36], uint8_t result);
#endif
