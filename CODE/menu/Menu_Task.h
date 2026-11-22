#ifndef MENU_TASK_H
#define MENU_TASK_H

/*
 * ============================================================================
 * 文件名称：Menu_Task.h
 * ============================================================================
 * 模块功能：板端菜单：三级导航（组 -> 项 -> 值/动作）、发车条件判定、
 *           内环 PID 在线修改与掉电保存入口、数独结果显示。
 *
 *   运行视角
 *     控制中断每拍把按键事件交给 Menu_Key()，它只返回“要做什么”
 *     （MENU_LAUNCH / MENU_SAVE_PARAMS），真正的发车、写 Flash 由平台层执行；
 *     主循环每 MENU_REFRESH_MS 刷新一行，10 行刷完即一整屏。
 *
 *   菜单结构（与实车一致）
 *     Race        ：Start race / Race status / Sudoku result
 *     Sensors     ：Wheel speeds / IMU / Vision
 *     Left wheel inner PID  ：Kp / Ki / Kd / Save parameters
 *     Right wheel inner PID ：同上
 * ============================================================================
 */

#include <stdint.h>

#include "Drv_Key.h"
#include "Task_Fsm.h"
#include "Track_Fsm.h"
#include "Pid_Storage.h"
#include "Proto_Vision.h"
#include "Sudoku.h"

/* ---------------------------------------------------------------------------
 * 菜单刷新节拍
 * 功能：主循环每隔多少毫秒重画一行，10 行刷完整屏约 1.2 s。
 * 取值：120 ms。调小会让屏幕更“跟手”，但会挤占陀螺采样与遥测发送。
 * ------------------------------------------------------------------------- */
#define MENU_REFRESH_MS          120U   /* 单行刷新间隔，毫秒 */

typedef enum { MENU_RACE, MENU_SENSORS, MENU_LEFT_PID, MENU_RIGHT_PID, MENU_GROUPS } MenuGroup;

/* 发车条件：只有 START_READY 才允许 OK 发车。 */
typedef enum { START_READY, START_STOP_FIRST, START_MOVING, START_RESULT,
               START_NO_ROAD, START_NEW_SESSION } StartCondition;

/* 菜单动作：返回给平台层执行。 */
typedef enum { MENU_IDLE, MENU_LAUNCH, MENU_SAVE_PARAMS } MenuAction;

typedef struct {
    uint8_t depth, group, item[MENU_GROUPS], feedback; /* 光标层级/组/项/反馈 */
    unsigned candidate;                                /* 正在编辑的增益，放大 100 倍 */
    CarParamsStatus params_status;                     /* 最近一次保存结果 */
} MenuState;

/* 一屏数据快照：平台层在临界区一次性填好，主循环逐行渲染。 */
typedef struct {
    MenuState menu;
    StartCondition start;
    uint8_t state, fault, motion, lap, phase, fresh, degraded; /* 整车/赛段/视觉状态 */
    uint8_t imu_present, gyro_ok, samples;                     /* 陀螺状态 */
    uint8_t tile_mask;                                         /* 已收到的数独子块位图 */
    uint8_t params_dirty, params_saved;                        /* 参数是否与 Flash 一致 */
    uint32_t vision_ok, vision_bad, vision_rejected;           /* 视觉接收统计 */
    float distance, yaw_deg, gyro_dps, bias_dps;
    float speed[2], reference[2], pwm[2], gains[3];
    CarVisionObservation obs;
} MenuView;

/*
 * 函数名称：Menu_StartCondition
 * 函数功能：综合整车状态、轮速、数独收尾、视觉路况与会话号，给出能否发车。
 * 参数说明：car —— 控制对象；still —— 轮速是否近似为零；
 *           pending —— 是否正在准备数独结果；road —— 最新道路观测；
 *           previous_session —— 上一场比赛的视觉会话号。
 * 返回参数：发车条件枚举。
 */
StartCondition Menu_StartCondition(const Car_Core *car, uint8_t still, uint8_t pending,
                                   const CarVisionObservation *road, uint32_t previous_session);

/*
 * 函数名称：Menu_Key
 * 函数功能：处理一个按键事件，推进光标或修改/应用增益，返回要执行的动作。
 * 参数说明：m —— 菜单状态；key —— 按键事件；car —— 控制对象；
 *           start —— 当前发车条件。
 * 返回参数：MENU_IDLE / MENU_LAUNCH / MENU_SAVE_PARAMS。
 * 备注信息：增益修改只在 STOP 生效；保存请求由平台层确认静止后执行。
 */
MenuAction Menu_Key(MenuState *m, Key_Event_Struct key, Car_Core *car, StartCondition start);

/*
 * 函数名称：Menu_ShowResult
 * 函数功能：把菜单位置跳到 “Race -> Sudoku result” 页。
 * 参数说明：m —— 菜单状态。
 * 返回参数：无。
 */
void Menu_ShowResult(MenuState *m);

/*
 * 函数名称：Menu_DrawRow
 * 函数功能：按当前快照画第 row 行（0..9）。
 * 参数说明：v —— 快照；row —— 行号；board —— 数独答案；result —— 求解状态。
 * 返回参数：无。
 */
void Menu_DrawRow(const MenuView *v, unsigned row, const uint8_t board[36], uint8_t result);

#endif
