#ifndef CAR_PURSUIT_H
#define CAR_PURSUIT_H

#include <stdint.h>

#include "Car_Steer.h"

/*
 * ============================================================================
 * 文件名称：Car_Pursuit.h
 * ============================================================================
 * 模块功能：纯跟踪（pure pursuit）路径跟随，移植自 HFK_STC32G144K 的
 *           alongLine()/setServoDuty() 并做差速底盘适配。
 *
 *   原实现：在 120x188 图像坐标中取预瞄点，计算
 *               d = atan2(2 * D * dy, dx^2 + dy^2)
 *           再查 tan 表得到转弯半径 r，按 sqrt(r*c4) 限制速度。
 *
 *   本实现：输入为视觉给出的车辆坐标系路径点（米，x 向右、y 向前），
 *           预瞄距离随车速变化（对应原实现按速度选 DTarget 的分段前瞻），
 *           输出曲率、舵机转角和曲率限速；CarPursuit_Wheels() 再用
 *           虚拟舵机模型把速度和转角换算成左右轮 rpm。
 *
 *   正方向约定：x 向右、y 向前；曲率、转角、横摆角速度均为左转为正，
 *   与 Car_Race/Car_Vision 的符号一致。
 *   本模块是纯算法，不访问外设；路径点须有限且至少 2 个。
 * ============================================================================
 */

/* 纯跟踪配置，来源：Car_Config.h 默认值或调用者覆盖。 */
typedef struct
{
    float wheelbase_m;           /* 虚拟轴距，单位米，用于转角/横摆换算 */
    float track_m;               /* 左右轮中心距，单位米，用于差速换算 */
    float wheel_radius_m;        /* 车轮半径，单位米，用于 rpm 换算 */
    float lookahead_min_m;       /* 最小预瞄距离，单位米 */
    float lookahead_per_mps;     /* 每 1 m/s 增加的预瞄距离，单位秒 */
    float lookahead_max_m;       /* 最大预瞄距离，单位米 */
    float steer_limit_rad;       /* 舵机转角上限，单位弧度 */
    float steer_gain;            /* 舵机转角增益 */
    float steer_rate_rad_s;      /* 舵机转角速率上限，单位 rad/s */
    float lateral_accel_limit_mps2; /* 横向加速度上限，用于曲率限速 */
    float speed_limit_mps;       /* 绝对速度上限，单位 m/s */
    float pwm_center_us;         /* 舵机中位脉宽，仅阿克曼底盘使用 */
    float pwm_per_rad_us;        /* 舵机每弧度脉宽，仅阿克曼底盘使用 */
} CarPursuitConfig;

/* 单次跟踪输出。 */
typedef struct
{
    uint8_t valid;              /* 1 表示本次结果可用于控制 */
    float lookahead_m;          /* 实际使用的预瞄距离，单位米 */
    float lateral_m;            /* 预瞄点横向偏差，右正，单位米 */
    float forward_m;            /* 预瞄点前向距离，单位米 */
    float curvature_inv_m;      /* 纯跟踪曲率，左转为正，单位 1/米 */
    float steering_rad;         /* 速率限制后的舵机转角，左转为正，单位弧度 */
    float speed_limit_mps;      /* 曲率限速，单位 m/s */
    float target_x_m;           /* 预瞄点 x，单位米 */
    float target_y_m;           /* 预瞄点 y，单位米 */
} CarPursuitResult;

/* 纯跟踪对象。 */
typedef struct
{
    CarPursuitConfig config;
    CarSteer steer;
    CarPursuitResult result;
    uint8_t fault;              /* 1 表示配置或输入非法，已锁存 */
} CarPursuit;

/**
 * @brief 初始化纯跟踪对象并校验配置。
 * @param p   跟踪对象。
 * @param cfg 配置，字段须为有限数且满足物理约束。
 * @return 1 成功；0 配置非法，对象已清零并置 fault。
 */
uint8_t CarPursuit_Init(CarPursuit *p, const CarPursuitConfig *cfg);

/**
 * @brief 清空跟踪结果和舵机动态状态，保留配置。
 * @param p 跟踪对象。
 */
void CarPursuit_Reset(CarPursuit *p);

/**
 * @brief 用车辆坐标系路径点计算一次纯跟踪输出。
 * @param p         跟踪对象。
 * @param points_xy 交错存放的路径点 [x0,y0,x1,y1,...]，单位米，
 *                  x 向右、y 向前，第一个点应离车辆最近。
 * @param count     路径点个数，至少 2。
 * @param speed_mps 当前前进速度，单位 m/s，用于动态预瞄和限速。
 * @param dt_s      控制周期，单位秒，范围 (0, 0.1]。
 * @return 1 结果有效；0 输入非法或预瞄点不在车辆前方。
 * @note 失败时 p->result.valid = 0，但已更新的舵机实际转角保留。
 */
uint8_t CarPursuit_Update(CarPursuit *p, const float *points_xy,
                          unsigned count, float speed_mps, float dt_s);

/**
 * @brief 把当前速度和舵机转角换算成左右轮目标转速。
 * @param p         跟踪对象。
 * @param speed_mps 前进速度，单位 m/s。
 * @param rpm       输出双轮 rpm，下标 0 左轮、1 右轮；正值为前进。
 * @note 结果未按 CAR_RPM_LIMIT 限幅，由调用者统一处理。
 */
void CarPursuit_Wheels(const CarPursuit *p, float speed_mps, float rpm[2]);

#endif
