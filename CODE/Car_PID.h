#ifndef CAR_PID_H
#define CAR_PID_H

#include <stdint.h>

/*
 * ============================================================================
 * 文件名称：Car_PID.h
 * ============================================================================
 * 模块功能：声明串级闭环使用的两种 PID 控制器。
 *
 *   1. 位置式 PID（Car_Pid_*）：输出 = P + I + D + 前馈，积分项显式累加。
 *      当前 Car_Core.c 把它用作串级控制的外环位置环，默认关闭。
 *   2. 增量式 PID（Car_PidInc_*）：输出按增量累加，u[k] = u[k-1] + Δu[k]，
 *      增益、滤波、前馈、限幅和抗饱和约定与位置式保持一致。
 *      当前 Car_Core.c 把它用作内环速度环，蓝牙调参只写这一组增益。
 *
 * 两种控制器的公共约定
 * ----------------------------------------------------------------------------
 *   - 内环速度环：目标/反馈为 rpm，加速度为 rpm/s，输出和前馈项为 PWM 数值；
 *     外环位置环：目标/反馈为输出轴圈数 rev，输出为内环转速给定 rpm；
 *   - Ki/Kd 沿用历史 5 ms 离散增益：
 *         I 增量 = Ki × error × dt / 0.005
 *         D 项   = -Kd × 0.005 × d(measure) / dt
 *   - 目标权重 beta 只作用于比例项，积分仍使用完整速度误差；
 *   - 本模块是纯算法，不访问外设；运行状态由上层（Car_Core.c）管理；
 *   - 所有接口要求 p 非空且独占访问：先 init，按需 configure，
 *     再周期调用 update；仅允许在停车时修改配置。
 * ============================================================================
 */

/* ========================================================================== */
/*                              位置式 PID                                    */
/* ========================================================================== */

/*
 * 位置式 PID 对象。
 *
 * 控制律：
 *     u = Kp × (beta × target - measure)
 *       + I
 *       + (-Kd × 0.005 × d(measure)/dt)
 *       + Kv × target + Ka × acceleration + Ks
 */
typedef struct
{
    /* ---------- 增益与限幅（来源：Car_Pid_init 参数或蓝牙命令 4） ---------- */
    float kp;             /* 比例增益，单位 PWM/rpm */
    float ki;             /* 积分增益，沿用 5 ms 离散约定 */
    float kd;             /* 微分增益，沿用 5 ms 离散约定 */
    float output_limit;   /* 输出绝对值上限，单位 PWM */

    /* ---------- 动态状态（由 update 维护） ---------- */
    float integral;       /* 已累积的积分输出，单位 PWM，受 integral_limit 限制 */
    float last_measure;   /* 上一周期实测转速，单位 rpm，用于计算测量变化率 */

    /* ---------- 配置（来源：Car_Pid_init 默认值或 configure 参数） ---------- */
    float beta;             /* 比例项目标权重，范围 [0,1] */
    float derivative_tau_s; /* 微分滤波时间常数，单位秒 */
    float integral_limit;   /* 积分项绝对值上限，单位 PWM */
    float ks;               /* 静摩擦前馈，单位 PWM */
    float kv;               /* 速度前馈系数，单位 PWM/rpm */
    float ka;               /* 加速度前馈系数，单位 PWM/(rpm/s) */

    /* ---------- 调试/遥测字段（由 update 写入） ---------- */
    float derivative;   /* 滤波后的测量变化率，单位 rpm/s */
    float p_term;       /* 本周期比例项输出，单位 PWM */
    float d_term;       /* 本周期微分项输出，单位 PWM */
    float ff_term;      /* 本周期前馈项输出，单位 PWM */
    float last_output;  /* 最近一次限幅后的输出，单位 PWM */

    /* ---------- 状态标志 ---------- */
    uint8_t started;    /* 1 表示已有上一周期测量值，可以计算微分 */
    uint8_t fault;      /* 1 表示数值异常已锁存故障，update 将返回 0 */
    uint8_t saturated;  /* 1 表示最近一次输出被限幅 */
} Car_Pid_Struct;

/**
 * @brief 清空对象并初始化增益和输出上限。
 * @param p     待初始化的位置式 PID 对象。
 * @param kp    比例增益，有限数，范围 [0, 655.35]。
 * @param ki    积分增益，有限数，范围 [0, 655.35]。
 * @param kd    微分增益，有限数，范围 [0, 655.35]。
 * @param limit 输出上限，有限数，范围 (0, 10000]。
 * @return 1 成功；0 参数无效，此时对象已清空并置 fault = 1。
 * @note 默认 beta = 1、微分滤波 0.02 s、积分上限等于输出上限、前馈为零。
 */
uint8_t Car_Pid_init(Car_Pid_Struct *p, float kp, float ki, float kd, float limit);

/**
 * @brief 在已初始化的对象上配置目标权重、微分滤波和前馈。
 * @param p        已初始化的位置式 PID 对象。
 * @param beta     比例项目标权重，范围 [0, 1]。
 * @param tau      微分滤波时间常数，范围 [0, 1] 秒，0 表示不滤波。
 * @param fraction 积分上限比例，范围 (0, 1]，
 *                 积分上限 = 该比例 × output_limit。
 * @param ks       静摩擦前馈，范围 [0, output_limit]。
 * @param kv       速度前馈系数，范围 [0, 10000]。
 * @param ka       加速度前馈系数，范围 [0, 10000]。
 * @return 1 成功并清除动态历史及故障；0 拒绝且保留原配置与历史。
 */
uint8_t Car_Pid_configure(Car_Pid_Struct *p, float beta, float derivative_tau_s,
                          float integral_fraction, float ks, float kv, float ka);

/**
 * @brief 保留配置，清空积分、微分历史、最近输出和状态标志。
 * @param p 位置式 PID 对象。
 * @note reset 会清除 fault；如果配置本身异常，须先修正参数再恢复控制。
 */
void Car_Pid_reset(Car_Pid_Struct *p);

/**
 * @brief 计算一次位置式速度控制输出，正负号表示驱动方向。
 * @param p            位置式 PID 对象。
 * @param target       参考转速（rpm），须为有限数。
 * @param measure      实测转速（rpm），须为有限数。
 * @param acceleration 参考加速度（rpm/s），用于 Ka 前馈。
 * @param dt           实际周期（秒），允许 [0.0001, 0.1]。
 * @return [-output_limit, output_limit] 内的 PWM 数值；故障时返回 0。
 * @note 零返回值也可能是正常输出，须结合 p->fault 判断；故障会锁存。
 */
float Car_Pid_update_dt(Car_Pid_Struct *p, float target, float measure,
                        float target_acceleration, float dt_s);

/**
 * @brief 固定 5 ms、参考加速度为 0 的兼容入口，其余语义同 update_dt。
 * @param p       位置式 PID 对象。
 * @param target  参考转速（rpm）。
 * @param measure 实测转速（rpm）。
 * @return 限幅后的 PWM 数值；故障时返回 0。
 */
float Car_Pid_update(Car_Pid_Struct *p, float target, float measure);

/* ========================================================================== */
/*                              增量式 PID                                    */
/* ========================================================================== */

/*
 * 增量式 PID 对象。
 *
 * 控制律：
 *     Δu[k] = ΔP + ΔI + ΔD + ΔFF
 *     u[k]  = u[k-1] + Δu[k]
 *     out   = clip(u[k], output_limit)
 *
 *   其中 P、D、FF 的绝对项定义与位置式完全相同，增量由相邻两周期作差得到；
 *   ΔI 是最终接纳的积分变化量。核心状态是未限幅递推值 unclamped_output，
 *   它保留 P 项的完整历史，因此 P 项顶到限幅再退出时不会产生反向冲击；
 *   last_output 只保存实际写到执行器的限幅输出，不能作为递推基准。
 *
 *   抗饱和用条件积分：本拍增量会把输出进一步推入同一方向饱和时，只撤掉
 *   积分增量，P/D/FF 仍照常响应。递推每 8 拍用绝对项重锚一次，纯为消除
 *   单精度累加漂移，不改变控制律，也不影响限幅判定。
 */
typedef struct
{
    /* ---------- 增益与限幅（来源：Car_PidInc_init 参数或本地默认值） ---------- */
    float kp;             /* 比例增益，单位 PWM/rpm */
    float ki;             /* 积分增益，沿用 5 ms 离散约定 */
    float kd;             /* 微分增益，沿用 5 ms 离散约定 */
    float output_limit;   /* 输出绝对值上限，单位 PWM */

    /* ---------- 动态状态（由 update 维护） ---------- */
    float integral;         /* 已累积的积分输出，单位 PWM，受 integral_limit 限制 */
    float last_measure;     /* 上一周期实测转速，单位 rpm，用于计算测量变化率 */
    float last_output;      /* 上一周期限幅后的执行器输出，单位 PWM */
    float unclamped_output; /* 上一周期未限幅的累加值 u[k-1]，单位 PWM */
    float last_p_term;      /* 上一周期比例项值，单位 PWM，用于求 ΔP */
    float last_d_term;      /* 上一周期微分项值，单位 PWM，用于求 ΔD */
    float last_ff_term;     /* 上一周期前馈项值，单位 PWM，用于求 ΔFF */

    /* ---------- 配置（来源：Car_PidInc_init 默认值或 configure 参数） ---------- */
    float beta;             /* 比例项目标权重，范围 [0,1] */
    float derivative_tau_s; /* 微分滤波时间常数，单位秒 */
    float integral_limit;   /* 积分项绝对值上限，单位 PWM */
    float ks;               /* 旧模型静摩擦前馈，单位 PWM */
    float kv;               /* 旧模型速度前馈系数，单位 PWM/rpm */
    float ka;               /* 加速度前馈系数，单位 PWM/(rpm/s) */

    /* ---------- 低速增益调度 / 积分分离 / 死区前馈 ---------- */
    float low_speed_rpm;       /* 低速调度起点，0 表示禁用；单位 rpm */
    float high_speed_rpm;      /* 低速调度终点；单位 rpm */
    float kp_scale_low;        /* 低速 Kp 缩放系数，0~1 */
    float ki_scale_low;        /* 低速 Ki 缩放系数，0~1 */
    float integral_band_rpm;   /* 积分分离误差带，0 表示禁用；单位 rpm */
    float integral_scale;      /* 误差带内 Ki 缩放系数，0~1 */
    float integral_speed_rpm;  /* 目标低于此值才启用积分分离；单位 rpm */
    float ff_dead_run;         /* 死区模型运行死区，单位 PWM */
    float ff_dead_start;       /* 死区模型起步断裂值，单位 PWM */
    float ff_gain;             /* 死区模型速度前馈，单位 PWM/rpm */
    float ff_decay_rpm;        /* 起步补偿衰减尺度，单位 rpm */
    uint8_t ff_model;          /* 1=死区前馈模型；0=旧 Ks/Kv 模型 */

    /* ---------- 调试/遥测字段（由 update 写入） ---------- */
    float derivative;   /* 滤波后的测量变化率，单位 rpm/s */
    float p_term;       /* 本周期比例项值，单位 PWM */
    float d_term;       /* 本周期微分项值，单位 PWM */
    float ff_term;      /* 本周期前馈项值，单位 PWM */
    float delta;        /* 本周期输出增量 Δu，单位 PWM */
    float kp_effective; /* 本拍实际 Kp = 基础 Kp × 低速调度系数 */
    float ki_effective; /* 本拍实际 Ki = 基础 Ki × 调度 × 积分分离系数 */
    float gain_blend;   /* 低速调度混合：0=低速系数，1=高速系数 */

    /* ---------- 状态标志 ---------- */
    uint8_t started;    /* 1 表示已有上一周期测量值，可以计算微分 */
    uint8_t fault;      /* 1 表示数值异常已锁存故障，update 将返回 0 */
    uint8_t saturated;  /* 1 表示最近一次输出被限幅 */
    uint8_t reanchor;   /* 递推重锚计数，每 8 拍用绝对项修正一次浮点漂移 */
} Car_PidInc_Struct;

/**
 * @brief 清空增量式 PID 对象并初始化增益和输出上限。
 * @param p     待初始化的增量式 PID 对象。
 * @param kp    比例增益，有限数，范围 [0, 655.35]。
 * @param ki    积分增益，有限数，范围 [0, 655.35]。
 * @param kd    微分增益，有限数，范围 [0, 655.35]。
 * @param limit 输出上限，有限数，范围 (0, 10000]。
 * @return 1 成功；0 参数无效，此时对象已清空并置 fault = 1。
 * @note 默认值与位置式一致：beta = 1、微分滤波 0.02 s、积分上限等于输出上限。
 */
uint8_t Car_PidInc_init(Car_PidInc_Struct *p, float kp, float ki, float kd,
                        float limit);

/**
 * @brief 在已初始化的增量式 PID 上配置目标权重、微分滤波和前馈。
 * @param p        已初始化的增量式 PID 对象。
 * @param beta     比例项目标权重，范围 [0, 1]。
 * @param tau      微分滤波时间常数，范围 [0, 1] 秒，0 表示不滤波。
 * @param fraction 积分上限比例，范围 (0, 1]。
 * @param ks       静摩擦前馈，范围 [0, output_limit]。
 * @param kv       速度前馈系数，范围 [0, 10000]。
 * @param ka       加速度前馈系数，范围 [0, 10000]。
 * @return 1 成功并清除动态历史及故障；0 拒绝且保留原配置与历史。
 */
uint8_t Car_PidInc_configure(Car_PidInc_Struct *p, float beta, float derivative_tau_s,
                             float integral_fraction, float ks, float kv, float ka);

/**
 * @brief 配置低速增益调度和积分分离；不清除积分/P/FF 历史。
 * @param p              增量式 PID 对象。
 * @param low_speed_rpm  低速调度起点（rpm），0 表示禁用调度。
 * @param high_speed_rpm 高速调度终点（rpm），必须大于 low_speed_rpm。
 * @param kp_scale_low   低速 Kp 缩放系数，(0, 1]。
 * @param ki_scale_low   低速 Ki 缩放系数，(0, 1]。
 * @param band_rpm       积分分离误差带（rpm），0 表示禁用。
 * @param band_scale     误差带内 Ki 缩放系数，[0, 1]。
 * @param band_speed_rpm 仅在目标低于此值时启用积分分离（rpm）。
 * @return 1 成功；0 参数无效，保留原配置。
 * @note 调度的系数乘在 kp/ki 基础增益上；基础增益仍由 init/蓝牙命令 4 决定。
 */
uint8_t Car_PidInc_set_low_speed(Car_PidInc_Struct *p, float low_speed_rpm,
                                 float high_speed_rpm, float kp_scale_low,
                                 float ki_scale_low, float band_rpm,
                                 float band_scale, float band_speed_rpm);

/**
 * @brief 配置死区前馈模型：PWM = dead_run + gain×|rpm|，起步时 dead_start 衰减。
 * @param p          增量式 PID 对象。
 * @param dead_run   运行死区，单位 PWM，>= 0。
 * @param dead_start 起步断裂值，单位 PWM，>= dead_run。
 * @param gain       速度前馈，单位 PWM/rpm，>= 0。
 * @param decay_rpm  起步补偿衰减尺度，单位 rpm，> 0。
 * @return 1 成功并启用死区模型；0 参数无效，保留旧 Ks/Kv 模型。
 * @note 该配置不清除积分/P/FF 历史；适合在停车状态调用一次。
 */
uint8_t Car_PidInc_set_friction_ff(Car_PidInc_Struct *p, float dead_run,
                                   float dead_start, float gain,
                                   float decay_rpm);

/**
 * @brief 保留配置，清空积分、微分历史、上一拍输出和状态标志。
 * @param p 增量式 PID 对象。
 * @note 限幅输出和未限幅累加值同时清零，下一次输出从 0 开始累加。
 */
void Car_PidInc_reset(Car_PidInc_Struct *p);

/**
 * @brief 计算一次增量式速度控制输出。
 * @param p            增量式 PID 对象。
 * @param target       参考转速（rpm），须为有限数。
 * @param measure      实测转速（rpm），须为有限数。
 * @param acceleration 参考加速度（rpm/s），用于 Ka 前馈。
 * @param dt           实际周期（秒），允许 [0.0001, 0.1]。
 * @return [-output_limit, output_limit] 内的 PWM 数值；故障时返回 0。
 * @note 与位置式一样，零返回值也可能是正常输出，须结合 p->fault 判断。
 */
float Car_PidInc_update_dt(Car_PidInc_Struct *p, float target, float measure,
                           float target_acceleration, float dt_s);

/**
 * @brief 固定 5 ms、参考加速度为 0 的增量式兼容入口。
 * @param p       增量式 PID 对象。
 * @param target  参考转速（rpm）。
 * @param measure 实测转速（rpm）。
 * @return 限幅后的 PWM 数值；故障时返回 0。
 */
float Car_PidInc_update(Car_PidInc_Struct *p, float target, float measure);

#endif
