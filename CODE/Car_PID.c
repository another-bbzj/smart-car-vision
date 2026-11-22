/*
 * ============================================================================
 * 文件名称：Car_PID.c
 * ============================================================================
 * 模块功能
 * ----------------------------------------------------------------------------
 * 本文件实现双轮串级闭环所用的两种纯算法 PID 控制器，不访问任何外设，
 * 也不直接输出 PWM。每个控制周期的调用关系如下：
 *
 *     Car_Core.c（控制周期任务）
 *         -> Car_Pid_update_dt()（外环位置环，默认关闭）
 *         -> Car_PidInc_update_dt()（内环速度环）
 *         -> 得到带符号的 PWM 数值
 *         -> Car_Hardware.c 把该数值写入电机驱动。
 *
 * 本文件包含两种实现：
 *
 *   1. 位置式 PID（Car_Pid_*）
 *      输出 = P + I + D + 前馈。积分项在结构体中显式累加；比例项支持目标
 *      权重 beta；微分项对“测量值”求导并做一阶低通，因此目标突然变化时
 *      不会产生微分冲击；积分项自身带限幅，并采用条件积分抗饱和。
 *      当前 Car_Core.c 把它用作串级控制的外环位置环；外环默认关闭。
 *
 *   2. 增量式 PID（Car_PidInc_*）
 *      不直接计算绝对输出，而是计算本周期相对上一周期的输出增量 Δu，
 *      再累加到上一周期的未限幅输出上：u[k] = u[k-1] + Δu[k]。
 *      Δu = ΔP + ΔI + ΔD + ΔFF，其中各绝对项的定义与位置式一致，
 *      增量由相邻两周期作差得到。递推保留未限幅的 u[k]，执行器只取
 *      clip(u[k])，因此 P 项在限幅期间的历史不会丢失，退出饱和时不会
 *      产生反向冲击。抗饱和采用条件积分：饱和时只拒绝积分增量。
 *      每 8 拍用绝对项重锚一次，消除单精度递推漂移。
 *      当前 Car_Core.c 把它用作串级控制的内环速度环，蓝牙命令 4 和
 *      板端菜单调整的就是这一组增益。
 *
 * 函数列表（按文件内出现顺序）
 * ----------------------------------------------------------------------------
 *   static uint8_t number(float x)
 *       —— 判断浮点数是否为有限值，过滤 NaN 和正负无穷。
 *   static float clip(float x, float limit)
 *       —— 把浮点数限制到 [-limit, +limit] 区间。
 *   void Car_Pid_reset(Car_Pid_Struct *p)
 *       —— 保留位置式 PID 的配置，清除动态历史与故障标志。
 *   uint8_t Car_Pid_init(Car_Pid_Struct *p, float kp, float ki, float kd,
 *                        float limit)
 *       —— 初始化位置式 PID，并校验增益和输出上限。
 *   uint8_t Car_Pid_configure(Car_Pid_Struct *p, float beta, float tau,
 *                             float fraction, float ks, float kv, float ka)
 *       —— 配置位置式 PID 的目标权重、微分滤波、积分限幅和前馈。
 *   float Car_Pid_update_dt(Car_Pid_Struct *p, float target, float measure,
 *                           float acceleration, float dt)
 *       —— 使用实际周期执行一次位置式 PID 计算。
 *   float Car_Pid_update(Car_Pid_Struct *p, float target, float measure)
 *       —— 固定 5 ms 周期的位置式 PID 兼容入口。
 *   void Car_PidInc_reset(Car_PidInc_Struct *p)
 *       —— 保留增量式 PID 的配置，清除动态历史与故障标志。
 *   uint8_t Car_PidInc_init(Car_PidInc_Struct *p, float kp, float ki,
 *                           float kd, float limit)
 *       —— 初始化增量式 PID，并校验增益和输出上限。
 *   uint8_t Car_PidInc_configure(Car_PidInc_Struct *p, float beta, float tau,
 *                                float fraction, float ks, float kv, float ka)
 *       —— 配置增量式 PID 的目标权重、微分滤波、积分限幅和前馈。
 *   float Car_PidInc_update_dt(Car_PidInc_Struct *p, float target,
 *                              float measure, float acceleration, float dt)
 *       —— 使用实际周期执行一次增量式 PID 计算。
 *   float Car_PidInc_update(Car_PidInc_Struct *p, float target, float measure)
 *       —— 固定 5 ms 周期的增量式 PID 兼容入口。
 *
 * 单位约定
 * ----------------------------------------------------------------------------
 *   内环速度环：target、measure 为 rpm，acceleration 为 rpm/s，输出为
 *               PWM 数值（NEUQ 库 PWM_MAX = 10000 对应 100%）；
 *   外环位置环：target、measure 为输出轴圈数 rev，acceleration 通常为 0，
 *               输出为内环转速给定 rpm；
 *   控制周期 dt：秒。
 *
 * Ki / Kd 的离散约定
 * ----------------------------------------------------------------------------
 *   为兼容既有蓝牙调参工具，Ki、Kd 仍按 5 ms 周期标定：
 *       积分增量 = Ki * error * (dt / 0.005)
 *       微分项   = -Kd * 0.005 * d(measure) / dt
 *   实际周期不是 5 ms 时，上面两式会自动按实际周期缩放。
 *
 * 数值安全
 * ----------------------------------------------------------------------------
 *   所有输入和中间结果都会检查是否为有限值。一旦发现 NaN、正负无穷、
 *   参数越界或周期异常，控制器会锁存 fault 标志并返回 0，由 Car_Core.c
 *   据此转入故障保护状态。
 * ============================================================================
 */

#include "Car_PID.h"

#include <float.h>
#include <math.h>
#include <string.h>

/* ========================================================================== */
/*                              内部工具函数                                   */
/* ========================================================================== */

/*
 * 函数名称：number
 * 函数功能：判断一个浮点数是否为有限值，过滤 NaN 和正负无穷。
 * 参数说明：x —— 待检查的浮点数。
 * 返回参数：1 表示有限值；0 表示 NaN 或正负无穷。
 * 使用示例：
 *     if (!number(kp))
 *     {
 *         return 0;
 *     }
 * 备注信息：NaN 与任何数比较都为假，因此只有有限值同时满足
 *           x <= FLT_MAX 和 x >= -FLT_MAX。
 */
static uint8_t number(float x)
{
    return x <= FLT_MAX && x >= -FLT_MAX;    /* 有限值返回 1，NaN/无穷返回 0 */
}

/*
 * 函数名称：clip
 * 函数功能：把浮点数限制到 [-limit, +limit] 区间。
 * 参数说明：x     —— 待限幅的数值；
 *           limit —— 正数形式的绝对值上限。
 * 返回参数：限幅后的数值。
 * 使用示例：
 *     output = clip(output, p->output_limit);
 * 备注信息：调用者需保证 limit 为正有限数；本函数不检查 limit 本身。
 */
static float clip(float x, float limit)
{
    if (x > limit)     /* 正向超限压到上限 */
    {
        return limit;
    }

    if (x < -limit)    /* 负向超限压到下限 */
    {
        return -limit;
    }

    return x;          /* 未超限原样返回 */
}

/* ========================================================================== */
/*                            位置式 PID 实现                                 */
/* ========================================================================== */

/*
 * 函数名称：Car_Pid_reset
 * 函数功能：保留增益、输出限幅、积分限幅和前馈配置，清除积分、微分历史、
 *           最近输出和全部状态标志，使控制器回到刚初始化完成的状态。
 * 参数说明：p —— 位置式 PID 对象，调用者需保证非空且独占访问。
 * 返回参数：无。
 * 使用示例：
 *     Car_Pid_reset(&car.pid_position[0]);   // 复位左轮外环位置环
 *     Car_Pid_reset(&car.pid_position[1]);   // 复位右轮外环位置环
 * 备注信息：fault 也会被清除；如果 fault 是配置参数异常引起的，必须先修正
 *           参数，再调用本函数恢复控制。
 */
void Car_Pid_reset(Car_Pid_Struct *p)
{
    p->integral = 0.0f;        /* 积分项清零 */
    p->last_measure = 0.0f;    /* 上一拍测量值清零 */
    p->derivative = 0.0f;      /* 滤波后的测量变化率清零 */
    p->p_term = 0.0f;          /* 比例项输出清零 */
    p->d_term = 0.0f;          /* 微分项输出清零 */
    p->ff_term = 0.0f;         /* 前馈项输出清零 */
    p->last_output = 0.0f;     /* 最近一次输出清零 */
    p->started = 0;            /* 重新等待首个测量周期 */
    p->fault = 0;              /* 清除锁存故障 */
    p->saturated = 0;          /* 清除饱和标志 */
}

/*
 * 函数名称：Car_Pid_init
 * 函数功能：清空位置式 PID 对象，并初始化增益与输出上限。
 * 参数说明：p     —— 待初始化的 PID 对象；
 *           kp    —— 比例增益，范围 [0, 655.35]；
 *           ki    —— 积分增益，范围 [0, 655.35]；
 *           kd    —— 微分增益，范围 [0, 655.35]；
 *           limit —— 输出绝对值上限，范围 (0, 10000]。
 * 返回参数：1 成功；0 参数无效，此时对象已清空并置 fault = 1。
 * 使用示例：
 *     Car_Pid_Struct pid;
 *     if (!Car_Pid_init(&pid, 2.0f, 0.12f, 0.0f, 9000.0f))
 *     {
 *         // 参数非法，pid.fault 已置 1
 *     }
 * 备注信息：默认 beta = 1、微分滤波时间常数 0.02 s、积分上限等于输出上限、
 *           前馈系数全为 0。
 */
uint8_t Car_Pid_init(Car_Pid_Struct *p, float kp, float ki, float kd, float limit)
{
    memset(p, 0, sizeof(*p));       /* 先整体清零，失败路径不留旧数据 */

    /* 统一检查增益和限幅：必须为有限值、非负且在允许范围内。 */
    if (!number(kp) || !number(ki) || !number(kd) || !number(limit) ||
        kp < 0.0f || ki < 0.0f || kd < 0.0f ||
        kp > 655.35f || ki > 655.35f || kd > 655.35f ||
        limit <= 0.0f || limit > 10000.0f)
    {
        p->fault = 1;               /* 锁存配置故障 */
        return 0;
    }

    p->kp = kp;                     /* 比例增益 */
    p->ki = ki;                     /* 积分增益 */
    p->kd = kd;                     /* 微分增益 */
    p->output_limit = limit;        /* 输出限幅 */
    p->integral_limit = limit;      /* 积分限幅默认等于输出限幅 */
    p->beta = 1.0f;                 /* 默认使用完整目标 */
    p->derivative_tau_s = 0.02f;    /* 默认微分滤波 20 ms */

    return 1;
}

/*
 * 函数名称：Car_Pid_configure
 * 函数功能：在已初始化的位置式 PID 上配置目标权重、微分滤波、积分限幅
 *           和前馈系数。
 * 参数说明：p        —— 已初始化的位置式 PID 对象；
 *           beta     —— 比例项目标权重，范围 [0, 1]；
 *           tau      —— 微分滤波时间常数，范围 [0, 1] 秒，0 表示不滤波；
 *           fraction —— 积分上限比例，范围 (0, 1]，
 *                       积分上限 = fraction × output_limit；
 *           ks       —— 静摩擦前馈，范围 [0, output_limit]，单位 PWM；
 *           kv       —— 速度前馈系数，范围 [0, 10000]，单位 PWM/rpm；
 *           ka       —— 加速度前馈系数，范围 [0, 10000]，单位 PWM/(rpm/s)。
 * 返回参数：1 成功并清除动态历史及故障；0 拒绝修改，原配置和历史保持不变。
 * 使用示例：
 *     Car_Pid_configure(&pid, 1.0f, 0.02f, 1.0f, 0.0f, 0.0f, 0.0f);
 * 备注信息：所有参数都必须为有限数；校验失败时不会写入任何一个字段。
 */
uint8_t Car_Pid_configure(Car_Pid_Struct *p, float beta, float tau,
                          float fraction, float ks, float kv, float ka)
{
    /* 配置参数必须有限且在合理范围内，否则整体拒绝，保持原配置不变。 */
    if (!number(beta) || !number(tau) || !number(fraction) ||
        !number(ks) || !number(kv) || !number(ka) ||
        beta < 0.0f || beta > 1.0f ||
        tau < 0.0f || tau > 1.0f ||
        fraction <= 0.0f || fraction > 1.0f ||
        ks < 0.0f || kv < 0.0f || ka < 0.0f ||
        ks > p->output_limit || kv > 10000.0f || ka > 10000.0f ||
        p->output_limit <= 0.0f)
    {
        return 0;
    }

    p->beta = beta;                                    /* 比例项目标权重 */
    p->derivative_tau_s = tau;                         /* 微分滤波时间常数 */
    p->integral_limit = fraction * p->output_limit;    /* 积分项幅值上限 */
    p->ks = ks;                                        /* 静摩擦前馈 */
    p->kv = kv;                                        /* 速度前馈 */
    p->ka = ka;                                        /* 加速度前馈 */

    /* 配置变化后旧的历史数据已不适用，清空动态状态再恢复控制。 */
    Car_Pid_reset(p);

    return 1;
}

/*
 * 函数名称：Car_Pid_update_dt
 * 函数功能：使用实际控制周期执行一次位置式 PID 计算，输出带符号 PWM 数值。
 * 参数说明：p            —— 位置式 PID 对象；
 *           target       —— 目标转速，单位 rpm；
 *           measure      —— 实测转速，单位 rpm；
 *           acceleration —— 参考加速度，单位 rpm/s，用于 Ka 前馈；
 *           dt           —— 实际控制周期，单位秒，允许 [0.0001, 0.1]。
 * 返回参数：[-output_limit, output_limit] 内的 PWM 数值；
 *           输入异常或对象已锁存故障时返回 0。
 * 使用示例：
 *     float rpm = Car_Pid_update_dt(&car.pid_position[0], 2.0f, position_rev,
 *                                   0.0f, 0.005f);
 *     if (car.pid_position[0].fault) { Car_Fault(&car, CAR_CONTROLLER_FAULT); }
 * 备注信息：返回 0 也可能是正常输出（例如目标为零且无扰动），调用者必须
 *           结合 p->fault 判断是否发生故障。故障一旦锁存，后续调用直接
 *           返回 0，直到 reset()。
 */
float Car_Pid_update_dt(Car_Pid_Struct *p, float target, float measure,
                       float acceleration, float dt)
{
    float error;                                           /* 速度误差 = target - measure，rpm */
    float raw_d = 0.0f;                                    /* 未滤波的测量变化率，rpm/s */
    float d_alpha;                                         /* 微分低通系数 */
    float base;                                            /* P + D + 前馈，不含积分 */
    float candidate;                                       /* 限幅后的积分项候选值 */
    float output;                                          /* 叠加积分后的总输出 */

    if (p->fault)                                          /* 已锁存故障，保持输出为 0 */
    {
        return 0.0f;
    }

    /* 统一检查输入、周期和对象内部数值，任一异常都进入 invalid。 */
    if (!number(target) || !number(measure) || !number(acceleration) || !number(dt) ||
        dt < 0.0001f || dt > 0.1f ||
        !number(p->kp) || !number(p->ki) || !number(p->kd) || !number(p->integral) ||
        !number(p->output_limit) || p->output_limit <= 0.0f ||
        p->output_limit > 10000.0f)
    {
        goto invalid;
    }

    error = target - measure;                              /* 完整速度误差，rpm */

    /*
     * 对测量值求导，而不是对误差求导：目标阶跃时不会出现微分冲击。
     * 首个控制周期没有上一拍测量值，因此不计算差分，避免上电瞬间
     * 产生虚假的微分尖峰。
     */
    if (p->started)
    {
        raw_d = (measure - p->last_measure) / dt;
    }

    p->last_measure = measure;                             /* 保存本拍测量值 */
    p->started = 1;                                        /* 从下一拍开始可以求微分 */

    /* 一阶低通抑制编码器量化噪声；系数随实际 dt 调整。 */
    d_alpha = dt / (p->derivative_tau_s + dt);
    p->derivative += d_alpha * (raw_d - p->derivative);

    /*
     * 比例项：beta 只作用于目标值。beta = 0 时相当于纯反馈比例，
     * beta = 1 时退化为常见的 Kp × (target - measure)。
     */
    p->p_term = p->kp * (p->beta * target - measure);

    p->d_term = -p->kd * 0.005f * p->derivative;           /* 微分项，5 ms 约定 */

    p->ff_term = p->kv * target + p->ka * acceleration;    /* 速度 + 加速度前馈 */

    if (target > 1.0f)                                     /* 正转时叠加正向静摩擦补偿 */
    {
        p->ff_term += p->ks;
    }
    else if (target < -1.0f)                               /* 反转时叠加负向静摩擦补偿 */
    {
        p->ff_term -= p->ks;
    }

    base = p->p_term + p->d_term + p->ff_term;             /* 不含积分的基础输出 */

    /*
     * 先限制积分项自身的幅值，再判断总输出是否允许接纳本次积分增量。
     * 按 5 ms 离散约定，积分步长为 Ki × error × (dt / 0.005)。
     */
    candidate = clip(p->integral + p->ki * error * (dt / 0.005f),
                     p->integral_limit);

    if (!number(base) || !number(candidate))
    {
        goto invalid;
    }

    output = base + candidate;                             /* 临时总输出，用于饱和判断 */

    /*
     * 条件积分抗饱和：
     * 如果总输出已经超过正限幅且误差仍为正，说明继续积分只会把积分项
     * 推得更大，退出饱和时容易产生超调，因此本次不接收积分增量；
     * 误差反向时允许积分退出饱和。负方向同理。
     */
    if (!((output > p->output_limit && error > 0.0f) ||
          (output < -p->output_limit && error < 0.0f)))
    {
        p->integral = candidate;                           /* 接收本次积分增量 */
    }

    output = base + p->integral;                           /* 用最终积分值重新计算总输出 */

    if (!number(output))
    {
        goto invalid;
    }

    p->saturated = (output > p->output_limit || output < -p->output_limit);
    p->last_output = clip(output, p->output_limit);

    return p->last_output;

invalid:
    /*
     * 输入或中间计算异常时锁存故障并返回零，
     * 上层（Car_Core.c）检测到 fault 后会进入故障状态并停车。
     */
    p->fault = 1;                                          /* 锁存数值故障 */
    p->last_output = 0.0f;                                 /* 立即撤去输出 */

    return 0.0f;
}

/*
 * 函数名称：Car_Pid_update
 * 函数功能：固定 5 ms 控制周期的位置式 PID 兼容入口。
 * 参数说明：p       —— 位置式 PID 对象；
 *           target  —— 目标转速，单位 rpm；
 *           measure —— 实测转速，单位 rpm。
 * 返回参数：同 Car_Pid_update_dt()。
 * 使用示例：
 *     float pwm = Car_Pid_update(&pid, 100.0f, speed_rpm);
 * 备注信息：参考加速度按 0 处理，适用于没有规划器的简单调用场景。
 */
float Car_Pid_update(Car_Pid_Struct *p, float target, float measure)
{
    return Car_Pid_update_dt(p, target, measure, 0.0f, 0.005f);
}

/* ========================================================================== */
/*                            增量式 PID 实现                                 */
/* ========================================================================== */

/* 每 8 拍用绝对项重锚一次递推输出，抑制单精度累加漂移。 */
#define CAR_PIDINC_REANCHOR_TICKS 8U

/*
 * 函数名称：Car_PidInc_reset
 * 函数功能：保留增量式 PID 的增益与限幅配置，清除积分、微分历史、
 *           上一拍输出和全部状态标志。
 * 参数说明：p —— 增量式 PID 对象，调用者需保证非空且独占访问。
 * 返回参数：无。
 * 使用示例：
 *     Car_PidInc_reset(&pid_inc);
 * 备注信息：last_output 清零意味着下一次调用的输出从 0 开始累加；
 *           如果车辆正在运动，应先确保执行器已经撤去驱动。
 */
void Car_PidInc_reset(Car_PidInc_Struct *p)
{
    p->integral = 0.0f;        /* 积分项清零 */
    p->last_measure = 0.0f;    /* 上一拍测量值清零 */
    p->derivative = 0.0f;      /* 滤波后的测量变化率清零 */
    p->last_output = 0.0f;     /* 上一拍执行器输出清零 */
    p->unclamped_output = 0.0f; /* 未限幅递推基准清零，下次从 0 累加 */
    p->reanchor = 0;           /* 重锚计数清零 */
    p->last_p_term = 0.0f;     /* 上一拍比例项清零 */
    p->last_d_term = 0.0f;     /* 上一拍微分项清零 */
    p->last_ff_term = 0.0f;    /* 上一拍前馈项清零 */
    p->p_term = 0.0f;          /* 本拍比例项清零 */
    p->d_term = 0.0f;          /* 本拍微分项清零 */
    p->ff_term = 0.0f;         /* 本拍前馈项清零 */
    p->delta = 0.0f;           /* 输出增量清零 */
    p->kp_effective = p->kp;   /* 调试：本拍实际 Kp 回到基础值 */
    p->ki_effective = p->ki;   /* 调试：本拍实际 Ki 回到基础值 */
    p->gain_blend = 1.0f;      /* 调试：调度混合系数回到高速端 */
    p->started = 0;            /* 重新等待首个测量周期 */
    p->fault = 0;              /* 清除锁存故障 */
    p->saturated = 0;          /* 清除饱和标志 */
}

/*
 * 函数名称：Car_PidInc_init
 * 函数功能：清空增量式 PID 对象，并初始化增益与输出上限。
 * 参数说明：p     —— 待初始化的增量式 PID 对象；
 *           kp    —— 比例增益，范围 [0, 655.35]；
 *           ki    —— 积分增益，范围 [0, 655.35]；
 *           kd    —— 微分增益，范围 [0, 655.35]；
 *           limit —— 输出绝对值上限，范围 (0, 10000]。
 * 返回参数：1 成功；0 参数无效，此时对象已清空并置 fault = 1。
 * 使用示例：
 *     Car_PidInc_Struct pid_inc;
 *     if (!Car_PidInc_init(&pid_inc, 2.0f, 0.12f, 0.0f, 9000.0f))
 *     {
 *         // 参数非法，pid_inc.fault 已置 1
 *     }
 * 备注信息：默认值与位置式一致：beta = 1、微分滤波 0.02 s、
 *           积分上限等于输出上限、前馈系数全为 0。
 */
uint8_t Car_PidInc_init(Car_PidInc_Struct *p, float kp, float ki, float kd,
                        float limit)
{
    memset(p, 0, sizeof(*p));       /* 先整体清零，失败路径不留旧数据 */

    /* 统一检查增益和限幅：必须为有限值、非负且在允许范围内。 */
    if (!number(kp) || !number(ki) || !number(kd) || !number(limit) ||
        kp < 0.0f || ki < 0.0f || kd < 0.0f ||
        kp > 655.35f || ki > 655.35f || kd > 655.35f ||
        limit <= 0.0f || limit > 10000.0f)
    {
        p->fault = 1;               /* 锁存配置故障 */
        return 0;
    }

    p->kp = kp;                     /* 比例增益 */
    p->ki = ki;                     /* 积分增益 */
    p->kd = kd;                     /* 微分增益 */
    p->output_limit = limit;        /* 输出限幅 */
    p->integral_limit = limit;      /* 积分限幅默认等于输出限幅 */
    p->beta = 1.0f;                 /* 默认使用完整目标 */
    p->derivative_tau_s = 0.02f;    /* 默认微分滤波 20 ms */

    /* 默认不启用低速调度、积分分离和死区前馈，保持旧行为。 */
    p->low_speed_rpm = 0.0f;        /* 0 = 不启用低速增益调度 */
    p->high_speed_rpm = 0.0f;       /* 0 = 不启用低速增益调度 */
    p->kp_scale_low = 1.0f;         /* 低速 Kp 系数，1 = 不缩放 */
    p->ki_scale_low = 1.0f;         /* 低速 Ki 系数，1 = 不缩放 */
    p->integral_band_rpm = 0.0f;    /* 0 = 不启用积分分离 */
    p->integral_scale = 1.0f;       /* 误差带内 Ki 系数，1 = 不缩放 */
    p->integral_speed_rpm = 0.0f;   /* 0 = 不对目标转速设限 */
    p->ff_dead_run = 0.0f;          /* 死区模型运行死区，默认 0 */
    p->ff_dead_start = 0.0f;        /* 死区模型起步断裂值，默认 0 */
    p->ff_gain = 0.0f;              /* 死区模型速度前馈，默认 0 */
    p->ff_decay_rpm = 0.0f;         /* 死区模型衰减尺度，默认 0（未启用） */
    p->ff_model = 0;                /* 0 = 使用旧 Ks/Kv 前馈 */
    p->kp_effective = kp;           /* 调试：本拍实际 Kp 初值 */
    p->ki_effective = ki;           /* 调试：本拍实际 Ki 初值 */
    p->gain_blend = 1.0f;           /* 调试：默认高速端 */

    return 1;
}

/*
 * 函数名称：Car_PidInc_configure
 * 函数功能：在已初始化的增量式 PID 上配置目标权重、微分滤波、积分限幅
 *           和前馈系数，参数含义与位置式完全一致。
 * 参数说明：p        —— 已初始化的增量式 PID 对象；
 *           beta     —— 比例项目标权重，范围 [0, 1]；
 *           tau      —— 微分滤波时间常数，范围 [0, 1] 秒，0 表示不滤波；
 *           fraction —— 积分上限比例，范围 (0, 1]；
 *           ks       —— 静摩擦前馈，单位 PWM；
 *           kv       —— 速度前馈系数，单位 PWM/rpm；
 *           ka       —— 加速度前馈系数，单位 PWM/(rpm/s)。
 * 返回参数：1 成功并清除动态历史及故障；0 拒绝修改，原配置和历史保持不变。
 * 使用示例：
 *     Car_PidInc_configure(&pid_inc, 1.0f, 0.02f, 1.0f, 0.0f, 0.0f, 0.0f);
 * 备注信息：所有参数都必须为有限数；校验失败时不会写入任何一个字段。
 */
uint8_t Car_PidInc_configure(Car_PidInc_Struct *p, float beta, float tau,
                             float fraction, float ks, float kv, float ka)
{
    /* 配置参数必须有限且在合理范围内，否则整体拒绝，保持原配置不变。 */
    if (!number(beta) || !number(tau) || !number(fraction) ||
        !number(ks) || !number(kv) || !number(ka) ||
        beta < 0.0f || beta > 1.0f ||
        tau < 0.0f || tau > 1.0f ||
        fraction <= 0.0f || fraction > 1.0f ||
        ks < 0.0f || kv < 0.0f || ka < 0.0f ||
        ks > p->output_limit || kv > 10000.0f || ka > 10000.0f ||
        p->output_limit <= 0.0f)
    {
        return 0;
    }

    p->beta = beta;                                    /* 比例项目标权重 */
    p->derivative_tau_s = tau;                         /* 微分滤波时间常数 */
    p->integral_limit = fraction * p->output_limit;    /* 积分项幅值上限 */
    p->ks = ks;                                        /* 静摩擦前馈 */
    p->kv = kv;                                        /* 速度前馈 */
    p->ka = ka;                                        /* 加速度前馈 */

    /* 配置变化后旧的历史数据已不适用，清空动态状态再恢复控制。 */
    Car_PidInc_reset(p);

    return 1;
}

/*
 * 函数名称：Car_PidInc_set_low_speed
 * 函数功能：配置低速 Kp/Ki 调度和误差带内的积分分离。
 * 参数说明：p              —— 增量式 PID 对象；
 *           low_speed_rpm  —— 调度起点（rpm），0 且 high=0 表示禁用；
 *           high_speed_rpm —— 调度终点（rpm），必须大于 low；
 *           kp_scale_low   —— 低速 Kp 系数，(0,1]；
 *           ki_scale_low   —— 低速 Ki 系数，(0,1]；
 *           band_rpm       —— 积分分离误差带（rpm），0 表示禁用；
 *           band_scale     —— 误差带内 Ki 系数，[0,1]；
 *           band_speed_rpm —— 仅在目标低于此值时启用积分分离（rpm）。
 * 返回参数：1 成功；0 参数无效，保留原配置。
 * 备注信息：只改调度参数，不清积分/P/FF 历史；基础 Kp/Ki 不变。
 */
uint8_t Car_PidInc_set_low_speed(Car_PidInc_Struct *p, float low_speed_rpm,
                                 float high_speed_rpm, float kp_scale_low,
                                 float ki_scale_low, float band_rpm,
                                 float band_scale, float band_speed_rpm)
{
    if (!number(low_speed_rpm) || !number(high_speed_rpm) ||
        !number(kp_scale_low) || !number(ki_scale_low) ||
        !number(band_rpm) || !number(band_scale) || !number(band_speed_rpm) ||
        low_speed_rpm < 0.0f || high_speed_rpm < 0.0f ||
        kp_scale_low <= 0.0f || kp_scale_low > 1.0f ||
        ki_scale_low <= 0.0f || ki_scale_low > 1.0f ||
        band_rpm < 0.0f || band_scale < 0.0f || band_scale > 1.0f ||
        band_speed_rpm < 0.0f)
    {
        return 0;
    }

    /* 允许 low=high=0 表示完全禁用；否则必须严格 high > low。 */
    if (high_speed_rpm != 0.0f && high_speed_rpm <= low_speed_rpm)
    {
        return 0;
    }

    p->low_speed_rpm = low_speed_rpm;
    p->high_speed_rpm = high_speed_rpm;
    p->kp_scale_low = kp_scale_low;
    p->ki_scale_low = ki_scale_low;
    p->integral_band_rpm = band_rpm;
    p->integral_scale = band_scale;
    p->integral_speed_rpm = band_speed_rpm;
    return 1;
}

/*
 * 函数名称：Car_PidInc_set_friction_ff
 * 函数功能：启用死区前馈模型 PWM = dead_run + gain×|rpm|，起步时 dead_start
 *           随速度平滑衰减到 dead_run。
 * 参数说明：p          —— 增量式 PID 对象；
 *           dead_run   —— 运行死区（PWM），>= 0；
 *           dead_start —— 起步断裂值（PWM），>= dead_run；
 *           gain       —— 速度前馈（PWM/rpm），>= 0；
 *           decay_rpm  —— 起步补偿衰减尺度（rpm），> 0。
 * 返回参数：1 成功并启用死区模型；0 参数无效，保留旧 Ks/Kv 模型。
 * 备注信息：只改前馈配置，不清积分/P/D 历史；适合停车状态调用一次。
 */
uint8_t Car_PidInc_set_friction_ff(Car_PidInc_Struct *p, float dead_run,
                                   float dead_start, float gain,
                                   float decay_rpm)
{
    if (!number(dead_run) || !number(dead_start) ||
        !number(gain) || !number(decay_rpm) ||
        dead_run < 0.0f || dead_start < dead_run ||
        dead_start > p->output_limit || gain < 0.0f || gain > 10000.0f ||
        decay_rpm <= 0.0f)
    {
        return 0;
    }

    p->ff_dead_run = dead_run;
    p->ff_dead_start = dead_start;
    p->ff_gain = gain;
    p->ff_decay_rpm = decay_rpm;
    p->ff_model = 1;
    return 1;
}

/*
 * 函数名称：Car_PidInc_update_dt
 * 函数功能：使用实际控制周期执行一次增量式 PID 计算，输出带符号 PWM 数值。
 * 参数说明：p            —— 增量式 PID 对象；
 *           target       —— 目标转速，单位 rpm；
 *           measure      —— 实测转速，单位 rpm；
 *           acceleration —— 参考加速度，单位 rpm/s，用于 Ka 前馈；
 *           dt           —— 实际控制周期，单位秒，允许 [0.0001, 0.1]。
 * 返回参数：[-output_limit, output_limit] 内的 PWM 数值；
 *           输入异常或对象已锁存故障时返回 0。
 * 使用示例：
 *     float pwm = Car_PidInc_update_dt(&pid_inc, 100.0f, speed_rpm,
 *                                      accel_rpm_s, 0.005f);
 *     if (pid_inc.fault) { Car_Fault(&car, CAR_CONTROLLER_FAULT); }
 * 备注信息：增量式公式为
 *               Δu = ΔP + ΔI + ΔD + ΔFF
 *               u[k] = u[k-1] + Δu
 *           其中 ΔP、ΔD、ΔFF 分别是本周期项值与上一周期项值之差，
 *           ΔI 是最终接纳的积分变化量，u 是未限幅的累加值；
 *           执行器限幅仅作用于返回值。递推基准是未限幅输出，不是限幅
 *           后的执行器输出，因此饱和期间 P/FF 的历史不会被丢掉。
 */
float Car_PidInc_update_dt(Car_PidInc_Struct *p, float target, float measure,
                           float acceleration, float dt)
{
    float error;                                           /* 速度误差 = target - measure，rpm */
    float raw_d = 0.0f;                                    /* 未滤波的测量变化率，rpm/s */
    float d_alpha;                                         /* 微分低通系数 */
    float p_term;                                          /* 本拍比例项绝对项值 */
    float d_term;                                          /* 本拍微分项绝对项值 */
    float ff_term;                                         /* 本拍前馈项绝对项值 */
    float base;                                            /* 本拍 P + D + FF */
    float integral_next;                                   /* 限幅后的积分项候选值 */
    float integral_step;                                   /* 本拍实际接纳的积分增量 */
    float delta;                                           /* 本拍输出增量 Δu */
    float unclamped;                                       /* 本拍未限幅递推输出 u[k] */
    float output;                                          /* 限幅后的最终输出 */
    float speed_abs;                                       /* |target|，用于调度和死区前馈 */
    float blend;                                           /* 低速调度混合：0=低速，1=高速 */
    float kp_eff;                                          /* 本拍实际 Kp */
    float ki_eff;                                          /* 本拍实际 Ki */
    float ff_ratio;                                        /* |target| / 起步补偿衰减尺度 */
    float ff_decay;                                        /* 起步补偿衰减因子，0..1 */
    float ff_dead;                                         /* 本拍有效死区 PWM */

    if (p->fault)                                          /* 已锁存故障，保持输出为 0 */
    {
        return 0.0f;
    }

    /* 与位置式相同的输入、周期和内部数值检查。 */
    if (!number(target) || !number(measure) || !number(acceleration) || !number(dt) ||
        dt < 0.0001f || dt > 0.1f ||
        !number(p->kp) || !number(p->ki) || !number(p->kd) || !number(p->integral) ||
        !number(p->output_limit) || p->output_limit <= 0.0f ||
        p->output_limit > 10000.0f)
    {
        goto invalid;
    }

    error = target - measure;                              /* 完整速度误差，rpm */

    /*
     * 低速增益调度：用目标转速（而不是实测转速）选择系数，避免堵转或
     * 测速噪声反过来改变增益。low~high 之间线性过渡，过渡段不会跳变。
     * 系数乘在基础 kp/ki 上；基础值仍由蓝牙命令 4 或 Flash 保存。
     */
    speed_abs = fabsf(target);
    blend = 1.0f;
    kp_eff = p->kp;
    ki_eff = p->ki;
    if (p->high_speed_rpm > p->low_speed_rpm)
    {
        blend = (speed_abs - p->low_speed_rpm) /
                (p->high_speed_rpm - p->low_speed_rpm);
        if (blend < 0.0f) blend = 0.0f;
        if (blend > 1.0f) blend = 1.0f;
        kp_eff = p->kp * (p->kp_scale_low +
                          (1.0f - p->kp_scale_low) * blend);
        ki_eff = p->ki * (p->ki_scale_low +
                          (1.0f - p->ki_scale_low) * blend);
    }

    /*
     * 积分分离：只在低速区、目标低于上限、误差进入带内时削弱积分。
     * 不直接清零，保留温漂和负载偏置的慢速修正能力。
     */
    if (p->integral_band_rpm > 0.0f &&
        (p->integral_speed_rpm <= 0.0f ||
         speed_abs < p->integral_speed_rpm) &&
        fabsf(error) < p->integral_band_rpm)
    {
        ki_eff *= p->integral_scale;
    }

    p->kp_effective = kp_eff;                              /* 调试：本拍实际 Kp */
    p->ki_effective = ki_eff;                              /* 调试：本拍实际 Ki */
    p->gain_blend = blend;                                 /* 调试：调度混合系数 */

    /* 与位置式一致：对测量值求导，首周期不计算差分，避免微分冲击。 */
    if (p->started)
    {
        raw_d = (measure - p->last_measure) / dt;
    }

    p->last_measure = measure;                             /* 保存本拍测量值 */
    p->started = 1;                                        /* 从下一拍开始可以求微分 */

    /* 一阶低通抑制编码器量化噪声，系数随实际周期调整。 */
    d_alpha = dt / (p->derivative_tau_s + dt);
    p->derivative += d_alpha * (raw_d - p->derivative);

    p_term = kp_eff * (p->beta * target - measure);        /* 比例项绝对项值 */
    d_term = -p->kd * 0.005f * p->derivative;              /* 微分项绝对项值 */

    if (p->ff_model && p->ff_decay_rpm > 0.0f)
    {
        /*
         * 死区前馈：PWM = D_run + G×|rpm|；起步时 D_start 按
         * 1 / (1 + (|rpm|/v0)²) 平滑衰减到 D_run。
         * 零目标不输出前馈，避免停车时残留偏置。
         */
        ff_ratio = speed_abs / p->ff_decay_rpm;
        ff_decay = 1.0f / (1.0f + ff_ratio * ff_ratio);
        ff_dead = p->ff_dead_run +
                  (p->ff_dead_start - p->ff_dead_run) * ff_decay;
        if (target > 0.0f)
        {
            ff_term = ff_dead + p->ff_gain * speed_abs;
        }
        else if (target < 0.0f)
        {
            ff_term = -(ff_dead + p->ff_gain * speed_abs);
        }
        else
        {
            ff_term = 0.0f;
        }
        ff_term += p->ka * acceleration;
    }
    else
    {
        ff_term = p->kv * target + p->ka * acceleration;   /* 旧模型：速度 + 加速度前馈 */

        if (target > 1.0f)                                 /* 正转时叠加正向静摩擦补偿 */
        {
            ff_term += p->ks;
        }
        else if (target < -1.0f)                           /* 反转时叠加负向静摩擦补偿 */
        {
            ff_term -= p->ks;
        }
    }

    base = p_term + d_term + ff_term;                      /* 本拍不含积分的绝对项 */

    /*
     * 先按积分上限裁剪本拍积分候选值，再求“本拍接纳的积分增量”。
     * ΔI 只把变化量计入输出，积分项自身仍受 integral_limit 约束。
     */
    integral_next = clip(p->integral + ki_eff * error * (dt / 0.005f),
                         p->integral_limit);
    integral_step = integral_next - p->integral;

    if (!number(base) || !number(integral_next) || !number(integral_step))
    {
        goto invalid;
    }

    /*
     * 增量式核心：Δu = ΔP + ΔI + ΔD + ΔFF，再累加到上一拍的未限幅输出。
     * 递推基准保留完整历史，限幅只发生在最终写执行器时。
     */
    delta = (p_term - p->last_p_term)
          + integral_step
          + (d_term - p->last_d_term)
          + (ff_term - p->last_ff_term);

    unclamped = p->unclamped_output + delta;

    if (!number(delta) || !number(unclamped))
    {
        goto invalid;
    }

    /*
     * 条件积分抗饱和：本拍增量会把输出进一步推入同一方向饱和时，
     * 只撤掉积分增量，P/D/FF 仍然响应，避免积分残留导致退出饱和后超调。
     */
    if ((unclamped > p->output_limit && error > 0.0f) ||
        (unclamped < -p->output_limit && error < 0.0f))
    {
        delta -= integral_step;
        unclamped = p->unclamped_output + delta;

        if (!number(delta) || !number(unclamped))
        {
            goto invalid;
        }
    }
    else
    {
        p->integral = integral_next;                       /* 接收本次积分增量 */
    }

    /*
     * 周期性重锚：递推和作差都会引入单精度舍入。用同一控制律的绝对项
     * base + integral 校正递推基准，数学上完全等价，只消除累计漂移；
     * 不与限幅后的输出比较，因此不改变饱和判定或输出历史。
     */
    p->reanchor++;
    if (p->reanchor >= CAR_PIDINC_REANCHOR_TICKS)
    {
        p->reanchor = 0;
        p->unclamped_output = base + p->integral;
    }
    else
    {
        p->unclamped_output = unclamped;
    }

    if (!number(p->unclamped_output))
    {
        goto invalid;
    }

    p->saturated = (p->unclamped_output > p->output_limit ||
                    p->unclamped_output < -p->output_limit);
    output = clip(p->unclamped_output, p->output_limit);   /* 仅对执行器输出限幅 */
    p->last_output = output;

    /* 保存本拍各项值，供下一周期作差得到增量。 */
    p->p_term = p_term;
    p->d_term = d_term;
    p->ff_term = ff_term;
    p->last_p_term = p_term;
    p->last_d_term = d_term;
    p->last_ff_term = ff_term;
    p->delta = delta;

    return output;

invalid:
    /* 与位置式一致：锁存故障、清零输出，由上层转入故障保护。 */
    p->fault = 1;                                          /* 锁存数值故障 */
    p->last_output = 0.0f;                                 /* 立即撤去输出 */

    return 0.0f;
}

/*
 * 函数名称：Car_PidInc_update
 * 函数功能：固定 5 ms 控制周期的增量式 PID 兼容入口。
 * 参数说明：p       —— 增量式 PID 对象；
 *           target  —— 目标转速，单位 rpm；
 *           measure —— 实测转速，单位 rpm。
 * 返回参数：同 Car_PidInc_update_dt()。
 * 使用示例：
 *     float pwm = Car_PidInc_update(&pid_inc, 100.0f, speed_rpm);
 * 备注信息：参考加速度按 0 处理，适用于没有规划器的简单调用场景。
 */
float Car_PidInc_update(Car_PidInc_Struct *p, float target, float measure)
{
    return Car_PidInc_update_dt(p, target, measure, 0.0f, 0.005f);
}
