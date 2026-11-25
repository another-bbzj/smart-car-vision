/*
 * ============================================================================
 * 文件名称：Car_Core.c
 * ============================================================================
 * 模块功能
 * ----------------------------------------------------------------------------
 * 本文件是蓝牙差速小车的“纯算法控制核心”，不直接访问任何 GPIO、PWM、
 * 编码器或串口。它把硬件层送来的字节流和编码器增量，转换为双轮 PWM 输出：
 *
 *     蓝牙字节 -> 协议解析 -> 状态机与目标转速
 *              -> 五次曲线参考规划
 *              -> [外环位置式位置 PID，默认关闭]
 *              -> 内环增量式速度 PID -> 双轮输出
 *              -> 运行保护（通信超时、堵转、方向异常、周期异常）
 *
 * 硬件采样、PWM 写入、串口收发和定时中断接入由 Car_Hardware.c 完成；
 * 对外的结构体和接口说明见 Car_Core.h。
 *
 * 控制流程（每个控制周期调用一次 Car_TickDt）
 * ----------------------------------------------------------------------------
 *   1. 检查实际周期是否在允许范围内，越界立即进入时序故障；
 *   2. 检查蓝牙半帧超时和运行通信看门狗；
 *   3. 运行状态下推进双轮共享进度的五次参考曲线；
 *   4. 用最近若干拍的编码器增量滑动窗口计算实测转速并做一阶低通，
 *      同时累计输出轴位置；
 *   5. 外环启用时用位置式位置 PID 生成转速给定，否则直接使用规划器参考；
 *   6. 对每一轮执行内环增量式速度 PID，得到带符号 PWM 输出；
 *   7. 检查堵转和方向异常，必要时锁存故障并停车。
 *
 * 函数列表（按文件内出现顺序）
 * ----------------------------------------------------------------------------
 *   static float absolute(float x)
 *       —— 求浮点数绝对值，用于速度大小判断。
 *   static float clamp(float x, float lim)
 *       —— 把浮点数限制到 [-lim, +lim]，用于目标转速限幅。
 *   static float wheel_ff_ks(unsigned wheel)
 *       —— 按左右轮返回各自的静摩擦前馈系数。
 *   static void speed_window_clear(Car_Core *c)
 *       —— 清空双轮测速滑动窗口，使下一次测速从空窗口开始。
 *   static void speed_window_push(Car_Core *c, unsigned i, int32_t counts,
 *                                 uint32_t dt_us)
 *       —— 把本拍编码器增量与周期写入指定车轮的滑动窗口。
 *   static float speed_window_rpm(const Car_Core *c, unsigned i)
 *       —— 用窗口计数和/时间和计算平均转速，抑制量化台阶。
 *   static void reset_motion(Car_Core *c)
 *       —— 停止运动：清空规划器、目标、参考、位置、输出和保护计时。
 *   void Car_Init(Car_Core *c)
 *       —— 上电初始化：清空控制对象、创建规划器和双轮串级 PID。
 *   void Car_Fault(Car_Core *c, uint8_t fault)
 *       —— 锁存首次故障原因、立即清零输出并丢弃未收完的蓝牙帧。
 *   uint8_t Car_Crc8(uint8_t crc, uint8_t byte)
 *       —— 计算 CRC-8（多项式 0x07），用于校验蓝牙命令帧。
 *   static uint16_t u16(const uint8_t *p)
 *       —— 从两个字节按大端顺序还原无符号 16 位整数。
 *   static float signed_rpm(const uint8_t *p)
 *       —— 从两个字节按大端顺序还原有符号 16 位转速，单位 rpm。
 *   static void execute(Car_Core *c)
 *       —— 执行一条已经通过长度和 CRC 校验的蓝牙命令。
 *   void Car_FeedByte(Car_Core *c, uint8_t byte)
 *       —— 逐字节解析蓝牙帧：C3 3C CMD LEN DATA CRC8。
 *   void Car_TickDt(Car_Core *c, int32_t left_count, int32_t right_count,
 *                   uint32_t dt_us)
 *       —— 按实际控制周期更新测速、参考、PID、输出和运行保护。
 *   void Car_Tick(Car_Core *c, int32_t left_count, int32_t right_count)
 *       —— 固定 CAR_PERIOD_MS 周期的兼容入口，主要用于主机仿真。
 *
 * 单位约定
 * ----------------------------------------------------------------------------
 *   目标转速、参考转速、实测转速：rpm；
 *   参考加速度：rpm/s；
 *   控制周期、各类计时：微秒（us）；
 *   PID 输出：PWM 数值，正负号表示转向。
 *
 * 上下文约定
 * ----------------------------------------------------------------------------
 *   本文件不依赖 HAL。调用者必须提供有效的非空对象指针，并保证：
 *   - Car_FeedByte 与 Car_TickDt 在同一执行上下文中串行调用；
 *   - 跨上下文读取 Car_Core 时自行保证快照一致（硬件层用临界区实现）。
 * ============================================================================
 */

#include "Car_Core.h"
#include "Car_Config.h"

#include <string.h>

/* ========================================================================== */
/*                              内部工具函数                                   */
/* ========================================================================== */

/*
 * 函数名称：absolute
 * 函数功能：求浮点数的绝对值。
 * 参数说明：x —— 任意有限浮点数。
 * 返回参数：x 的绝对值。
 * 使用示例：
 *     abs_speed = absolute(c->speed[i]);
 * 备注信息：仅用于速度、误差等量的数值大小判断，不改变控制方向。
 */
static float absolute(float x)
{
    return x < 0.0f ? -x : x;
}

/*
 * 函数名称：clamp
 * 函数功能：把浮点数限制到 [-lim, +lim]。
 * 参数说明：x   —— 待限幅的数值；
 *           lim —— 正数形式的绝对值上限。
 * 返回参数：限幅后的数值。
 * 使用示例：
 *     c->target[0] = clamp(rpm, CAR_RPM_LIMIT);
 * 备注信息：调用者需保证 lim 为正有限数。
 */
static float clamp(float x, float lim)
{
    if (x > lim)     /* 正向超限压到上限 */
    {
        return lim;
    }

    if (x < -lim)    /* 负向超限压到下限 */
    {
        return -lim;
    }

    return x;        /* 未超限原样返回 */
}

/*
 * 函数名称：wheel_ff_ks
 * 函数功能：取指定车轮的静摩擦前馈系数。
 * 参数说明：wheel —— 轮号，0 左轮、1 右轮。
 * 返回参数：对应车轮的 Ks，单位 PWM 数值。
 * 使用示例：
 *     Car_PidInc_configure(&c->pid_speed[i], ..., wheel_ff_ks(i), ...);
 * 备注信息：两轮静摩擦门槛实测不同（左轮约 12%、右轮约 7%），共用一组
 *           Ks 会让右轮低速过冲、左轮起不来。默认值见 Car_Config.h。
 */
static float wheel_ff_ks(unsigned wheel)
{
    return wheel ? CAR_FF_KS_RIGHT : CAR_FF_KS_LEFT;
}

/*
 * 函数名称：speed_window_clear
 * 函数功能：清空双轮测速滑动窗口，使下一次测速从空窗口重新开始。
 * 参数说明：c —— 控制对象。
 * 返回参数：无。
 * 使用示例：
 *     speed_window_clear(c);      // 停车、故障或重新发车时
 * 备注信息：窗口清空后测速会在接下来 CAR_SPEED_WINDOW_TICKS 拍内逐步
 *           恢复满窗；期间按实际已填拍数求平均，不会出现虚假零速。
 */
static void speed_window_clear(Car_Core *c)
{
    unsigned i;                                      /* 双轮循环下标，0 左轮 / 1 右轮 */

    for (i = 0; i < 2; i++)
    {
        c->speed_index[i] = 0;                       /* 写指针回到队首 */
        c->speed_fill[i] = 0;                        /* 有效拍数清零 */
        memset(c->speed_counts[i], 0, sizeof(c->speed_counts[i]));
        memset(c->speed_dt_us[i], 0, sizeof(c->speed_dt_us[i]));
        /* 窗口重开后，守卫也从空窗口重新播种；诊断计数保留累计值。 */
        c->speed_guard_raw[i] = 0.0f;
        c->speed_guard_hold[i] = 0;
        c->count_guard_hold[i] = 0;
    }
}

/*
 * 函数名称：speed_window_push
 * 函数功能：把本拍编码器增量和实测周期写入指定车轮的滑动窗口。
 * 参数说明：c      —— 控制对象；
 *           i      —— 轮号，0 左轮 / 1 右轮；
 *           counts —— 本拍有符号编码器增量；
 *           dt_us  —— 与增量对应的实测周期，单位微秒。
 * 返回参数：无。
 * 使用示例：
 *     speed_window_push(c, 0, left_counts, dt_us);
 * 备注信息：窗口满后自动覆盖最旧的一拍；写指针在 0..N-1 之间循环。
 */
static void speed_window_push(Car_Core *c, unsigned i, int32_t counts,
                              uint32_t dt_us)
{
    unsigned index = c->speed_index[i];               /* 当前写指针 */

    c->speed_counts[i][index] = counts;
    c->speed_dt_us[i][index] = dt_us;

    index++;
    if (index >= CAR_SPEED_WINDOW_TICKS)
    {
        index = 0;                                    /* 环形回绕 */
    }

    c->speed_index[i] = (uint8_t)index;

    if (c->speed_fill[i] < CAR_SPEED_WINDOW_TICKS)
    {
        c->speed_fill[i]++;                           /* 上电初期逐步填满 */
    }
}

/*
 * 函数名称：speed_window_rpm
 * 函数功能：用滑动窗口内的计数和与时间和计算平均转速。
 * 参数说明：c —— 控制对象；
 *           i —— 轮号，0 左轮 / 1 右轮。
 * 返回参数：窗口平均转速，单位 rpm；窗口时间为 0 时返回 0。
 * 使用示例：
 *     raw = speed_window_rpm(c, i);
 * 备注信息：把有效拍数内的计数和一次性换算，等效于对窗口时间做平均，
 *           而不是先逐拍求速再平均，因此不会把量化误差再次放大。
 */
static float speed_window_rpm(const Car_Core *c, unsigned i)
{
    int32_t counts = 0;                               /* 窗口计数和 */
    uint32_t dt_us = 0;                               /* 窗口时间和，微秒 */
    unsigned k;                                       /* 有效拍循环下标 */

    for (k = 0; k < c->speed_fill[i]; k++)
    {
        counts += c->speed_counts[i][k];
        dt_us += c->speed_dt_us[i][k];
    }

    if (dt_us == 0U)                                  /* 空窗口或异常周期 */
    {
        return 0.0f;
    }

    /* 转/秒 = counts / (CPR × 秒)，再乘 60 成 rpm；时间用微秒，补 1e6。 */
    return (float)counts * (60.0f * 1000000.0f) /
           (CAR_ENCODER_CPR * (float)dt_us);
}

/*
 * 函数名称：open_loop_clear
 * 函数功能：退出双轮开环 PWM 测试并清除命令和超时计数。
 * 参数说明：c —— 控制对象。
 * 返回参数：无。
 * 备注信息：STOP、故障、ARM、轮速命令和恢复闭环时都必须调用，
 *           保证不会把上一次测试的占空比留在执行器上。
 */
static void open_loop_clear(Car_Core *c)
{
    unsigned i;

    for (i = 0; i < 2; i++)
    {
        c->open_loop[i] = 0U;
        c->open_loop_pwm[i] = 0.0f;
        c->open_loop_us[i] = 0U;
    }
}

/*
 * 函数名称：reset_motion
 * 函数功能：停止运动并清除规划器、PID、测速窗口和保护计时。
 * 参数说明：c —— 控制对象。
 * 返回参数：无。
 * 使用示例：
 *     reset_motion(c);            // STOP 命令、ARM 启动或故障时调用
 * 备注信息：保留实测转速 speed[]，供后续测速滤波继续使用；
 *           内环、外环积分和位置反馈全部清零，下一次运行从位置零点开始。
 *           测速窗口一并清空，避免把停车前的计数带入下一次发车。
 */
static void reset_motion(Car_Core *c)
{
    unsigned i;                                      /* 双轮循环下标，0 左轮 / 1 右轮 */

    CarReference_Reset(&c->profile);
    speed_window_clear(c);                           /* 双轮窗口同步清空 */
    open_loop_clear(c);                              /* 退出开环测试 */

    for (i = 0; i < 2; i++)
    {
        c->target[i] = 0.0f;                         /* 目标转速清零 */
        c->reference[i] = 0.0f;                      /* 内环转速给定清零 */
        c->position[i] = 0.0f;                       /* 累计位置回到零点 */
        c->position_target[i] = 0.0f;                /* 外环目标位置回到零点 */
        c->output[i] = 0.0f;                         /* 控制输出清零 */
        c->output_last[i] = 0.0f;                    /* 守卫保持值同步清零 */
        c->stall_us[i] = 0;                          /* 堵转计时清零 */
        c->overspeed_us[i] = 0;
        c->direction_us[i] = 0;                      /* 方向异常计时清零 */
        c->slow_speed[i] = absolute(c->speed[i]);    /* 慢速滤波重新播种 */
        Car_PidInc_reset(&c->pid_speed[i]);          /* 清除内环速度历史与故障 */
        Car_Pid_reset(&c->pid_position[i]);          /* 清除外环位置历史与故障 */
    }
}

/* ========================================================================== */
/*                              初始化与故障                                   */
/* ========================================================================== */

/*
 * 函数名称：Car_Init
 * 函数功能：上电初始化控制对象，创建双轮共享进度的参考规划器和左右轮
 *           两套独立的串级 PID（外环位置式位置环 + 内环增量式速度环），
 *           初始化后进入 CAR_STOP。
 * 参数说明：c —— 待初始化的控制对象。
 * 返回参数：无。
 * 使用示例：
 *     Car_Core car;
 *     Car_Init(&car);
 * 备注信息：规划器限制和 PID 默认参数均来自 Car_Config.h；
 *           PID 配置失败时直接进入故障状态。外环即使默认关闭也会初始化，
 *           便于以后只改编译开关就能启用。
 */
void Car_Init(Car_Core *c)
{
    unsigned i;                                    /* 双轮循环下标，0 左轮 / 1 右轮 */

    memset(c, 0, sizeof(*c));                      /* 清空全部状态与历史 */

    /* 规划器限制来源：Car_Config.h 的指令限速、加速度和加加速度上限。 */
    CarReference_Init(&c->profile,
                      CAR_RPM_LIMIT,
                      CAR_ACCEL_LIMIT_RPM_S,
                      CAR_JERK_LIMIT_RPM_S2);

    for (i = 0; i < 2; i++)
    {
        /* 内环：增量式速度 PID，默认增益和输出限幅来自 Car_Config.h，
         * 停车时可由蓝牙命令 4 或板端菜单修改。 */
        Car_PidInc_init(&c->pid_speed[i],
                        CAR_DEFAULT_KP,
                        CAR_DEFAULT_KI,
                        CAR_DEFAULT_KD,
                        CAR_PWM_LIMIT);

        if (!Car_PidInc_configure(&c->pid_speed[i],
                                  CAR_PID_SETPOINT_WEIGHT,
                                  CAR_PID_D_TAU_S,
                                  1.0f,
                                  wheel_ff_ks(i),
                                  CAR_FF_KV,
                                  CAR_FF_KA))
        {
            Car_Fault(c, CAR_CONTROLLER_FAULT);    /* 配置失败立即进入故障 */
        }

        /* 外环：位置式位置 PID，输出是内环转速给定，因此限幅用转速上限。
         * 默认增益全 0，只有 CAR_CASCADE_OUTER_ENABLE=1 时才参与控制。 */
        Car_Pid_init(&c->pid_position[i],
                     CAR_POS_DEFAULT_KP,
                     CAR_POS_DEFAULT_KI,
                     CAR_POS_DEFAULT_KD,
                     CAR_RPM_LIMIT);

        if (!Car_Pid_configure(&c->pid_position[i],
                               CAR_PID_SETPOINT_WEIGHT,
                               CAR_PID_D_TAU_S,
                               1.0f,
                               0.0f,
                               0.0f,
                               0.0f))
        {
            Car_Fault(c, CAR_CONTROLLER_FAULT);    /* 配置失败立即进入故障 */
        }
    }
}

/*
 * 函数名称：Car_Fault
 * 函数功能：锁存故障并立即停车。
 * 参数说明：c     —— 控制对象；
 *           fault —— 调用者传入的 CAR_*_FAULT 故障码。
 * 返回参数：无。
 * 使用示例：
 *     if (dt_us > CAR_MAX_CONTROL_US)
 *     {
 *         Car_Fault(&car, CAR_TIMING_FAULT);
 *     }
 * 备注信息：重复调用不会覆盖第一次记录的故障原因；
 *           清零输出只代表控制量为 0，不代表车轮已经物理停止。
 */
void Car_Fault(Car_Core *c, uint8_t fault)
{
    c->autonomous = 0;
    if (c->state != CAR_FAULT)    /* 只记录第一次故障原因 */
    {
        c->fault = fault;
    }

    reset_motion(c);              /* 清空目标、参考和输出 */
    c->state = CAR_FAULT;         /* 进入故障状态 */
    c->parse_state = 0;           /* 丢弃未收完的蓝牙帧 */
}

void Car_LocalStop(Car_Core *c)
{
    reset_motion(c);
    c->state = CAR_STOP;
    c->fault = c->autonomous = c->parse_state = 0;
    c->link_us = 0;
}

int Car_StartRace(Car_Core *c)
{
    if (c->state != CAR_STOP) return 0;
    Car_LocalStop(c);
    c->autonomous = 1;
    c->state = CAR_RUN;
    return 1;
}

uint8_t Car_SetPid(Car_Core *c, unsigned wheel, float kp, float ki, float kd)
{
    Car_PidInc_Struct next;
    if (c->state != CAR_STOP) return 1;
    if (wheel > 1 || !Car_PidInc_init(&next, kp, ki, kd, CAR_PWM_LIMIT) ||
        !Car_PidInc_configure(&next, CAR_PID_SETPOINT_WEIGHT, CAR_PID_D_TAU_S,
                              1.0f, wheel_ff_ks(wheel), CAR_FF_KV, CAR_FF_KA))
    {
        return 2;
    }
    c->pid_speed[wheel] = next;      /* 蓝牙命令 4 只写内环速度环 */
    return 0;
}

uint8_t Car_SetOpenLoop(Car_Core *c, unsigned wheel, float pwm)
{
    uint8_t entering;

    if (c->state != CAR_RUN) return 1;
    if (wheel > 1 || pwm != pwm ||
        pwm < -CAR_OPENLOOP_PWM_LIMIT || pwm > CAR_OPENLOOP_PWM_LIMIT)
    {
        return 2;
    }

    /*
     * 首次进入开环时要求双轮目标为零且车轮基本停住，防止在正常行驶中
     * 直接切到开环。周期刷新时 open_loop 已置位，不再做这项检查，
     * 否则阶跃期间转速上升会让后续刷新被拒绝。
     */
    entering = (c->open_loop[0] == 0U && c->open_loop[1] == 0U) ? 1U : 0U;
    if (entering &&
        (c->target[0] != 0.0f || c->target[1] != 0.0f ||
         absolute(c->speed[0]) > 10.0f || absolute(c->speed[1]) > 10.0f))
    {
        return 2;
    }

    /*
     * 一次只测试一轮：先清掉双轮开环和规划器，避免另一轮带着旧目标运动。
     * target/reference 归零后，本拍不会再走正常 PID 分支。
     */
    open_loop_clear(c);
    CarReference_Reset(&c->profile);
    c->target[0] = 0.0f;
    c->target[1] = 0.0f;
    c->reference[0] = 0.0f;
    c->reference[1] = 0.0f;

    c->open_loop[wheel] = 1U;
    c->open_loop_pwm[wheel] = pwm;
    c->open_loop_us[wheel] = 0U;
    Car_PidInc_reset(&c->pid_speed[wheel]);
    Car_Pid_reset(&c->pid_position[wheel]);
    c->link_us = 0U;                 /* 开环命令同样刷新通信看门狗 */
    return 0;
}

void Car_RaceWheels(Car_Core *c, const float rpm[2])
{
    if (!c->autonomous || c->state != CAR_RUN) return;
    c->target[0] = clamp(rpm[0], CAR_RPM_LIMIT);
    c->target[1] = clamp(rpm[1], CAR_RPM_LIMIT);
    if (rpm[0] == 0 && rpm[1] == 0) reset_motion(c);
    c->link_us = 0;
}

/* ========================================================================== */
/*                              蓝牙协议解析                                   */
/* ========================================================================== */

/*
 * 函数名称：Car_Crc8
 * 函数功能：对一个字节更新 CRC-8 校验值。
 * 参数说明：crc  —— 当前累计 CRC；首字节传 0，后续传上次返回值；
 *           byte —— 本次参与计算的字节。
 * 返回参数：更新后的 CRC-8 值。
 * 使用示例：
 *     c->crc = Car_Crc8(0, c->cmd);       // 命令字节
 *     c->crc = Car_Crc8(c->crc, length);  // 长度字节
 * 备注信息：多项式 0x07，不反射，初值 0；调用方依次送入命令、长度和负载。
 */
uint8_t Car_Crc8(uint8_t crc, uint8_t byte)
{
    unsigned i;                           /* 位循环下标，每个字节处理 8 位 */

    crc ^= byte;                          /* 当前字节并入 CRC */

    for (i = 0; i < 8; i++)
    {
        if (crc & 0x80U)                  /* 最高位为 1 时异或多项式 */
        {
            crc = (uint8_t)((crc << 1) ^ 0x07U);
        }
        else
        {
            crc = (uint8_t)(crc << 1);    /* 否则仅左移 */
        }
    }

    return crc;
}

/*
 * 函数名称：u16
 * 函数功能：从两个字节按大端顺序还原无符号 16 位整数。
 * 参数说明：p —— 指向帧中第一个字节的指针，要求 p[0]、p[1] 可读。
 * 返回参数：高字节在前组成的 uint16_t 数值。
 * 使用示例：
 *     uint16_t kp_raw = u16(c->data + 1);
 * 备注信息：蓝牙协议约定多字节字段均为大端序。
 */
static uint16_t u16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] * 256U + p[1]);
}

/*
 * 函数名称：signed_rpm
 * 函数功能：从两个字节按大端顺序还原有符号 16 位转速。
 * 参数说明：p —— 指向帧中第一个字节的指针，要求 p[0]、p[1] 可读。
 * 返回参数：有符号转速值，单位 rpm，负值表示反转。
 * 使用示例：
 *     c->target[0] = signed_rpm(c->data);
 * 备注信息：协议把转速存成补码大端序；最高位为 1 时减去 65536 还原负值。
 */
static float signed_rpm(const uint8_t *p)
{
    uint16_t n = u16(p);    /* 原始无符号 16 位值 */

    if (n & 0x8000U)        /* 最高位为 1 表示负数 */
    {
        return (float)((int32_t)n - 65536);
    }

    return (float)((int32_t)n);
}

/*
 * 函数名称：execute
 * 函数功能：执行一条已经通过长度和 CRC 校验的蓝牙命令。
 * 参数说明：c —— 控制对象，命令号保存在 c->cmd，负载保存在 c->data。
 * 返回参数：无；结果写入 c->result：0 成功，1 状态不允许，
 *           2 参数无效，3 命令无效。
 * 使用示例：
 *     // 仅在 CRC 校验通过后由 Car_FeedByte 调用
 *     if (byte == c->crc)
 *     {
 *         execute(c);
 *     }
 * 备注信息：frames 计数包含被状态或参数校验拒绝的命令，
 *           便于上位机区分“收到了帧”和“命令生效”。
 */
static void execute(Car_Core *c)
{
    if (c->autonomous) { c->result = 1; return; }
    c->frames++;                          /* 统计进入执行阶段的帧数 */
    c->result = 0;                        /* 默认结果：成功 */

    switch (c->cmd)
    {
        case 1:
            /* ARM：只有停止状态能启动；故障后必须先发送 STOP 清除故障。 */
            if (c->state != CAR_STOP)
            {
                c->result = 1;            /* 状态不允许 */
                break;
            }

            reset_motion(c);              /* 清空旧目标和历史 */
            c->link_us = 0;               /* 重置通信看门狗 */
            c->state = CAR_RUN;           /* 进入运行状态 */
            break;

        case 2:
            /* STOP：清除故障并立即归零，不等待速度规划曲线自然结束。 */
            reset_motion(c);              /* 立即清零输出 */
            c->state = CAR_STOP;          /* 回到停车状态 */
            c->fault = 0;                 /* 清除故障码 */
            c->link_us = 0;               /* 重置通信看门狗 */
            break;

        case 3:
            /* WHEELS：左、右目标转速；双轮都为零时立即复位运动状态。
             * 合法的轮速命令同时退出开环测试，恢复内环闭环控制。 */
            if (c->state != CAR_RUN)
            {
                c->result = 1;            /* 非运行状态拒绝转速命令 */
                break;
            }

            open_loop_clear(c);
            c->target[0] = clamp(signed_rpm(c->data), CAR_RPM_LIMIT);
            c->target[1] = clamp(signed_rpm(c->data + 2), CAR_RPM_LIMIT);

            if (c->target[0] == 0.0f && c->target[1] == 0.0f)
            {
                reset_motion(c);          /* 双轮零目标立即停车 */
            }

            c->link_us = 0;               /* 合法转速命令喂通信看门狗 */
            break;

        case 4:
        {
            c->result = Car_SetPid(c, c->data[0], u16(c->data + 1) / 100.0f,
                                   u16(c->data + 3) / 100.0f, u16(c->data + 5) / 100.0f);
            break;
        }

        case 5:
            /* HEARTBEAT：运行时刷新通信看门狗，不改变目标转速。 */
            if (c->state == CAR_RUN)
            {
                c->link_us = 0;           /* 运行中喂看门狗 */
            }
            break;

        case 6:
            /* OPENLOOP：自动调参用的开环 PWM 测试，轮号 + 大端 int16。 */
            c->result = Car_SetOpenLoop(c, c->data[0],
                                        signed_rpm(c->data + 1));
            break;

        default:
            c->result = 3;                /* 未知命令 */
            break;
    }
}

/*
 * 函数名称：Car_FeedByte
 * 函数功能：逐字节解析蓝牙帧。
 * 参数说明：c    —— 控制对象；
 *           byte —— 本次收到的原始字节，来源：蓝牙串口接收 FIFO。
 * 返回参数：无。
 * 使用示例：
 *     // 控制周期内从 FIFO 取出字节后逐个送入
 *     while (NEUQ_FIFO_read_byte(&rx_fifo, &byte) == NEUQ_FIFO_OK)
 *     {
 *         Car_FeedByte(&car, byte);
 *     }
 * 备注信息：帧格式为 C3 3C CMD LEN DATA CRC8。
 *           解析状态 0/1 用于寻找帧头，2/3 读取命令和长度，4 接收负载，
 *           5 校验。异常字节如果本身是 0xC3，则保留为下一帧的首字节，
 *           避免漏掉紧邻的新帧。
 */
void Car_FeedByte(Car_Core *c, uint8_t byte)
{
    /*
     * 命令 1~7 对应的负载长度：0、0、4、7、0、3、0。
     * 下标 0 占位，保证可以直接用命令号索引；先核验长度，
     * 才能保证后续写入 c->data[7] 不会越界。
     */
    static const uint8_t lengths[] =
    {
        255,                                         /* 下标 0：无效命令占位 */
        0,                                           /* 命令 1：ARM，无负载 */
        0,                                           /* 命令 2：STOP，无负载 */
        4,                                           /* 命令 3：WHEELS，左右各 2 字节 */
        7,                                           /* 命令 4：PID，轮号 + 3 个 uint16 */
        0,                                           /* 命令 5：HEARTBEAT，无负载 */
        3,                                           /* 命令 6：OPENLOOP，轮号 + int16 */
        0                                            /* 命令 7：LINK_PROBE，无负载 */
    };

    /*
     * 此处只刷新“帧字节间隔”；运行通信看门狗要等合法 ARM/WHEELS/
     * HEARTBEAT 执行时才刷新（STOP 也会清零），因此噪声或错误帧
     * 不能维持运行状态。
     */
    c->frame_us = 0;

    switch (c->parse_state)
    {
        case 0:
            /* 状态 0：寻找第一个帧头字节 0xC3。 */
            if (byte == 0xC3)
            {
                c->parse_state = 1;
            }
            break;

        case 1:
            /* 状态 1：确认第二个帧头字节 0x3C；若又收到 0xC3 则保持状态 1。 */
            if (byte == 0x3C)
            {
                c->parse_state = 2;
            }
            else if (byte == 0xC3)
            {
                c->parse_state = 1;
            }
            else
            {
                c->parse_state = 0;
            }
            break;

        case 2:
            /* 状态 2：接收命令号，并开始累计 CRC。 */
            if (byte < 1 || byte >= sizeof lengths)
            {
                c->result = 3;
                c->parse_state = (byte == 0xC3) ? 1 : 0;
                break;
            }
            c->cmd = byte;                           /* 保存命令号 */
            c->crc = Car_Crc8(0, byte);              /* CRC 从命令字节开始 */
            c->parse_state = 3;
            break;

        case 3:
            /* 状态 3：接收长度；命令号或长度非法时丢弃当前帧。 */
            if (byte != lengths[c->cmd])
            {
                c->parse_state = (byte == 0xC3) ? 1 : 0;
                c->result = 3;                       /* 帧格式错误 */
                break;
            }

            c->length = byte;                        /* 期望负载长度 */
            c->index = 0;                            /* 已收负载字节数 */
            c->crc = Car_Crc8(c->crc, byte);         /* 长度字节并入 CRC */
            c->parse_state = (byte != 0) ? 4 : 5;    /* 无负载直接校验 */
            break;

        case 4:
            /* 状态 4：逐字节接收负载，收满后进入校验状态。 */
            c->data[c->index++] = byte;              /* 保存负载字节 */
            c->crc = Car_Crc8(c->crc, byte);         /* 负载字节并入 CRC */

            if (c->index == c->length)
            {
                c->parse_state = 5;
            }
            break;

        default:
            /* 状态 5：最后一个字节应为 CRC；匹配后才会执行命令。 */
            c->parse_state = 0;

            if (byte == c->crc)
            {
                if (c->cmd == 7)
                    c->bt_probe_us = 2500000U;       /* 仅用于显示，禁止刷新运行看门狗 */
                else
                    execute(c);
            }
            else
            {
                c->result = 3;                       /* CRC 错误 */

                if (byte == 0xC3)                    /* 当前字节可能是下一帧帧头 */
                {
                    c->parse_state = 1;
                }
            }
            break;
    }
}

/* ========================================================================== */
/*                              周期控制与保护                                 */
/* ========================================================================== */

/*
 * 函数名称：Car_TickDt
 * 函数功能：使用实际控制周期更新测速、参考、PID 输出和运行保护。
 * 参数说明：c           —— 控制对象；
 *           left_count  —— 左轮本周期编码器增量（有符号，前进为正）；
 *           right_count —— 右轮本周期编码器增量（有符号，前进为正）；
 *           dt_us       —— 与计数增量对应的实际时间，单位微秒。
 * 返回参数：无；双轮控制量写入 c->output[0] 和 c->output[1]。
 * 使用示例：
 *     // 在 PIT 控制中断中，取完编码器增量后调用
 *     Car_TickDt(&car, left_delta, right_delta, dt_us);
 *     output(0, car.output[0]);
 *     output(1, car.output[1]);
 * 备注信息：周期越界、通信超时、堵转或方向异常都会触发故障并停车。
 *           本函数不操作 GPIO/PWM，硬件层负责把 output 应用到电机。
 */
void Car_TickDt(Car_Core *c, int32_t left_count, int32_t right_count,
                uint32_t dt_us)
{
    int32_t counts[2];                            /* 双轮编码器增量，0 左轮 / 1 右轮 */
    unsigned i;                                   /* 双轮循环下标 */
    float dt;                                     /* 实际控制周期，单位秒 */

    dt = (float)dt_us * 0.000001f;                /* 微秒换算为秒 */
    c->bt_probe_us = c->bt_probe_us > dt_us ? c->bt_probe_us - dt_us : 0U;

    /* 周期越界时停止控制，避免用异常采样间隔计算速度和积分。 */
    if (dt_us < CAR_MIN_CONTROL_US || dt_us > CAR_MAX_CONTROL_US)
    {
        if (c->state != CAR_FAULT) c->timing_fault_us = dt_us;
        Car_Fault(c, CAR_TIMING_FAULT);
        return;
    }

    counts[0] = left_count;                       /* 左轮增量 */
    counts[1] = right_count;                      /* 右轮增量 */

    /* 蓝牙半帧超时：长时间收不到后续字节则丢弃当前半帧。 */
    if (c->parse_state &&
        (c->frame_us += dt_us) >= CAR_FRAME_TIMEOUT_MS * 1000U)
    {
        c->parse_state = 0;                       /* 丢弃半帧，重新寻找帧头 */
    }

    /* 运行通信看门狗：运行中长时间没有有效命令则判定通信故障。 */
    if (c->state == CAR_RUN &&
        (c->link_us += dt_us) >= CAR_LINK_TIMEOUT_MS * 1000U)
    {
        Car_Fault(c, CAR_LINK_FAULT);
    }

    /* 运行状态下推进双轮共享进度的五次参考曲线。 */
    if (c->state == CAR_RUN)
    {
        if (c->autonomous) {
            /* A streaming reference changes every camera frame. A slew limiter
             * advances both wheels together without restarting a point-to-point profile. */
            float left = c->target[0] - c->profile.value[0];
            float right = c->target[1] - c->profile.value[1];
            float peak = absolute(left) > absolute(right) ? absolute(left) : absolute(right);
            float step = CAR_ACCEL_LIMIT_RPM_S * dt;
            float fraction = peak > step ? step / peak : 1.0f;
            for (i = 0; i < 2; ++i) {
                float change = (c->target[i] - c->profile.value[i]) * fraction;
                c->profile.acceleration[i] = change / dt;
                c->profile.value[i] += change;
            }
        } else CarReference_Update(&c->profile, c->target, dt);
    }

    for (i = 0; i < 2; i++)
    {
        /*
         * 先把本拍计数压入滑动窗口，再用窗口内的计数和/时间和求平均转速。
         * 单拍 5 ms 时 1 个编码器计数折算 27.3 rpm，直接送 PID 会形成
         * 27 rpm 的量化台阶；窗口平均后台阶缩小到 60/(CPR×窗口时间)。
         * 左右轮使用完全相同的窗口长度和复位时机，测速口径一致。
         */
        float raw;                                /* 窗口平均转速，rpm */
        float abs_speed;                          /* 实测转速绝对值，rpm */
#if CAR_SPEED_GUARD_ENABLE
        uint8_t guarded = 0U;                     /* 本拍测速是否被判为野值 */
#endif

#if CAR_SPEED_GUARD_ENABLE
        /*
         * 单拍计数物理上限：转速上限 × CPR × dt，再乘裕量。超过后用上一拍
         * 滤波转速推算的计数替换，避免野值进入窗口。窗口未填满时还没有
         * 可信的转速基准，先放行，让滤波器建立初值；STOP 状态不拦截，
         * 保证停车后的位置和速度仍按真实计数累计。
         */
        if (c->state == CAR_RUN&&!c->open_loop[i]&&
            c->speed_fill[i] >= CAR_SPEED_WINDOW_TICKS)
        {
            float dt_s = (float)dt_us * 0.000001f;
            float max_counts = CAR_ENCODER_CPR * CAR_RPM_MAX / 60.0f *
                               dt_s * CAR_COUNT_GUARD_FACTOR;

            if (absolute((float)counts[i]) > max_counts)
            {
                /*
                 * 连续超限达到保持拍数后放行，交给超速/堵转保护判断，
                 * 避免把真实的持续超速永久挡在窗口外；计数回到限内后
                 * 重新开始计数。
                 */
                if (c->count_guard_hold[i] <
                    CAR_SPEED_GUARD_HOLD_TICKS)
                {
                    c->count_guard_hold[i]++;
                    counts[i] = (int32_t)(c->speed[i] * CAR_ENCODER_CPR /
                                          60.0f * dt_s);
                    c->count_reject[i]++;
                    guarded = 1U;
                }
            }
            else
            {
                c->count_guard_hold[i] = 0;
            }
        }
#endif

        speed_window_push(c, i, counts[i], dt_us);
        raw = speed_window_rpm(c, i);

#if CAR_SPEED_GUARD_ENABLE
        /*
         * 窗口转速跳变守卫：正常参考加速度 200 rpm/s，5 ms 只变化 1 rpm；
         * 单拍 50 rpm 以上的跳变只可能来自坏计数或真实冲击。短于
         * CAR_SPEED_GUARD_HOLD_TICKS 的跳变保持上一次有效值；持续超过该
         * 拍数的变化视为真实变化并接受，避免把真实加减速永久挡住。
         */
        if (c->state == CAR_RUN &&!c->open_loop[i]&&
            c->speed_fill[i] >= CAR_SPEED_WINDOW_TICKS)
        {
            if (absolute(raw - c->speed_guard_raw[i]) >
                CAR_SPEED_GUARD_STEP_RPM)
            {
                /*
                 * 连续跳变达到保持拍数后接受新电平，避免真实加减速被
                 * 永久挡住；回到 50 rpm 以内后重新开始计数。
                 */
                if (c->speed_guard_hold[i] <
                    CAR_SPEED_GUARD_HOLD_TICKS)
                {
                    c->speed_guard_hold[i]++;
                    raw = c->speed_guard_raw[i];
                    c->speed_reject[i]++;
                    guarded = 1U;
                }
            }
            else
            {
                c->speed_guard_hold[i] = 0;
            }
            c->speed_guard_raw[i] = raw;
        }
        else
        {
            /* STOP 或窗口未满：守卫跟随真实值，不拦截。 */
            c->speed_guard_raw[i] = raw;
            c->speed_guard_hold[i] = 0;
        }
#endif

        /*
         * 一阶低通滤波：残量化噪声由窗口压制，这里只做最后的平滑。
         * 系数随实际周期变化。停车时也继续更新，便于上位机观察是否停住。
         */
        c->speed[i] += (dt / (CAR_SPEED_FILTER_TAU_S + dt)) *
                       (raw - c->speed[i]);

        /* 累计输出轴位置，单位圈（rev），供外环位置环使用。 */
        c->position[i] += (float)counts[i] / CAR_ENCODER_CPR;

        if (c->state != CAR_RUN)
        {
            c->output[i] = 0.0f;                  /* 非运行状态撤去输出 */
            c->output_last[i] = 0.0f;             /* 守卫保持值同步清零 */
            continue;
        }

        c->reference[i] = c->profile.value[i];    /* 本拍规划器参考转速 */

        /*
         * 开环 PWM 测试（蓝牙命令 6）：直接给执行器，绕过外环/内环。
         * 上位机必须周期刷新命令；超时后该轮立即撤去输出。开环期间
         * target/reference 已被清零，下面的超速、堵转和方向保护仍生效。
         */
        if (c->open_loop[i])
        {
            c->open_loop_us[i] += dt_us;

            if (c->open_loop_us[i] >= CAR_OPENLOOP_TIMEOUT_MS * 1000U)
            {
                c->open_loop[i] = 0U;
                c->open_loop_pwm[i] = 0.0f;
                c->output[i] = 0.0f;
                c->output_last[i] = 0.0f;
                Car_PidInc_reset(&c->pid_speed[i]);
                Car_Pid_reset(&c->pid_position[i]);
            }
            else
            {
                Car_PidInc_reset(&c->pid_speed[i]);
                Car_Pid_reset(&c->pid_position[i]);
                c->output[i] = c->open_loop_pwm[i];
                c->output_last[i] = c->output[i];
            }
        }
        else
        {
            /*
             * 外环位置式位置环。默认由 CAR_CASCADE_OUTER_ENABLE = 0 关闭：
             * 此时位置环不参与控制，内环直接跟随规划器参考转速。
             * 启用后位置误差换算为内环转速给定，并受 CAR_RPM_LIMIT 限幅。
             */
#if CAR_CASCADE_OUTER_ENABLE
            c->reference[i] = Car_Pid_update_dt(&c->pid_position[i],
                                                c->position_target[i],
                                                c->position[i],
                                                0.0f,
                                                dt);
#endif

            /*
             * 单轮目标和参考均归零后撤去驱动、清除积分；
             * 不以 PID 主动“抱住”零速。此处清零的是控制输出，
             * 车轮是否已经停住仍以 speed 反馈判断。
             */
            if (c->reference[i] == 0.0f && c->target[i] == 0.0f)
            {
                c->output[i] = 0.0f;
                c->output_last[i] = 0.0f;
                Car_PidInc_reset(&c->pid_speed[i]);
                Car_Pid_reset(&c->pid_position[i]);
            }
            else
            {
                /*
                 * 内环增量式速度 PID。外环关闭时用规划器加速度做 Ka 前馈；
                 * 外环启用时内环给定来自位置环，规划器加速度不再对应它，
                 * 因此按 0 处理。
                 */
#if CAR_CASCADE_OUTER_ENABLE
                c->output[i] = Car_PidInc_update_dt(&c->pid_speed[i],
                                                    c->reference[i],
                                                    c->speed[i],
                                                    0.0f,
                                                    dt);
#else
                c->output[i] = Car_PidInc_update_dt(&c->pid_speed[i],
                                                    c->reference[i],
                                                    c->speed[i],
                                                    c->profile.acceleration[i],
                                                    dt);
#endif
            }
        }

#if CAR_SPEED_GUARD_ENABLE
        /*
         * 本拍测速被判为野值时保持上一拍输出。PID 内部仍按保持后的测速
         * 更新，但执行器不会因为一个坏样本在 1~2 拍内掉到零，也就不会
         * 误触发反向或让 DIR 抖动。开环命令不经过这条保持逻辑。
         */
        if (guarded && !c->open_loop[i])
        {
            c->output[i] = c->output_last[i];
        }
#endif
        c->output_last[i] = c->output[i];

        /* 任一环内部检测到数值异常都会锁存 fault，这里立即转入整车故障。 */
        if (c->pid_speed[i].fault || c->pid_position[i].fault)
        {
            c->output_last[i] = 0.0f;
            Car_Fault(c, CAR_CONTROLLER_FAULT);
            continue;
        }

        abs_speed = absolute(c->speed[i]);        /* 取实测转速绝对值 */

        /* 实测转速持续超过满量程的 90% 才停车，短暂超调由 100 ms 计时滤除。 */
        if (abs_speed > CAR_OVERSPEED_RPM)
        {
            c->overspeed_us[i] += dt_us;
            if (c->overspeed_us[i] >= 100000U)
            {
                Car_Fault(c, CAR_OVERSPEED_FAULT);
                return;
            }
        }
        else
        {
            c->overspeed_us[i] = 0;
        }
        /* 转速慢速滤波：0.095 s 时间常数，用于识别换向时是否仍在减速。 */
        c->slow_speed[i] += (dt / (0.095f + dt)) *
                            (abs_speed - c->slow_speed[i]);

        /*
         * 堵转判断：
         * 参考转速和输出都比较大、实测转速却很低，且持续超过阈值时间，
         * 才判定为堵转，避免起步瞬间或短暂过载误触发。开环测试没有
         * 参考转速，因此只要输出超过门槛就按同样的条件计时。
         */
        if ((absolute(c->reference[i]) >= 40.0f || c->open_loop[i]) &&
            absolute(c->output[i]) >= CAR_STALL_PWM &&
            abs_speed < 10.0f)
        {
            c->stall_us[i] += dt_us;

            if (c->stall_us[i] >= CAR_STALL_TIMEOUT_MS * 1000U)
            {
                Car_Fault(c, CAR_STALL_FAULT);
            }
        }
        else
        {
            c->stall_us[i] = 0;                   /* 条件解除，堵转计时清零 */
        }

        /*
         * 方向异常判断：
         * 参考方向、输出方向都与实测转速方向相反，且转速较高，
         * 并持续超过阈值时间，说明车轮可能被卡住或接线方向错误。
         * slow_speed 用于排除明显减速过程中的正常换向；条件中断即清除计时。
         */
        if ((c->reference[i] * c->speed[i] < 0.0f || c->open_loop[i]) &&
            c->output[i] * c->speed[i] < 0.0f &&
            abs_speed > 100.0f &&
            abs_speed >= c->slow_speed[i] - 5.0f)
        {
            c->direction_us[i] += dt_us;

            if (c->direction_us[i] >= CAR_DIRECTION_TIMEOUT_MS * 1000U)
            {
                Car_Fault(c, CAR_DIRECTION_FAULT);
            }
        }
        else
        {
            c->direction_us[i] = 0;               /* 条件解除，方向计时清零 */
        }
    }
}

/*
 * 函数名称：Car_Tick
 * 函数功能：固定周期的兼容入口，供主机仿真或固定周期调用者使用。
 * 参数说明：c           —— 控制对象；
 *           left_count  —— 左轮本周期编码器增量；
 *           right_count —— 右轮本周期编码器增量。
 * 返回参数：无。
 * 使用示例：
 *     Car_Tick(&car, left_delta, right_delta);
 * 备注信息：实际硬件使用 Car_TickDt，传入 TIM5 测得的真实周期。
 */
void Car_Tick(Car_Core *c, int32_t left_count, int32_t right_count)
{
    Car_TickDt(c, left_count, right_count, CAR_PERIOD_MS * 1000U);
}
