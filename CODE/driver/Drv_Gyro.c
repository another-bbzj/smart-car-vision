/*
 * ============================================================================
 * 文件名称：Drv_Gyro.c
 * ============================================================================
 * 模块功能
 * ----------------------------------------------------------------------------
 * MPU6050 的 Z 轴角速度读取与零偏标定：
 *   1. Gyro_Init()  ：初始化器件，记录在位标志；
 *   2. Gyro_Poll()  ：主循环里每 10 ms 读一次，前 100 个静止样本求零偏，
 *                     之后把去零偏的角速度写进快照（临界区保护）；
 *   3. Gyro_*()     ：给控制中断/菜单提供只读快照。
 *
 * 为什么标定要求“车静止”：
 *   零偏是静止时的读数均值。标定期间车身一动，均值里就混进真实角速度，
 *   整段比赛的 yaw 都会漂，所以一动就清零重来。
 *
 * 函数列表
 *   void    Gyro_Init(void)
 *   void    Gyro_Poll(uint8_t chassis_still)
 *   uint8_t Gyro_Present(void)
 *   uint8_t Gyro_Fresh(void)
 *   float   Gyro_RateRad(void)
 *   float   Gyro_BiasRad(void)
 * ============================================================================
 */
#include "NEUQ_lib_include.h"
#include "Board_Config.h"
#include "Drv_Gyro.h"
#include <math.h>

/* 1 表示 MPU6050 初始化成功。 */
static uint8_t s_present;

/* 1 表示 s_rate 是有效快照（控制中断读它）。 */
static volatile uint8_t s_valid;

/* 去零偏后的 Z 轴角速度，rad/s；由主循环写、控制中断读。 */
static volatile float s_rate;

/* 快照时间，单位毫秒。 */
static volatile uint32_t s_stamp;

/* 零偏估计，rad/s。 */
static float s_bias;

/* 已累计的标定样本数与样本和。 */
static unsigned s_samples;
static float s_sum;

/* 上一次采样时间，单位毫秒（仅主循环访问）。 */
static uint32_t s_last_ms;

/*
 * 函数名称：Gyro_Init
 * 函数功能：初始化 MPU6050 并记录在位标志。
 * 参数说明：无。
 * 返回参数：无。
 * 备注信息：必须在 I²C 初始化之后调用；失败不影响其他模块启动。
 */
void Gyro_Init(void)
{
    s_present = NEUQ_MPU6050_init();
    s_valid = 0;
    s_samples = 0;
    s_sum = 0.0f;
    s_bias = 0.0f;
}

/*
 * 函数名称：Gyro_Poll
 * 函数功能：读一次 Z 轴角速度；标定阶段累计静止样本求零偏。
 * 参数说明：chassis_still —— 1 表示整车静止。
 * 返回参数：无。
 * 备注信息：读数失败把 s_valid 清零（数据过期），巡线自动退回轮速推算；
 *           标定完成后以临界区方式一次性更新快照，避免读到半新半旧的值。
 */
void Gyro_Poll(uint8_t chassis_still)
{
    uint8_t raw[2];          /* 原始高/低字节 */
    float rate;              /* 本次读数，rad/s */
    uint32_t now = HAL_GetTick();

    if (!s_present || now - s_last_ms < GYRO_POLL_MS)
    {
        return;
    }
    s_last_ms = now;

    if (!NEUQ_Hard_IIC_read_8bit_reg(MPU6050_HARD_IIC_USE, MPU6050_ADDR,
                                     MPU6050_GYRO_ZOUT_H, raw, 2))
    {
        s_valid = 0;
        return;
    }

    rate = GYRO_SIGN * MPU6050_gyro_transition((int16_t)((raw[0] << 8) | raw[1]))
         * 0.017453293f;

    /* 标定阶段：只有静止且角速度足够小才累计样本。 */
    if (s_samples < GYRO_BIAS_SAMPLES)
    {
        if (!chassis_still || fabsf(rate) > GYRO_STILL_RATE_RAD_S)
        {
            s_samples = 0;
            s_sum = 0.0f;
            return;
        }
        s_sum += rate;
        if (++s_samples == GYRO_BIAS_SAMPLES)
        {
            s_bias = s_sum / (float)GYRO_BIAS_SAMPLES;
        }
        return;
    }

    {
        uint32_t mask = __get_PRIMASK();
        __disable_irq();
        s_rate = rate - s_bias;
        s_stamp = now;
        s_valid = 1;
        __set_PRIMASK(mask);
    }
}

/*
 * 函数名称：Gyro_Present
 * 函数功能：查询 MPU6050 是否在位。
 * 参数说明：无。
 * 返回参数：1 在位；0 不在位。
 */
uint8_t Gyro_Present(void)
{
    return s_present;
}

/*
 * 函数名称：Gyro_Fresh
 * 函数功能：查询陀螺快照是否在 GYRO_STALE_MS 内更新过。
 * 参数说明：无。
 * 返回参数：1 可用；0 过期。
 */
uint8_t Gyro_Fresh(void)
{
    return (s_valid && (HAL_GetTick() - s_stamp) <= GYRO_STALE_MS) ? 1U : 0U;
}

/*
 * 函数名称：Gyro_RateRad
 * 函数功能：取去零偏后的 Z 轴角速度。
 * 参数说明：无。
 * 返回参数：rad/s，逆时针为正。
 */
float Gyro_RateRad(void)
{
    return s_rate;
}

/*
 * 函数名称：Gyro_Samples
 * 函数功能：返回零偏标定已累计的样本数。
 * 参数说明：无。
 * 返回参数：0..GYRO_BIAS_SAMPLES。
 */
unsigned Gyro_Samples(void)
{
    return s_samples;
}

/*
 * 函数名称：Gyro_BiasRad
 * 函数功能：取当前零偏估计，供 LCD 显示与调试。
 * 参数说明：无。
 * 返回参数：rad/s。
 */
float Gyro_BiasRad(void)
{
    return s_bias;
}
