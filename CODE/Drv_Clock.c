/*
 * ============================================================================
 * 文件名称：Drv_Clock.c
 * ============================================================================
 * 模块功能
 * ----------------------------------------------------------------------------
 * 用一路通用定时器做 1 MHz 自由运行计数器，给控制中断提供“实际周期”。
 *   1. Clock_Init()     ：选 TIM5 或 TIM7，分频到 1 MHz，启动计数；
 *   2. Clock_PeriodUs() ：读计数、与上次相减、返回本拍微秒数。
 *
 * 为什么不用 SysTick：
 *   SysTick 只有 1 ms 分辨率，5 ms 周期上量化误差 20%，PID 的 dt 会抖。
 *   1 µs 分辨率下 5 ms 的量化误差 0.02%，可以忽略。
 *
 * 函数列表
 *   uint8_t  Clock_Init(void)
 *   uint32_t Clock_PeriodUs(void)
 * ============================================================================
 */
#include "NEUQ_lib_include.h"
#include "Board_Config.h"
#include "Drv_Clock.h"

/* 控制时基定时器：默认 TIM5，C 方案下为 TIM7。 */
static TIM_HandleTypeDef s_clock;

/* 上一次控制周期的时基计数，单位微秒。 */
static uint32_t s_last_us;

/* 时基初始化结果：1 可用，0 初始化失败。 */
static uint8_t s_ready;

/*
 * 函数名称：Clock_Init
 * 函数功能：把控制时基配置成 1 MHz 自由运行计数器。
 * 参数说明：无。
 * 返回参数：1 成功；0 失败。
 * 备注信息：APB1 分频不为 1 时定时器时钟是 PCLK1 的两倍；预分频只能取
 *           整数，因此要求定时器时钟能被 1 MHz 整除。
 */
uint8_t Clock_Init(void)
{
    RCC_ClkInitTypeDef clocks;      /* 当前系统时钟配置 */
    uint32_t latency;               /* Flash 等待周期，本函数不使用 */
    uint32_t timer_hz;              /* 时基定时器输入时钟，Hz */

    HAL_RCC_GetClockConfig(&clocks, &latency);

    timer_hz = HAL_RCC_GetPCLK1Freq();

    /* APB1 分频不为 1 时，定时器时钟为 PCLK1 的两倍。 */
    if (clocks.APB1CLKDivider != RCC_HCLK_DIV1)
    {
        timer_hz *= 2U;
    }

    if (timer_hz < CLOCK_TICK_HZ || (timer_hz % CLOCK_TICK_HZ) != 0U)
    {
        return 0;
    }

#if ENCODER_RIGHT_ON_TIM5
    __HAL_RCC_TIM7_CLK_ENABLE();                          /* 打开 TIM7 时钟 */
    s_clock.Instance = TIM7;                              /* TIM5 让给右编码器 */
    s_clock.Init.Period = 0xFFFFU;                        /* 16 位自由运行 */
#else
    __HAL_RCC_TIM5_CLK_ENABLE();                          /* 打开 TIM5 时钟 */
    s_clock.Instance = TIM5;
    s_clock.Init.Period = 0xFFFFFFFFU;                    /* 32 位自由运行 */
#endif
    s_clock.Init.Prescaler = timer_hz / CLOCK_TICK_HZ - 1U;  /* 分频到 1 MHz */
    s_clock.Init.CounterMode = TIM_COUNTERMODE_UP;
    s_clock.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    s_clock.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_Base_Init(&s_clock) != HAL_OK)
    {
        return 0;
    }

    if (HAL_TIM_Base_Start(&s_clock) != HAL_OK)
    {
        return 0;
    }

    s_last_us = __HAL_TIM_GET_COUNTER(&s_clock);
    s_ready = 1;
    return 1;
}

/*
 * 函数名称：Clock_PeriodUs
 * 函数功能：返回距上一次调用经过的实际微秒数。
 * 参数说明：无。
 * 返回参数：本拍周期，单位微秒；时基不可用时返回 0。
 * 备注信息：无符号减法允许计数器自然回绕；TIM7 方案下先截断到 16 位。
 */
uint32_t Clock_PeriodUs(void)
{
    uint32_t now;
    uint32_t dt;

    if (!s_ready)
    {
        return 0;
    }

    now = __HAL_TIM_GET_COUNTER(&s_clock);

#if ENCODER_RIGHT_ON_TIM5
    dt = (uint16_t)((uint16_t)now - (uint16_t)s_last_us);
#else
    dt = now - s_last_us;
#endif
    s_last_us = now;

    return dt;
}
