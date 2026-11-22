/*
 * ============================================================================
 * 文件名称：Drv_Encoder.c
 * ============================================================================
 * 模块功能
 * ----------------------------------------------------------------------------
 * 正交编码器输入的底层读法：
 *   1. Encoder_Init()  ：NEUQ 库编码器模式初始化 + IC1F/IC2F 输入滤波 +
 *                        记录上电基准计数；
 *   2. Encoder_Delta() ：读出 16 位计数器，用模差算出本拍增量，按配置取反。
 *
 * 为什么不用“读后清零”：
 *   读后清零会把两次读取之间的脉冲丢掉一部分；模差只依赖前后两次读数，
 *   计数连续，回绕也能正确还原符号。
 *
 * 函数列表
 *   static void encoder_input_filter(TIM_TypeDef *tim, uint32_t filter)
 *   void        Encoder_Init(void)
 *   int32_t     Encoder_Delta(unsigned wheel)
 * ============================================================================
 */
#include "NEUQ_lib_include.h"
#include "Board_Config.h"
#include "Drv_Encoder.h"

/* 编码器定时器句柄，来源：NEUQ_Encoder_init 初始化。 */
extern TIM_HandleTypeDef Encoder_TIM3_Handle;
extern TIM_HandleTypeDef Encoder_TIM4_Handle;
#if ENCODER_RIGHT_ON_TIM5
extern TIM_HandleTypeDef Encoder_TIM5_Handle;
#endif

/* 上一次读到的 16 位计数值，下标 0 左、1 右。 */
static uint16_t s_previous[2];

/*
 * 函数名称：encoder_input_filter
 * 函数功能：为编码器定时器的 TI1/TI2 设置输入滤波。
 * 参数说明：tim    —— TIM3 / TIM4 / TIM5；
 *           filter —— 0..15，写入 CCMR1 的 IC1F/IC2F。
 * 返回参数：无。
 * 备注信息：编码器模式 TI12 只用 TI1/TI2，对应 CCMR1 的 IC1F(bit7:4) 与
 *           IC2F(bit15:12)；只改这两个字段，保留 CC1S/CC2S 的输入模式。
 */
static void encoder_input_filter(TIM_TypeDef *tim, uint32_t filter)
{
    uint32_t f = filter & 0x0FU;
    tim->CCMR1 = (tim->CCMR1 & ~0x0000F0F0U) | (f << 4) | (f << 12);
}

/*
 * 函数名称：Encoder_Init
 * 函数功能：初始化左右轮编码器、写入输入滤波、建立计数基准。
 * 参数说明：无。
 * 返回参数：无。
 * 备注信息：顺序不能变——先 NEUQ_Encoder_init 打开定时器，再写滤波，
 *           最后读一次计数作基准，首拍增量才只包含此后产生的脉冲。
 */
void Encoder_Init(void)
{
    NEUQ_Encoder_init(Encoder_TIM4,
                      Encoder_TIM4_PhaseA_PB6,
                      Encoder_TIM4_PhaseB_PB7);
#if ENCODER_RIGHT_ON_TIM5
    NEUQ_Encoder_init(Encoder_TIM5,
                      Encoder_TIM5_PhaseA_PA0,
                      Encoder_TIM5_PhaseB_PA1);
#else
    NEUQ_Encoder_init(Encoder_TIM3,
                      Encoder_TIM3_PhaseA_PA6,
                      Encoder_TIM3_PhaseB_PA7);
#endif

    /* 两路都加输入滤波：PIT 未启动，不会把配置瞬间的半拍数据送进速度环。 */
    encoder_input_filter(TIM4, ENCODER_IC_FILTER);
#if ENCODER_RIGHT_ON_TIM5
    encoder_input_filter(TIM5, ENCODER_IC_FILTER);
#else
    encoder_input_filter(TIM3, ENCODER_IC_FILTER);
#endif

    /* 进中断之前建立基准：首拍只计算此后产生的脉冲。 */
    s_previous[0] = (uint16_t)__HAL_TIM_GET_COUNTER(&Encoder_TIM4_Handle);
#if ENCODER_RIGHT_ON_TIM5
    s_previous[1] = (uint16_t)__HAL_TIM_GET_COUNTER(&Encoder_TIM5_Handle);
#else
    s_previous[1] = (uint16_t)__HAL_TIM_GET_COUNTER(&Encoder_TIM3_Handle);
#endif
}

/*
 * 函数名称：Encoder_Delta
 * 函数功能：读取计数器并计算本周期有符号增量。
 * 参数说明：wheel —— 0 左轮、1 右轮。
 * 返回参数：本周期计数增量，正数表示车轮向前。
 * 备注信息：16 位模差最高位为 1 表示负数，减 65536 还原符号；
 *           再按 ENCODER_*_INVERT 取反，对上“向前为正”的口径。
 */
int32_t Encoder_Delta(unsigned wheel)
{
    uint16_t now;        /* 本次读取的 16 位计数值 */
    uint16_t diff;       /* 两次读取的 16 位模差 */
    int32_t value;       /* 还原符号后的增量 */

    if (wheel > 1U)
    {
        return 0;
    }

    now = (uint16_t)__HAL_TIM_GET_COUNTER(
            (wheel == 0U) ? &Encoder_TIM4_Handle :
#if ENCODER_RIGHT_ON_TIM5
                            &Encoder_TIM5_Handle
#else
                            &Encoder_TIM3_Handle
#endif
          );
    diff = (uint16_t)(now - s_previous[wheel]);
    s_previous[wheel] = now;

    value = (diff & 0x8000U) ? ((int32_t)diff - 65536) : (int32_t)diff;

    if ((wheel == 0U) ? ENCODER_LEFT_INVERT : ENCODER_RIGHT_INVERT)
    {
        value = -value;
    }

    return value;
}
