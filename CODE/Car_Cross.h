#ifndef CAR_CROSS_H
#define CAR_CROSS_H

#include <stdint.h>

#include "Car_Vision.h"

/*
 * ============================================================================
 * 文件名称：Car_Cross.h
 * ============================================================================
 * 模块功能：十字路口状态机，移植自 HFK_STC32G144K 的 runCross() 两相位逻辑
 *           （CROSS_READY -> CROSS_IN）。
 *
 *   HFK 在 CROSS_IN 中从单侧边线重建远端中线并继续巡线，穿过路口后退出。
 *   W 车的视觉已把十字识别为 VISION_ELEMENT_CROSS，因此这里改为：
 *     IDLE   -> 连续两帧识别到 CROSS 后进入；
 *     INSIDE -> 保持直行（曲率强制为 0），走过固定里程后退出。
 *   十字期间不采用视觉曲率，避免路口内的假边线把车带偏。
 *
 *   本模块只输出“是否直行”和速度缩放，不直接产生转速。
 * ============================================================================
 */

/* 十字相位。 */
typedef enum
{
    CAR_CROSS_IDLE,   /* 未进入十字 */
    CAR_CROSS_INSIDE, /* 正在穿过路口，强制直行 */
    CAR_CROSS_FAULT   /* 超时，需人工停车清除 */
} CarCrossPhase;

/* 十字状态机对象。 */
typedef struct
{
    CarCrossPhase phase;     /* 当前相位 */
    uint8_t confirm;         /* 连续识别到 CROSS 的帧数 */
    uint8_t have_sequence;   /* 1 表示 last_sequence 有效 */
    uint16_t last_sequence;  /* 上一帧视觉序号，用于去重 */
    float entry_distance_m;  /* 进入时的累计里程，单位米 */
    uint32_t entered_ms;     /* 进入时刻，单位毫秒 */
} CarCross;

/* 十字输出。 */
typedef struct
{
    uint8_t active;       /* 1 表示当前处于十字相位 */
    uint8_t fault;        /* 1 表示超时故障 */
    uint8_t straight;     /* 1 表示强制直行 */
    float speed_scale;    /* 速度缩放系数，0~1 */
} CarCrossOutput;

/**
 * @brief 清空十字状态机。
 * @param c 十字对象。
 */
void CarCross_Reset(CarCross *c);

/**
 * @brief 推进一次十字状态机。
 * @param c          十字对象。
 * @param o          当前视觉观测，可为 NULL。
 * @param distance_m 本场累计里程，单位米。
 * @param now_ms     当前毫秒时基。
 * @return 本周期的直行输出；未激活时 active = 0。
 */
CarCrossOutput CarCross_Step(CarCross *c, const CarVisionObservation *o,
                             float distance_m, uint32_t now_ms);

#endif
