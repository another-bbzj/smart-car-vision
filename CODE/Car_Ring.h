#ifndef CAR_RING_H
#define CAR_RING_H

#include <stdint.h>

#include "Car_Vision.h"

/*
 * ============================================================================
 * 文件名称：Car_Ring.h
 * ============================================================================
 * 模块功能：环岛状态机，移植自 HFK_STC32G144K 的 runLRound()/runRRound()
 *           四相位逻辑（START -> IN -> END -> EXIT），并适配 W 车的视觉元素。
 *
 *   HFK 用图像中左右边线长度和角点判断环岛相位；W 车的香橙派视觉已把
 *   环岛识别成 VISION_ELEMENT_RING，因此这里改用车载里程和视觉元素驱动：
 *     IDLE   -> 连续两帧识别到 RING 后进入，
 *     ENTER  -> 短暂过渡，输出半幅转向偏置，
 *     INSIDE -> 按固定环岛半径输出转向偏置，
 *     EXIT   -> 偏置线性衰减回 0，回到巡线。
 *   转向偏置以曲率增量形式交给 Race，正值为左转，与视觉曲率符号一致。
 *
 *   本模块只输出偏置，不直接产生转速；超时或元素持续不可用由 Race 判故障。
 * ============================================================================
 */

/* 环岛相位。 */
typedef enum
{
    CAR_RING_IDLE,   /* 未进入环岛 */
    CAR_RING_ENTER,  /* 已确认元素，进入过渡区 */
    CAR_RING_INSIDE, /* 环岛内侧，输出完整转向偏置 */
    CAR_RING_EXIT,   /* 驶出环岛，偏置衰减 */
    CAR_RING_FAULT   /* 超时，需人工停车清除 */
} CarRingPhase;

/* 环岛状态机对象。 */
typedef struct
{
    CarRingPhase phase;      /* 当前相位 */
    int8_t side;             /* 环岛方向：+1 左环岛，-1 右环岛 */
    uint8_t confirm;         /* 连续识别到 RING 的帧数 */
    uint8_t have_sequence;   /* 1 表示 last_sequence 有效 */
    uint16_t last_sequence;  /* 上一帧视觉序号，用于去重 */
    float entry_distance_m;  /* 进入时的累计里程，单位米 */
    uint32_t entered_ms;     /* 进入时刻，单位毫秒 */
} CarRing;

/* 环岛输出。 */
typedef struct
{
    uint8_t valid;           /* 1 表示当前处于环岛相位 */
    uint8_t fault;           /* 1 表示超时故障 */
    float curvature_bias;    /* 叠加到视觉曲率上的偏置，单位 1/米，左转为正 */
    float speed_scale;       /* 速度缩放系数，0~1 */
} CarRingOutput;

/**
 * @brief 清空环岛状态机。
 * @param r 环岛对象。
 */
void CarRing_Reset(CarRing *r);

/**
 * @brief 推进一次环岛状态机。
 * @param r            环岛对象。
 * @param o            当前视觉观测，可为 NULL。
 * @param distance_m   本场累计里程，单位米。
 * @param now_ms       当前毫秒时基。
 * @param side_hint    视觉给出的环岛方向提示：+1 左、-1 右、0 未知。
 * @return 本周期的偏置输出；未激活时 valid = 0。
 */
CarRingOutput CarRing_Step(CarRing *r, const CarVisionObservation *o,
                           float distance_m, uint32_t now_ms,
                           int8_t side_hint);

#endif
