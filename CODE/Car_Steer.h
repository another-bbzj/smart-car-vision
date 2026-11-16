#ifndef CAR_STEER_H
#define CAR_STEER_H

#include <stdint.h>

/*
 * ============================================================================
 * 文件名称：Car_Steer.h
 * ============================================================================
 * 模块功能：虚拟转向舵机模型，移植自 HFK_STC32G144K 的舵机控制链并做差速适配。
 *
 *   HFK 车使用真实舵机：转角命令 × 增益后叠加中位，再输出 PWM。
 *   当前 W 车为差速底盘，没有舵机，因此本模块只保留“转角命令 -> 限幅 ->
 *   速率限制 -> 虚拟转角”的模型，后续可用 CarSteer_YawRate() 换算成
 *   横摆角速度，再由 Car_Pursuit 转成左右轮转速；若实车改阿克曼底盘，
 *   直接使用 CarSteer_PwmUs() 驱动舵机即可，无需改上层。
 *
 *   角度约定：弧度，正值为左转（与 Race/vision 曲率正方向一致）。
 *   本模块是纯算法，不访问 GPIO/PWM。
 * ============================================================================
 */

/* 虚拟转向舵机对象。 */
typedef struct
{
    float gain;             /* 转角命令缩放系数，来源：CarSteer_Init 参数 */
    float limit_rad;        /* 实际转角绝对值上限，单位弧度 */
    float rate_limit_rad_s; /* 转角变化率上限，单位 rad/s，模拟舵机响应速度 */
    float pwm_center_us;    /* 中位脉宽，单位微秒，仅阿克曼底盘使用 */
    float pwm_per_rad_us;   /* 每弧度脉宽增量，单位微秒/rad，仅阿克曼底盘使用 */
    float angle_rad;        /* 速率限制后的实际转角，单位弧度 */
    float target_rad;       /* 限幅后的目标转角，单位弧度 */
    uint8_t fault;          /* 1 表示参数或输入非法，已锁存 */
} CarSteer;

/**
 * @brief 初始化虚拟舵机并校验参数。
 * @param s              舵机对象。
 * @param gain           转角命令缩放系数，范围 [0, 10]。
 * @param limit_rad      转角上限，范围 (0, 1.57]。
 * @param rate_limit_rad_s 转角速率上限，范围 (0, 100]。
 * @param pwm_center_us  中位脉宽，范围 [500, 2500]。
 * @param pwm_per_rad_us 每弧度脉宽，范围 [0, 2000]。
 * @return 1 成功；0 参数非法，对象已清零并置 fault。
 */
uint8_t CarSteer_Init(CarSteer *s, float gain, float limit_rad,
                      float rate_limit_rad_s, float pwm_center_us,
                      float pwm_per_rad_us);

/**
 * @brief 清空动态状态与故障，保留配置。
 * @param s 舵机对象。
 */
void CarSteer_Reset(CarSteer *s);

/**
 * @brief 设置目标转角：乘以增益并限幅，不立即改变实际转角。
 * @param s         舵机对象。
 * @param angle_rad 命令转角，单位弧度，正值为左转。
 * @return 1 成功；0 对象故障或输入非有限数（此时置 fault）。
 */
uint8_t CarSteer_SetAngle(CarSteer *s, float angle_rad);

/**
 * @brief 按实际周期推进转角速率限制。
 * @param s    舵机对象。
 * @param dt_s 周期，单位秒，允许 (0, 0.1]；越界时保持当前转角。
 * @return 速率限制后的实际转角，单位弧度。
 */
float CarSteer_Update(CarSteer *s, float dt_s);

/**
 * @brief 实际转角对应的舵机脉宽，供阿克曼底盘直接输出。
 * @param s 舵机对象。
 * @return 脉宽，单位微秒。
 */
float CarSteer_PwmUs(const CarSteer *s);

/**
 * @brief 由实际转角和前进速度计算横摆角速度。
 * @param s          舵机对象。
 * @param speed_mps  前进速度，单位 m/s。
 * @param wheelbase_m 轴距，单位米。
 * @return 横摆角速度，单位 rad/s，正值为左转；参数非法时返回 0。
 */
float CarSteer_YawRate(const CarSteer *s, float speed_mps, float wheelbase_m);

#endif
