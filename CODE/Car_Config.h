#ifndef CAR_CONFIG_H
#define CAR_CONFIG_H

/* 按参考原理图约定：通道 1 为左轮，通道 2 为右轮。
 * 实际轮位、方向电平及编码器符号需在车轮架空时核对。 */
/* 本板固定接线：蓝牙独占 UART_0（USART1，PA9 TX / PA10 RX）。 */
#define CAR_UART_INDEX          0
#define CAR_UART_BAUD           UART_BAUD_9600
/* 临时诊断：1 = 不做任何外设初始化，只用忙等闪核心板 LED2(PA8)，
 * 用于确证独立上电时 MCU 是否在执行本固件。测完改回 0。 */
#define CAR_LED_TEST            0
/* 香橙派视觉串口：USART3，PB10 TX / PB11 RX。
 * 与蓝牙分属不同串口，可同时启用。 */
#define CAR_VISION_UART_INDEX   2
#define CAR_VISION_UART_BAUD    UART_BAUD_115200
#ifndef CAR_VISION_ENABLE
#define CAR_VISION_ENABLE       1
#endif
#if CAR_UART_INDEX != 0 || CAR_VISION_UART_INDEX != 2
#error This board requires UART_0 for Bluetooth and UART_2 for vision
#endif
#if CAR_VISION_ENABLE != 0 && CAR_VISION_ENABLE != 1
#error CAR_VISION_ENABLE must be 0 or 1
#endif
#define CAR_MOTOR_NOLOAD_RPM    1000.0f /* 用户给定的输出轴空载转速；指令限速由 CAR_RPM_LIMIT 决定。 */
#define CAR_FLASH_CS            PB0  /* V2 核心板 W25Q 片选；复用 PA5/6/7 前拉高 */
#define CAR_LEFT_PWM            TIM8_CH1_PC6
#define CAR_LEFT_DIR            PC7
#define CAR_RIGHT_PWM           TIM2_CH3_PA2
#define CAR_RIGHT_DIR           PA5
/* 正转时方向引脚的电平，以及编码器计数是否反号；前进时反馈应为正。 */
#define CAR_LEFT_FORWARD_HIGH   1
#define CAR_RIGHT_FORWARD_HIGH  1
#define CAR_LEFT_ENCODER_INVERT 1
#define CAR_RIGHT_ENCODER_INVERT 0
#define CAR_PWM_HZ              8000U
/* 标称控制周期 5 ms；每 20 次控制周期发送一次遥测，标称 100 ms。 */
#define CAR_PERIOD_MS           5U
/* task2 board keys (PE10 is also the immediate stop input while driving). */
#define KEY_UP_PIN              PE10
#define KEY_DOWN_PIN            PE8
#define KEY_OK_PIN              PE7
#define KEY_BACK_PIN            PE9
#define KEY_UP_ACTIVE_LEVEL     GPIO_Low
#define KEY_DOWN_ACTIVE_LEVEL   GPIO_Low
#define KEY_OK_ACTIVE_LEVEL     GPIO_Low
#define KEY_BACK_ACTIVE_LEVEL   GPIO_Low
#define KEY_SCAN_PERIOD_MS      10U
#define KEY_DEBOUNCE_COUNT      3U
#define KEY_REPEAT_DELAY_MS     500U
#define KEY_REPEAT_PERIOD_MS    100U
#define CAR_TELEMETRY_TICKS     20U
/* 接线图标注电机轴每圈每相 11 个脉冲；TIM3/TIM4 正交四倍频，减速比 10:1。
 * 输出轴每圈理论计数 440；左右轮各向前转 10 圈，核对带符号累计计数。 */
#define CAR_ENCODER_CPR         (11.0f * 4.0f * 10.0f)
/* 指令限速（rpm）、PWM 数值限幅、参考加速度（rpm/s）及加加速度（rpm/s²）。 */
#define CAR_RPM_LIMIT           180.0f
/* NEUQ 库 PWM_MAX=10000 对应 100%，3500 即最大 35% 占空比。 */
#define CAR_PWM_LIMIT           3500.0f
#define CAR_ACCEL_LIMIT_RPM_S   200.0f
#define CAR_JERK_LIMIT_RPM_S2    800.0f
/* 允许的实测控制周期范围（微秒）；超出范围触发时序故障。 */
#define CAR_MIN_CONTROL_US      1000U
#define CAR_MAX_CONTROL_US      15000U
/* 测速滑动窗口长度（控制周期数）。内环测速不再用单拍计数，而是对最近
 * CAR_SPEED_WINDOW_TICKS 拍的计数和时间求和：
 *     转速 = 窗口计数和 × 60 / (CPR × 窗口时间和)
 * 5 ms 单拍时 1 个计数 = 27.3 rpm，PID 只能在这条 27 rpm 的台阶上工作；
 * 8 拍（40 ms）后 1 个计数 = 3.4 rpm，测速噪声下降约 8 倍。
 * 窗口越长分辨率越高，但测速延迟约为窗口的一半；调大时需同步复核增益。 */
#define CAR_SPEED_WINDOW_TICKS  8U
#if CAR_SPEED_WINDOW_TICKS < 1U || CAR_SPEED_WINDOW_TICKS > 32U
#error CAR_SPEED_WINDOW_TICKS must be 1..32
#endif
/* 实测速度一阶低通时间常数（秒）。窗口已经滤掉大部分量化噪声，这里只做
 * 残余抑制；值越大越平滑，同时反馈延迟越大。 */
#define CAR_SPEED_FILTER_TAU_S  0.015f
/* 前馈系数须在本车辨识：Ks 为 PWM 数值，Kv 为 PWM/rpm，Ka 为 PWM/(rpm/s)。 */
#define CAR_FF_KS               0.0f
#define CAR_FF_KV               0.0f
#define CAR_FF_KA               0.0f
/* 微分测量低通时间常数（秒）；目标权重只影响比例项，1 表示使用完整目标。 */
#define CAR_PID_D_TAU_S         0.020f
#define CAR_PID_SETPOINT_WEIGHT 1.0f
/* 各项超时单位为毫秒：有效运行通信、帧字节间隔、堵转、方向异常。 */
#define CAR_LINK_TIMEOUT_MS     500U
#define CAR_FRAME_TIMEOUT_MS    100U
#define CAR_STALL_TIMEOUT_MS    1500U
#define CAR_DIRECTION_TIMEOUT_MS 1000U
/* Ki/Kd 沿用 5 ms 离散增益，具体换算见 Car_PID.h。
 * 这组默认值配合 8 拍测速窗口使用：单计数噪声约 3.4 rpm，Kp=8 时对应
 * 约 27 PWM（0.27% 占空比）的残余纹波，既能明显快于旧默认值，又不会
 * 像 Kp=100 那样把噪声放大成开关式抖动。转速—增益参考表见
 * Docs/Speed_PID_Tuning_Guide.md。 */
#define CAR_DEFAULT_KP         8.0f
#define CAR_DEFAULT_KI         0.30f
#define CAR_DEFAULT_KD         0.0f
/* 串级 PID：外环位置环用位置式（Car_Pid_*），内环速度环用增量式
 * （Car_PidInc_*）。内环增益沿用上面的 CAR_DEFAULT_KP/KI/KD，蓝牙命令 4
 * 和板端 PID 菜单只写内环。
 *
 * CAR_CASCADE_OUTER_ENABLE = 0 时外环完全旁路：规划器参考转速直接送内环，
 * 与改动前的速度闭环行为一致。外环的反馈是累计位置（输出轴圈数），输出是
 * 内环转速给定；位置目标 position_target[] 目前没有蓝牙协议来源，默认保持
 * 为 0，启用前必须先接入任务层的位置指令并按实车重新整定外环增益。 */
#ifndef CAR_CASCADE_OUTER_ENABLE
#define CAR_CASCADE_OUTER_ENABLE 0
#endif
#if CAR_CASCADE_OUTER_ENABLE != 0 && CAR_CASCADE_OUTER_ENABLE != 1
#error CAR_CASCADE_OUTER_ENABLE must be 0 or 1
#endif
/* 外环位置式 PID 的占位增益，单位：Kp 为 rpm/圈，Ki/Kd 沿用 5 ms 离散约定。
 * 默认全 0，即使误开外环也不会突然驱动车轮；启用前必须重新标定。 */
#define CAR_POS_DEFAULT_KP     0.0f
#define CAR_POS_DEFAULT_KI     0.0f
#define CAR_POS_DEFAULT_KD     0.0f
#endif
