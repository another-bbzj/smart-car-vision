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
/* 实测速度低通时间常数（秒）；值越大越平滑，同时反馈延迟越大。 */
#define CAR_SPEED_FILTER_TAU_S  0.005f
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
/* Ki/Kd 沿用 5 ms 离散增益，具体换算见 Car_PID.h。 */
#define CAR_DEFAULT_KP         2.5f
#define CAR_DEFAULT_KI         0.15f
#define CAR_DEFAULT_KD         0.0f
#endif
