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
/* 香橙派视觉串口：UART4，PC10 TX / PC11 RX。
 * 与蓝牙分属不同串口，可同时启用。 */
#define CAR_VISION_UART_INDEX   3
#define CAR_VISION_UART_BAUD    UART_BAUD_115200
#ifndef CAR_VISION_ENABLE
#define CAR_VISION_ENABLE       1
#endif
#if CAR_UART_INDEX != 0 || CAR_VISION_UART_INDEX != 3
#error This board requires UART_0 for Bluetooth and UART_3 for vision
#endif
#if CAR_VISION_ENABLE != 0 && CAR_VISION_ENABLE != 1
#error CAR_VISION_ENABLE must be 0 or 1
#endif
#define CAR_MOTOR_NOLOAD_RPM    1000.0f /* 输出轴空载转速，作为转速满量程。 */
#define CAR_FLASH_CS            PB0  /* V2 核心板 W25Q 片选；复用 PA5/6/7 前拉高 */
#define CAR_LEFT_PWM            TIM8_CH1_PC6
#define CAR_LEFT_DIR            PC7
#define CAR_RIGHT_PWM           TIM2_CH3_PA2
#define CAR_RIGHT_DIR           PA5
/* 方向标定（架空车轮，分别给 +60 rpm 后核对）：
 *   1. 车轮应向前转；若反转，改对应轮的 *_FORWARD_HIGH；
 *   2. 遥测实测转速应为正；若方向正确但实测为负，改对应轮的 *_ENCODER_INVERT。
 * 两个开关只能改一个：同时改会互相抵消，闭环仍会正反馈跑到限幅并触发
 * 方向/超速保护。前进时反馈必须为正，这是内环 PID 和方向保护的前提。
 * 2026-11-18 实车核对：左轮前进电平 High，编码器取反；右轮方向引脚完全
 * 相反（High 时反转），因此前进电平改为 Low，编码器不取反。 */
#define CAR_LEFT_FORWARD_HIGH   1
#define CAR_RIGHT_FORWARD_HIGH  0
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
/* 编码器输入滤波（IC1F/IC2F，取值 0..15）。0 表示不滤波；0xF 对应
 * fDTS/32、N=8，约 3 µs（TIM3/TIM4 时钟 84 MHz）。1000 rpm 时一个计数
 * 边沿间隔约 136 µs，该滤波不会影响正常计数，只用于压掉电机 PWM、地弹和
 * 线缆耦合引入的窄毛刺。电机干扰优先在这一层滤除，不要用降低 Kp 掩盖。 */
#define CAR_ENCODER_IC_FILTER   0xFU
#if CAR_ENCODER_IC_FILTER > 15U
#error CAR_ENCODER_IC_FILTER must be 0..15
#endif
/* C 方案：把右编码器从与板载 W25Q 复用的 TIM3 PA6/PA7 移到 TIM5 PA0/PA1。
 * 默认 0 = 保持现有接线（右编码器 TIM3 PA6/PA7，TIM5 作 1 MHz 控制时基）。
 * 改为 1 之前必须先把右编码器 A/B 两线从 PA6/PA7 改接到 PA0/PA1；此时
 * 控制时基自动切换到 TIM7（16 位、1 MHz，约 65.5 ms 回绕），左编码器仍为
 * TIM4 PB6/PB7 不动。PA0/PA1 当前未被本工程占用。 */
#ifndef CAR_RIGHT_ENCODER_ON_TIM5
#define CAR_RIGHT_ENCODER_ON_TIM5 0
#endif
#if CAR_RIGHT_ENCODER_ON_TIM5 != 0 && CAR_RIGHT_ENCODER_ON_TIM5 != 1
#error CAR_RIGHT_ENCODER_ON_TIM5 must be 0 or 1
#endif
/* D 方案：测速合理性守卫。电机物理上限是 CAR_RPM_MAX，单拍计数超过
 * CAR_COUNT_GUARD_FACTOR 倍物理上限即判为野值，用上一拍滤波速度推算的
 * 计数替换；窗口转速单拍跳变超过 CAR_SPEED_GUARD_STEP_RPM、且持续不足
 * CAR_SPEED_GUARD_HOLD_TICKS 拍时，保持上一次有效值。这样单拍坏数据不会
 * 直接进入内环把 PWM 打到零。守卫事件计入 Car_Core.count_reject /
 * speed_reject，不占用 16 通道遥测；可在 Keil 调试器中观察。 */
/* 低速起步/堵转时的坏样本可能持续 200~300 ms；旧的 50 rpm/20 拍会在
 * 20~60 rpm 的起步过程中先冻结 100 ms 再放行。放宽到 150 rpm、保持 3 拍：
 * 单拍尖峰仍由测速窗口和编码器滤波压制，守卫只拦截明显不合理的跳变，
 * 不再把真实的低速变化冻成一个 100 ms 的平台。 */
#define CAR_SPEED_GUARD_ENABLE      1
#define CAR_COUNT_GUARD_FACTOR      1.5f
#define CAR_SPEED_GUARD_STEP_RPM    150.0f
#define CAR_SPEED_GUARD_HOLD_TICKS  3U
#if CAR_SPEED_GUARD_ENABLE != 0 && CAR_SPEED_GUARD_ENABLE != 1
#error CAR_SPEED_GUARD_ENABLE must be 0 or 1
#endif
#if CAR_SPEED_GUARD_HOLD_TICKS < 1U
#error CAR_SPEED_GUARD_HOLD_TICKS must be >= 1
#endif
/* 安全限幅统一取硬件满量程的 90%，给电池电压波动、测速噪声和闭环超调
 * 留 10% 余量。PWM 满量程为 NEUQ 库 PWM_MAX = 10000（100% 占空比），
 * 转速满量程为电机空载 1000 rpm。限幅和超速报错必须同时修改，否则两者
 * 会互相矛盾。 */
#define CAR_PWM_MAX             10000.0f
#define CAR_RPM_MAX             CAR_MOTOR_NOLOAD_RPM
#define CAR_LIMIT_FRACTION      0.90f
/* 指令限速与 PWM 数值限幅：900 rpm / 9000，即满量程的 90%。 */
#define CAR_RPM_LIMIT           (CAR_RPM_MAX * CAR_LIMIT_FRACTION)
#define CAR_PWM_LIMIT           (CAR_PWM_MAX * CAR_LIMIT_FRACTION)
/* 超速报错：实测转速持续超过满量程的 90% 即判定反馈或驱动失控。 */
#define CAR_OVERSPEED_RPM       (CAR_RPM_MAX * CAR_LIMIT_FRACTION)
/* 堵转判定中“控制器确实在驱动”的 PWM 门槛，15% 满量程。 */
#define CAR_STALL_PWM           1500.0f
/* 自动调参用的开环 PWM 测试通道（蓝牙命令 6）。
 * 只在 RUN 状态接受，绝对值上限取满量程的 30%，给轮速环一个安全阶跃；
 * 上位机必须每 CAR_OPENLOOP_TIMEOUT_MS 内刷新一次，否则该轮自动撤去输出。
 * 开环期间仍保留超速、堵转和方向保护；进入前必须把车轮架空。 */
#define CAR_OPENLOOP_PWM_LIMIT  3000.0f
#define CAR_OPENLOOP_TIMEOUT_MS 500U
#if CAR_OPENLOOP_TIMEOUT_MS < (CAR_PERIOD_MS * 2U)
#error CAR_OPENLOOP_TIMEOUT_MS must allow at least two control periods
#endif
/* 参考加速度（rpm/s）及加加速度（rpm/s²）。 */
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
/* 前馈系数须在本车辨识：Ks 为 PWM 数值，Kv 为 PWM/rpm，Ka 为 PWM/(rpm/s)。
 * 2026-11-18 第二轮开环阶跃（车轮架空、每档从静止起步）：
 *   左轮 5% 稳定 20.8 rpm，5.5% 30.0，6% 33.5，…，12% 87.4；
 *   右轮 5%/6% 不动，7% 仅 23.1 且会间歇卡住，8% 稳定 35.6。
 * 线性拟合：左轮死区约 2.3%、右轮约 4.1%，两轮增益都约 11 PWM/rpm。
 * Ks 取“从静止能稳定起步”的占空比，而不是运行死区：左 5%、右 8%。
 * 取大了会在低速目标起步时先冲出去（例：左 12% 对应约 87 rpm），
 * 取小了右轮会间歇不动。以 100 为步长微调，判据是 20 rpm 目标约 100 ms
 * 内起步，冲过头则回调。 */
#define CAR_FF_KS_LEFT          500.0f   /* 5% PWM，左轮静摩擦前馈 */
#define CAR_FF_KS_RIGHT         800.0f   /* 8% PWM，右轮静摩擦前馈 */
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
