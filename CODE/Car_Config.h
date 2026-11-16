#ifndef CAR_CONFIG_H
#define CAR_CONFIG_H

/* ============================================================================
 * 整车配置：通道分配、控制周期、指令限幅、保护阈值和默认 PID。
 * 这里的值直接影响上电后的外设初始化和运行安全，改动前须整组核对。
 * ========================================================================== */

#define CAR_UART_INDEX          3                      /* 蓝牙串口在 NEUQ 库中的设备索引，硬件层用它初始化 UART4 */
#define CAR_UART_BAUD           UART_BAUD_9600         /* 蓝牙 JDY-31 波特率，须与模块自身配置一致 */
#define CAR_VISION_UART_INDEX   2                      /* 视觉串口在 NEUQ 库中的设备索引，对应 USART3，PB10 TX / PB11 RX */
#define CAR_VISION_UART_BAUD    UART_BAUD_115200       /* 视觉波特率，须与香橙派发送端一致 */
#if CAR_UART_INDEX != 3 || CAR_VISION_UART_INDEX != 2
#error This board adapter currently supports Bluetooth UART4 and vision USART3 only
#endif
#define CAR_MOTOR_NOLOAD_RPM    1000.0f                /* 电机输出轴空载转速（rpm），仅作标称参考；实际指令限速看 CAR_RPM_LIMIT */
#define CAR_FLASH_CS            PB0                    /* V2 核心板 W25Q 片选；启动时先拉高，之后 PA5/6/7 才可复用为方向/编码器 */
#define CAR_LEFT_PWM            TIM8_CH1_PC6           /* 左轮 PWM 通道，PID 输出换算成占空比后写入 */
#define CAR_LEFT_DIR            PC7                    /* 左轮方向引脚，与 CAR_LEFT_FORWARD_HIGH 配合决定前进方向 */
#define CAR_RIGHT_PWM           TIM2_CH3_PA2           /* 右轮 PWM 通道 */
#define CAR_RIGHT_DIR           PA5                    /* 右轮方向引脚；与 Flash 片选复用，必须先拉高 CS 再配置 */
#define CAR_LEFT_FORWARD_HIGH   1                      /* 1 表示左轮前进时 DIR 输出高电平 */
#define CAR_RIGHT_FORWARD_HIGH  1                      /* 1 表示右轮前进时 DIR 输出高电平 */
#define CAR_LEFT_ENCODER_INVERT 1                      /* 1 表示左轮编码器计数取反，保证前进时增量为正 */
#define CAR_RIGHT_ENCODER_INVERT 1                     /* 1 表示右轮编码器计数取反，保证前进时增量为正 */
#define CAR_PWM_HZ              8000U                  /* PWM 载波 8 kHz，高于可听频段，兼顾驱动发热 */
#define CAR_PERIOD_MS           5U                     /* PIT 控制中断周期；实际周期由 TIM5 测量并参与时序故障判断 */
#define KEY_UP_PIN              PE10                   /* 菜单“上”键；行驶中也作为最高优先级的立即停车输入 */
#define KEY_DOWN_PIN            PE8                    /* 菜单“下”键 */
#define KEY_OK_PIN              PE7                    /* 菜单“确认/进入”键 */
#define KEY_BACK_PIN            PE9                    /* 菜单“返回/取消”键 */
#define KEY_UP_ACTIVE_LEVEL     GPIO_Low               /* 上键按下时读到的电平：低有效 */
#define KEY_DOWN_ACTIVE_LEVEL   GPIO_Low               /* 下键按下时读到的电平：低有效 */
#define KEY_OK_ACTIVE_LEVEL     GPIO_Low               /* 确认键按下时读到的电平：低有效 */
#define KEY_BACK_ACTIVE_LEVEL   GPIO_Low               /* 返回键按下时读到的电平：低有效 */
#define KEY_SCAN_PERIOD_MS      10U                    /* 按键扫描周期；PIT 按此分频调用 My_Key_scan */
#define KEY_DEBOUNCE_COUNT      3U                     /* 连续 3 次采样一致才认定状态变化，即 30 ms 消抖 */
#define KEY_REPEAT_DELAY_MS     500U                   /* 上/下键按住 500 ms 后开始连发 */
#define KEY_REPEAT_PERIOD_MS    100U                   /* 连发间隔 100 ms，用于快速移动光标或加减数值 */
#define CAR_TELEMETRY_TICKS     20U                    /* 遥测分频：每 20 个控制周期把一帧数据推入 VOFA+ FIFO */
#define CAR_ENCODER_CPR         (11.0f * 4.0f * 10.0f) /* 输出轴每圈 440 计数，测速换算的除数（11 脉冲/相 ×4 倍频 ×10 减速比） */
#define CAR_RPM_LIMIT           180.0f                 /* 目标/参考转速绝对值上限，比赛层输出也受它限幅 */
#define CAR_PWM_LIMIT           3500.0f                /* 双轮 PWM 输出绝对值上限，PID 与硬件层都按它限幅 */
#define CAR_ACCEL_LIMIT_RPM_S   200.0f                 /* 参考曲线加速度上限，抑制起步和换速冲击 */
#define CAR_JERK_LIMIT_RPM_S2    800.0f                /* 参考曲线加加速度上限，决定曲线平滑程度 */
#define CAR_MIN_CONTROL_US      1000U                  /* 周期下限：小于 1 ms 视为中断节拍异常 */
#define CAR_MAX_CONTROL_US      15000U                 /* 周期上限：大于 15 ms 视为控制节拍丢失 */
#define CAR_SPEED_FILTER_TAU_S  0.005f                 /* 实测转速一阶低通时间常数，越大越平滑但反馈延迟越大 */
#define CAR_FF_KS               0.0f                   /* 静摩擦前馈，尚未辨识，置 0 时不起作用，须在本车辨识 */
#define CAR_FF_KV               0.0f                   /* 速度前馈，尚未辨识，置 0 时不起作用 */
#define CAR_FF_KA               0.0f                   /* 加速度前馈，尚未辨识，置 0 时不起作用 */
#define CAR_PID_D_TAU_S         0.020f                 /* 微分项测量低通时间常数，抑制编码器量化噪声 */
#define CAR_PID_SETPOINT_WEIGHT 1.0f                   /* 比例项目标权重 beta：1 完整跟随目标，0 只用测量反馈 */
#define CAR_LINK_TIMEOUT_MS     500U                   /* RUN 状态下 500 ms 未收到有效命令即锁存通信故障 */
#define CAR_FRAME_TIMEOUT_MS    100U                   /* 一帧内字节间隔超过 100 ms 丢弃半帧，重新找帧头 */
#define CAR_STALL_TIMEOUT_MS    1500U                  /* 参考与输出都大、实测转速却低并持续 1.5 s 判堵转 */
#define CAR_DIRECTION_TIMEOUT_MS 1000U                 /* 指令方向与实测方向持续相反 1 s 判方向异常 */
#define CAR_DEFAULT_KP         2.0f                    /* 上电默认比例增益；Flash 无有效参数时使用 */
#define CAR_DEFAULT_KI         0.12f                   /* 上电默认积分增益 */
#define CAR_DEFAULT_KD         0.0f                    /* 上电默认微分增益 */

/* ---------------------------- 纯跟踪与虚拟舵机 ---------------------------- */
/* 以下默认值来自 HFK_STC32G144K 的纯跟踪/舵机链，按 W 差速车参数换算；
 * 换底盘或重新标定后须整组复核。 */
#define CAR_SPEED_PID_INCREMENTAL 0                    /* 1 表示速度环改用增量式 PID；0 保持位置式 */
#define CAR_PURSUIT_WHEELBASE_M 0.152f                 /* 虚拟轴距，单位米，视觉纯跟踪使用同一值 */
#define CAR_PURSUIT_TRACK_M     0.173f                 /* 左右轮中心距，单位米，与 RACE_TRACK_M 保持一致 */
#define CAR_PURSUIT_WHEEL_RADIUS_M 0.0325f             /* 车轮半径，单位米，与 RACE_RADIUS_M 保持一致 */
#define CAR_PURSUIT_LOOKAHEAD_MIN_M 0.20f              /* 最小预瞄距离，单位米 */
#define CAR_PURSUIT_LOOKAHEAD_PER_MPS 0.35f            /* 每 1 m/s 增加的预瞄距离，单位秒 */
#define CAR_PURSUIT_LOOKAHEAD_MAX_M 1.20f              /* 最大预瞄距离，单位米 */
#define CAR_PURSUIT_STEER_LIMIT_RAD 0.52f              /* 虚拟舵机转角上限，约 30 度 */
#define CAR_PURSUIT_STEER_GAIN  1.0f                   /* 虚拟舵机转角增益 */
#define CAR_PURSUIT_STEER_RATE_RAD_S 6.0f              /* 虚拟舵机转角速率上限，单位 rad/s */
#define CAR_PURSUIT_LAT_ACCEL_MPS2 0.55f               /* 横向加速度上限，单位 m/s²，与巡线曲率限速一致 */
#define CAR_PURSUIT_SPEED_LIMIT_MPS 1.20f              /* 纯跟踪输出的绝对速度上限，单位 m/s */
#define CAR_PURSUIT_PWM_CENTER_US 1500.0f              /* 虚拟舵机中位脉宽，仅阿克曼底盘使用 */
#define CAR_PURSUIT_PWM_PER_RAD_US 500.0f              /* 虚拟舵机每弧度脉宽，仅阿克曼底盘使用 */

/* ------------------------------- 环岛 / 十字 ------------------------------ */
#define CAR_RING_CONFIRM_FRAMES   2U                   /* 连续识别到环岛元素的帧数，防止单帧误检 */
#define CAR_RING_MIN_CONFIDENCE   0.50f                /* 进入环岛所需的最低置信度 */
#define CAR_RING_TRIGGER_M        0.80f                /* 保留：按元素距离触发时使用的阈值，单位米 */
#define CAR_RING_ENTER_M          0.30f                /* 进入过渡区里程，单位米 */
#define CAR_RING_INSIDE_M         1.20f                /* 环岛内侧持续里程，单位米 */
#define CAR_RING_EXIT_M           0.40f                /* 驶出过渡里程，单位米 */
#define CAR_RING_RADIUS_M         0.45f                /* 环岛等效半径，决定转向偏置，单位米 */
#define CAR_RING_SPEED_SCALE      0.80f                /* 环岛内速度缩放系数 */
#define CAR_RING_TIMEOUT_MS       12000U               /* 环岛相位超时，单位毫秒 */
#define CAR_CROSS_CONFIRM_FRAMES  2U                   /* 连续识别到十字元素的帧数 */
#define CAR_CROSS_MIN_CONFIDENCE  0.50f                /* 进入十字所需的最低置信度 */
#define CAR_CROSS_HOLD_M          0.60f                /* 十字内强制直行里程，单位米 */
#define CAR_CROSS_SPEED_SCALE     0.80f                /* 十字内速度缩放系数 */
#define CAR_CROSS_TIMEOUT_MS      5000U                /* 十字相位超时，单位毫秒 */
#endif
