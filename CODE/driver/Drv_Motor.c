/*
 * ============================================================================
 * 文件名称：Drv_Motor.c
 * ============================================================================
 * 模块功能
 * ----------------------------------------------------------------------------
 * 双轮直流电机的执行器驱动，只做三件事：
 *   1. Motor_HoldOff()：初始化前把 EN 脚拉低，电机强制不转；
 *   2. Motor_Init()   ：方向引脚定电平 + PWM 通道起步占空比 0；
 *   3. Motor_Apply()  ：把带符号 PWM 数值变成“方向电平 + 占空比”。
 *
 * 换向保护
 *   方向改变时先写占空比 0 并置 waiting，下一拍再翻转方向引脚，
 *   保证 H 桥上管先关断一个控制周期（5 ms），避免直通炸管。
 *
 * 函数列表
 *   void Motor_Init(void)
 *   void Motor_HoldOff(void)
 *   void Motor_Apply(unsigned wheel, float pwm)
 * ============================================================================
 */
#include "NEUQ_lib_include.h"
#include "Board_Config.h"
#include "Drv_Motor.h"

/* 左右轮 PWM 通道，来源：Drv_Motor.h，下标 0 左、1 右。 */
static const PWM_Ports_ENUM s_pwm[2] =
{
    MOTOR_LEFT_PWM,
    MOTOR_RIGHT_PWM
};

/* 左右轮方向引脚，来源：Drv_Motor.h，下标 0 左、1 右。 */
static const GPIO_Ports_ENUM s_dir[2] =
{
    MOTOR_LEFT_DIR,
    MOTOR_RIGHT_DIR
};

/* 各轮“前进”对应的方向引脚电平，来源：Drv_Motor.h，下标 0 左、1 右。 */
static const uint8_t s_forward_high[2] =
{
    MOTOR_LEFT_FORWARD_HIGH,
    MOTOR_RIGHT_FORWARD_HIGH
};

/* 当前逻辑方向：1 表示反向。上电按“前进”初始化。 */
static uint8_t s_reverse[2];

/* 1 表示已经撤去 PWM、正在等待下一拍切换方向引脚。 */
static uint8_t s_waiting[2];

/*
 * 函数名称：Motor_Init
 * 函数功能：方向引脚置停止电平，初始化两路 PWM 并给出 0 占空比。
 * 参数说明：无。
 * 返回参数：无。
 * 备注信息：调用前必须已把 BOARD_FLASH_CS 拉高，否则 PA5 上会同时出现
 *           Flash SCK 与本模块的方向电平。
 */
void Motor_Init(void)
{
    unsigned i;

    for (i = 0; i < 2; i++)
    {
        /* 方向脚先给低电平，等第一条有效命令到来再决定方向。 */
        NEUQ_GPIO_init(s_dir[i], GPIO_Mode_Output_PushPull, GPIO_Low);
        s_reverse[i] = s_forward_high[i];
        s_waiting[i] = 0;
        NEUQ_PWM_init(s_pwm[i], MOTOR_PWM_HZ, 0);
    }
}

/*
 * 函数名称：Motor_HoldOff
 * 函数功能：在 PWM 初始化前把两路 EN 脚拉低，保证上电过程电机不转。
 * 参数说明：无。
 * 返回参数：无。
 * 备注信息：参考驱动板上两个 EN 均为高有效，因此这里直接给低电平。
 *           必须在时钟配置、参数读取、LED 自检之前调用。
 */
void Motor_HoldOff(void)
{
    NEUQ_GPIO_init(PC6, GPIO_Mode_Output_PushPull, GPIO_Low);   /* 左轮 EN 低 */
    NEUQ_GPIO_init(PA2, GPIO_Mode_Output_PushPull, GPIO_Low);   /* 右轮 EN 低 */
}

/*
 * 函数名称：Motor_Apply
 * 函数功能：把带符号 PWM 数值写到指定车轮。
 * 参数说明：wheel —— 0 左轮、1 右轮（越界直接返回）；
 *           pwm   —— 带符号 PWM 数值，正数正转、负数反转。
 * 返回参数：无。
 * 备注信息：0 输出只撤 PWM、不翻方向，避免停车时方向脚抖动；
 *           换向必须先撤 PWM 一个控制周期，再翻方向脚。
 */
void Motor_Apply(unsigned wheel, float pwm)
{
    uint8_t next;        /* 期望方向：1 反向，0 正向 */
    uint32_t duty;       /* 四舍五入后的整数占空比 */

    if (wheel > 1U)
    {
        return;
    }

    next = (pwm < 0.0f) ? 1U : 0U;

    /* 零输出：撤去 PWM、结束换向等待，方向保持不变。 */
    if (pwm == 0.0f)
    {
        NEUQ_PWM_set_duty(s_pwm[wheel], 0);
        s_waiting[wheel] = 0;
        return;
    }

    /* 方向需要改变时，先撤 PWM，等下一拍再翻转方向引脚。 */
    if (next != s_reverse[wheel])
    {
        if (!s_waiting[wheel])
        {
            NEUQ_PWM_set_duty(s_pwm[wheel], 0);
            s_waiting[wheel] = 1;
            return;
        }

        /* forward_high 为 1 表示高电平前进；next 为 1 时取反。 */
        if (next)
        {
            NEUQ_GPIO_write(s_dir[wheel],
                            s_forward_high[wheel] ? GPIO_Low : GPIO_High);
        }
        else
        {
            NEUQ_GPIO_write(s_dir[wheel],
                            s_forward_high[wheel] ? GPIO_High : GPIO_Low);
        }

        s_reverse[wheel] = next;
    }

    s_waiting[wheel] = 0;

    /* 四舍五入到整数占空比；幅值上限由 NEUQ 库 PWM_MAX 限制。 */
    duty = (uint32_t)((pwm < 0.0f ? -pwm : pwm) + 0.5f);
    NEUQ_PWM_set_duty(s_pwm[wheel], duty);
}
