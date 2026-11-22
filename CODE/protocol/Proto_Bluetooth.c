/*
 * ============================================================================
 * 文件名称：Proto_Bluetooth.c
 * ============================================================================
 * 模块功能
 * ----------------------------------------------------------------------------
 * 蓝牙命令下行协议（帧格式与命令语义不可改）：
 *   帧格式 C3 3C CMD LEN DATA CRC8；CRC-8 多项式 0x07、初值 0、覆盖 CMD..DATA；
 *   命令 1 ARM / 2 STOP / 3 WHEELS(4B) / 4 PID(7B) / 5 HEARTBEAT / 6 OPENLOOP(3B)。
 *
 *   1. Car_Crc8()      ：逐字节累计 CRC-8；
 *   2. u16 / signed_rpm：大端还原无符号/有符号 16 位；
 *   3. execute()       ：按命令号执行，写 result（0 成功 1 状态不允许 2 参数无效）；
 *   4. Car_FeedByte()  ：6 状态解析机，凑满一帧并校验后才执行。
 *
 * 运行视角
 *   接收中断只入 FIFO；控制中断每拍最多解析 32 字节，逐字节送 Car_FeedByte()。
 *   自动行驶（motion == CAR_MOTION_RACE）期间拒绝蓝牙命令，result=1；
 *   手动与自动共用同一条执行通路，区别只在“谁写 target”。
 *
 * 函数列表
 *   uint8_t Car_Crc8(uint8_t crc, uint8_t byte)
 *   static uint16_t u16(const uint8_t *p)
 *   static float signed_rpm(const uint8_t *p)
 *   static void execute(Car_Core *c)
 *   void Car_FeedByte(Car_Core *c, uint8_t byte)
 * ============================================================================
 */
#include "Proto_Bluetooth.h"
#include "Task_Fsm.h"
#include <string.h>

/*
 * 函数名称：clamp
 * 函数功能：把浮点数限制到 [-bound, +bound]。
 * 参数说明：x —— 待限幅值；bound —— 正的上限。
 * 返回参数：限幅后的值。
 * 备注信息：协议层只用于转速与 PWM 输入校验，与任务层的同名工具等价。
 */
static float clamp(float x, float bound)
{
    if (x > bound)
    {
        return bound;
    }
    if (x < -bound)
    {
        return -bound;
    }
    return x;
}

/*
 * 函数名称：Car_Crc8
 * 函数功能：对一个字节更新 CRC-8 校验值。
 * 参数说明：crc  —— 当前累计 CRC；首字节传 0，后续传上次返回值；
 *           byte —— 本次参与计算的字节。
 * 返回参数：更新后的 CRC-8 值。
 * 使用示例：
 *     c->crc = Car_Crc8(0, c->cmd);       // 命令字节
 *     c->crc = Car_Crc8(c->crc, length);  // 长度字节
 * 备注信息：多项式 0x07，不反射，初值 0；调用方依次送入命令、长度和负载。
 */
uint8_t Car_Crc8(uint8_t crc, uint8_t byte)
{
    unsigned i;                           /* 位循环下标，每个字节处理 8 位 */

    crc ^= byte;                          /* 当前字节并入 CRC */

    for (i = 0; i < 8; i++)
    {
        if (crc & 0x80U)                  /* 最高位为 1 时异或多项式 */
        {
            crc = (uint8_t)((crc << 1) ^ 0x07U);
        }
        else
        {
            crc = (uint8_t)(crc << 1);    /* 否则仅左移 */
        }
    }

    return crc;
}

/*
 * 函数名称：u16
 * 函数功能：从两个字节按大端顺序还原无符号 16 位整数。
 * 参数说明：p —— 指向帧中第一个字节的指针，要求 p[0]、p[1] 可读。
 * 返回参数：高字节在前组成的 uint16_t 数值。
 * 使用示例：
 *     uint16_t kp_raw = u16(c->data + 1);
 * 备注信息：蓝牙协议约定多字节字段均为大端序。
 */
static uint16_t u16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] * 256U + p[1]);
}

/*
 * 函数名称：signed_rpm
 * 函数功能：从两个字节按大端顺序还原有符号 16 位转速。
 * 参数说明：p —— 指向帧中第一个字节的指针，要求 p[0]、p[1] 可读。
 * 返回参数：有符号转速值，单位 rpm，负值表示反转。
 * 使用示例：
 *     c->target[0] = signed_rpm(c->data);
 * 备注信息：协议把转速存成补码大端序；最高位为 1 时减去 65536 还原负值。
 */
static float signed_rpm(const uint8_t *p)
{
    uint16_t n = u16(p);    /* 原始无符号 16 位值 */

    if (n & 0x8000U)        /* 最高位为 1 表示负数 */
    {
        return (float)((int32_t)n - 65536);
    }

    return (float)((int32_t)n);
}

/*
 * 函数名称：execute
 * 函数功能：执行一条已经通过长度和 CRC 校验的蓝牙命令。
 * 参数说明：c —— 控制对象，命令号保存在 c->cmd，负载保存在 c->data。
 * 返回参数：无；结果写入 c->result：0 成功，1 状态不允许，
 *           2 参数无效，3 命令无效。
 * 使用示例：
 *     // 仅在 CRC 校验通过后由 Car_FeedByte 调用
 *     if (byte == c->crc)
 *     {
 *         execute(c);
 *     }
 * 备注信息：frames 计数包含被状态或参数校验拒绝的命令，
 *           便于上位机区分“收到了帧”和“命令生效”。
 */
static void execute(Car_Core *c)
{
    if (c->motion == CAR_MOTION_RACE) { c->result = 1; return; }
    c->frames++;                          /* 统计进入执行阶段的帧数 */
    c->result = 0;                        /* 默认结果：成功 */

    switch (c->cmd)
    {
        case 1:
            /* ARM：只有停止状态能启动；故障后必须先发送 STOP 清除故障。 */
            if (c->state != CAR_STOP)
            {
                c->result = 1;            /* 状态不允许 */
                break;
            }

            Car_ClearMotion(c);              /* 清空旧目标和历史 */
            c->link_us = 0;               /* 重置通信看门狗 */
            c->state = CAR_RUN;           /* 进入运行状态 */
            break;

        case 2:
            /* STOP：清除故障并立即归零，不等待速度规划曲线自然结束。 */
            Car_ClearMotion(c);              /* 立即清零输出 */
            c->state = CAR_STOP;          /* 回到停车状态 */
            c->fault = 0;                 /* 清除故障码 */
            c->link_us = 0;               /* 重置通信看门狗 */
            break;

        case 3:
            /* WHEELS：左、右目标转速；双轮都为零时立即复位运动状态。
             * 合法的轮速命令同时退出开环测试，恢复内环闭环控制。 */
            if (c->state != CAR_RUN)
            {
                c->result = 1;            /* 非运行状态拒绝转速命令 */
                break;
            }

            Car_OpenLoopClear(c);
            c->target[0] = clamp(signed_rpm(c->data), BOARD_RPM_LIMIT);
            c->target[1] = clamp(signed_rpm(c->data + 2), BOARD_RPM_LIMIT);
            c->motion = CAR_MOTION_BT;   /* 蓝牙手动调速：只换目标来源，执行通路不变 */

            if (c->target[0] == 0.0f && c->target[1] == 0.0f)
            {
                Car_ClearMotion(c);          /* 双轮零目标立即停车 */
            }

            c->link_us = 0;               /* 合法转速命令喂通信看门狗 */
            break;

        case 4:
        {
            c->result = Car_SetPid(c, c->data[0], u16(c->data + 1) / 100.0f,
                                   u16(c->data + 3) / 100.0f, u16(c->data + 5) / 100.0f);
            break;
        }

        case 5:
            /* HEARTBEAT：运行时刷新通信看门狗，不改变目标转速。 */
            if (c->state == CAR_RUN)
            {
                c->link_us = 0;           /* 运行中喂看门狗 */
            }
            break;

        case 6:
            /* OPENLOOP：自动调参用的开环 PWM 测试，轮号 + 大端 int16。 */
            c->result = Car_SetOpenLoop(c, c->data[0],
                                        signed_rpm(c->data + 1));
            break;

        default:
            c->result = 3;                /* 未知命令 */
            break;
    }
}

/*
 * 函数名称：Car_FeedByte
 * 函数功能：逐字节解析蓝牙帧。
 * 参数说明：c    —— 控制对象；
 *           byte —— 本次收到的原始字节，来源：蓝牙串口接收 FIFO。
 * 返回参数：无。
 * 使用示例：
 *     // 控制周期内从 FIFO 取出字节后逐个送入
 *     while (NEUQ_FIFO_read_byte(&rx_fifo, &byte) == NEUQ_FIFO_OK)
 *     {
 *         Car_FeedByte(&car, byte);
 *     }
 * 备注信息：帧格式为 C3 3C CMD LEN DATA CRC8。
 *           解析状态 0/1 用于寻找帧头，2/3 读取命令和长度，4 接收负载，
 *           5 校验。异常字节如果本身是 0xC3，则保留为下一帧的首字节，
 *           避免漏掉紧邻的新帧。
 */
void Car_FeedByte(Car_Core *c, uint8_t byte)
{
    /*
     * 命令 1~6 对应的负载长度：0、0、4、7、0、3。
     * 下标 0 占位，保证可以直接用命令号索引；先核验长度，
     * 才能保证后续写入 c->data[7] 不会越界。
     */
    static const uint8_t lengths[7] =
    {
        255,                                         /* 下标 0：无效命令占位 */
        0,                                           /* 命令 1：ARM，无负载 */
        0,                                           /* 命令 2：STOP，无负载 */
        4,                                           /* 命令 3：WHEELS，左右各 2 字节 */
        7,                                           /* 命令 4：PID，轮号 + 3 个 uint16 */
        0,                                           /* 命令 5：HEARTBEAT，无负载 */
        3                                            /* 命令 6：OPENLOOP，轮号 + int16 */
    };

    /*
     * 此处只刷新“帧字节间隔”；运行通信看门狗要等合法 ARM/WHEELS/
     * HEARTBEAT 执行时才刷新（STOP 也会清零），因此噪声或错误帧
     * 不能维持运行状态。
     */
    c->frame_us = 0;

    switch (c->parse_state)
    {
        case 0:
            /* 状态 0：寻找第一个帧头字节 0xC3。 */
            if (byte == 0xC3)
            {
                c->parse_state = 1;
            }
            break;

        case 1:
            /* 状态 1：确认第二个帧头字节 0x3C；若又收到 0xC3 则保持状态 1。 */
            if (byte == 0x3C)
            {
                c->parse_state = 2;
            }
            else if (byte == 0xC3)
            {
                c->parse_state = 1;
            }
            else
            {
                c->parse_state = 0;
            }
            break;

        case 2:
            /* 状态 2：接收命令号，并开始累计 CRC。 */
            if (byte < 1 || byte > 6)
            {
                c->result = 3;
                c->parse_state = (byte == 0xC3) ? 1 : 0;
                break;
            }
            c->cmd = byte;                           /* 保存命令号 */
            c->crc = Car_Crc8(0, byte);              /* CRC 从命令字节开始 */
            c->parse_state = 3;
            break;

        case 3:
            /* 状态 3：接收长度；命令号或长度非法时丢弃当前帧。 */
            if (byte != lengths[c->cmd])
            {
                c->parse_state = (byte == 0xC3) ? 1 : 0;
                c->result = 3;                       /* 帧格式错误 */
                break;
            }

            c->length = byte;                        /* 期望负载长度 */
            c->index = 0;                            /* 已收负载字节数 */
            c->crc = Car_Crc8(c->crc, byte);         /* 长度字节并入 CRC */
            c->parse_state = (byte != 0) ? 4 : 5;    /* 无负载直接校验 */
            break;

        case 4:
            /* 状态 4：逐字节接收负载，收满后进入校验状态。 */
            c->data[c->index++] = byte;              /* 保存负载字节 */
            c->crc = Car_Crc8(c->crc, byte);         /* 负载字节并入 CRC */

            if (c->index == c->length)
            {
                c->parse_state = 5;
            }
            break;

        default:
            /* 状态 5：最后一个字节应为 CRC；匹配后才会执行命令。 */
            c->parse_state = 0;

            if (byte == c->crc)
            {
                execute(c);                          /* CRC 正确，执行命令 */
            }
            else
            {
                c->result = 3;                       /* CRC 错误 */

                if (byte == 0xC3)                    /* 当前字节可能是下一帧帧头 */
                {
                    c->parse_state = 1;
                }
            }
            break;
    }
}

/* ========================================================================== */
/*                              周期控制与保护                                 */
/* ========================================================================== */

