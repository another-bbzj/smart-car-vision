#ifndef PROTO_BLUETOOTH_H
#define PROTO_BLUETOOTH_H

/*
 * ============================================================================
 * 文件名称：Proto_Bluetooth.h
 * ============================================================================
 * 模块功能：蓝牙（JDY-31，USART1）命令下行协议：帧格式、CRC、命令号、超时。
 *
 *   外设资源：USART1，9600 8N1，PA9 TX / PA10 RX。
 *
 *   线上帧格式（不可改）：
 *     C3 3C | CMD | LEN | DATA[0..LEN-1] | CRC8
 *     CRC8：多项式 0x07，初值 0，覆盖 CMD、LEN、DATA。
 *
 *   命令与负载（不可改）：
 *     1 ARM      无负载    启动（STOP -> RUN）
 *     2 STOP     无负载    停车 / 清故障
 *     3 WHEELS   4 字节    左右轮目标转速，两个大端 int16，单位 rpm
 *     4 PID      7 字节    轮号 0/1 + Kp/Ki/Kd（大端 uint16，放大 100 倍）
 *     5 HEARTBEAT 无负载   心跳，刷新通信看门狗
 *     6 OPENLOOP 3 字节    轮号 0/1 + 大端 int16 PWM（架空调参用）
 *   result：0 成功；1 状态不允许；2 参数无效；3 帧或命令无效。
 *
 *   运行视角
 *     运行中超过 BT_LINK_TIMEOUT_MS 没有有效命令即判 CAR_LINK_FAULT；
 *     自动行驶（CAR_MOTION_RACE）期间所有命令都回 result=1，不改变车轮。
 * ============================================================================
 */

#include <stdint.h>

#include "Task_Fsm.h"

/* ---------------------------------------------------------------------------
 * 外设：蓝牙帧结构与 CRC
 * 功能：帧头、最大负载与 CRC 多项式。负载最长 7 字节（命令 4）。
 * 取值：0xC3/0x3C；0x07。改这里等于改协议，禁止。
 * ------------------------------------------------------------------------- */
#define BT_HEAD0                 0xC3U    /* 帧头第 1 字节 */
#define BT_HEAD1                 0x3CU    /* 帧头第 2 字节 */
#define BT_MAX_PAYLOAD           7U       /* 单帧最大负载字节数（命令 4） */
#define BT_CRC8_POLY             0x07U    /* CRC-8 多项式，不反射、初值 0 */

/* 命令号（不可改，与上位机/网页调试台一一对应）。 */
#define BT_CMD_ARM               1U       /* 启动：STOP -> RUN */
#define BT_CMD_STOP              2U       /* 停车 / 清故障 */
#define BT_CMD_WHEELS            3U       /* 双轮目标转速，两个大端 int16 */
#define BT_CMD_PID               4U       /* 内环 PID 增益，轮号 + 三个大端 uint16 */
#define BT_CMD_HEARTBEAT         5U       /* 心跳，刷新通信看门狗 */
#define BT_CMD_OPENLOOP          6U       /* 开环 PWM 测试，轮号 + 大端 int16 */

/* ---------------------------------------------------------------------------
 * 外设：协议超时（单位毫秒）
 * 功能：运行通信看门狗、帧字节间隔。
 * 取值：通信 500 ms（上位机 10 Hz 遥测 + 心跳足够）；帧间隔 100 ms
 *       （半帧超过它说明链路丢字节，丢弃重同步）。
 * ------------------------------------------------------------------------- */
#define BT_LINK_TIMEOUT_MS       500U     /* 运行中无有效命令的允许时长，毫秒 */
#define BT_FRAME_TIMEOUT_MS      100U     /* 帧内字节间隔上限，毫秒 */

/* ---------------------------------------------------------------------------
 * 外设：开环 PWM 测试（命令 6）
 * 功能：自动调参用的安全阶跃：绝对值上限取满量程的 30%，且必须周期刷新。
 * 取值：上限 3000（30% 占空比）；超时 500 ms（≥ 2 个控制周期）。
 * 注意：只在 RUN 接受，首次进入要求双轮目标为零且实测低于 10 rpm。
 * ------------------------------------------------------------------------- */
#define BT_OPENLOOP_PWM_LIMIT    3000.0f  /* 开环 PWM 绝对值上限 */
#define BT_OPENLOOP_TIMEOUT_MS   500U     /* 开环命令刷新超时，毫秒 */
#if BT_OPENLOOP_TIMEOUT_MS < (BOARD_PERIOD_MS * 2U)
#error BT_OPENLOOP_TIMEOUT_MS must allow at least two control periods
#endif

/*
 * 函数名称：Car_Crc8
 * 函数功能：对一个字节更新 CRC-8。
 * 参数说明：crc  —— 首字节传 0，之后传上次返回值；
 *           byte —— 本次参与计算的字节。
 * 返回参数：更新后的 CRC-8。
 * 备注信息：多项式 0x07、不反射；调用方依次送入 CMD、LEN、DATA。
 */
uint8_t Car_Crc8(uint8_t crc, uint8_t byte);

/*
 * 函数名称：Car_FeedByte
 * 函数功能：输入一个蓝牙字节；只有完整合法帧才执行命令。
 * 参数说明：c    —— 控制对象；
 *           byte —— 刚收到的原始字节。
 * 返回参数：无。
 * 备注信息：结果通过 c->result / state / frames 观察；异常字节如果本身是
 *           0xC3 会保留为下一帧首字节，避免漏掉紧邻的新帧。
 */
void Car_FeedByte(Car_Core *c, uint8_t byte);

#endif
