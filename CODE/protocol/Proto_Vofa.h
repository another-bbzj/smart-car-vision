#ifndef PROTO_VOFA_H
#define PROTO_VOFA_H
/*
 * ============================================================================
 * 文件名称：Proto_Vofa.h
 * ============================================================================
 * 模块功能：向上位机 VOFA+ 发送波形数据（JustFloat 协议）。
 *
 *   帧格式：N 个 float（每个 4 字节，小端）+ 帧尾 00 00 80 7F。
 *   VOFA+ 把第 1 个 float 画成 I0，第 2 个画成 I1……
 *   本工程的通道顺序固定在 Platform_Main.c 的遥测装配处，共 16 通道。
 *
 *   运行视角
 *     控制中断每 20 拍（100 ms）组一帧 -> Vofa_Push()；主循环 Vofa_Poll()
 *     逐字节发送。蓝牙带宽 960 B/s，遥测 680 B/s，留出下行命令的余量。
 * ============================================================================
 */

#include <stdint.h>

#define VOFA_MAX_CHANNELS       (16U)       // 一帧最多几个通道
#define VOFA_FIFO_SIZE          (2048U)     // 发送缓冲大小（字节），主循环忙于刷屏、写 Flash 时先存在这里

void    Vofa_Init(uint8_t uart_dev);                        // uart_dev 取 UART_0 ~ UART_3，串口需要已经初始化
uint8_t Vofa_Push(const float *data, uint8_t channels);     // 中断中调用：放入一帧，1 成功 0 缓冲满被丢弃
void    Vofa_Poll(void);                                    // 主循环中调用：把缓冲里的数据发出去

#endif
