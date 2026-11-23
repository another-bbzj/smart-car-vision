/*
 * ============================================================================
 * 文件名称：Proto_Vofa.c
 * ============================================================================
 * 模块功能
 * ----------------------------------------------------------------------------
 * 蓝牙上行遥测的发送缓冲。帧（A5/A6）在 Platform_Main.c 里组装，
 * 本模块只做两件事：
 *   1. Vofa_PushFrame()：控制中断里把一整帧放进发送 FIFO（微秒级返回）；
 *   2. Vofa_Poll()     ：主循环里慢慢把 FIFO 字节送出去。
 *
 * 为什么分 push / poll：
 *   9600 波特率下 1 字节约 1 ms，一帧 7 字节约 7 ms。在控制中断里直接发
 *   会把 5 ms 的控制周期拖垮；入队 + 主循环发送才能既不断流又不影响控制。
 *   缓冲放不下时整帧丢弃，绝不发半帧，上位机不会解析错位。
 *
 * 帧格式（由 Platform_Main 组装，本模块不解释内容）：
 *   A5 | 左 rpm int16 | 右 rpm int16 | 状态 | XOR     （7 字节，100 Hz）
 *   A6 | 左位置 int16 | 右位置 int16 | 状态 | XOR     （7 字节，20 Hz）
 *
 * 函数列表
 *   void    Vofa_Init(uint8_t uart_dev)
 *   uint8_t Vofa_PushFrame(const uint8_t *data, uint32_t length)
 *   void    Vofa_Poll(void)
 * ============================================================================
 */
#include "NEUQ_lib_include.h"
#include "Proto_Vofa.h"

/* 发送缓冲区：由 NEUQ_FIFO 管理，中断写入、主循环读取。 */
static uint8_t s_buffer[VOFA_FIFO_SIZE];

/* 发送 FIFO 对象，来源：Vofa_Init 初始化。 */
static NEUQ_FIFO_t s_fifo;

/* 遥测串口编号，来源：Vofa_Init 参数；默认 UART_0。 */
static UART_DEV_ENUM s_uart = UART_0;

/*
 * 函数名称：Vofa_Init
 * 函数功能：初始化遥测发送模块，保存遥测串口并建立发送 FIFO。
 * 参数说明：uart_dev —— NEUQ 库串口编号，必须与蓝牙串口一致。
 * 返回参数：无。
 * 使用示例：
 *     NEUQ_Uart_init((UART_DEV_ENUM)BOARD_UART_BT_INDEX, BOARD_UART_BT_BAUD);
 *     Vofa_Init(BOARD_UART_BT_INDEX);
 * 备注信息：只初始化软件 FIFO，串口本身由 Car_Main 初始化。
 */
void Vofa_Init(uint8_t uart_dev)
{
    s_uart = (UART_DEV_ENUM)uart_dev;    /* 保存遥测串口编号 */
    NEUQ_FIFO_init(&s_fifo, s_buffer, sizeof(s_buffer));
}

/*
 * 函数名称：Vofa_PushFrame
 * 函数功能：把一帧已经组装好的字节整体放入发送 FIFO（控制中断中调用）。
 * 参数说明：data   —— 完整帧的首地址；
 *           length —— 帧长度，字节。
 * 返回参数：1 整帧放入；0 参数非法或剩余空间不足，整帧丢弃。
 * 使用示例：
 *     uint8_t frame[7] = {0xA5, ...};
 *     Vofa_PushFrame(frame, sizeof(frame));
 * 备注信息：使用 write_all，空间足够才整帧写入，空间不足时一个字节都不写。
 */
uint8_t Vofa_PushFrame(const uint8_t *data, uint32_t length)
{
    if (data == NULL || length == 0U || length > VOFA_FIFO_SIZE)
    {
        return 0U;                                         /* 参数非法，整帧丢弃 */
    }

    return (NEUQ_FIFO_write_all(&s_fifo, (uint8_t *)data, length) == NEUQ_FIFO_OK)
         ? 1U
         : 0U;
}

/*
 * 函数名称：Vofa_Poll
 * 函数功能：从发送 FIFO 向串口推进最多 4 字节（主循环中调用）。
 * 参数说明：无。
 * 返回参数：无。
 * 使用示例：
 *     while (1)
 *     {
 *         Vofa_Poll();
 *     }
 * 备注信息：限制单次串口等待时间，供主循环及时轮询陀螺仪与 LCD。
 *           帧的剩余字节留在 FIFO 中，由后续调用按原顺序继续发送。
 */
void Vofa_Poll(void)
{
    uint32_t count;                            /* 本次准备发送的字节数 */
    uint8_t byte;                              /* 从 FIFO 读出的临时字节 */

    count = NEUQ_FIFO_get_used(&s_fifo);
    /* At 9600 baud four bytes occupy about 4 ms. Service IMU and LCD
     * between slices even when an entire telemetry frame is queued. */
    if (count > 4U) count = 4U;

    while (count > 0U &&
           NEUQ_FIFO_read_byte(&s_fifo, &byte) == NEUQ_FIFO_OK)
    {
        NEUQ_Uart_write_byte(s_uart, byte);    /* 发送一个字节 */
        count--;                               /* 剩余字节数减一 */
    }
}
