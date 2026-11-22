#ifndef PID_STORAGE_H
#define PID_STORAGE_H

/*
 * ============================================================================
 * 文件名称：Pid_Storage.h
 * ============================================================================
 * 模块功能：内环 PID 增益的掉电保存：记录编码、双扇区轮换写入、上电恢复。
 *
 *   存储介质：片内 Flash 的 S6/S7 两个扇区（不占用外部 SPI Flash 引脚）。
 *   记录内容：左右轮各 3 个增益，共 6 个 float，一条记录 64 字节。
 *   运行视角：菜单 “Save parameters” -> 平台层确认停车静止 -> 写入；
 *             上电 Car_Main() 读取，读不到就用 Pid.h 的默认值。
 * ============================================================================
 */

#include <stdint.h>

#include "Task_Fsm.h"

/* ---------------------------------------------------------------------------
 * 记录格式
 * 功能：一条记录 64 字节（magic + 版本 + 6 个增益 + CRC，按 16 字对齐填充）。
 * 取值：64。改布局必须同时改 PID_PARAM_MAGIC。
 * ------------------------------------------------------------------------- */
#define PID_PARAM_RECORD_BYTES   64U      /* 单条记录的字节数 */

/* ---------------------------------------------------------------------------
 * 外设：片内 Flash 扇区
 * 功能：两个 S6/S7 扇区轮换写入，保证写坏一个还有上一个有效记录。
 * 取值：基址 0x08040000；扇区大小 0x20000（128 KB）。
 * 注意：与 Keil 链接脚本预留的 IROM 区间一致，改地址必须同步改 .sct。
 * ------------------------------------------------------------------------- */
#define PID_FLASH_BASE           0x08040000U  /* 参数区起始地址 */
#define PID_FLASH_SECTOR_BYTES   0x20000U     /* 单扇区大小，字节 */

typedef struct { float gain[2][3]; } CarParams;

/* 读写回调：把 Flash 访问与算法解耦，便于在 PC 上跑单元测试。 */
typedef struct {
    void *ctx;
    uint32_t sector_bytes;
    int (*read)(void *, unsigned, uint32_t, uint32_t words[16]);
    int (*erase)(void *, unsigned);
    int (*program)(void *, unsigned, uint32_t, uint32_t);
} CarParamsIO;

/* 保存结果，顺序参与菜单显示。 */
typedef enum {
    PARAMS_DEFAULTS, PARAMS_LOADED, PARAMS_SAVED, PARAMS_UNCHANGED,
    PARAMS_ERROR, PARAMS_INVALID, PARAMS_BUSY, PARAMS_STOP_FIRST
} CarParamsStatus;

/* 片内 Flash 的读写实现与“保存当前参数”封装。 */
extern const CarParamsIO CarParams_FlashIO;
CarParamsStatus CarParams_FlashSave(const CarParams *params);

/*
 * 函数名称：CarParams_Capture
 * 函数功能：从控制对象抓取左右轮内环增益。
 * 参数说明：car —— 控制对象；out —— 抓取结果。
 * 返回参数：无。
 */
void CarParams_Capture(const Car_Core *car, CarParams *out);

/*
 * 函数名称：CarParams_Apply
 * 函数功能：把参数写回控制对象（内部会做数值校验）。
 * 参数说明：car —— 控制对象；params —— 待应用参数。
 * 返回参数：1 成功；0 参数不合法（保持原值）。
 */
int CarParams_Apply(Car_Core *car, const CarParams *params);

/*
 * 函数名称：CarParams_Equal
 * 函数功能：比较两组参数是否完全相同。
 * 参数说明：a/b —— 待比较参数。
 * 返回参数：1 相同；0 不同。
 */
int CarParams_Equal(const CarParams *a, const CarParams *b);

/*
 * 函数名称：CarParams_Load
 * 函数功能：从 Flash 读取最近一条有效记录。
 * 参数说明：io —— 读写接口；out —— 读取结果。
 * 返回参数：1 有效；0 无有效记录；-1 I/O 错误（失败时不改 out）。
 */
int CarParams_Load(const CarParamsIO *io, CarParams *out);

/*
 * 函数名称：CarParams_Save
 * 函数功能：擦除另一扇区并写入新记录（双扇区轮换）。
 * 参数说明：io —— 读写接口；params —— 待保存参数。
 * 返回参数：保存状态枚举。
 */
CarParamsStatus CarParams_Save(const CarParamsIO *io, const CarParams *params);

#endif
