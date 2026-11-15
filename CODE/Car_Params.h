#ifndef CAR_PARAMS_H
#define CAR_PARAMS_H
#include "Car_Core.h"
#include <stdint.h>

#define CAR_PARAMS_RECORD_BYTES 64U /* 一条记录的字节数：16 个 32 位字，含魔数、序号、增益、CRC 和提交字 */

/* 需要持久化的 6 个 PID 增益：一维 0 左轮 / 1 右轮，二维依次 Kp / Ki / Kd。 */
typedef struct { float gain[2][3]; } CarParams;

/* Flash 读写接口：把参数算法与具体存储介质解耦，偏移相对单个可独立擦除的 bank。 */
typedef struct {
    void *ctx;                                                   /* 回调上下文，内部 Flash 实现中未使用 */
    uint32_t sector_bytes;                                       /* 单个 bank 的字节数，决定能追加多少条记录 */
    int (*read)(void *, unsigned, uint32_t, uint32_t words[16]); /* 读一条记录：bank、偏移、输出 16 字 */
    int (*erase)(void *, unsigned);                              /* 擦除整个 bank，写满切换到另一半时调用 */
    int (*program)(void *, unsigned, uint32_t, uint32_t);        /* 编程一个 32 位字：bank、偏移、数据 */
} CarParamsIO;

/* 参数加载/保存结果：菜单据此显示提示，并决定是否保持“保存中”忙状态。 */
typedef enum {
    PARAMS_DEFAULTS,  /* 上电未找到有效记录，使用默认增益 */
    PARAMS_LOADED,    /* 上电从 Flash 成功加载参数 */
    PARAMS_SAVED,     /* 本次保存成功并已回读校验 */
    PARAMS_UNCHANGED, /* 与已保存内容相同，未重复写入 */
    PARAMS_ERROR,     /* Flash 解锁、读写或校验失败 */
    PARAMS_INVALID,   /* 增益非法（NaN、负值或超量程），拒绝保存 */
    PARAMS_BUSY,      /* 保存进行中，菜单忽略按键 */
    PARAMS_STOP_FIRST /* 未停车或车轮仍在转，拒绝保存 */
} CarParamsStatus;

void CarParams_Capture(const Car_Core *car, CarParams *out);
int CarParams_Apply(Car_Core *car, const CarParams *params);
int CarParams_Equal(const CarParams *a, const CarParams *b);
/* 1=找到有效记录，0=没有记录，-1=接口或读取失败；失败时不改写 out。 */
int CarParams_Load(const CarParamsIO *io, CarParams *out);
CarParamsStatus CarParams_Save(const CarParamsIO *io, const CarParams *params);
#endif
