#ifndef CAR_PARAMS_FLASH_H
#define CAR_PARAMS_FLASH_H
#include "Car_Params.h"
#define CAR_PARAMS_FLASH_BASE 0x08040000U                     /* 参数区起始地址：扇区 6 起点，位于程序区之后；S6/S7 已由链接配置预留 */
#define CAR_PARAMS_FLASH_SECTOR_BYTES 0x20000U                /* 每个 bank 128 KB；bank0 = 扇区 6，bank1 = 扇区 7 */
extern const CarParamsIO CarParams_FlashIO;                   /* 绑定内部 Flash 的接口表，传给 CarParams_Load/Save */
CarParamsStatus CarParams_FlashSave(const CarParams *params); /* 停车保存入口：解锁、关缓存、写入回读、上锁 */
#endif
