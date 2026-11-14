#ifndef CAR_PARAMS_FLASH_H
#define CAR_PARAMS_FLASH_H
#include "Car_Params.h"
/* S6/S7 are reserved by the Keil link configuration; no external SPI pins used. */
#define CAR_PARAMS_FLASH_BASE 0x08040000U
#define CAR_PARAMS_FLASH_SECTOR_BYTES 0x20000U
extern const CarParamsIO CarParams_FlashIO;
CarParamsStatus CarParams_FlashSave(const CarParams *params);
#endif
