#include "Car_ParamsFlash.h"
#include "stm32f4xx_hal.h"

static int flash_read(void *ctx,unsigned bank,uint32_t off,uint32_t words[16]) {
    const volatile uint32_t *src;
    unsigned i;
    (void)ctx;
    if(bank>1 || off%64 || off>CAR_PARAMS_FLASH_SECTOR_BYTES-64) return 0;
    src=(const volatile uint32_t *)(CAR_PARAMS_FLASH_BASE+bank*CAR_PARAMS_FLASH_SECTOR_BYTES+off);
    for(i=0;i<16;i++) words[i]=src[i];
    return 1;
}
static int flash_erase(void *ctx,unsigned bank) {
    FLASH_EraseInitTypeDef request={0};
    uint32_t failed;
    (void)ctx;
    if(bank>1) return 0;
    request.TypeErase=FLASH_TYPEERASE_SECTORS;
    request.Sector=bank?FLASH_SECTOR_7:FLASH_SECTOR_6;
    request.NbSectors=1;
    request.VoltageRange=FLASH_VOLTAGE_RANGE_3; /* Board supply 3.3 V. */
    return HAL_FLASHEx_Erase(&request,&failed)==HAL_OK;
}
static int flash_program(void *ctx,unsigned bank,uint32_t off,uint32_t word) {
    (void)ctx;
    if(bank>1 || off%4 || off>=CAR_PARAMS_FLASH_SECTOR_BYTES) return 0;
    return HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
        CAR_PARAMS_FLASH_BASE+bank*CAR_PARAMS_FLASH_SECTOR_BYTES+off,word)==HAL_OK;
}
const CarParamsIO CarParams_FlashIO={0,CAR_PARAMS_FLASH_SECTOR_BYTES,flash_read,flash_erase,flash_program};

CarParamsStatus CarParams_FlashSave(const CarParams *params) {
    CarParamsStatus result;
    uint32_t cache=FLASH->ACR & FLASH_ACR_DCEN;
    if(HAL_FLASH_Unlock()!=HAL_OK) return PARAMS_ERROR;
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP|FLASH_FLAG_OPERR|FLASH_FLAG_WRPERR|
                          FLASH_FLAG_PGAERR|FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);
    /* Parameter addresses may already be in the data cache after boot scanning. */
    __HAL_FLASH_DATA_CACHE_DISABLE();
    __HAL_FLASH_DATA_CACHE_RESET();
    result=CarParams_Save(&CarParams_FlashIO,params);
    __HAL_FLASH_DATA_CACHE_RESET();
    if(cache) __HAL_FLASH_DATA_CACHE_ENABLE();
    if(HAL_FLASH_Lock()!=HAL_OK) return PARAMS_ERROR;
    return result;
}
