/*
 * 文件名称：Car_ParamsFlash.c
 * 模块功能：把参数持久化接到 STM32F407 内部 Flash。bank0 使用扇区 6、bank1 使用扇区 7，
 *           提供按 bank/偏移读字、擦除扇区和按字编程，均带 bank、对齐和越界检查；
 *           保存前关闭并复位数据缓存，避免读到启动时缓存下来的旧参数，保存后恢复缓存。
 * 运行位置：仅在上电和停车保存时由主循环调用；擦写期间 CPU 停等，禁止在控制中断里执行。
 */

#include "Car_ParamsFlash.h"
#include "stm32f4xx_hal.h"

/* 函数名称：flash_read
 * 函数功能：从参数 bank（扇区 6/7）的指定偏移读出 16 个 32 位字，带 bank、对齐和越界检查。
 * 参数说明：ctx —— 未使用；bank —— 0/1；off —— 字节偏移；words —— 输出 16 字。
 * 返回参数：1 成功；0 参数越界。
 */
static int flash_read(void *ctx,unsigned bank,uint32_t off,uint32_t words[16]) {
    const volatile uint32_t *src;
    unsigned i;
    (void)ctx;
    if(bank>1 || off%64 || off>CAR_PARAMS_FLASH_SECTOR_BYTES-64) return 0;
    src=(const volatile uint32_t *)(CAR_PARAMS_FLASH_BASE+bank*CAR_PARAMS_FLASH_SECTOR_BYTES+off);
    for(i=0;i<16;i++) words[i]=src[i];
    return 1;
}
/* 函数名称：flash_erase
 * 函数功能：擦除参数 bank 所在的 Flash 扇区（bank0 扇区 6、bank1 扇区 7），电压范围按 3.3V 板供电设置。
 * 参数说明：ctx —— 未使用；bank —— 0/1。
 * 返回参数：1 擦除成功；0 bank 非法或 HAL 失败。
 */
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
/* 函数名称：flash_program
 * 函数功能：向参数 bank 的 4 字节对齐偏移编程一个 32 位字，带越界检查。
 * 参数说明：ctx —— 未使用；bank —— 0/1；off —— 字节偏移；word —— 待写入字。
 * 返回参数：1 成功；0 越界或 HAL 失败。
 */
static int flash_program(void *ctx,unsigned bank,uint32_t off,uint32_t word) {
    (void)ctx;
    if(bank>1 || off%4 || off>=CAR_PARAMS_FLASH_SECTOR_BYTES) return 0;
    return HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
        CAR_PARAMS_FLASH_BASE+bank*CAR_PARAMS_FLASH_SECTOR_BYTES+off,word)==HAL_OK;
}
const CarParamsIO CarParams_FlashIO={0,CAR_PARAMS_FLASH_SECTOR_BYTES,flash_read,flash_erase,flash_program};

/* 函数名称：CarParams_FlashSave
 * 函数功能：停车保存参数的硬件入口：解锁 Flash、清错误标志、关闭并复位数据缓存（防止读到缓存的旧参数）、执行追加写入、回读校验，最后恢复缓存并上锁。
 * 参数说明：params —— 待保存参数。
 * 返回参数：CarParams_Save 的结果；解锁或上锁失败返回 PARAMS_ERROR。
 * 备注信息：必须在停车状态调用；擦写期间 CPU 停等，不能放在控制中断里执行。
 */
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
