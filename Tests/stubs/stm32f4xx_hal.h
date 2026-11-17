#ifndef TEST_STM32F4XX_HAL_H
#define TEST_STM32F4XX_HAL_H
#include <stdint.h>

/* Only the Cortex-M registers used by NEUQ_Delay.c. */
typedef struct { uint32_t DEMCR; } TestCoreDebug;
typedef struct { uint32_t CTRL, CYCCNT; } TestDwt;
extern TestCoreDebug test_debug;
extern uint32_t SystemCoreClock;
TestDwt *test_dwt_access(void);
#define CoreDebug (&test_debug)
#define DWT (test_dwt_access())
#define CoreDebug_DEMCR_TRCENA_Msk (1u << 24)
#define DWT_CTRL_CYCCNTENA_Msk 1u
#define __DSB() ((void)0)
#define __ISB() ((void)0)
#endif
