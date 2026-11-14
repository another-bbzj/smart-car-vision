#ifndef CAR_PARAMS_H
#define CAR_PARAMS_H
#include "Car_Core.h"
#include <stdint.h>

#define CAR_PARAMS_RECORD_BYTES 64U
typedef struct { float gain[2][3]; } CarParams;
/* Offsets are relative to one of two independently erasable sectors. */
typedef struct {
    void *ctx;
    uint32_t sector_bytes;
    int (*read)(void *, unsigned, uint32_t, uint32_t words[16]);
    int (*erase)(void *, unsigned);
    int (*program)(void *, unsigned, uint32_t, uint32_t);
} CarParamsIO;
typedef enum {
    PARAMS_DEFAULTS, PARAMS_LOADED, PARAMS_SAVED, PARAMS_UNCHANGED,
    PARAMS_ERROR, PARAMS_INVALID, PARAMS_BUSY, PARAMS_STOP_FIRST
} CarParamsStatus;

void CarParams_Capture(const Car_Core *car, CarParams *out);
int CarParams_Apply(Car_Core *car, const CarParams *params);
int CarParams_Equal(const CarParams *a, const CarParams *b);
/* Load returns 1=valid record, 0=no valid record, -1=I/O error; output unchanged on failure. */
int CarParams_Load(const CarParamsIO *io, CarParams *out);
CarParamsStatus CarParams_Save(const CarParamsIO *io, const CarParams *params);
#endif
