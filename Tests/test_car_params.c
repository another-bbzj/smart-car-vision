#include "Car_Params.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

/* Two tiny NOR sectors force rollover quickly. Bits can only change 1 -> 0. */
typedef struct {
    uint32_t mem[2][32];
    int budget, writes, erases, read_fail, erase_fail;
} Flash;
static int rd(void *ctx, unsigned bank, uint32_t off, uint32_t w[16]) {
    Flash *f=ctx; assert(bank<2 && off+64<=128 && off%64==0);
    if(f->read_fail) return 0;
    memcpy(w,&f->mem[bank][off/4],64); return 1;
}
static int erase(void *ctx, unsigned bank) {
    Flash *f=ctx; assert(bank<2); ++f->erases;
    if(f->erase_fail) { memset(f->mem[bank],0xff,32); return 0; }
    memset(f->mem[bank],0xff,128); return 1;
}
static int prog(void *ctx,unsigned bank,uint32_t off,uint32_t w) {
    Flash *f=ctx; uint32_t *p; assert(bank<2 && off<128 && off%4==0);
    p=&f->mem[bank][off/4]; assert((*p&w)==w);
    if(f->budget==0) { *p &= w|0x55555555U; return 0; }
    if(f->budget>0) --f->budget;
    *p &= w; ++f->writes; return 1;
}
static void reset(Flash *f) { memset(f,0,sizeof *f); memset(f->mem,0xff,sizeof f->mem); f->budget=-1; }
int main(void) {
    Flash f, baseline;
    CarParamsIO io={&f,128,rd,erase,prog};
    CarParams a={{{1.25f,0.12f,0.03f},{2.5f,0.23f,0.04f}}}, b=a, got=a;
    Car_Core car;
    int writes, erases, cut;
    reset(&f);
    assert(CarParams_Load(&io,&got)==0 && CarParams_Equal(&got,&a));
    for(cut=0;cut<12;cut++) {
        reset(&f); f.budget=cut;
        assert(CarParams_Save(&io,&a)==PARAMS_ERROR);
        assert(CarParams_Load(&io,&got)==0); /* First save interrupted: retain defaults. */
    }
    reset(&f);
    assert(CarParams_Save(&io,&a)==PARAMS_SAVED);
    Car_Init(&car);
    assert(CarParams_Load(&io,&got)==1 && CarParams_Apply(&car,&got));
    CarParams_Capture(&car,&got);
    assert(CarParams_Equal(&a,&got));
    writes=f.writes; erases=f.erases;
    assert(CarParams_Save(&io,&a)==PARAMS_UNCHANGED && f.writes==writes && f.erases==erases);
    b.gain[1][2]=0.75f;
    baseline=f;
    for(cut=0;cut<16;cut++) {
        f=baseline; f.budget=cut;
        (void)CarParams_Save(&io,&b);
        assert(CarParams_Load(&io,&got)==1);
        assert(CarParams_Equal(&got,cut<12 ? &a : &b));
        f.budget=-1;
        assert(CarParams_Save(&io,&b)!=PARAMS_ERROR);
        assert(CarParams_Load(&io,&got)==1 && CarParams_Equal(&got,&b));
    }
    f=baseline;
    assert(CarParams_Save(&io,&b)==PARAMS_SAVED);
    f.mem[0][16+4] ^= 1U; /* Bad CRC: recover preceding complete record. */
    assert(CarParams_Load(&io,&got)==1 && CarParams_Equal(&got,&a));
    reset(&f);
    for(cut=0;cut<4;cut++) { a.gain[0][0]=(float)cut; assert(CarParams_Save(&io,&a)==PARAMS_SAVED); }
    baseline=f; b=a; b.gain[1][0]=9;
    for(cut=0;cut<16;cut++) {
        f=baseline; f.budget=cut;
        (void)CarParams_Save(&io,&b); /* Full sectors: erase only the old bank. */
        assert(CarParams_Load(&io,&got)==1);
        assert(CarParams_Equal(&got,cut<12 ? &a : &b));
    }
    f=baseline; f.erase_fail=1;
    assert(CarParams_Save(&io,&b)==PARAMS_ERROR);
    assert(CarParams_Load(&io,&got)==1 && CarParams_Equal(&got,&a));
    f=baseline; writes=f.writes;
    b.gain[0][0]=NAN; assert(CarParams_Save(&io,&b)==PARAMS_INVALID);
    b.gain[0][0]=-1; assert(CarParams_Save(&io,&b)==PARAMS_INVALID);
    b.gain[0][0]=INFINITY; assert(CarParams_Save(&io,&b)==PARAMS_INVALID);
    b.gain[0][0]=656; assert(CarParams_Save(&io,&b)==PARAMS_INVALID);
    assert(f.writes==writes);
    Car_Init(&car); CarParams_Capture(&car,&got);
    assert(!CarParams_Apply(&car,&b));
    b=a; car.state=CAR_RUN; assert(!CarParams_Apply(&car,&b));
    f.read_fail=1; assert(CarParams_Load(&io,&got)==-1);
    assert(CarParams_Save(&io,&a)==PARAMS_ERROR);
    puts("PASS PID reboot, both wheels, unchanged, corrupt records, interrupted writes/erase and rollover");
    return 0;
}
