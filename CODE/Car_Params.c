#include "Car_Params.h"
#include <math.h>
#include <string.h>

#define PARAM_MAGIC 0x50494431U
#define PARAM_COMMIT 0x434f4d54U
typedef struct {
    uint32_t used[2], sequence;
    unsigned bank;
    int found;
    CarParams params;
} Scan;

static int valid(const CarParams *p) {
    unsigned w,k;
    for(w=0;w<2;w++) for(k=0;k<3;k++)
        if(!isfinite(p->gain[w][k]) || p->gain[w][k]<0 || p->gain[w][k]>655.35f) return 0;
    return 1;
}
void CarParams_Capture(const Car_Core *c, CarParams *p) {
    unsigned w;
    for(w=0;w<2;w++) { p->gain[w][0]=c->pid_speed[w].kp; p->gain[w][1]=c->pid_speed[w].ki; p->gain[w][2]=c->pid_speed[w].kd; }
}
int CarParams_Apply(Car_Core *c,const CarParams *p) {
    Car_Core next;
    unsigned w;
    if(c->state!=CAR_STOP || !valid(p)) return 0;
    next=*c;
    for(w=0;w<2;w++) if(Car_SetPid(&next,w,p->gain[w][0],p->gain[w][1],p->gain[w][2])) return 0;
    *c=next; return 1;
}
int CarParams_Equal(const CarParams *a,const CarParams *b) {
    unsigned w,k;
    for(w=0;w<2;w++) for(k=0;k<3;k++) if(a->gain[w][k]!=b->gain[w][k]) return 0;
    return 1;
}
/* IEEE CRC32 over 14 little-endian words; independent of host byte order. */
static uint32_t crc(const uint32_t *words) {
    uint32_t c=0xffffffffU;
    unsigned i,b,bit;
    for(i=0;i<14;i++) for(b=0;b<4;b++) {
        c ^= (words[i]>>(b*8))&255U;
        for(bit=0;bit<8;bit++) c=(c>>1)^((c&1U)?0xedb88320U:0U);
    }
    return ~c;
}
static int decode(const uint32_t *r,CarParams *p) {
    if(r[0]!=PARAM_MAGIC || r[1]!=1 || r[3]!=6 || r[15]!=PARAM_COMMIT || r[14]!=crc(r)) return 0;
    memcpy(p->gain,r+4,sizeof p->gain);
    return valid(p);
}
static int scan(const CarParamsIO *io,Scan *s) {
    unsigned bank,i;
    uint32_t off,r[16];
    CarParams p;
    memset(s,0,sizeof *s);
    if(!io || !io->read || io->sector_bytes<64 || io->sector_bytes%64) return 0;
    for(bank=0;bank<2;bank++) for(off=0;off<io->sector_bytes;off+=64) {
        if(!io->read(io->ctx,bank,off,r)) return 0;
        for(i=0;i<16;i++) if(r[i]!=0xffffffffU) break;
        if(i!=16) s->used[bank]=off+64; /* Never overwrite a torn record. */
        if(decode(r,&p) && (!s->found || (r[2]!=s->sequence && r[2]-s->sequence<0x80000000U))) {
            s->found=1; s->sequence=r[2]; s->bank=bank; s->params=p;
        }
    }
    return 1;
}
int CarParams_Load(const CarParamsIO *io,CarParams *p) {
    Scan s;
    if(!scan(io,&s)) return -1;
    if(s.found) *p=s.params;
    return s.found;
}
CarParamsStatus CarParams_Save(const CarParamsIO *io,const CarParams *p) {
    Scan s;
    uint32_t r[16],check[16],off;
    unsigned bank,i;
    if(!valid(p)) return PARAMS_INVALID;
    if(!scan(io,&s) || !io->program || !io->erase) return PARAMS_ERROR;
    if(s.found && CarParams_Equal(&s.params,p)) return PARAMS_UNCHANGED;
    bank=s.found?s.bank:0;
    off=s.used[bank];
    if(off==io->sector_bytes) {
        bank^=1U; off=0;
        if(s.used[bank] && !io->erase(io->ctx,bank)) return PARAMS_ERROR;
    }
    memset(r,0xff,sizeof r);
    r[0]=PARAM_MAGIC; r[1]=1; r[2]=s.found?s.sequence+1U:1U; r[3]=6;
    memcpy(r+4,p->gain,sizeof p->gain); r[14]=crc(r); r[15]=PARAM_COMMIT;
    /* Commit is programmed last. The previous valid record is never erased here. */
    for(i=0;i<16;i++) if(r[i]!=0xffffffffU && !io->program(io->ctx,bank,off+i*4,r[i])) return PARAMS_ERROR;
    if(!io->read(io->ctx,bank,off,check) || memcmp(r,check,sizeof r)) return PARAMS_ERROR;
    return PARAMS_SAVED;
}
