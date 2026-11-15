/*
 * 文件名称：Car_Params.c
 * 模块功能：PID 参数的持久化流程。一条记录 64 字节：魔数、版本、序号、参数个数、6 个
 *           增益、CRC32 和提交字。上电时扫描 Flash 两个 bank，选出序号最新且校验通过
 *           的记录，没有记录则沿用默认值；保存时追加一条记录（数据字先写、提交字最后
 *           写），写满后切换 bank 并擦除，半写或掉电不会破坏上一条有效参数。
 * 运行位置：上电初始化与主循环参数保存；只在停车状态写入。
 */

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

/* 函数名称：valid
 * 函数功能：校验 6 个 PID 增益均为有限值且在 [0, 655.35] 内，阻止 NaN、负值和超量程参数被应用或写进 Flash。
 * 参数说明：p —— 待校验参数。
 * 返回参数：1 合法；0 非法。
 */
static int valid(const CarParams *p) {
    unsigned w,k;
    for(w=0;w<2;w++) for(k=0;k<3;k++)
        if(!isfinite(p->gain[w][k]) || p->gain[w][k]<0 || p->gain[w][k]>655.35f) return 0;
    return 1;
}
/* 函数名称：CarParams_Capture
 * 函数功能：把当前双轮 PID 增益抓取到 CarParams，用于与 Flash 已保存值比较、判断界面是否显示“有未保存修改”。
 * 参数说明：c —— 控制对象；p —— 输出参数。
 * 返回参数：无。
 */
void CarParams_Capture(const Car_Core *c, CarParams *p) {
    unsigned w;
    for(w=0;w<2;w++) { p->gain[w][0]=c->pid[w].kp; p->gain[w][1]=c->pid[w].ki; p->gain[w][2]=c->pid[w].kd; }
}
/* 函数名称：CarParams_Apply
 * 函数功能：把参数应用到控制对象：先校验，再在副本上逐轮 Car_SetPid，全部成功才整体替换，失败时原对象保持不变。
 * 参数说明：c —— 控制对象；p —— 参数。
 * 返回参数：1 成功；0 车辆未停车或参数非法。
 * 备注信息：只在停车状态生效，保证应用过程中控制对象不会出现半更新。
 */
int CarParams_Apply(Car_Core *c,const CarParams *p) {
    Car_Core next;
    unsigned w;
    if(c->state!=CAR_STOP || !valid(p)) return 0;
    next=*c;
    for(w=0;w<2;w++) if(Car_SetPid(&next,w,p->gain[w][0],p->gain[w][1],p->gain[w][2])) return 0;
    *c=next; return 1;
}
/* 函数名称：CarParams_Equal
 * 函数功能：逐项比较两组 PID 增益是否完全一致，用于判断是否需要写 Flash。
 * 参数说明：a, b —— 两组参数。
 * 返回参数：1 相同；0 不同。
 */
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
/* 函数名称：decode
 * 函数功能：解码一条 64 字节参数记录：校验魔数、版本、长度、提交字和 CRC32，全部通过才取出 6 个增益并做范围校验。
 * 参数说明：r —— 16 个 32 位原始字；p —— 输出参数。
 * 返回参数：1 记录有效；0 记录损坏或未提交。
 */
static int decode(const uint32_t *r,CarParams *p) {
    if(r[0]!=PARAM_MAGIC || r[1]!=1 || r[3]!=6 || r[15]!=PARAM_COMMIT || r[14]!=crc(r)) return 0;
    memcpy(p->gain,r+4,sizeof p->gain);
    return valid(p);
}
/* 函数名称：scan
 * 函数功能：扫描两个参数扇区：统计每个 bank 已写到哪个偏移（半写记录只标记不覆盖），并选出序号最新且校验通过的那条记录。
 * 参数说明：io —— Flash 读写接口；s —— 扫描结果。
 * 返回参数：1 扫描完成；0 接口非法或读取失败。
 */
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
/* 函数名称：CarParams_Load
 * 函数功能：扫描 Flash 参数区并取出最新有效参数，上电初始化时调用。
 * 参数说明：io —— Flash 读写接口；p —— 输出参数。
 * 返回参数：1 找到有效记录；0 没有记录（沿用默认值）；-1 接口或读取失败。
 */
int CarParams_Load(const CarParamsIO *io,CarParams *p) {
    Scan s;
    if(!scan(io,&s)) return -1;
    if(s.found) *p=s.params;
    return s.found;
}
/* 函数名称：CarParams_Save
 * 函数功能：把参数追加写入 Flash：与已存记录相同则直接返回；当前 bank 写满则切换并擦除另一 bank；先写数据字、最后写提交字，写完回读校验。
 * 参数说明：io —— Flash 读写接口；p —— 待保存参数。
 * 返回参数：SAVED / UNCHANGED / INVALID / ERROR。
 * 备注信息：只在停车状态由菜单触发；不擦除上一个有效记录，掉电最多丢失最新一条。
 */
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
