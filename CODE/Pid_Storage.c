/*
 * ============================================================================
 * 文件名称：Pid_Storage.c
 * ============================================================================
 * 模块功能
 * ----------------------------------------------------------------------------
 * 内环 PID 增益的掉电保存。分三层：
 *
 *   第一层  参数搬运    CarParams_Capture / Apply / Equal
 *           在“控制对象里的 pid_speed[]”和“6 个 float 的参数结构”之间转换。
 *
 *   第二层  记录编解码   valid / crc / decode / scan / Load / Save
 *           一条记录 64 字节 = 16 个 32 位字：
 *             [0] magic 0x50494431        [1] 版本 = 1
 *             [2] 序号（越大越新）         [3] 增益个数 = 6
 *             [4..9] 左右轮 Kp/Ki/Kd      [10..13] 保留（0xFFFFFFFF）
 *             [14] CRC32（覆盖前 14 字）  [15] commit 0x434F4D54
 *           写入顺序固定：先写数据，最后写 commit。掉电时只要 commit 没写进去，
 *           这条记录就作废，上一份有效记录仍在另一个扇区。
 *
 *   第三层  片内 Flash  S6/S7 两个扇区轮换读写（可编译开关 PID_STORAGE_WITH_FLASH）
 *           写之前先擦“另一个”扇区，保证任何时刻至少有一份完整旧参数。
 *
 * 参数传递约定（重要）
 *   CarParamsIO 把 Flash 访问抽象成三个回调（read/erase/program），
 *   参数是 (ctx, bank, 扇区内偏移, 数据)。bank=0 对应 S6、bank=1 对应 S7，
 *   这样上层逻辑完全不关心具体地址，PC 单测可以用内存数组实现同样的接口。
 *
 * 函数列表
 *   static int      valid(const CarParams *p)
 *   void            CarParams_Capture / CarParams_Apply / CarParams_Equal
 *   static uint32_t crc(const uint32_t *words)
 *   static int      decode(const uint32_t *r, CarParams *p)
 *   static int      scan(const CarParamsIO *io, Scan *s)
 *   int             CarParams_Load(const CarParamsIO *io, CarParams *p)
 *   CarParamsStatus CarParams_Save(const CarParamsIO *io, const CarParams *p)
 *   static int      flash_read / flash_erase / flash_program
 *   CarParamsStatus CarParams_FlashSave(const CarParams *params)
 * ============================================================================
 */
#include "Pid_Storage.h"
#include "stm32f4xx_hal.h"
#include <math.h>
#include <string.h>

/* 1=编译片内 Flash 读写实现（固件）；0=只在 PC 单测里编译记录编解码。
 * 片内 Flash 部分依赖 HAL_FLASH，单测环境没有 HAL，用 0 关掉即可。 */
#ifndef PID_STORAGE_WITH_FLASH
#define PID_STORAGE_WITH_FLASH 1
#endif

#define PID_PARAM_MAGIC  0x50494431U   /* 记录头 "PID1"，识别这是参数区而不是随机数据 */
#define PID_PARAM_COMMIT 0x434F4D54U   /* 提交标记 "COMT"，最后一个写入，写不进去则记录作废 */

/* 扫描结果：每个扇区用到哪里、最新一条有效记录的序号/所在扇区/内容。 */
typedef struct {
    uint32_t used[2];    /* 每个扇区已用字节数（至少写到这里），用于追加写 */
    uint32_t sequence;   /* 当前最新记录的序号，0 表示还没有 */
    unsigned bank;       /* 最新记录所在扇区：0=S6、1=S7 */
    int found;           /* 是否找到有效记录 */
    CarParams params;    /* 有效记录的内容 */
} Scan;

/*
 * 函数名称：valid
 * 函数功能：检查 6 个增益是否都是有限、非负且不超过 655.35 的合法值。
 * 参数说明：p —— 待检查参数。
 * 返回参数：1 合法；0 非法。
 * 备注信息：上限 655.35 不是随手取的：蓝牙命令 4 与菜单都用“uint16 放大 100 倍”
 *           表示增益，655.35 正好是 uint16 能表示的最大增益值，
 *           这样从任何入口写进来的参数都能被无损保存和回读。
 */
static int valid(const CarParams *p) {
    unsigned w,k;
    for(w=0;w<2;w++) for(k=0;k<3;k++)
        if(!isfinite(p->gain[w][k]) || p->gain[w][k]<0 || p->gain[w][k]>655.35f) return 0;
    return 1;
}

/*
 * 函数名称：CarParams_Capture
 * 函数功能：从控制对象抓取左右轮内环增益。
 * 参数说明：c —— 控制对象；p —— 抓取结果（6 个 float）。
 * 返回参数：无。
 * 备注信息：只读 pid_speed[]（内环增量式），外环位置环参数不参与掉电保存。
 */
void CarParams_Capture(const Car_Core *c, CarParams *p) {
    unsigned w;
    for(w=0;w<2;w++) { p->gain[w][0]=c->pid_speed[w].kp; p->gain[w][1]=c->pid_speed[w].ki; p->gain[w][2]=c->pid_speed[w].kd; }
}

/*
 * 函数名称：CarParams_Apply
 * 函数功能：把参数写回控制对象（上电恢复时调用）。
 * 参数说明：c —— 控制对象；p —— 待应用参数。
 * 返回参数：1 全部应用成功；0 状态不对或参数非法。
 * 备注信息：参数传递上做了“整体成功或整体不变”：
 *           1) 先要求 STOP：运行中改增益会让电流突变；
 *           2) 先把 c 复制到临时对象 next，6 个增益全部经 Car_SetPid 校验并写入 next；
 *           3) 只有 6 个都成功才一次性赋回 *c。
 *           这样即使某个增益非法，控制对象也不会出现“两个轮参数不一致”的中间态。
 */
int CarParams_Apply(Car_Core *c,const CarParams *p) {
    Car_Core next;
    unsigned w;
    if(c->state!=CAR_STOP || !valid(p)) return 0;
    next=*c;
    for(w=0;w<2;w++) if(Car_SetPid(&next,w,p->gain[w][0],p->gain[w][1],p->gain[w][2])) return 0;
    *c=next; return 1;
}

/*
 * 函数名称：CarParams_Equal
 * 函数功能：逐项比较两组参数是否完全相同。
 * 参数说明：a/b —— 待比较参数。
 * 返回参数：1 完全相同；0 有差异。
 * 备注信息：用精确相等而不是容差比较：这里比较的是“要不要写 Flash”，
 *           只要有一个 bit 不同就值得重写；用容差反而会漏掉微小改动。
 */
int CarParams_Equal(const CarParams *a,const CarParams *b) {
    unsigned w,k;
    for(w=0;w<2;w++) for(k=0;k<3;k++) if(a->gain[w][k]!=b->gain[w][k]) return 0;
    return 1;
}

/*
 * 函数名称：crc
 * 函数功能：对前 14 个 32 位字计算 IEEE CRC32。
 * 参数说明：words —— 记录首地址（16 字，只用前 14 字）。
 * 返回参数：CRC32 值。
 * 备注信息：按“小端字节序”逐字节喂入，与主机字节序无关：
 *           Flash 里存的是 STM32 的小端字，PC 单测在 x86 上算出的结果一致。
 *           最后取反是 IEEE CRC32 的标准做法。
 */
static uint32_t crc(const uint32_t *words) {
    uint32_t c=0xffffffffU;
    unsigned i,b,bit;
    for(i=0;i<14;i++) for(b=0;b<4;b++) {
        c ^= (words[i]>>(b*8))&255U;
        for(bit=0;bit<8;bit++) c=(c>>1)^((c&1U)?0xedb88320U:0U);
    }
    return ~c;
}

/*
 * 函数名称：decode
 * 函数功能：校验一条 16 字记录并解出 6 个增益。
 * 参数说明：r —— 16 字记录；p —— 解出的参数。
 * 返回参数：1 有效；0 无效（magic/版本/个数/commit/CRC 任一不符，或增益非法）。
 * 备注信息：检查顺序是“便宜的在前、贵的在后”：magic -> 版本 -> 个数 ->
 *           commit -> CRC -> 数值范围。commit 在 CRC 之前查，
 *           这样“写了一半”的记录不必算 CRC 就能快速丢弃。
 */
static int decode(const uint32_t *r,CarParams *p) {
    if(r[0]!=PID_PARAM_MAGIC || r[1]!=1 || r[3]!=6 || r[15]!=PID_PARAM_COMMIT || r[14]!=crc(r)) return 0;
    memcpy(p->gain,r+4,sizeof p->gain);
    return valid(p);
}

/*
 * 函数名称：scan
 * 函数功能：扫描两个扇区，统计每区已用长度，并挑出序号最新的一条有效记录。
 * 参数说明：io —— Flash 访问接口；s —— 扫描结果。
 * 返回参数：1 扫描完成；0 接口非法或读失败。
 * 备注信息：序号比较用“无符号回绕差”：
 *             r[2]-s->sequence < 0x80000000U
 *           而不是简单的 >，这样 32 位序号从 0xFFFFFFFF 回到 0 时仍然正确。
 *           used[bank] 记录“最后一个非 0xFFFFFFFF 字的末尾”，
 *           用于追加写（不覆盖撕裂缝）；全 0xFF 表示该扇区还没写过。
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
        if(i!=16) s->used[bank]=off+64;   /* 只记录“确实写过”的槽位，避免覆盖撕裂缝 */
        if(decode(r,&p) && (!s->found || (r[2]!=s->sequence && r[2]-s->sequence<0x80000000U))) {
            s->found=1; s->sequence=r[2]; s->bank=bank; s->params=p;
        }
    }
    return 1;
}

/*
 * 函数名称：CarParams_Load
 * 函数功能：读取最新一条有效参数（上电调用）。
 * 参数说明：io —— Flash 访问接口；p —— 输出参数。
 * 返回参数：1 读到有效记录；0 没有有效记录；-1 I/O 错误。
 * 备注信息：三种返回值的含义不同，调用方据此决定菜单提示：
 *           1 -> “Loaded from Flash”，0 -> “Using defaults”，-1 -> “Save/read failed”。
 *           失败时不修改 p，避免把随机值当参数用。
 */
int CarParams_Load(const CarParamsIO *io,CarParams *p) {
    Scan s;
    if(!scan(io,&s)) return -1;
    if(s.found) *p=s.params;
    return s.found;
}

/*
 * 函数名称：CarParams_Save
 * 函数功能：把参数写成一条新记录（双扇区轮换 + 最后写 commit）。
 * 参数说明：io —— Flash 访问接口；p —— 待保存参数。
 * 返回参数：PARAMS_SAVED / PARAMS_UNCHANGED / PARAMS_INVALID / PARAMS_ERROR。
 * 备注信息：关键的几个判断：
 *           1) 与 Flash 里最新记录完全相同 -> PARAMS_UNCHANGED，不浪费擦写寿命；
 *           2) 当前扇区写满 -> 换另一个扇区并先擦除它（旧的完整记录在刚写满的
 *              那个扇区里，所以擦除不会丢参数）；
 *           3) 只写非 0xFF 的字，最后写 commit，写完回读比对；
 *           4) 整个过程不擦当前有效记录所在扇区，掉电最坏只损失“这次的新参数”。
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
    r[0]=PID_PARAM_MAGIC; r[1]=1; r[2]=s.found?s.sequence+1U:1U; r[3]=6;
    memcpy(r+4,p->gain,sizeof p->gain); r[14]=crc(r); r[15]=PID_PARAM_COMMIT;
    /* 提交字最后写：先写数据再写 commit，半途掉电时这条记录不算数。 */
    for(i=0;i<16;i++) if(r[i]!=0xffffffffU && !io->program(io->ctx,bank,off+i*4,r[i])) return PARAMS_ERROR;
    if(!io->read(io->ctx,bank,off,check) || memcmp(r,check,sizeof r)) return PARAMS_ERROR;
    return PARAMS_SAVED;
}

#if PID_STORAGE_WITH_FLASH
/* ========================================================================== */
/*                        片内 Flash 读写实现（S6/S7）                         */
/* ========================================================================== */

/*
 * 函数名称：flash_read
 * 函数功能：从参数区读一个 16 字记录到内存。
 * 参数说明：ctx  —— 未使用；bank —— 0=S6、1=S7；off —— 扇区内字节偏移；
 *           words—— 输出缓冲。
 * 返回参数：1 成功；0 参数越界。
 * 备注信息：直接按 volatile 指针读，避免编译器把重复读优化掉；
 *           越界检查保证不会读到别的扇区。
 */
static int flash_read(void *ctx,unsigned bank,uint32_t off,uint32_t words[16]) {
    const volatile uint32_t *src;
    unsigned i;
    (void)ctx;
    if(bank>1 || off%64 || off>PID_FLASH_SECTOR_BYTES-64) return 0;
    src=(const volatile uint32_t *)(PID_FLASH_BASE+bank*PID_FLASH_SECTOR_BYTES+off);
    for(i=0;i<16;i++) words[i]=src[i];
    return 1;
}

/*
 * 函数名称：flash_erase
 * 函数功能：擦除一个参数扇区（bank 0 -> S6，bank 1 -> S7）。
 * 参数说明：ctx —— 未使用；bank —— 扇区号。
 * 返回参数：1 成功；0 参数非法或 HAL 报错。
 * 备注信息：VoltageRange 固定 3.3 V（板级供电），改硬件供电必须同步改这一项。
 */
static int flash_erase(void *ctx,unsigned bank) {
    FLASH_EraseInitTypeDef request={0};
    uint32_t failed;
    (void)ctx;
    if(bank>1) return 0;
    request.TypeErase=FLASH_TYPEERASE_SECTORS;
    request.Sector=bank?FLASH_SECTOR_7:FLASH_SECTOR_6;
    request.NbSectors=1;
    request.VoltageRange=FLASH_VOLTAGE_RANGE_3;
    return HAL_FLASHEx_Erase(&request,&failed)==HAL_OK;
}

/*
 * 函数名称：flash_program
 * 函数功能：向参数区写一个 32 位字。
 * 参数说明：ctx —— 未使用；bank —— 扇区号；off —— 扇区内字节偏移；word —— 待写数据。
 * 返回参数：1 成功；0 参数非法或 HAL 报错。
 * 备注信息：要求 4 字节对齐；地址计算与 flash_read 完全一致，
 *           避免“读写的不是同一个位置”这种低级错误。
 */
static int flash_program(void *ctx,unsigned bank,uint32_t off,uint32_t word) {
    (void)ctx;
    if(bank>1 || off%4 || off>=PID_FLASH_SECTOR_BYTES) return 0;
    return HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
        PID_FLASH_BASE+bank*PID_FLASH_SECTOR_BYTES+off,word)==HAL_OK;
}

/* 片内 Flash 版读写接口：交给上层逻辑使用，接口与 PC 单测的内存实现一致。 */
const CarParamsIO CarParams_FlashIO={0,PID_FLASH_SECTOR_BYTES,flash_read,flash_erase,flash_program};

/*
 * 函数名称：CarParams_FlashSave
 * 函数功能：解锁 Flash、关数据 Cache、保存参数、恢复现场并加锁。
 * 参数说明：params —— 待保存参数。
 * 返回参数：保存状态；解锁/加锁失败返回 PARAMS_ERROR。
 * 备注信息：必须关数据 Cache 再读参数区，否则可能读到 Cache 里的旧内容；
 *           保存完重置 Cache 并按原状态决定是否重新使能，避免影响其他外设性能。
 *           调用方必须保证此时已停车、且 PIT 不会重入（平台层已做）。
 */
CarParamsStatus CarParams_FlashSave(const CarParams *params) {
    CarParamsStatus result;
    uint32_t cache=FLASH->ACR & FLASH_ACR_DCEN;
    if(HAL_FLASH_Unlock()!=HAL_OK) return PARAMS_ERROR;
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP|FLASH_FLAG_OPERR|FLASH_FLAG_WRPERR|
                          FLASH_FLAG_PGAERR|FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);
    /* 上电扫描时参数区可能已经进了数据 Cache，写前必须关掉并复位。 */
    __HAL_FLASH_DATA_CACHE_DISABLE();
    __HAL_FLASH_DATA_CACHE_RESET();
    result=CarParams_Save(&CarParams_FlashIO,params);
    __HAL_FLASH_DATA_CACHE_RESET();
    if(cache) __HAL_FLASH_DATA_CACHE_ENABLE();
    if(HAL_FLASH_Lock()!=HAL_OK) return PARAMS_ERROR;
    return result;
}

#endif /* PID_STORAGE_WITH_FLASH */
