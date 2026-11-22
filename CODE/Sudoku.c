/*
 * ============================================================================
 * 文件名称：Sudoku.c
 * ============================================================================
 * 模块功能
 * ----------------------------------------------------------------------------
 * 6×6 数独求解：把视觉收到的 4 个 3×3 子块拼成 6×6，求“唯一解”。
 *
 *   棋盘展开方式（与视觉协议严格对应）
 *     子块索引 k = (行/3)*2 + (列/3)  -> 0 左上、1 右上、2 左下、3 右下
 *     子块内索引 = (行%3)*3 + (列%3)
 *     拼好的 6×6 按行展开成 36 格，0 表示空格。
 *
 *   求解策略
 *     位掩码 + 最少候选优先（MRV）的回溯：
 *       row/col/box 各存 6 位掩码，bit1..bit6 表示该数字已用过；
 *       每次选“候选数最少”的空格展开，能显著减少搜索树；
 *       节点数上限 20000，超限返回 SUDOKU_LIMIT（菜单不能被卡住）；
 *       搜到第 2 个解就停（只需要判断“唯一解”，不需要枚举全部）。
 *
 *   运行视角
 *     完赛停车后由平台层调用，结果原子写入 result（失败不改输出），
 *     菜单“Race -> Sudoku result”页直接显示状态或答案。
 *
 * 函数列表
 *   static unsigned box_at(unsigned i)
 *   static void     toggle(Search *s, unsigned i, unsigned bit)
 *   static void     search(Search *s)
 *   SudokuStatus    Sudoku_SolveDetailed(const uint8_t tiles[4][9], uint8_t result[36])
 *   int             Sudoku_Solve(const uint8_t tiles[4][9], uint8_t result[36])
 * ============================================================================
 */
#include "Sudoku.h"
#include <string.h>

/* 搜索状态：棋盘 + 三个维度的位掩码 + 第一个解 + 解计数。 */
typedef struct {
    uint8_t board[36], row[6], col[6], box[6], answer[36], solutions;
    unsigned nodes;        /* 已展开节点数，用于限制最坏耗时 */
    uint8_t exhausted;     /* 1 表示触发节点上限，结果不可信 */
} Search;

/*
 * 函数名称：box_at
 * 函数功能：把 0..35 的一维下标映射到 2×3 的宫编号（0..5）。
 * 参数说明：i —— 棋盘下标，按行优先。
 * 返回参数：宫编号：0 左上、1 右上、2 左下、3 右下…（每 3 行换一排宫）。
 * 备注信息：i/12 每 12 个格子（两行宫）进一排，i%6/3 再区分左右宫，
 *           所以公式是 (i/12)*2 + (i%6)/3；求解和落子共用同一个映射。
 */
static unsigned box_at(unsigned i) { return (i / 12) * 2 + (i % 6) / 3; }

/*
 * 函数名称：toggle
 * 函数功能：在行/列/宫三个掩码里同时置位或清除某个数字。
 * 参数说明：s   —— 搜索状态；
 *           i   —— 棋盘下标；
 *           bit —— 数字对应的位（1U << digit）。
 * 返回参数：无。
 * 备注信息：用异或实现“落子/悔棋同一段代码”：落子时置位，回溯时再异或一次即复位。
 *           三个掩码必须同步修改，否则“填数合法但后续搜索误判”的隐蔽 bug 很难查。
 */
static void toggle(Search *s, unsigned i, unsigned bit) {
    s->row[i/6] ^= bit;
    s->col[i%6] ^= bit;
    s->box[box_at(i)] ^= bit;
}

/*
 * 函数名称：search
 * 函数功能：回溯搜索，统计解的个数并保存第一个解。
 * 参数说明：s —— 搜索状态（board/掩码/计数）。
 * 返回参数：无；结果写回 s。
 * 备注信息：三个关键判断：
 *           1) 节点上限 20000：超限立即置 exhausted 退出，保证菜单响应；
 *           2) 某个空格候选掩码为 0 -> 当前分支无解，直接 return；
 *           3) 没有空格 -> 记一个解，只保存第一个，解数到 2 即停止继续搜索。
 *           候选掩码用 0x7e（bit1..bit6）与“已占用”取反，天然排除 0 和 7。
 */
static void search(Search *s) {
    unsigned i, best = 36, choices = 0, smallest = 7, digit;
    if (++s->nodes > 20000U) { s->exhausted = 1; return; }
    for (i = 0; i < 36; ++i) {
        unsigned mask, count = 0, bits;
        if (s->board[i]) continue;
        mask = 0x7eU & ~(s->row[i/6] | s->col[i%6] | s->box[box_at(i)]);
        if (!mask) return;                              /* 该空格无论填什么都冲突：本分支无解 */
        for (bits = mask; bits; bits &= bits-1) ++count; /* 统计候选个数（Brian Kernighan 法） */
        if (count < smallest) { best = i; choices = mask; smallest = count; } /* MRV：选候选最少的格子 */
    }
    if (best == 36) {                                    /* 没有空格：找到一个完整解 */
        if (!s->solutions) memcpy(s->answer, s->board, 36);
        ++s->solutions;
        return;
    }
    for (digit = 1; digit <= 6 && s->solutions < 2 && !s->exhausted; ++digit) {
        unsigned bit = 1U << digit;
        if (!(choices & bit)) continue;
        s->board[best] = (uint8_t)digit;                 /* 落子 */
        toggle(s, best, bit);
        search(s);
        toggle(s, best, bit);                            /* 悔棋：异或复位 */
        s->board[best] = 0;
    }
}

/*
 * 函数名称：Sudoku_SolveDetailed
 * 函数功能：拼盘、校验初始冲突、求解，返回详细状态。
 * 参数说明：tiles  —— 四个 3×3 子块，tiles[块][格]，0 表示空白，1..6 为数字；
 *           result —— 36 格答案缓冲，只在 SUDOKU_SOLVED 时写入。
 * 返回参数：SUDOKU_INVALID / SUDOKU_LIMIT / SUDOKU_AMBIGUOUS / SUDOKU_SOLVED。
 * 备注信息：参数传递上做“原子输出”：先用局部 Search 求解，成功后才 memcpy 到
 *           result，任何失败都不会污染调用方的显示缓冲。
 *           拼盘时逐个校验 0..6、并检查行/列/宫冲突，把“视觉给错数据”挡在搜索之前，
 *           否则搜索会在大分支里白白跑满节点上限。
 */
SudokuStatus Sudoku_SolveDetailed(const uint8_t tiles[4][9], uint8_t result[36]) {
    Search s;
    unsigned i;
    if (!tiles || !result) return SUDOKU_INVALID;
    memset(&s, 0, sizeof(s));
    for (i = 0; i < 36; ++i) {
        unsigned r = i/6, c = i%6;
        unsigned digit = tiles[(r/3)*2 + c/3][(r%3)*3 + c%3];  /* 6×6 坐标 -> 子块/格内坐标 */
        unsigned bit;
        if (digit > 6) return SUDOKU_INVALID;                   /* 只允许 0..6，7 及以上视为脏数据 */
        if (!digit) continue;                                   /* 空格 */
        bit = 1U << digit;
        if ((s.row[r] | s.col[c] | s.box[box_at(i)]) & bit) return SUDOKU_INVALID; /* 初始冲突 */
        s.board[i] = (uint8_t)digit;
        toggle(&s, i, bit);
    }
    search(&s);
    if (s.exhausted) return SUDOKU_LIMIT;
    if (!s.solutions) return SUDOKU_INVALID;
    if (s.solutions > 1) return SUDOKU_AMBIGUOUS;
    memcpy(result, s.answer, 36);
    return SUDOKU_SOLVED;
}

/*
 * 函数名称：Sudoku_Solve
 * 函数功能：简化封装：只告诉调用方“是否解出唯一解”。
 * 参数说明：tiles —— 四个子块；result —— 36 格答案。
 * 返回参数：1 唯一解且已写入；0 其他所有情况。
 * 备注信息：需要区分“多解/无解/超限”时用 Sudoku_SolveDetailed。
 */
int Sudoku_Solve(const uint8_t tiles[4][9], uint8_t result[36]) {
    return Sudoku_SolveDetailed(tiles, result) == SUDOKU_SOLVED;
}
