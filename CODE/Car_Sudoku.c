/*
 * 文件名称：Car_Sudoku.c
 * 模块功能：6x6 数独求解。把视觉收到的四块 3x3 瓦片拼成棋盘，先检查给定数字的行/列/
 *           宫冲突，再用“候选最少优先”的回溯搜索求解；搜索节点上限 20000，解数达到 2
 *           即判定多解，最终区分为唯一解、多解、无解和搜索超限。
 * 运行位置：完赛停车后在前台 poll_menu() 中一次性求解，不占控制中断。
 */

#include "Car_Race.h"
#include <string.h>

typedef struct {
    uint8_t board[36], row[6], col[6], box[6], answer[36], solutions;
    unsigned nodes;
    uint8_t exhausted;
} Search;

/* 函数名称：box_at
 * 函数功能：把 6x6 棋盘的线性下标映射到 2 行 x 3 列的小宫格编号，供行列宫冲突位图使用。
 * 参数说明：i —— 0..35 的格子下标。
 * 返回参数：小宫格编号 0..5。
 */
static unsigned box_at(unsigned i) { return (i / 12) * 2 + (i % 6) / 3; }
/* 函数名称：toggle
 * 函数功能：在行、列、宫三个位图里同时置位或清除某个数字，用于回溯搜索时试填与撤销。
 * 参数说明：s —— 搜索状态；i —— 格子下标；bit —— 数字位（1<<digit）。
 * 返回参数：无。
 */
static void toggle(Search *s, unsigned i, unsigned bit) {
    s->row[i/6] ^= bit;
    s->col[i%6] ^= bit;
    s->box[box_at(i)] ^= bit;
}
/* 函数名称：search
 * 函数功能：数独回溯求解主体：每步选候选数最少的空格试填，记录第一个解并统计解数；节点数超过 20000 置 exhausted 后退出。
 * 参数说明：s —— 搜索状态。
 * 返回参数：无。
 * 备注信息：解数达到 2 即判定多解并停止；只在前台完赛结算时调用，不占控制中断。
 */
static void search(Search *s) {
    unsigned i, best = 36, choices = 0, smallest = 7, digit;
    if (++s->nodes > 20000U) { s->exhausted = 1; return; }
    for (i = 0; i < 36; ++i) {
        unsigned mask, count = 0, bits;
        if (s->board[i]) continue;
        mask = 0x7eU & ~(s->row[i/6] | s->col[i%6] | s->box[box_at(i)]);
        if (!mask) return;
        for (bits = mask; bits; bits &= bits-1) ++count;
        if (count < smallest) { best = i; choices = mask; smallest = count; }
    }
    if (best == 36) {
        if (!s->solutions) memcpy(s->answer, s->board, 36);
        ++s->solutions;
        return;
    }
    for (digit = 1; digit <= 6 && s->solutions < 2 && !s->exhausted; ++digit) {
        unsigned bit = 1U << digit;
        if (!(choices & bit)) continue;
        s->board[best] = (uint8_t)digit;
        toggle(s, best, bit);
        search(s);
        toggle(s, best, bit);
        s->board[best] = 0;
    }
}
/* 函数名称：Race_SolveDetailed
 * 函数功能：把视觉收到的四块 3x3 瓦片拼成 6x6 数独，先检查给定数字冲突，再回溯求解，返回唯一解或具体失败原因。
 * 参数说明：tiles —— 四块瓦片（每块 9 格）；result —— 输出 36 格答案。
 * 返回参数：SOLVED / INVALID / AMBIGUOUS / LIMIT。
 */
RaceSudokuStatus Race_SolveDetailed(const uint8_t tiles[4][9], uint8_t result[36]) {
    Search s;
    unsigned i;
    if (!tiles || !result) return SUDOKU_INVALID;
    memset(&s, 0, sizeof(s));
    for (i = 0; i < 36; ++i) {
        unsigned r = i/6, c = i%6;
        unsigned digit = tiles[(r/3)*2 + c/3][(r%3)*3 + c%3];
        unsigned bit;
        if (digit > 6) return SUDOKU_INVALID;
        if (!digit) continue;
        bit = 1U << digit;
        if ((s.row[r] | s.col[c] | s.box[box_at(i)]) & bit) return SUDOKU_INVALID;
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

/* 函数名称：Race_Solve
 * 函数功能：Race_SolveDetailed 的布尔封装，只有唯一解时返回 1。
 * 参数说明：tiles —— 四块瓦片；result —— 输出 36 格答案。
 * 返回参数：1 唯一解；0 无解/多解/超限。
 */
int Race_Solve(const uint8_t tiles[4][9], uint8_t result[36]) {
    return Race_SolveDetailed(tiles, result) == SUDOKU_SOLVED;
}
