/*
 * ============================================================================
 * 文件名称：Sudoku.c
 * ============================================================================
 * 模块功能
 * ----------------------------------------------------------------------------
 * 6×6 数独的唯一解搜索：
 *   1. box_at()        ：把一维下标映射到 2×3 的宫编号；
 *   2. toggle()        ：行/列/宫的位掩码置位或清除；
 *   3. search()        ：回溯搜索，记录解的个数，超过 1 个即判多解；
 *   4. Sudoku_SolveDetailed()：拼盘、校验、搜索，原子写输出；
 *   5. Sudoku_Solve()  ：只返回“是否唯一解”的简化封装。
 *
 * 为什么在停车时求解：
 *   搜索最坏要遍历几百个节点，放在控制中断里会拖长 5 ms 节拍；
 *   完赛停车后由主循环求解，显示到菜单即可。
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

typedef struct {
    uint8_t board[36], row[6], col[6], box[6], answer[36], solutions;
    unsigned nodes;
    uint8_t exhausted;
} Search;

static unsigned box_at(unsigned i) { return (i / 12) * 2 + (i % 6) / 3; }
static void toggle(Search *s, unsigned i, unsigned bit) {
    s->row[i/6] ^= bit;
    s->col[i%6] ^= bit;
    s->box[box_at(i)] ^= bit;
}
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
SudokuStatus Sudoku_SolveDetailed(const uint8_t tiles[4][9], uint8_t result[36]) {
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

int Sudoku_Solve(const uint8_t tiles[4][9], uint8_t result[36]) {
    return Sudoku_SolveDetailed(tiles, result) == SUDOKU_SOLVED;
}
