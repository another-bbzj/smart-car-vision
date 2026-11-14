#include "Car_Race.h"
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

int Race_Solve(const uint8_t tiles[4][9], uint8_t result[36]) {
    return Race_SolveDetailed(tiles, result) == SUDOKU_SOLVED;
}
