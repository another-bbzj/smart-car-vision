#ifndef SUDOKU_H
#define SUDOKU_H

/*
 * ============================================================================
 * 文件名称：Sudoku.h
 * ============================================================================
 * 模块功能：数独求解：把视觉收到的四个 3×3 子块拼成 6×6，求唯一解。
 *
 *   棋盘约定（与视觉协议一致，不可改）
 *     四个子块按阅读顺序：0 左上、1 右上、2 左下、3 右下；
 *     每块 9 格按行排列，0 表示空白，1~6 为数字；
 *     拼成的 6×6 棋盘按行展开为 36 格答案。
 *
 *   运行视角
 *     完赛停车后由平台层调用 Sudoku_SolveDetailed()，结果状态直接显示在
 *     菜单 “Race -> Sudoku result” 页；求解在停车状态完成，不参与控制。
 * ============================================================================
 */

#include <stdint.h>

/* ---------------------------------------------------------------------------
 * 数独规模
 * 功能：4 个子块、每块 9 格、整盘 36 格。
 * 取值：固定，与视觉帧负载（10 字节：类型 + 9 格）一一对应。
 * ------------------------------------------------------------------------- */
#define SUD_BLOCKS              4U     /* 子块个数：左上/右上/左下/右下 */
#define SUD_CELLS               9U     /* 每个子块格数：3×3 */
#define SUD_SOLUTION            36U    /* 整盘格数：6×6 */

/* 求解状态，枚举值参与菜单显示，顺序不可改。 */
typedef enum {
    SUDOKU_COLLECTING = 0,  /* 0：还在收子块 */
    SUDOKU_MISSING = 1,     /* 1：缺子块 */
    SUDOKU_SOLVED = 2,      /* 2：唯一解，答案已写入 */
    SUDOKU_INVALID = 3,     /* 3：有冲突 / 无解 */
    SUDOKU_AMBIGUOUS = 4,   /* 4：多解 */
    SUDOKU_LIMIT = 5,       /* 5：搜索超出上限 */
    SUDOKU_WAITING = 6      /* 6：完赛后等待最后一块 */
} SudokuStatus;

/*
 * 函数名称：Sudoku_SolveDetailed
 * 函数功能：拼盘并求解，返回详细状态；失败时不改动输出。
 * 参数说明：tiles  —— 四个子块的格值，tiles[块][格]；
 *           result —— 36 格答案缓冲。
 * 返回参数：SudokuStatus，只有 SUDOKU_SOLVED 时 result 才被写入。
 * 备注信息：搜索有节点上限，保证菜单不被长时间阻塞。
 */
SudokuStatus Sudoku_SolveDetailed(const uint8_t tiles[4][9], uint8_t result[36]);

/*
 * 函数名称：Sudoku_Solve
 * 函数功能：Sudoku_SolveDetailed 的简化封装。
 * 参数说明：tiles —— 四个子块；result —— 36 格答案。
 * 返回参数：1 唯一解且已写入；0 失败。
 */
int Sudoku_Solve(const uint8_t tiles[4][9], uint8_t result[36]);

#endif
