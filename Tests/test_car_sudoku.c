#include "Car_Race.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

int main(void)
{
    uint8_t tiles[4][9]={{1,0,3,0,5,0,2,0,1},{4,0,0,0,2,3,5,0,4},
                         {5,6,0,0,1,2,6,0,0},{0,3,1,6,0,5,0,1,0}};
    const uint8_t expected[36]={1,2,3,4,5,6,4,5,6,1,2,3,2,3,1,5,6,4,
        5,6,4,2,3,1,3,1,2,6,4,5,6,4,5,3,1,2};
    uint8_t answer[36], saved[36];
    unsigned r,c;
    assert(Race_SolveDetailed(tiles,answer)==SUDOKU_SOLVED);
    assert(!memcmp(answer,expected,36));
    memcpy(saved,answer,36);
    tiles[0][0]=7;
    assert(Race_SolveDetailed(tiles,answer)==SUDOKU_INVALID && !memcmp(answer,saved,36));
    tiles[0][0]=1;
    tiles[0][1]=1;
    assert(Race_SolveDetailed(tiles,answer)==SUDOKU_INVALID);
    memset(tiles,0,sizeof tiles);
    assert(Race_SolveDetailed(tiles,answer)==SUDOKU_AMBIGUOUS && !memcmp(answer,saved,36));
    for (r=0;r<6;++r)
        for (c=0;c<6;++c) tiles[(r/3)*2+c/3][(r%3)*3+c%3]=expected[r*6+c];
    assert(Race_Solve(tiles,answer) && !memcmp(answer,expected,36));
    tiles[0][0]=tiles[0][1];
    assert(!Race_Solve(tiles,answer) && !memcmp(answer,saved,36));
    assert(Race_SolveDetailed(NULL,answer)==SUDOKU_INVALID);
    puts("PASS sudoku tile mapping, unique solution, contradictions, ambiguity, atomic output");
    return 0;
}
