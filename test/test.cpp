#include "stdio.h"
#include "stdlib.h"

#include "8puzzle.h"
#include "findpath.h"
#include "Romania.h"

void doMemPoolTest()
{
    printf(" Test MemPool class \n");
    printf("==============================\n");
    MemPool<double> mp(20);
    mp.Debug();
    printf("---\n");
    double *a1 = mp.Alloc();
    double *a2 = mp.Alloc();
    mp.Debug();
    printf("---\n");
    mp.Free(a1);
    mp.Debug();
    printf("---\n");
    double *a3 = mp.Alloc();
    double *a4 = mp.Alloc();
    mp.Debug();
    printf("---\n");
    mp.FreeMem();
    mp.Debug();
}

int main()
{
    doMemPoolTest();
    doPuzzleTest();
    system("pause");
    return 0;
}