#include "stdio.h"
#include "stdlib.h"

#include "8puzzle.h"
#include "findpath.h"
#include "Romania.h"

void doMemPoolTest()
{
    printf(" Test MemPool class \n");
    printf("==============================\n");

    const int n = 20;
    printf("Create MemPool with %d elements: \n\n", n);
    MemPool<double> mp(n);
    mp.Debug();

    printf("---\n\n");

    printf("Alloc 2 elements: \n\n");
    double *a1 = mp.Alloc();
    double *a2 = mp.Alloc();
    mp.Debug();

    printf("---\n\n");

    printf("Free the 1st element: \n\n");
    mp.Free(a1);
    mp.Debug();

    printf("---\n\n");

    printf("Alloc another 2 elements: \n\n");
    double *a3 = mp.Alloc();
    double *a4 = mp.Alloc();
    mp.Debug();

    printf("---\n\n");

    printf("Free MemPool: \n\n");
    mp.FreeMem();
    mp.Debug();
}

int main()
{
    doMemPoolTest();
    printf("\n^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n");

    doPuzzleTest();
    printf("\n^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n");

    doFindPathTest();
    printf("\n^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n");

    doRomaniaTest();
    printf("\n^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n");

    system("pause");
    return 0;
}