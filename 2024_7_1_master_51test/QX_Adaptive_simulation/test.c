#include <stdio.h>
void swap(int **a, int **b)
{
    printf("Before swap: a = %d, b = %d\n", a, b);
    printf("After swap: *a = %d, *q = %d\n", *a, *b);
    printf("After swap: **a = %d, **q = %d\n", **a, **b);
    int *tp;
    tp = *a;
    *a = *b;
    *b = tp;
}

int main() {
    int i = 3, j = 7;
    int *p = &i, *q = &j;

    printf("Before swap: *p = %d, *q = %d\n", *p, *q);
    swap(&p, &q);
   // printf("After swap: *p = %d, *q = %d\n", *p, *q);
    return 0;
}
