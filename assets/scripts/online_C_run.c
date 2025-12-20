
// https://godbolt.org/

#include <math.h>
#include <stdio.h>

int square(int num)
{
	return num * num;
}

int i = 0;
float x = 2, y = 3;

int main()
{
	printf("Hsdfgello, world!\n");
	printf("  B%d: X=%7.2f  Z=%7.2f\n", i, x, y);
	return 0;
}
