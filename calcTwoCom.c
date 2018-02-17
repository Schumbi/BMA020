#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>

#define MAX 10

int twoCompToDec(unsigned int in, int max)
{
	int erg = in;
	if((erg >> (max -1)) > 0)
	{
		erg |= 1 << (max - 1);
		erg -= 1 << max;
	}
	return erg;
}

int main(int argc, char** argv)
{
	int max = MAX;
	int buf[MAX];
	if(argc == 2)
	{
		char* t = argv[1];

		unsigned int erg = 0;
		
		int len = strnlen(t, max);
		max = max > len ? len : max;

		for(size_t ctr = 0; ctr < max; ctr++)
		{
			char diga[] = {t[ctr], '\0' };
			int dig = atoi(diga) > 0 ? 1 : 0;

			erg |= (dig << max - 1 - ctr);
		}
		for(size_t ctr = 0; ctr < max; ctr++)
			printf("|%d|", max - 1 - ctr);
		printf("\n");
		for(size_t ctr = 0; ctr < max; ctr++)
			printf("---");
		printf("\n");
		for(size_t ctr = 0; ctr < max; ctr++)
			printf("'%c'",  t[ctr] == '0' ? '0' : '1');

		printf(" => %d", twoCompToDec(erg, max));
		printf("\n");
	}

	return 0;
}
