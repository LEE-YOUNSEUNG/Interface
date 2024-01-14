#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>

void main() {
	int arr[8][8], brr[4][4], i, k,sum=0;
	for (i = 0; i < 8; i++) {
		for (k = 0; k < 8; k++) {
			arr[i][k] = sum;
			sum += 1;
			printf("%d", arr[i][k]);
		}
		printf("\n");
	}
}