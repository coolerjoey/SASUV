#ifndef __KF_TEST_H
#define __KF_TEST_H

typedef enum{
	KF_Line = 0,
	KF_PolyLine = 1,
	KF_Square = 2,
	KF_Circule = 3,
}KF_TEST_TYPE;

void kF_test(int sec, int type);
void Test_print_info(int num);


#endif

