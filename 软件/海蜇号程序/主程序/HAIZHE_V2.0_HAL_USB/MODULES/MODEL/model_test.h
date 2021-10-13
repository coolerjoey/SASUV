#ifndef _MODEL_TEST_H
#define _MODEL_TEST_H

typedef enum{
	MODEL_RISE = 0,
	MODEL_Line = 1,
	MODEL_PolyLine = 2,
	MODEL_Square = 3,
	MODEL_Circule = 4,
	MODEL_SPIN = 5,
	MODEL_ROLL = 6,
	MODEL_PITCH = 7,
	MODEL_DIV = 8,
}MODEL_TEST_TYPE;

void model_test(int sec, MODEL_TEST_TYPE type);

#endif


