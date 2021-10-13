#include "MathWrapper.h"
#include "math.hpp"

using namespace matrix;

struct vector_3f{
	Vector3f v3f;
};
Vector3f a;

#ifdef __cplusplus
extern "C" {
#endif


struct vector_3f *new_vector3f(){
	return new struct vector_3f;
}
	
#ifdef __cplusplus
};
#endif

