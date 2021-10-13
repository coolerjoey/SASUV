#ifndef __MYMATH_H
#define __MYMATH_H	
#include <math.h>
#include "sys.h"

typedef struct{ 
	float w;
	float x;
	float y;  
	float z;
}Vector4f;

typedef struct{ 
	float x;
	float y;  
	float z;
}Vector3f;

#define GRAVITY_MSS 9.80665f

#ifndef true
#define true 1
#endif
#ifndef false
#define false ~true
#endif

#ifdef M_PI
#undef M_PI
#endif
#define M_PI      (3.141592653589793f)

#ifdef M_PI_2
#undef M_PI_2
#endif
#define M_PI_2    (M_PI / 2)

#define M_GOLDEN  1.6180339f

#define M_2PI         (M_PI * 2)

#define CHAR_BIT    8
#define SCHAR_MIN  (-SCHAR_MAX - 1)
#define SCHAR_MAX   127
#define UCHAR_MAX   255

/* These could be different on machines where char is unsigned */

#ifdef __CHAR_UNSIGNED__
#define CHAR_MIN    0
#define CHAR_MAX    UCHAR_MAX
#else
#define CHAR_MIN    SCHAR_MIN
#define CHAR_MAX    SCHAR_MAX
#endif

#define SHRT_MIN    (-SHRT_MAX - 1)
#define SHRT_MAX    32767
#define USHRT_MAX   65535U

#define INT_MIN     (-INT_MAX - 1)
#define INT_MAX     2147483647
#define UINT_MAX    4294967295U

/* These change on 32-bit and 64-bit platforms */

#define LONG_MIN    (-LONG_MAX - 1)
#define LONG_MAX    2147483647L
#define ULONG_MAX   4294967295UL

#define LLONG_MIN   (-LLONG_MAX - 1)
#define LLONG_MAX   9223372036854775807LL
#define ULLONG_MAX  18446744073709551615ULL

/* A pointer is 4 bytes */

#define PTR_MIN     (-PTR_MAX - 1)
#define PTR_MAX     2147483647
#define UPTR_MAX    4294967295U

#endif 

