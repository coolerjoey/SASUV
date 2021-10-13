#ifndef __MYMATH_H
#define __MYMATH_H	
#include <math.h>
#include "sys.h"
#include "arm_math.h"

typedef union{
	u8 u8vals[4];
	u32 u32val;
	float fval;
}int2f_t;

typedef float mat3f[3][3];
typedef float mat6f[6][6];
typedef float mat12f[12][12];
typedef float mat15f[15][15];
typedef float vec3f[3];
typedef double vec3d[3];
typedef float vec4f[4];
typedef float vec6f[6];
typedef float vec12f[12];
typedef float vec15f[15];
typedef float mat6_15f[6][15];
typedef float mat15_6f[15][6];
typedef float mat12_6f[12][6];

extern vec3f O31;
extern vec3f I31;
extern mat3f O33;
extern mat3f I33;
extern vec3f O61;
extern mat6f O66;
extern mat6f I66;
extern mat6_15f O6_15;
extern vec12f O12_1;
extern mat12_6f O12_6;
extern mat12f O12_12;
extern vec15f O15_1;
extern mat15f O15_15;
extern mat15_6f O15_6;

//注意：经纬度已经扩大了1e7倍 
#define LATLON_TO_M 0.01113166f
#define LATLON_TO_CM 1.113166f	//(111.316666 *10^5 / 1e7) 
#define M_TO_LATLON	(1/0.01113166f)
#define CM_TO_LATLON (1/1.113166f)

#define GRAVITY_MSS 9.7803267714f

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

#define DEG_TO_RAD (M_PI / 180.0f)	//角度 -> 弧度
#define RAD_TO_DEG (180.0f / M_PI)	//弧度 -> 角度

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

#define DEG_TO_RAD      (M_PI / 180.0f)
#define RAD_TO_DEG      (180.0f / M_PI)

#ifndef DEG
#define DEG		(M_PI/180.0)
#endif
#ifndef SEC
#define SEC		(DEG/3600.0)
#endif

#ifndef EPS
#define EPS		2.220446049e-16F
#endif
#ifndef INF
#define INF		3.402823466e+30F
#endif

bool is_zero(float val);
bool is_equal(float val_1, float val_2);
float int2float(float *,u8 *);
float norm(float x, float y);
float length_of_vec3f(vec3f vec);
float constrain_float(float val, float min, float max);
void set2vecf(float* data,u16 dim,float *vec,arm_matrix_instance_f32 *mat);
void set2vecf_sq(float *data,u16 dim,float *vec,arm_matrix_instance_f32 *mat);
void set2vecf_scale(float *data, float scale, u16 dim, float *vec,arm_matrix_instance_f32 *mat);
void set2matf(float* data,u16 rows,u16 cols,float *vec,arm_matrix_instance_f32 *mat);
void set2diag(float *data,u16 rows,float *diag,arm_matrix_instance_f32 *mat);
void set2diag_sq(float *data,u16 rows,float *diag,arm_matrix_instance_f32 *mat);
void askew(vec3f data,arm_matrix_instance_f32 *mat);
void setMat3(mat3f src,arm_matrix_instance_f32 *mat,int i,int j);
void setMat(float *src, int row, int cols, arm_matrix_instance_f32 *des,int i,int j);
void setMat2Mat(arm_matrix_instance_f32 *src,arm_matrix_instance_f32 *des,int i,int j);
void op_mat(float *src,float *des,	int rows, int cols);
void add_eye2mat(arm_matrix_instance_f32 *mat);
bool vecf_isZero(float *vec, int dims);
void symmetry(float *mat, int dims);
void mymath_test(void);
void print_mat(arm_matrix_instance_f32 mat);
void print_mat_name(arm_matrix_instance_f32 mat, const char* mat_name);

#endif 

