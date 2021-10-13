#pragma once

#ifdef __PX4_QURT
#include "dspal_math.h"
#endif
#include "Matrix.hpp"
#include "SquareMatrix.hpp"
#include "Vector.hpp"
#include "Vector2.hpp"
#include "Vector3.hpp"
#include "Euler.hpp"
#include "Dcm.hpp"
#include "Scalar.hpp"
#include "Quaternion.hpp"

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
