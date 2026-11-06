#pragma once
#ifndef TINYMPC_MATLIB_TYPES_H
#define TINYMPC_MATLIB_TYPES_H

#include <float.h>

// USE_TYPE_FP16 is set by the CMake plumbing (matlib.cmake / matlib_header.cmake)
// when USE_TYPE=float16. When undefined, matlib defaults to single precision.

#ifdef USE_TYPE_FP16
typedef _Float16 tinytype;
// Wider accumulator used in dot-product kernels (matmul/matvec/matnorm).
// For fp16 storage we accumulate in fp32 to keep dot-product roundoff
// from dominating; can be disabled with -DMATLIB_NARROW_ACC=1.
#ifndef MATLIB_NARROW_ACC
typedef float tinytype_acc;
#else
typedef _Float16 tinytype_acc;
#endif
#define MATLIB_FLOAT_MIN ((_Float16)6.103515625e-05f)
#define MATLIB_FLOAT_MAX ((_Float16)65504.0f)
#define MATLIB_COMPARE_TOL 1.0e-2f
#else
typedef float tinytype;
typedef float tinytype_acc;
#define MATLIB_FLOAT_MIN FLT_MIN
#define MATLIB_FLOAT_MAX FLT_MAX
#define MATLIB_COMPARE_TOL 1.0e-6f
#endif

#endif // TINYMPC_MATLIB_TYPES_H
