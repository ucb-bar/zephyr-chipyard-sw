#pragma once

#define NSTATES 12
#define NINPUTS 4
#define NHORIZON 10
#define NTOTAL 301

// tinytype now comes from matlib_types.h (driven by USE_TYPE_FP16).
// Pull it in here too so headers that don't include matlib first still see it.
#include "matlib_types.h"
