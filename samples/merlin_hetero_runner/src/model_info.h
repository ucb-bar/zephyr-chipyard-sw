/* Per-model shape / dtype / entry-function metadata.
 *
 * Selected at CMake configure time via -DMERLIN_MODEL=<name>, which sets
 * the MERLIN_MODEL_<NAME> preprocessor define. Source MLIRs were sampled
 * from /scratch2/agustin/merlin/models/*.q.int8.mlir; inputs/outputs are
 * all f32 even though the model is quantized internally (QDQ pattern). */

#ifndef MERLIN_MODEL_INFO_H_
#define MERLIN_MODEL_INFO_H_

#include <stddef.h>
#include <stdint.h>
#include "iree/hal/api.h"

typedef struct merlin_tensor_spec {
  const char* name;
  iree_hal_element_type_t element_type;
  int rank;
  const iree_hal_dim_t* shape;   /* length = rank */
  size_t element_count;          /* product of shape */
  size_t byte_count;             /* element_count * sizeof(element) */
} merlin_tensor_spec_t;

/* ----- mlp_wide -----
 * func.func @main_graph(%arg0: !torch.vtensor<[16,16],f32>)
 *   -> !torch.vtensor<[16,16],f32>                                         */
#if defined(MERLIN_MODEL_MLP_WIDE)
#define MERLIN_ENTRY_FUNCTION_NAME "module.main_graph"
#define MERLIN_INPUT_COUNT 1
#define MERLIN_OUTPUT_COUNT 1
static const iree_hal_dim_t merlin_in0_shape[]  = {16, 16};
static const iree_hal_dim_t merlin_out0_shape[] = {16, 16};
static const merlin_tensor_spec_t merlin_inputs[] = {
  {"arg0", IREE_HAL_ELEMENT_TYPE_FLOAT_32, 2, merlin_in0_shape, 16*16, 16*16*4},
};
static const merlin_tensor_spec_t merlin_outputs[] = {
  {"out0", IREE_HAL_ELEMENT_TYPE_FLOAT_32, 2, merlin_out0_shape, 16*16, 16*16*4},
};

/* ----- dronet -----
 * func.func @dronet(%arg0: !torch.vtensor<[1,3,112,112],f32>)
 *   -> (!torch.vtensor<[1,1],f32>, !torch.vtensor<[1,1],f32>)              */
#elif defined(MERLIN_MODEL_DRONET)
#define MERLIN_ENTRY_FUNCTION_NAME "module.dronet"
#define MERLIN_INPUT_COUNT 1
#define MERLIN_OUTPUT_COUNT 2
static const iree_hal_dim_t merlin_in0_shape[]  = {1, 3, 112, 112};
static const iree_hal_dim_t merlin_out0_shape[] = {1, 1};
static const iree_hal_dim_t merlin_out1_shape[] = {1, 1};
static const merlin_tensor_spec_t merlin_inputs[] = {
  {"arg0", IREE_HAL_ELEMENT_TYPE_FLOAT_32, 4, merlin_in0_shape,
   1*3*112*112, 1*3*112*112*4},
};
static const merlin_tensor_spec_t merlin_outputs[] = {
  {"steer", IREE_HAL_ELEMENT_TYPE_FLOAT_32, 2, merlin_out0_shape, 1, 4},
  {"coll",  IREE_HAL_ELEMENT_TYPE_FLOAT_32, 2, merlin_out1_shape, 1, 4},
};

/* ----- dronet with intermediate checkpoints (diagnostic) -----
 * func.func @main_graph(%arg0: !torch.vtensor<[1,3,112,112],f32>) ->
 *   (!torch.vtensor<[1,1],f32>,  // steer
 *    !torch.vtensor<[1,1],f32>,  // collision
 *    !torch.vtensor<[1,1],f32>,  // steer_QuantizeLinear_Input
 *    !torch.vtensor<[1,1],f32>)  // linear_1
 * Used to localize WHICH dispatch range first diverges between gemmini
 * and the ONNX-runtime golden — see tools/diff_dronet_dispatches.py for
 * the four golden checkpoint hashes.                                       */
#elif defined(MERLIN_MODEL_DRONET_WITH_INTERMEDIATE)
#define MERLIN_ENTRY_FUNCTION_NAME "module.main_graph"
#define MERLIN_INPUT_COUNT 1
#define MERLIN_OUTPUT_COUNT 4
static const iree_hal_dim_t merlin_in0_shape[]  = {1, 3, 112, 112};
static const iree_hal_dim_t merlin_out0_shape[] = {1, 1};
static const iree_hal_dim_t merlin_out1_shape[] = {1, 1};
static const iree_hal_dim_t merlin_out2_shape[] = {1, 1};
static const iree_hal_dim_t merlin_out3_shape[] = {1, 1};
static const merlin_tensor_spec_t merlin_inputs[] = {
  {"arg0", IREE_HAL_ELEMENT_TYPE_FLOAT_32, 4, merlin_in0_shape,
   1*3*112*112, 1*3*112*112*4},
};
static const merlin_tensor_spec_t merlin_outputs[] = {
  {"steer",                       IREE_HAL_ELEMENT_TYPE_FLOAT_32, 2, merlin_out0_shape, 1, 4},
  {"collision",                   IREE_HAL_ELEMENT_TYPE_FLOAT_32, 2, merlin_out1_shape, 1, 4},
  {"steer_QuantizeLinear_Input",  IREE_HAL_ELEMENT_TYPE_FLOAT_32, 2, merlin_out2_shape, 1, 4},
  {"linear_1",                    IREE_HAL_ELEMENT_TYPE_FLOAT_32, 2, merlin_out3_shape, 1, 4},
};

/* ----- yolov8n -----
 * func.func @main_graph(%arg0: !torch.vtensor<[1,3,320,320],f32>)
 *   -> !torch.vtensor<[1,84,2100],f32>                                     */
#elif defined(MERLIN_MODEL_YOLOV8N)
#define MERLIN_ENTRY_FUNCTION_NAME "module.main_graph"
#define MERLIN_INPUT_COUNT 1
#define MERLIN_OUTPUT_COUNT 1
static const iree_hal_dim_t merlin_in0_shape[]  = {1, 3, 320, 320};
static const iree_hal_dim_t merlin_out0_shape[] = {1, 84, 2100};
static const merlin_tensor_spec_t merlin_inputs[] = {
  {"arg0", IREE_HAL_ELEMENT_TYPE_FLOAT_32, 4, merlin_in0_shape,
   1*3*320*320, 1*3*320*320*4},
};
static const merlin_tensor_spec_t merlin_outputs[] = {
  {"out0", IREE_HAL_ELEMENT_TYPE_FLOAT_32, 3, merlin_out0_shape,
   1*84*2100, 1*84*2100*4},
};

#else
#error "Define one of MERLIN_MODEL_MLP_WIDE / MERLIN_MODEL_DRONET / MERLIN_MODEL_YOLOV8N (set via -DMERLIN_MODEL=<name>)"
#endif

#endif  /* MERLIN_MODEL_INFO_H_ */
