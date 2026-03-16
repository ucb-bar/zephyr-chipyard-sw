// Simple embedding sample adapted from the FireSim bare-metal example.
// NOTE: This expects IREE runtime headers and libraries to be available
// on the include path and link line.

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <zephyr/sys/reboot.h>

#include "iree/base/api.h"
#include "iree/hal/api.h"
#include "iree/modules/hal/module.h"
#include "iree/vm/api.h"
#include "iree/vm/bytecode/module.h"

#define L_TRACE_ENCODER_BASE_ADDRESS 0x3000000

// NOTE: The trace encoder is a platform-specific MMIO device. Under Spike (and
// many Zephyr targets) this address may be unmapped and will fault if touched.
// Enable explicitly only when running on a platform that provides the device.
#ifndef L_TRACE_ENCODER_ENABLE
#define L_TRACE_ENCODER_ENABLE 0
#endif

// Debug helpers. We fflush() so output isn't lost if Spike aborts abruptly.
#define DBG_PRINTF(...)         \
  do {                          \
    printf(__VA_ARGS__);        \
    fflush(stdout);             \
  } while (0)
#define DBG_FPRINTF(...)        \
  do {                          \
    fprintf(stderr, __VA_ARGS__); \
    fflush(stderr);             \
  } while (0)

static iree_status_t DebugPrintStatus(iree_status_t status, const char* what) {
  if (iree_status_is_ok(status)) return status;
  DBG_FPRINTF("\n[Run] IREE call failed at: %s\n", what);
  iree_status_fprint(stderr, status);
  DBG_FPRINTF("\n");
  return status;
}

#define IREE_RETURN_IF_ERROR_DBG(expr, what) \
  IREE_RETURN_IF_ERROR(DebugPrintStatus((expr), (what)))

typedef struct {
  volatile uint32_t TR_TE_CTRL;
  volatile uint32_t TR_TE_INFO;
  volatile uint32_t TR_TE_BUBBLE[6];
  volatile uint32_t TR_TE_TARGET;
  volatile uint32_t TR_TE_BRANCH_MODE;
} LTraceEncoderType;

static inline void l_trace_encoder_start(uint32_t hart_id) {
#if L_TRACE_ENCODER_ENABLE
  uintptr_t base_addr = (uintptr_t)L_TRACE_ENCODER_BASE_ADDRESS;
  LTraceEncoderType* encoder =
      (LTraceEncoderType*)(base_addr + (hart_id * 0x1000));
  encoder->TR_TE_CTRL |= (0x1 << 1);
#else
  (void)hart_id;
#endif
}

static inline void l_trace_encoder_stop(uint32_t hart_id) {
#if L_TRACE_ENCODER_ENABLE
  uintptr_t base_addr = (uintptr_t)L_TRACE_ENCODER_BASE_ADDRESS;
  LTraceEncoderType* encoder =
      (LTraceEncoderType*)(base_addr + (hart_id * 0x1000));
  encoder->TR_TE_CTRL &= ~(0x1 << 1);
#else
  (void)hart_id;
#endif
}

// Provided by the platform-specific IREE HAL backend.
extern iree_status_t create_sample_device(iree_allocator_t host_allocator,
                                          iree_hal_device_t** out_device);
extern const iree_const_byte_span_t load_bytecode_module_data(void);

static iree_status_t Run(void) {
  DBG_PRINTF("[Run] enter\n");
  // Use the runtime-provided system allocator.
  iree_allocator_t host_allocator = iree_allocator_system();

  iree_vm_instance_t* instance = NULL;
  DBG_PRINTF("[Run] creating vm instance...\n");
  IREE_RETURN_IF_ERROR_DBG(
      iree_vm_instance_create(IREE_VM_TYPE_CAPACITY_DEFAULT, host_allocator,
                              &instance),
      "iree_vm_instance_create");
  DBG_PRINTF("[Run] vm instance = %p\n", (void*)instance);

  DBG_PRINTF("[Run] registering HAL module types...\n");
  IREE_RETURN_IF_ERROR_DBG(iree_hal_module_register_all_types(instance),
                           "iree_hal_module_register_all_types");

  iree_hal_device_t* device = NULL;
  DBG_PRINTF("[Run] creating sample device...\n");
  IREE_RETURN_IF_ERROR_DBG(create_sample_device(host_allocator, &device),
                           "create_sample_device");
  DBG_PRINTF("[Run] device = %p\n", (void*)device);

  iree_vm_module_t* hal_module = NULL;
  DBG_PRINTF("[Run] creating HAL module...\n");
  IREE_RETURN_IF_ERROR_DBG(
      iree_hal_module_create(instance, iree_hal_module_device_policy_default(),
                             /*device_count=*/1, &device,
                             IREE_HAL_MODULE_FLAG_SYNCHRONOUS,
                             iree_hal_module_debug_sink_null(), host_allocator,
                             &hal_module),
      "iree_hal_module_create");
  DBG_PRINTF("[Run] hal_module = %p\n", (void*)hal_module);

  DBG_PRINTF("[Run] loading bytecode module data...\n");
  const iree_const_byte_span_t module_data = load_bytecode_module_data();
  DBG_PRINTF("[Run] module_data: ptr=%p size=%zu\n", (void*)module_data.data,
             (size_t)module_data.data_length);

  iree_vm_module_t* bytecode_module = NULL;
  DBG_PRINTF("[Run] creating bytecode module...\n");
  IREE_RETURN_IF_ERROR_DBG(
      iree_vm_bytecode_module_create(instance, module_data,
                                     iree_allocator_null(), host_allocator,
                                     &bytecode_module),
      "iree_vm_bytecode_module_create");
  DBG_PRINTF("[Run] bytecode_module = %p\n", (void*)bytecode_module);

  iree_vm_context_t* context = NULL;
  iree_vm_module_t* modules[] = {hal_module, bytecode_module};
  DBG_PRINTF("[Run] creating vm context with modules...\n");
  IREE_RETURN_IF_ERROR_DBG(
      iree_vm_context_create_with_modules(
          instance, IREE_VM_CONTEXT_FLAG_NONE, IREE_ARRAYSIZE(modules),
          &modules[0], host_allocator, &context),
      "iree_vm_context_create_with_modules");
  DBG_PRINTF("[Run] context = %p\n", (void*)context);
  iree_vm_module_release(hal_module);
  iree_vm_module_release(bytecode_module);

  const char kMainFunctionName[] = "module.vanilla_matmul_large";
  iree_vm_function_t main_function;
  DBG_PRINTF("[Run] resolving function: %s\n", kMainFunctionName);
  IREE_RETURN_IF_ERROR_DBG(
      iree_vm_context_resolve_function(
          context, iree_make_cstring_view(kMainFunctionName), &main_function),
      "iree_vm_context_resolve_function");
  DBG_PRINTF("[Run] function resolved\n");

  const int kDim = 128;
  const int kCount = kDim * kDim;

  DBG_PRINTF("[Run] allocating host input buffers (kCount=%d)...\n", kCount);
  int8_t* kInt8_4 = (int8_t*)malloc(kCount * sizeof(int8_t));
  int8_t* kInt8_2 = (int8_t*)malloc(kCount * sizeof(int8_t));
  int32_t* kInt32_Zero = (int32_t*)malloc(kCount * sizeof(int32_t));
  DBG_PRINTF("[Run] mallocs: kInt8_4=%p kInt8_2=%p kInt32_Zero=%p\n",
             (void*)kInt8_4, (void*)kInt8_2, (void*)kInt32_Zero);

  if (!kInt8_4 || !kInt8_2 || !kInt32_Zero) {
    return iree_make_status(IREE_STATUS_RESOURCE_EXHAUSTED,
                            "buffer allocation failed");
  }

  DBG_PRINTF("[Run] initializing host inputs...\n");
  for (int i = 0; i < kCount; ++i) {
    kInt8_4[i] = 4;
    kInt8_2[i] = 2;
    kInt32_Zero[i] = 0;
  }

  iree_hal_dim_t shape[2] = {kDim, kDim};
  DBG_PRINTF("[Run] tensor shape = [%d, %d]\n", kDim, kDim);

  iree_hal_buffer_view_t* arg0_buffer_view = NULL;
  iree_hal_buffer_view_t* arg1_buffer_view = NULL;
  iree_hal_buffer_view_t* arg2_buffer_view = NULL;

  DBG_PRINTF("[Run] uploading arg0...\n");
  IREE_RETURN_IF_ERROR_DBG(
      iree_hal_buffer_view_allocate_buffer_copy(
          device, iree_hal_device_allocator(device), IREE_ARRAYSIZE(shape),
          shape, IREE_HAL_ELEMENT_TYPE_SINT_8,
          IREE_HAL_ENCODING_TYPE_DENSE_ROW_MAJOR,
          (iree_hal_buffer_params_t){
              .type = IREE_HAL_MEMORY_TYPE_DEVICE_LOCAL,
              .usage = IREE_HAL_BUFFER_USAGE_DEFAULT,
          },
          iree_make_const_byte_span(kInt8_4, kCount * sizeof(int8_t)),
          &arg0_buffer_view),
      "iree_hal_buffer_view_allocate_buffer_copy(arg0)");
  DBG_PRINTF("[Run] arg0_buffer_view = %p\n", (void*)arg0_buffer_view);

  DBG_PRINTF("[Run] uploading arg1...\n");
  IREE_RETURN_IF_ERROR_DBG(
      iree_hal_buffer_view_allocate_buffer_copy(
          device, iree_hal_device_allocator(device), IREE_ARRAYSIZE(shape),
          shape, IREE_HAL_ELEMENT_TYPE_SINT_8,
          IREE_HAL_ENCODING_TYPE_DENSE_ROW_MAJOR,
          (iree_hal_buffer_params_t){
              .type = IREE_HAL_MEMORY_TYPE_DEVICE_LOCAL,
              .usage = IREE_HAL_BUFFER_USAGE_DEFAULT,
          },
          iree_make_const_byte_span(kInt8_2, kCount * sizeof(int8_t)),
          &arg1_buffer_view),
      "iree_hal_buffer_view_allocate_buffer_copy(arg1)");
  DBG_PRINTF("[Run] arg1_buffer_view = %p\n", (void*)arg1_buffer_view);

  DBG_PRINTF("[Run] uploading arg2...\n");
  IREE_RETURN_IF_ERROR_DBG(
      iree_hal_buffer_view_allocate_buffer_copy(
          device, iree_hal_device_allocator(device), IREE_ARRAYSIZE(shape),
          shape, IREE_HAL_ELEMENT_TYPE_SINT_32,
          IREE_HAL_ENCODING_TYPE_DENSE_ROW_MAJOR,
          (iree_hal_buffer_params_t){
              .type = IREE_HAL_MEMORY_TYPE_DEVICE_LOCAL,
              .usage = IREE_HAL_BUFFER_USAGE_DEFAULT,
          },
          iree_make_const_byte_span(kInt32_Zero, kCount * sizeof(int32_t)),
          &arg2_buffer_view),
      "iree_hal_buffer_view_allocate_buffer_copy(arg2)");
  DBG_PRINTF("[Run] arg2_buffer_view = %p\n", (void*)arg2_buffer_view);

  iree_vm_list_t* inputs = NULL;
  DBG_PRINTF("[Run] creating inputs list...\n");
  IREE_RETURN_IF_ERROR_DBG(
      iree_vm_list_create(iree_vm_make_undefined_type_def(),
                          /*capacity=*/3, host_allocator, &inputs),
      "iree_vm_list_create(inputs)");
  DBG_PRINTF("[Run] inputs = %p\n", (void*)inputs);

  iree_vm_ref_t arg0_ref = iree_hal_buffer_view_move_ref(arg0_buffer_view);
  iree_vm_ref_t arg1_ref = iree_hal_buffer_view_move_ref(arg1_buffer_view);
  iree_vm_ref_t arg2_ref = iree_hal_buffer_view_move_ref(arg2_buffer_view);

  DBG_PRINTF("[Run] pushing input refs...\n");
  IREE_RETURN_IF_ERROR_DBG(iree_vm_list_push_ref_move(inputs, &arg0_ref),
                           "iree_vm_list_push_ref_move(arg0)");
  IREE_RETURN_IF_ERROR_DBG(iree_vm_list_push_ref_move(inputs, &arg1_ref),
                           "iree_vm_list_push_ref_move(arg1)");
  IREE_RETURN_IF_ERROR_DBG(iree_vm_list_push_ref_move(inputs, &arg2_ref),
                           "iree_vm_list_push_ref_move(arg2)");

  iree_vm_list_t* outputs = NULL;
  DBG_PRINTF("[Run] creating outputs list...\n");
  IREE_RETURN_IF_ERROR_DBG(
      iree_vm_list_create(iree_vm_make_undefined_type_def(),
                          /*capacity=*/1, host_allocator, &outputs),
      "iree_vm_list_create(outputs)");
  DBG_PRINTF("[Run] outputs = %p\n", (void*)outputs);

  DBG_PRINTF("[Run] invoking 128x128 matmul...\n");
  IREE_RETURN_IF_ERROR_DBG(
      iree_vm_invoke(context, main_function, IREE_VM_INVOCATION_FLAG_NONE,
                     /*policy=*/NULL, inputs, outputs, host_allocator),
      "iree_vm_invoke");
  DBG_PRINTF("[Run] invocation complete\n");

  iree_hal_buffer_view_t* ret_buffer_view =
      iree_vm_list_get_buffer_view_assign(outputs, 0);
  if (ret_buffer_view == NULL) {
    return iree_make_status(IREE_STATUS_NOT_FOUND,
                            "can't find return buffer view");
  }
  DBG_PRINTF("[Run] ret_buffer_view = %p\n", (void*)ret_buffer_view);

  DBG_PRINTF("[Run] allocating host results buffer...\n");
  int32_t* results = (int32_t*)malloc(kCount * sizeof(int32_t));
  if (!results) {
    return iree_make_status(IREE_STATUS_RESOURCE_EXHAUSTED,
                            "results buffer allocation failed");
  }
  DBG_PRINTF("[Run] results = %p\n", (void*)results);

  DBG_PRINTF("[Run] transferring results device->host (%zu bytes)...\n",
             (size_t)(kCount * sizeof(int32_t)));
  IREE_RETURN_IF_ERROR_DBG(
      iree_hal_device_transfer_d2h(
          device, iree_hal_buffer_view_buffer(ret_buffer_view), 0, results,
          kCount * sizeof(int32_t), IREE_HAL_TRANSFER_BUFFER_FLAG_DEFAULT,
          iree_infinite_timeout()),
      "iree_hal_device_transfer_d2h");
  DBG_PRINTF("[Run] transfer complete\n");

  DBG_PRINTF("[Run] validating results...\n");
  int errors = 0;
  for (iree_host_size_t i = 0; i < kCount; ++i) {
    if (results[i] != 1024) {
      if (errors < 10) {
        printf("Mismatch at %d: Expected 1024, got %d\n", (int)i, results[i]);
      }
      errors++;
    }
  }

  if (errors > 0) {
    printf("Total errors: %d\n", errors);
    return iree_make_status(IREE_STATUS_UNKNOWN, "result mismatches");
  }

  free(kInt8_4);
  free(kInt8_2);
  free(kInt32_Zero);
  free(results);
  iree_vm_list_release(inputs);
  iree_vm_list_release(outputs);
  iree_hal_device_release(device);
  iree_vm_context_release(context);
  iree_vm_instance_release(instance);
  DBG_PRINTF("[Run] exit ok\n");
  return iree_ok_status();
}

int main(void) {
  // Optional trace hooks kept to mirror the FireSim example.
  // l_trace_encoder_start(0);

  printf("simple_embedding started\n");
  const iree_status_t result = Run();
  if (!iree_status_is_ok(result)) {
    iree_status_fprint(stderr, result);
    iree_status_free(result);
    return 1;
  }
  printf("simple_embedding done\n");

  // Only touch trace encoder MMIO when explicitly enabled.
  if (L_TRACE_ENCODER_ENABLE) {
    l_trace_encoder_stop(0);
    printf("Trace Stopped.\n");
  }
	sys_reboot(SYS_REBOOT_COLD);
  return 0;
}

