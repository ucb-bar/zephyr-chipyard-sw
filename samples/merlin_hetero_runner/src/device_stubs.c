/* Local-sync device factory + bytecode-module accessor.
 *
 * The IREE archives in this Zephyr build include both local-sync and
 * local-task HAL drivers. We instantiate the local-sync device here:
 *  - inline executor — no internal threading required from IREE
 *  - the iree_thread_zephyr.c + iree_futex_zephyr.c bridge files are still
 *    linked in for iree_slim_mutex / iree_notification on the bytecode-VM
 *    side (those use futex)
 *  - app-level multi-hart parallelism remains available: spawn one
 *    k_thread per Shuttle hart, each driving its own iree_vm_context.
 *
 * Switching to iree_hal_task_device_create would let IREE schedule
 * dispatch across multiple workers in a SINGLE inference, but requires a
 * non-stub proactor backend on Zephyr (the upstream stub vtable leaves
 * notification_signal/wait + relay methods as NULL, which the task HAL's
 * semaphore-init path calls during iree_vm_context_create — load access
 * fault). Real backend = separate workstream.
 */

#include <stdio.h>

#include <zephyr/sys/printk.h>

#include "iree/async/util/proactor_pool.h"
#include "iree/base/api.h"
#include "iree/hal/api.h"
#include "iree/hal/device_group.h"
#include "iree/hal/drivers/local_sync/sync_device.h"
#include "iree/hal/local/executable_loader.h"
#include "iree/hal/local/loaders/embedded_elf_loader.h"
/* hetero_runner variant: bytecode comes from per-worker state
 * (merlin_vmfb_h0/h1.h embedded in main.c), not from a single global
 * symbol. So load_bytecode_module_data() is omitted below. */

iree_status_t create_sample_device(iree_allocator_t host_allocator,
                                   iree_hal_device_t** out_device) {
  *out_device = NULL;
  printk("[bp] device: sync_params_init\n");

  iree_hal_sync_device_params_t params;
  iree_hal_sync_device_params_initialize(&params);

  iree_hal_device_create_params_t create_params =
      iree_hal_device_create_params_default();

  printk("[bp] device: proactor_pool_create\n");
  iree_async_proactor_pool_t* proactor_pool = NULL;
  iree_status_t pool_status = iree_async_proactor_pool_create(
      /*node_count=*/1, /*node_ids=*/NULL,
      iree_async_proactor_pool_options_default(), host_allocator,
      &proactor_pool);
  if (!iree_status_is_ok(pool_status)) {
    return pool_status;
  }
  create_params.proactor_pool = proactor_pool;
  printk("[bp] device: pool OK\n");

  iree_hal_executable_loader_t* loader = NULL;
  iree_status_t status = iree_hal_embedded_elf_loader_create(
      /*plugin_manager=*/NULL, host_allocator, &loader);
  if (!iree_status_is_ok(status)) {
    iree_async_proactor_pool_release(proactor_pool);
    return status;
  }

  iree_string_view_t identifier = iree_make_cstring_view("local-sync");

  printk("[bp] device: allocator_create_heap\n");
  iree_hal_allocator_t* device_allocator = NULL;
  status = iree_hal_allocator_create_heap(identifier, host_allocator,
                                          host_allocator, &device_allocator);
  if (iree_status_is_ok(status)) {
    printk("[bp] device: hal_sync_device_create\n");
    status = iree_hal_sync_device_create(
        identifier, &params, &create_params,
        /*loader_count=*/1, &loader, device_allocator, host_allocator,
        out_device);
    printk("[bp] device: sync_device_create rc=%d\n",
           (int)iree_status_code(status));
  }

  iree_hal_allocator_release(device_allocator);
  iree_hal_executable_loader_release(loader);
  iree_async_proactor_pool_release(proactor_pool);
  return status;
}

/* load_bytecode_module_data() intentionally not provided in hetero_runner;
 * each worker reads its own VMFB blob via merlin_worker_state_t.vmfb. */
