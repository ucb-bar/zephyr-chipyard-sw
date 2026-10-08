/*
 * iree_picolibc_compat.c — bridge newlib symbol references emitted by the
 * IREE static archives (built by riscv_firesim.toolchain.cmake against
 * chipyard's riscv-tools NEWLIB sysroot) into the picolibc-based Zephyr
 * SDK environment.
 *
 * IREE archives reference: _ctype_, _impure_ptr, __errno, iree_thread_create,
 * iree_thread_release, plus picolibc's sbrk wants __heap_start/__heap_end.
 *
 * None of these references is reachable on the IREE_PLATFORM_GENERIC +
 * IREE_ENABLE_THREADING=OFF + IREE_FILE_IO_ENABLE=0 code paths we exercise
 * (no stdio, no async, no scratch-allocator sbrk), but the linker pulls
 * them in via archive symbol resolution. Providing dummies here lets the
 * unused code be GC'd by --gc-sections at the final ELF link step. If any
 * of these are actually called at runtime, IREE itself is misconfigured.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include <zephyr/sys/printk.h>

/* ---- newlib stdio reentrancy struct (_reent) ---------------------------
 * IREE's stdio paths reference `_impure_ptr->...`. We provide a large
 * opaque buffer so any field accesses land in valid memory; the buffer
 * is BSS-zeroed so any field read returns 0/NULL. */
__attribute__((aligned(8))) static char _iree_compat_reent[4096];
void *_impure_ptr = _iree_compat_reent;

/* ---- newlib ctype table -----------------------------------------------
 * `_ctype_` is a 257-byte table in newlib; index -1 is the EOF slot,
 * 0..255 are character flags. We zero-fill — `isalpha(c)` etc. will
 * return false for every character, which is benign for the parse paths
 * that pull this reference in. */
static const char _iree_compat_ctype_table[257] = {0};
const char *_ctype_ = &_iree_compat_ctype_table[1];

/* ---- newlib errno indirection ------------------------------------------
 * Newlib stdio uses `*__errno() = X` to set errno; picolibc has plain
 * `errno`. Bridge by returning &errno (Zephyr/picolibc's thread-local). */
int *__errno(void) {
  return &errno;
}

/* ---- fprintf shim ------------------------------------------------------
 * IREE compiled against chipyard newlib calls fprintf(stderr, ...) for
 * status / diag prints. Newlib's fprintf dereferences `_impure_ptr->_stdout`
 * et al. — our `_impure_ptr` stub is zero-filled so those FILE* are NULL
 * and fprintf load-faults on the first field access.
 *
 * Override fprintf to route through Zephyr's printk (which doesn't need
 * a FILE* and goes directly to the console driver). The FILE* arg is
 * ignored — IREE only ever passes stdout/stderr equivalents.
 *
 * The function lives in our app object file, so the linker resolves the
 * fprintf undef-ref in IREE archives to this implementation before
 * picolibc's libc.a is searched. */
int fprintf(FILE *stream, const char *fmt, ...) {
  (void)stream;
  va_list ap;
  va_start(ap, fmt);
  vprintk(fmt, ap);
  va_end(ap);
  return 0;
}

/* Same for the stderr-rooted iree_status_fprint internal helpers if they
 * end up referencing fputs / fputc directly. */
int fputs(const char *s, FILE *stream) {
  (void)stream;
  printk("%s", s);
  return 0;
}

int fputc(int c, FILE *stream) {
  (void)stream;
  printk("%c", c);
  return c;
}

/* fflush shim — newlib's fflush reads `stream->_seek` at offset 0x18 and
 * jumps through it; with our zero-filled _impure_ptr that's a NULL deref.
 * Since fprintf/fputs/fputc above already go through printk synchronously,
 * fflush is a no-op. */
int fflush(FILE *stream) {
  (void)stream;
  return 0;
}

/* ---- iree async/threading stubs ----------------------------------------
 * No longer needed: real iree_thread_create / iree_thread_release impls
 * are provided by src/iree_thread_zephyr.c (k_thread-backed). Same for
 * iree_thread_join / yield / id / priority overrides / affinity. */

/* ---- heap boundary symbols --------------------------------------------
 * Picolibc's libc_picolib_picosbrk.c uses __heap_start / __heap_end as
 * the BSS heap bounds (treated as `extern char __heap_start[]` and
 * accessed by address). Define them via inline asm so they end up as
 * real linker symbols bracketing a BSS region — picolibc's malloc
 * then has a working sbrk-backed heap and we avoid the multiple-defn
 * conflict that CONFIG_COMMON_LIBC_MALLOC=y triggers with picolibc's
 * own nano-malloc.
 *
 * 2026-05-22: bumped from 4 MiB to 64 MiB. Note this is SEPARATE from
 * CONFIG_HEAP_MEM_POOL_SIZE (= Zephyr's k_heap kernel heap). IREE's
 * `iree_allocator_libc()` calls libc malloc which goes through picolibc
 * sbrk, which uses these symbols. yolov8n's 9.35 MB Gemmini VMFB
 * expands to ~20 MB of loaded dispatch ELFs and tripped
 * RESOURCE_EXHAUSTED in iree_hal_executable_create with 4 MiB. The
 * chipyard SoC has 256 MiB SRAM at 0x80000000 so 64 MiB is well within
 * budget. */
__asm__(
    ".section .bss.iree_heap, \"aw\", @nobits\n"
    ".balign 16\n"
    ".global __heap_start\n"
    ".global __heap_end\n"
    "__heap_start:\n"
    ".space 64 * 1024 * 1024\n"
    "__heap_end:\n"
    ".previous\n"
);
