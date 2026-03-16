#ifndef IREE_ZEPHYR_COMPAT_H_
#define IREE_ZEPHYR_COMPAT_H_

// Force IREE runtime internals into a minimal generic mode that is compatible
// with Zephyr cross builds from an ExternalProject sub-build.
#define IREE_SYNCHRONIZATION_DISABLE_UNSAFE 1
#define IREE_FILE_IO_ENABLE 0
#define IREE_WAIT_UNTIL_FN sizeof

// Generic platform time fallback used by iree/base/internal/time.c.
#define IREE_TIME_NOW_FN return 0;

#endif  // IREE_ZEPHYR_COMPAT_H_

