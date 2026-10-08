Merlin multithreaded multicore RVV (SAXPY)
##########################################

Demonstrates correct concurrent RISC-V Vector (RVV) compute across SMP harts, exercising
the per-thread vector context save/restore in ``arch/riscv/core/v.c``.

``NUM_THREADS = 3 * MP_MAX_NUM_CPUS`` worker threads are pinned round-robin across the harts
(so several V-using threads time-share each hart). Each repeatedly accumulates
``y[i] += a * x[i]`` over its own 2048-float slice with an RVV ``vfmacc`` kernel
(``.option arch, +v`` inline asm), ``k_yield()``-ing between rounds so the scheduler
interleaves the V-using threads on each hart. Each thread then validates its slice against a
scalar reference and prints PASS/FAIL; ``main`` tallies the result.

If ``v.c`` did not save/restore each thread's ``v0..v31`` + V CSRs on context switch, a
thread preempted mid-kernel would resume with another thread's vector registers and its
slice would be wrong — so 12/12 PASS is a positive correctness check on the V context
switch under SMP.

Build & run (Zephyr SDK 0.17.0 + chipyard spike)::

   export ZEPHYR_BASE=.../zephyr_ws/zephyr
   export ZEPHYR_TOOLCHAIN_VARIANT=zephyr ZEPHYR_SDK_INSTALL_DIR=.../zephyr-sdk-0.17.0
   cmake -B build -G Ninja -DBOARD=spike_riscv64 -S samples/merlin_mt_rvv_saxpy
   ninja -C build
   spike -p4 --isa=rv64gcv_zicntr build/zephyr/zephyr.elf

Required config (see ``prj.conf``): ``CONFIG_RISCV_ISA_EXT_V_LAZY=n`` selects the eager V
save/restore (the Kconfig default ``y`` gates it out, leaving ``MSTATUS.VS`` OFF so the
first vector instruction traps); ``CONFIG_RISCV_V_KERNEL_ONLY=y`` keeps libc/kernel scalar
so they link against the SDK's non-V picolibc while the app's V comes from inline asm.

Expected output::

   Merlin multicore RVV SAXPY: 12 threads over 4 harts (...)
   thread  N on hart H: PASS (2048 elems, a=...)
   ...
   RESULT: 12/12 threads PASS
