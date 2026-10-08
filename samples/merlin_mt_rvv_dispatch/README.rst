Merlin multicore RVV dispatch executor
######################################

A level-synchronous executor for a Merlin dispatch schedule on Zephyr SMP, with RVV
kernels. A fixed thread pool (one worker pinned per hart) runs all dispatches of one
dependency level concurrently across harts, barriers, then advances to the next level —
the structure ``merlin/xdsl_dialects/lowering/schedule_dispatch.py`` emits
(``partition_dispatches`` -> levels + per-node hart assignment; ``emit_schedule_c`` writes
the matching C dispatch table).

The embedded schedule is the canonical diamond::

   P = A * X   (level 0, hart 0)   ┐ run concurrently
   Q = B * X   (level 0, hart 1)   ┘
   R = P * Q   (level 1, hart 0)   waits on the level-0 barrier

Each matmul is an RVV kernel (rank-1 ``vfmacc`` updates, ``.option arch,+v`` inline asm), so
V-using worker threads run on distinct harts and rely on ``arch/riscv/core/v.c`` to keep
their vector state separate. ``R`` is validated against a scalar reference.

Build & run::

   cmake -B build -G Ninja -DBOARD=spike_riscv64 -S samples/merlin_mt_rvv_dispatch
   ninja -C build
   spike -p4 --isa=rv64gcv_zicntr build/zephyr/zephyr.elf

Expected::

   level 0: 2 dispatch(es) done
   level 1: 1 dispatch(es) done
   RESULT: PASS (1024/1024 elems correct)

See ``prj.conf`` for the required V config (eager ``v.c`` path: ``ISA_EXT_V_LAZY=n`` +
``RISCV_V_KERNEL_ONLY=y``).
