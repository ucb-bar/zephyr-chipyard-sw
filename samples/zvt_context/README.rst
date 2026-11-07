Zvt tile context switch test
############################

Checks that ``CONFIG_RISCV_ISA_EXT_ZVT`` preserves per-thread Zvt tile
state (all 16 tiles) and the ``mtype`` tile configuration across
preemptive (time-slice) and cooperative context switches, and that a
thread which never called ``arch_riscv_zvt_enable()`` faults on tile
instructions (``mstatus.MS`` = Off).

Needs a spike with Zvt (riscv-isa-sim >= 72958551)::

  west build -p -b spike_riscv64 samples/zvt_context
  spike --isa=rv64gcv_zicntr_zvtbase_zvt32t build/zephyr/zephyr.elf

The test prints ``ZVT_CONTEXT: PASS`` on success.
