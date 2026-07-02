RoSE bridge interface samples
=============================

Self-checking Zephyr apps that exercise the different usages of the RoSE co-sim
bridge interface, via the ``rose`` Zephyr module (driver + protocol layer) in
``$ROSE_DIR/soc/sw/zephyr-rose``. Each test receives the known ``PatternEnv``
ramp (``0xC0DE0000 + 0,1,...``) over the bridge and validates every word,
printing a single ``ROSE <name>: PASS/FAIL ...`` marker (see
``common/rose_check.h``).

Samples
-------

``reqrsp``    Low-level reqrsp RX: ``rose_tx`` the request, read the framed
              ``[header, num_bytes, data...]`` response word-by-word with
              ``rose_rx`` on channel 2, validate. (cmd ``0x12`` -> ch2)

``dma``       Camera-DMA RX, interrupt-driven: ``rose_dma_arm`` + request +
              ``rose_dma_wait`` (blocks on the DMA-complete PLIC IRQ), then
              validate the buffer the engine wrote to memory. (cmd ``0x11`` -> ch0)

``protocol``  High-level transport layer: ``rose_request_camera`` +
              ``rose_recv_reqrsp`` (framing stripped automatically), validate.
              (cmd ``0x12`` -> ch2)

``selftest``  Combined regression: runs the DMA path (ch0) and the reqrsp path
              (ch2) in one binary and prints a final
              ``ROSE selftest: dma=.. reqrsp=.. => PASS/FAIL`` summary. Use this
              for one-shot FPGA validation of both paths.

Building
--------

The samples need the out-of-tree ``rose`` module::

    ROSE=$ROSE_DIR   # repo root (…/RoSE)
    west build -p always -b spike_riscv64 samples/rose/<name> \
        -- -DZEPHYR_EXTRA_MODULES=$ROSE/soc/sw/zephyr-rose

Running (metasim or FPGA)
-------------------------

All samples share ONE synchronizer config
(``$ROSE_DIR/deploy/config/config_gym_PatternEnv-v0.yaml``), which serves the
pattern on both routes (``0x11`` -> ch0 DMA, ``0x12`` -> ch2 reqrsp). Start it
with ``run_sync_only.py`` (PatternEnv), then run the built ``zephyr.elf`` as the
FireSim ``SIM_BINARY`` (metasim) or as a ``rose-*`` workload (FPGA). On FPGA set
``plusarg_passthrough: "+partitioned=1"`` so the intentional bridge stalls are
not flagged as a deadlock.
