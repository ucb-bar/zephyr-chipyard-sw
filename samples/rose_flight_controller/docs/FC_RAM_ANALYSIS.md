# Flight-Controller SRAM Analysis — freeing RAM for a WiFi/BLE telemetry stack

**Target:** ESP32-C6 (`esp32c6_devkitc/esp32c6/hpcore`), Zephyr, `samples/rose_flight_controller`.
**Question:** the FC sits at ~94–96 % of the ~437 KB `sram0_0_seg`. Where does the SRAM go, what can we
free, and does a WiFi stack fit (or only BLE)?

**Bottom line (spoiler):** one line of config frees ~290 KB. The FC is *not* actually full of controller
data — it reserves a **320 KB main-thread stack for a TinyMPC solve that isn't even linked into the flight
build**. Right-size that stack and the FC drops from **96.8 % → 29.4 %** of SRAM, leaving **~300 KB free** —
enough for a full WiFi + telemetry stack, not just BLE.

---

## 0. Method / how these numbers were produced

- SRAM region size from the linker: `zephyr.map` → *Memory Configuration* → `sram0_0_seg 0x40800000 len 0x6ad08`
  = **437,512 B**. On the ESP32-C6 HP-SRAM this one segment holds *both* the IRAM (`.iram0.*`, code that runs
  from RAM) and DRAM (`.dram0.*`, data/bss/noinit) output sections, so everything below counts against it.
- Section totals: `riscv64-zephyr-elf-size -A zephyr.elf` (Zephyr SDK 0.17.2).
- Per-symbol sizes: `riscv64-zephyr-elf-nm --size-sort --print-size`.
- Stack / heap / buffer placement: the `.dram0.noinit` and `.dram0.bss` blocks of `zephyr.map`.
- Two **empirical** scratch builds (separate build dirs, no source/CMake changes — flags + a one-line
  overlay only) confirmed the savings; see the Appendix for exact commands.

Builds analysed: `build_fly` (flight: PID + complementary + BUMPER + FLOW + FLIGHTLOG + AUTOFLIGHT, ~96.8 %)
and `build_flow` (PID + complementary + BUMPER + FLOW, ~94.7 %). Both build with **`ROSE_USE_PID=1`,
`ROSE_USE_EKF=0`, `ROSE_THREADED=0`**.

---

## 1. Ranked SRAM (sram0) breakdown — `build_fly`

`sram0_0_seg` = **437,512 B**. Used ≈ **423,332 B (96.8 %)**; free ≈ **14,180 B (3.2 %)**.

| # | Component (symbol / section) | Bytes | KB | % sram0 |
|--:|------------------------------|------:|----:|-------:|
| 1 | **`z_main_stack`** — main-thread stack (`CONFIG_MAIN_STACK_SIZE=327680`) | 327,680 | 320.0 | **74.9 %** |
| 2 | `.iram0.text` — Zephyr/ESP code that must run from RAM (flash/SPI, RTC, PMU, I²C, GPIO, fault handler) | 38,428 | 37.5 | 8.8 % |
| 3 | `.dram0.bss` — static globals (see split below) | 21,152 | 20.7 | 4.8 % |
| 4 | `_ring_buffer_data_g_rb` — flightlog RAM ring buffer (`RB_SIZE=8192`, in `.noinit`) | 8,192 | 8.0 | 1.9 % |
| 5 | `.dram0.data` — initialized statics | 6,080 | 5.9 | 1.4 % |
| 6 | `kheap__system_heap` — Zephyr system heap | 4,096 | 4.0 | 0.9 % |
| 7 | `tof_stack` — down-ToF thread stack | 4,096 | 4.0 | 0.9 % |
| 8 | `g_side_stack` — side-ToF (bumper) thread stack | 4,096 | 4.0 | 0.9 % |
| 9 | `z_interrupt_stacks` — ISR stack (`CONFIG_ISR_STACK_SIZE=2048`) | 2,048 | 2.0 | 0.5 % |
| 10 | `g_logger_stack` — flightlog writer thread stack | 2,048 | 2.0 | 0.5 % |
| 11 | `flow_stack` — optical-flow thread stack | 2,048 | 2.0 | 0.5 % |
| 12 | `.loader.text` — 2nd-stage loader shim | 1,840 | 1.8 | 0.4 % |
| 13 | `z_idle_stacks` — idle thread stack | 512 | 0.5 | 0.1 % |
| 14 | misc kernel (`sw_isr_table` 256, `device_states` 32, `k_heap/mutex/sem_area` ~116, `.loader.data` 592, `.iram0.bss` 16) | ~1,012 | 1.0 | 0.2 % |
| | **Total** | **~423,332** | **413.4** | **96.8 %** |

**`.dram0.bss` (#3) split** — this is where the app/driver static globals live:

| Sub-item | Bytes | Note |
|----------|------:|------|
| `vl53l5x_data_0..3` (4 × 4,152) | 16,608 | **VL53L5CX side-ToF driver instance data (bumper)** — dominates bss |
| `vl53l1x_data_0` | 960 | down-ToF driver data (altitude sensor — needed) |
| `g_wbuf` (flightlog flush scratch) | ~1,000 | consumer-thread scratch |
| thread control blocks, `s_log_cache`, misc | ~2,584 | small |

### Component attribution (what the task asked to pin down)

- **TinyMPC / matlib static data + workspace: 0 B in the flight build.** The factory (`controller_factory.cpp`)
  `#if`-selects only the PID controller, `estimator_factory.cpp` selects only the complementary filter, and
  Zephyr links with `-ffunction-sections -fdata-sections --gc-sections`. So with `ROSE_USE_PID=1` /
  `ROSE_USE_EKF=0`, **the entire TinyMPC + ADMM + matlib and the whole EKF are garbage-collected** — verified:
  `nm` finds **zero** `tiny_solve` / `admm` / `TinympcController` / `EkfEstimator` symbols in `build_fly`.
  For reference, when TinyMPC *is* selected it costs **≈16.9 KB** of static SRAM (measured: `build_fc_mpc` vs
  `build_fc_pid` → data +2,456 B, bss +14,464 B). **This "compile-out" win is therefore already banked** in the
  flight build — it is *not* an available optimization, and it is not the reason we're at 96 %.
- **EKF vs complementary:** complementary is a 96-B object (`g_estimator`); the EKF is gc'd. No delta today.
- **PID vs TinyMPC controller:** PID selected; a handful of scalar floats. TinyMPC would re-add the 16.9 KB
  static *and* is the reason the 320 KB stack exists (below).
- **Thread stacks:** main 320 KB **+** tof 4 KB + side 4 KB + logger 2 KB + flow 2 KB + ISR 2 KB + idle 0.5 KB
  = **~334.5 KB, i.e. 79 % of everything** — utterly dominated by the main stack.
- **Zephyr subsystem heaps / net / flash / log:** `CONFIG_HEAP_MEM_POOL_SIZE=0`, yet a **4 KB** system heap is
  still present (`kheap__system_heap`). `CONFIG_LOG=n`, `CONFIG_NET_BUF=n`, `CONFIG_BT=n`, `CONFIG_WIFI=n` — so
  there is **no** net-buffer / log-buffer spend to trim today (WiFi/BLE will *add* to this later, not subtract).
- **App static globals (`.bss`/`.data`):** ~27.2 KB total, dominated by the **16.6 KB VL53L5CX bumper buffers**.

### Why the main stack is 320 KB

It is deliberate and documented, e.g. `prj.conf`:
> *"a large main stack for the TinyMPC solve (single-loop default runs it in main…)"* — `CONFIG_MAIN_STACK_SIZE=327680`

and `main.cpp:780` (the threaded variant's control-thread stack):
> `K_THREAD_STACK_DEFINE(ctrl_stack, 327680); /* TinyMPC solve working set (was the main stack) */`

So the 320 KB was provisioned for the TinyMPC ADMM solve. **But the flight build runs PID**, and TinyMPC is
gc'd out — the PID `compute()` + complementary `update()` + sensor reads use only small scalar locals (`float
R[9]`, `float u[4]`, a `sensor_value`, printk formatting). **The single biggest SRAM consumer is a working set
for code that isn't in the binary.**

---

## 2. Optimizations (ranked by KB freed)

| # | Optimization | Est. saving | Measured? | Tradeoff |
|--:|--------------|------------:|:---------:|----------|
| 1 | **Right-size `CONFIG_MAIN_STACK_SIZE` 327680 → 32768** (PID build) | **~294.9 KB** | ✅ exact | Valid *only* while `ROSE_USE_PID=1`. Restore/raise it if you rebuild TinyMPC (`ROSE_USE_PID=0`). |
| 2 | **Disable the VL53L5CX side-ToF driver** (`CONFIG_VL53L5CX=n` + drop its 4 DT nodes) | ~16.6 KB (+4 KB stack) | ✅ present-check | Loses the wall "bumper". **Note:** `ROSE_BUMPER=0` alone does *not* free this. |
| 3 | **Drop the flightlog** (`-DROSE_FLIGHTLOG` off) for a wireless build | ~11.2 KB | ✅ | No on-board black-box; the live link captures the same stream. Also drops the FLASH_MAP dependency. |
| 4 | **Right-size background thread stacks** (tof 4096, side 4096 → ~1536) | ~4–8 KB | reasoned | Needs a high-water check; each only does one `sensor_fetch` + scalars. Marginal after #1. |
| 5 | **Drop optical-flow app pieces** (`ROSE_FLOW=0`) | ~2.2 KB | ✅ | Flow is a core velocity sensor — **keep for flight**; only for a bench/telemetry-only build. |
| — | System heap (4 KB), IRAM (37.5 KB), `.data` (5.9 KB) | ~0 | — | Effectively fixed Zephyr/ESP overhead; not safely reclaimable. |

### #1 — Right-size the main stack (the whole ballgame)

**Empirically measured (scratch build E1 = `build_fly` flags + overlay `CONFIG_MAIN_STACK_SIZE=32768`):**

| | main stack | `.dram0.noinit` | sram0 used | % of 437,512 |
|---|---:|---:|---:|---:|
| `build_fly` (320 KB stack) | 327,680 | 354,816 | 423,332 | 96.8 % |
| E1 (32 KB stack) | 32,768 | 59,904 | **128,420** | **29.4 %** |
| **Δ** | −294,912 | −294,912 | **−294,912** | **−67.4 pts** |

Nothing else changed (iram 38,428, bss 21,152, data 6,080 all identical) — the delta is *exactly* the stack
shrink. **This one config line frees ~288 KB of usable headroom** (and the build links cleanly with
`CONFIG_STACK_SENTINEL=y` still on).

**How small is safe?** 32 KB is already luxurious for the PID path (deepest use is an I²C sensor read + a float
`printk`, a few KB). You could go to 16 KB. Before committing, lock it in with a runtime high-water reading:
add `CONFIG_INIT_STACKS=y` + `CONFIG_THREAD_ANALYZER=y` + `CONFIG_THREAD_ANALYZER_AUTO=y` and read the reported
`main` unused bytes (or run the `spike_riscv64` co-sim). Recommend **24–32 KB** given we now have RAM to spare.

**Caveat:** if TinyMPC is ever reselected (`ROSE_USE_PID=0`), it needs its working set again. Cleaner long-term
fix: TinyMPC's `TinyWorkspace`/`TinyCache` are *already file-scope statics* in `controller_tinympc.cpp`, so the
320 KB may be historical over-provisioning even for the MPC path — measure the MPC high-water and shrink both
the stack and, if needed, tie `CONFIG_MAIN_STACK_SIZE` to `ROSE_USE_PID` in `CMakeLists`.

### #2 — VL53L5CX side-ToF buffers (important nuance)

`vl53l5x_data_0..3` = **16,608 B**. **Confirmed by measurement:** building with `ROSE_BUMPER=0` does *not*
remove them — they are instance data of the `CONFIG_VL53L5CX` driver + the 4 devicetree sensor nodes, not the
app flag. To actually reclaim them, set `CONFIG_VL53L5CX=n` in `boards/esp32c6_devkitc_hpcore.conf` and
disable/remove the 4 side-ToF nodes in the board overlay. Setting `ROSE_BUMPER=0` still frees `g_side_stack`
(4 KB) via gc. Total ≈ **20.7 KB** if the bumper is dropped for a telemetry/flight build.

### #3 — Flightlog (clean app-flag removal)

Ring buffer 8,192 + `g_wbuf` ~1,000 + logger stack 2,048 + TCB ~176 ≈ **11.2 KB**, all removed by building
without `-DROSE_FLIGHTLOG`. Also removes the flash-map/`storage`-partition dependency. The wireless link is the
live capture path, so the on-board logger is redundant for a telemetry build.

---

## 3. Total realistically-freeable headroom

Starting point: `build_fly` uses **423.3 KB / 437.5 KB (96.8 %)**, only **~14 KB** free.

| Scenario | sram0 used | Free | % free | How |
|----------|-----------:|-----:|------:|-----|
| Current flight build | 423.3 KB | 14.2 KB | 3.2 % | — |
| **+ #1 stack → 32 KB (keep ALL features)** | **128.4 KB** | **309.1 KB** | **70.6 %** | measured (E1) |
| + #1 + drop bumper-driver + flightlog + flow | ~109.9 KB* | ~327.6 KB* | ~74.9 % | measured (E2), *VL53L5CX driver still on |
| + also `CONFIG_VL53L5CX=n` | ~93.3 KB | ~344.2 KB | ~78.7 % | E2 − 16.6 KB |

**We can realistically free ~290 KB with a single one-line change (`CONFIG_MAIN_STACK_SIZE` 327680 → 32768),
without dropping any flight feature, landing at ~30 % SRAM. Dropping the side-ToF driver + flightlog for a
dedicated telemetry build pushes that to ~330–345 KB free (~75–79 %).**

### WiFi vs BLE verdict

- A Zephyr **BLE** stack (host + controller) is on the order of ~30–60 KB RAM — fits with enormous margin.
- A full **WiFi** path on the ESP32-C6 (Wi-Fi driver + LwIP + net-buffer pool + WPA supplicant) is heavier,
  roughly ~80–150 KB RAM depending on socket/buffer tuning.

With **~290 KB freed by optimization #1 alone** — before touching any feature — **the heavy WiFi + UDP/TCP
telemetry stack is viable, not just BLE.** The remaining action item is to actually enable `CONFIG_WIFI` /
`CONFIG_NETWORKING` (or `CONFIG_BT`) and re-measure, since those *add* heap and net-buffer demand that this
analysis has now made room for. Right-size the main stack first, then dial WiFi buffer counts to taste.

---

## Appendix — reproduction

Env: `source .../esp32env.sh` (west + Zephyr SDK 0.17.2). Tools:
`riscv64-zephyr-elf-{size,nm}` from `~/zephyr-sdk-0.17.2/riscv64-zephyr-elf/bin`.

```
# Region size + section totals + biggest symbols (any build)
riscv64-zephyr-elf-size -A build_fly/zephyr/zephyr.elf
riscv64-zephyr-elf-nm --size-sort --print-size --radix=d build_fly/zephyr/zephyr.elf | tail -40
# noinit (stack/heap) placement:
awk '/^\.dram0\.noinit/{f=1} f{print} /^\.dram0\.bss/{if(f)exit}' build_fly/zephyr/zephyr.map

# E1 — right-sized stack (frees ~294.9 KB), leanstack.conf = "CONFIG_MAIN_STACK_SIZE=32768"
west build -b esp32c6_devkitc/esp32c6/hpcore -d build_e1 samples/rose_flight_controller -- \
  -DROSE_USE_PID=1 -DROSE_USE_EKF=0 -DROSE_BUMPER=1 -DROSE_FLOW=1 -DROSE_THREADED=0 \
  -DCTRL_ITERS=20000 -DEXTRA_CONF_FILE=leanstack.conf \
  -DEXTRA_CPPFLAGS="-DROSE_AUTOFLIGHT=1 -DROSE_FLIGHTLOG=1 ..."

# E2 — telemetry-lean (no bumper/flow/flightlog + 32K stack)
west build -b esp32c6_devkitc/esp32c6/hpcore -d build_e2 samples/rose_flight_controller -- \
  -DROSE_USE_PID=1 -DROSE_USE_EKF=0 -DROSE_BUMPER=0 -DROSE_FLOW=0 -DROSE_THREADED=0 \
  -DCTRL_ITERS=20000 -DEXTRA_CONF_FILE=leanstack.conf
```

Measured results: `build_fly` 423,332 B (96.8 %); E1 128,420 B (29.4 %); E2 109,864 B (25.1 %) of 437,512 B.
TinyMPC static cost (informational): `build_fc_mpc` vs `build_fc_pid` → +16,920 B (already gc'd from the PID
flight build).
