# riskybird v3 — Flight Build Recipe (autoflight + flow + flightlog)

The exact build used for the untethered autoflight hops. **Replicates the last flight test**:
lift‑and‑place autoflight, optical flow in the loop, on‑board flight log (with horizontal
velocity + raw flow), the two motor chirps, and motors active. Reproduce with
`tools/build_flight.sh` or the command below.

## Build command
```bash
source /home/cobble/.claude/projects/-home-cobble-Tools-riskybird/esp32env.sh
cd /home/cobble/Tools/zephyr-chipyard-sw
west build -b esp32c6_devkitc/esp32c6/hpcore samples/rose_flight_controller -d build_fly -- \
  -DROSE_USE_PID=1 -DROSE_USE_EKF=1 -DROSE_FLOW=1 -DROSE_BUMPER=1 -DROSE_BUMPER_READDRESS_ONLY=1 \
  -DROSE_ACTUATE_TIMEOUT_MS=0 -DCTRL_ITERS=20000 \
  -DEXTRA_CPPFLAGS="-DROSE_AUTOFLIGHT=1 -DROSE_FLIGHTLOG=1 -DVL53L1X_TIMING_BUDGET_US=100000 \
     -DHOVER_Z_M=0.20f -DT_CLIMB_MS=1500 -DT_HOVER_MS=1500 -DT_DESCEND_MS=1500 \
     -DAUTOFLIGHT_MAX_DUTY=0.8f -DPID_MASS_KG=0.060f -DSAFE_MAX_VEL_MPS=1000.0f"
```
ELF: `build_fly/zephyr/zephyr.elf`. (SRAM 96.8% — the 320 KB main stack is still in; that is a
TinyMPC leftover, fine for flight, only matters for the wireless build — see `FC_RAM_ANALYSIS.md`.)

## Parameters

### CMake knobs (`-D…`, these ARE in CMakeLists)
| Knob | Value | Meaning |
|---|---|---|
| `ROSE_USE_PID` | 1 | hierarchical PID cascade (not TinyMPC) |
| `ROSE_FLOW` | 1 | PMW3901 optical flow → estimator horizontal velocity |
| `ROSE_BUMPER` | 1 | side‑ToF bring‑up (readdress 0x31‑0x34). **Required to boot the fully‑populated board** (else the 4 sides collide at 0x29 → `i2c_hw_fsm_reset` wedge). Adds ~12 s boot. |
| `ROSE_BUMPER_READDRESS_ONLY` | 1 | **fast boot** — park the 4 sides off 0x29 (bus stays clear for the down VL53L1X) but skip the ~84 KB-per-sensor ULD firmware upload + ranging → **~0.5 s vs ~12 s**. No wall telemetry (FC doesn't use it); drop this flag for live walls. |
| `ROSE_ACTUATE_TIMEOUT_MS` | 0 | no bench actuation cutoff (autoflight manages its own timing) |
| `CTRL_ITERS` | 20000 | control‑loop iteration cap |

### EXTRA_CPPFLAGS defines (⚠ these are NOT CMake knobs)
> **CRITICAL:** `ROSE_AUTOFLIGHT` and `ROSE_FLIGHTLOG` are **not** in `target_compile_definitions`.
> Passing them as bare `-DROSE_AUTOFLIGHT=1` is **silently ignored** → autoflight is compiled out →
> the build falls through to the always‑armed bench path (motors at 10%, no chirp). They **must** go
> inside `EXTRA_CPPFLAGS`. (This exact mistake cost us the "props spin at boot, no chirp" debug.)

| Define | Value | Meaning |
|---|---|---|
| `ROSE_AUTOFLIGHT` | 1 | arm‑and‑fly: lift‑and‑place gesture → altitude profile → land → disarm; enables **both chirps** |
| `ROSE_FLIGHTLOG` | 1 | on‑board flash log @ ~50 Hz (t, roll/pitch/yaw, z, vz, **vx, vy, fvx, fvy**, duty[4], flags) |
| `VL53L1X_TIMING_BUDGET_US` | 100000 | down‑ToF 100 ms ranging budget (weak/dark‑floor reliability) |
| `HOVER_Z_M` | 0.20f | hover altitude = 20 cm |
| `T_CLIMB_MS` / `T_HOVER_MS` / `T_DESCEND_MS` | 1500 each | profile: 1.5 s climb → 1.5 s hover → 1.5 s descend (~4.5 s hop) |
| `AUTOFLIGHT_MAX_DUTY` | 0.8f | per‑motor duty ceiling in flight (safety cap; hover needs ~65%) |
| `PID_MASS_KG` | 0.060f | mass feedforward (real v3 ≈ 60 g); the altitude PD has no integrator so this must match weight |
| `SAFE_MAX_VEL_MPS` | 1000.0f | **velocity watchdog DISABLED** (intentionally off to characterize flow without premature cuts) |

## Source features baked into this build (not build flags)
- **Two motor chirps** (main.cpp): `motors_boot_chirp()` (1‑2‑3‑4 sweep, fires early = "board reset")
  and `motors_ready_chirp()` (double all‑together blip, fires after sensor bring‑up, right
  before the arm gate = "ready — do the lift‑and‑place"). No ready chirp ⇒ the IMU didn't come up ⇒
  power‑cycle and retry.
- **Watchdog is FLIGHT‑only** (`if (g_armed && !g_estop)`): pre‑arm hand‑handling (the lift‑and‑place)
  can't latch estop. Once armed: tilt > 1.0 rad (57°) / rate > 10 rad/s guard the flight (velocity
  guard is off via `SAFE_MAX_VEL_MPS=1000`).
- **Flightlog record** carries horizontal velocity + raw flow input (flightlog.{h,c}) for drift/flow
  diagnosis; dump parser is `tools/flightlog_dump.py`.

## Flight + log procedure
1. Flash via USB, then power the board from a **freshly‑charged 1S LiPo** (a POR boots the app; the
   RTS/esptool reset lands `d5:00` in ROM download — use battery/power‑cycle).
2. Boot (~2-3 s with readdress-only; ~15 s with the full bumper) → **boot chirp** (reset), then the
   **ready chirp** shortly after = arm gate live.
3. **Lift‑and‑place** after the ready chirp: lift >15 cm over open floor (nothing under the down‑ToF),
   set level on the ground, hold still ~1.5 s → arms → climb/hover/descend/land → disarm (one‑shot).
4. Dump the log: reflash a dump build and power‑cycle while capturing:
   ```bash
   west build -b esp32c6_devkitc/esp32c6/hpcore samples/rose_flight_controller -d build_dump -- \
     -DROSE_USE_PID=1 -DEXTRA_CPPFLAGS="-DROSE_FLIGHTLOG_DUMP=1"
   west flash -d build_dump   # then power-cycle; it prints FLIGHTLOG_CSV_BEGIN…END once at boot
   ```
   (The dump build reads + halts *before* any erase, so the log survives the reflash.)

## Known caveat — why the last hop "flew too high"
The log showed the **down‑ToF reading 0 mm as soon as the props spun** (vibration / occlusion / the
VL53L1X driver returning a bad `RangeStatus` as 0) → the altitude loop saw "on ground" → commanded
**max thrust** → it climbed away; and with `height < 0.02 m` the flow gated OFF, so the estimator
velocity dead‑reckoned. **This is unfixed** — replicating this build will likely recur unless the
down‑ToF is made robust (gate on `RangeStatus` + hold last‑good) or the physical vibration/occlusion
is addressed. Watch the down‑ToF; keep clear space.
