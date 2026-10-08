# DroNet navigation, 2026-09-28

`ROSE_DRONET_NAV=1` connects live camera inference to the existing flight
controller in `samples/rose_fc_dronet`. This describes the `integration-dronet`
working tree over `35bf7c9`. The default remains the static-input co-residency
benchmark. **The committed grayscale model has seeded random weights; its
commands are not meaningful navigation.** No hardware qualification was performed.

## Data flow and controller frame

1. `camera_dma_grab_gray()` posts a generation-tagged request to the camera
   thread. A timeout cancels that generation; an old reply cannot satisfy a new
   request. The API supports one caller at a time. The thread refuses a grab on
   a busy DMA engine because `DMA_CLEAR` only clears status; it does not abort.
   It pre-sleeps toward a frame edge, then locks the scheduler while catching
   the edge and arming capture. Interrupts remain enabled; the grab's edge spin
   is bounded by `CAM_GRAB_EDGE_SPIN_US`. The capture timestamp is taken at arm,
   before unlocking the scheduler.
2. A grab is valid only if the edge was caught, the FIFO fill check passed
   (`FIFOCOUNT >= CAP_PIXELS - 4096`), DMA reports `DONE` without `ERROR`, and
   `DMA_BYTES == CAP_PIXELS` (104,000 bytes). The 324 × 320 source region is box
   averaged to 112 × 112 grayscale, extending odd box spans where possible to
   suppress period-two row/column noise. Failed grabs leave the destination
   untouched. The thread converts pixels to int8 with
   `clamp(pixel - DRONET_IN_ZERO, -128, 127)`.
3. The DroNet thread first runs an IRQ-locked golden baseline and classifies
   operations for masking. A baseline error above `DRONET_VERIFY_ATOL` parks
   the thread before any publication. By default, each live frame runs twice
   under the steady-state preemption/masking policy. Both outputs must agree
   within the tolerance. Steering comes from the first run; collision uses a
   stop code if either run reports one, otherwise the larger collision output.
   Every `DRONET_VERIFY_EVERY` frames, a golden inference runs **after** the live
   command is published. A dual-run or golden-check failure invalidates the
   mailbox and parks the thread indefinitely. Capture errors instead retry
   after 50 ms, leaving the previous command to age out.
4. `dronet_cmd.c` uses a spinlock around a small mailbox copy: sequence, raw
   steering/collision outputs, capture timestamp, and validity. Invalidation
   makes `dronet_cmd_latest()` return false immediately. `dronet_nav_tick()`
   reads it once per control iteration, before `solve_control()`.
5. `fc_dronet_nav_step()` dequantizes the outputs and forms
   `v_target = (1 - collision) * DRONET_V_MAX` and
   `w_target = DRONET_YAW_SIGN * clamp(steer, -1, 1) * DRONET_YAW_RATE_MAX`.
   Time-based first-order smoothing uses `1 - exp(-dt/tau)`. The smoothed yaw
   rate integrates into a bounded yaw reference. The mapping is
   `g_setpoint[5] = tan(psi_ref/2)` (Gibbs yaw), `[6] = v` (forward m/s),
   and `[7] = 0` (lateral velocity).
6. `solve_control()` obtains the **current** heading with `fc_yaw_from_gibbs()`
   and calls `fc_state_to_yaw_frame()`. This rotates horizontal position and
   velocity and removes that yaw from the Gibbs attitude; body rates stay
   unchanged. The copied yaw target becomes `tan(wrap(psi_ref - yaw)/2)`.
   The PID receives this transformed state and relative yaw target. Envelope
   checks, telemetry and logs retain the estimator's world-frame state.

The estimator supplies world-frame velocity and Gibbs roll/pitch components
that mix with yaw, while the PID was tuned at heading zero. For example, pure
pitch 0.2 rad at yaw 1 rad produces a raw Gibbs roll component near -0.055.
Transforming the full attitude as well as velocity restores the heading-zero
control problem. Using current heading keeps the forward request, wall bumper
and velocity integral aligned with body directions even while yaw lags its
reference. The integral consequently acts as body-fixed trim.

Compile-time guards reject navigation without the `rose_fc_dronet` thread
(`ROSE_HAVE_DRONET`), `ROSE_FLOW`, `ROSE_CAMERA_DMA`, or the heading frame; with
`ROSE_VEL_LOOP=0`; without autoflight unless motors are inhibited; or with live
motors unless `DRONET_MODEL_VALIDATED` is enabled. Heading-frame builds also
reject `ROSE_POS_LOOP` and `ROSE_USE_ROCC`: the former mixes world-position
control with heading-frame velocity, and the latter ignores the supplied
state argument in favor of its own accelerator state copy.

## Engagement and failure handling

Autoflight permits navigation only during the hover segment
`climb_ms <= flight_time < climb_ms + hover_ms`, above
`DRONET_NAV_MIN_Z_M`, with no abort landing or estop. The last flow sample
accepted as valid must be within `DRONET_FLOW_MAX_GAP_MS`. The age comes from
`flow_sample_ms()`, not the time the control loop read the cached sample.
A newly invalid flow read does not refresh this timestamp; the remaining gap
budget can still elapse before disengagement. Non-autoflight bench builds
require inhibited motors and omit the hover/height gate, but retain flow and
estop gating.

Leaving engagement zeros smoothed forward velocity and yaw rate on that tick
and holds the yaw reference. Within engagement, a missing, invalidated, or
stale command instead decays both rates through the smoothing. Freshness
requires a capture age from zero through `DRONET_CMD_TIMEOUT_MS`, inclusive,
so the budget covers capture, inference and hand-off. The yaw reference can
advance through the decaying yaw-rate tail, then holds.

Collision output 127 or any negative value decodes to probability 1 and a
zero **forward target**. Forward speed still smooths toward zero; steering
can still request yaw. Disarm and soft `RESET` reset navigation velocity,
yaw rate and yaw reference to zero. They do not restart a parked inference
thread. `DRONET_NAV` status defaults to once per second only when motors are
inhibited and `ROSE_TELEM_CONSOLE` is enabled; the DroNet thread's own
`DRONET` reports have a separate, unconditional cadence.

## Navigation and capture knobs

Pass CMake options after `--`; pass C/C++ macros through `EXTRA_CPPFLAGS`.
Defaults below are from the source. Parameter fields in `fc_dronet_nav.h`
have no separate defaults: `main.cpp` fills them from the corresponding macros.

| Knob / parameter | Default | Meaning |
| --- | --- | --- |
| CMake `ROSE_DRONET_NAV` | `0` | Live navigation when 1; static-input benchmark when 0. |
| `ROSE_VEL_HEADING_FRAME` | `ROSE_DRONET_NAV` | Transform controller inputs into current heading. |
| `DRONET_MODEL_VALIDATED` | Unset | Must be enabled for navigation with live motors; declaration for a trained, validated model. |
| `DRONET_V_MAX` / `v_max` | `0.30` m/s | Forward target at collision probability zero. |
| `DRONET_YAW_RATE_MAX` / `yaw_rate_max` | `0.50` rad/s | Yaw rate at steering magnitude 1. |
| `DRONET_YAW_SIGN` / `yaw_sign` | `1.0` | Positive steering turns nose-left (+z); use -1 for right-positive output. |
| `DRONET_YAW_MAX` / `yaw_max` | `1.00` rad | Absolute yaw-reference bound, away from the Gibbs singularity. |
| `DRONET_TAU_V` / `tau_v` | `0.50` s | Forward-speed smoothing; nonpositive bypasses smoothing. |
| `DRONET_TAU_W` / `tau_w` | `0.30` s | Yaw-rate smoothing; nonpositive bypasses smoothing. |
| `DRONET_CMD_TIMEOUT_MS` / `timeout_ms` | `600` ms | Maximum fresh command age from capture. |
| `DRONET_FLOW_MAX_GAP_MS` | `200` ms | Maximum age of the last flow sample accepted as valid. |
| `DRONET_STEER_SCALE` / `steer_scale` | `0.0034265661` | Steering output dequantization, zero point 0. |
| `DRONET_COLL_SCALE` / `coll_scale` | `0.0031727139` | Collision output dequantization, zero point 0; stop codes override it. |
| `DRONET_NAV_MIN_Z_M` | `0.20` m | Autoflight engagement requires estimated height strictly above this. |
| `DRONET_NAV_PRINT_MS` | `1000` if motors inhibited and console telemetry enabled; otherwise `0` | Control-loop status interval; 0 disables. A line costs about 9 ms of polled console output per source comment. |
| `DRONET_PRIO` | `7` | DroNet thread priority, below control-loop priority 0. |
| `DRONET_STACK_SIZE` | `256 * 1024` bytes | DroNet stack allocation. |
| `DRONET_START_DELAY_MS` | `4000` ms | Delay before the inference thread starts. |
| `DRONET_REPORT_EVERY` | `1` | Print every N live frames (or benchmark iterations); requires N > 0. Live compute/wall fields measure the first inference, excluding the repeat and golden check. |
| `DRONET_IRQ_GATE` | `0` | 1 IRQ-masks every operation and skips explicit boundary yields; correctness fallback with long control stalls. |
| `DRONET_YIELD_DIV` | `1` | Yield every N operations when IRQ gate is off; requires N > 0. |
| `DRONET_MASK_OPS` | `"batchnorm", "add_s8", "relu", "linear", "sigmoid", "maxpool"` | IR-kind substrings selecting whole-operation IRQ masking. |
| `DRONET_VERIFY_ATOL` | `3` | Maximum absolute int8 difference accepted by the nav baseline, dual-run and periodic golden checks. |
| `DRONET_IN_ZERO` | `128` | Pixel centering before int8 clamping; placeholder preprocessing. |
| `DRONET_GRAB_TIMEOUT_MS` | `500` ms | Wait budget passed to `camera_dma_grab_gray()`. |
| `DRONET_VERIFY_EVERY` | `20` | Golden check every N live frames; 0 means boot baseline only. |
| `DRONET_DUAL_RUN` | `1` | Run each live frame twice and compare/merge outputs. |
| `CAM_GRAB_EDGE_SPIN_US` | `5000` µs | Time bound on the scheduler-locked edge spin while servicing a grab. |
| `ROSE_CAM_AUTOSNAP` | `0` | 0 disables periodic UART snapshots; positive N selects every Nth camera-loop iteration. |
| `ROSE_CAM_AUTOSNAP_W`, `ROSE_CAM_AUTOSNAP_H` | `128`, `126` | Automatic snapshot dimensions; live DroNet input remains 112 × 112. |

The camera also fixes `CAP_PIXELS=104000`, `MCLK_DIV=1`, `SRC_W=324`, and
`SRC_H=320`; `DRONET_IN_W=112`, `DRONET_IN_H=112` and `CAMERA_DMA_GRAB_MAX=12544`
are fixed size constants, not override knobs. Input size and grab capacity have build
assertions. Camera thread stack/priority are fixed at 8192 bytes/12, with
priority 5 during initialization.

Related build switches used by the path and guards:

| Switch | Default | Meaning |
| --- | --- | --- |
| CMake `ROSE_FLOW` | `0` | Enable PMW3901 optical flow; navigation requires 1. |
| CMake `ROSE_CAMERA_DMA` | Empty; source defaults to `0` | Enable OSPI DMA capture; navigation requires 1. Empty preserves a macro override. |
| CMake `ROSE_UART_TELEM` | Empty; source defaults to `0` | Enable raw uart1 telemetry/command transport. |
| CMake `ROSE_HAVE_DRONET` definition | `1` in this sample | Marks availability of the inference thread. |
| `ROSE_AUTOFLIGHT` | Unset/off | Enable the flight profile and hover-only navigation gate. |
| `ROSE_MOTORS_INHIBIT` | `0` | 1 inhibits motor actuation and permits non-autoflight navigation bench builds. |
| `ROSE_TELEM_CONSOLE` | `1` | Also controls the default navigation status-print interval. |
| `ROSE_VEL_LOOP` | `1` in PID | Horizontal velocity regulation; required for navigation. |
| `ROSE_POS_LOOP` | `0` in PID | Position outer loop; incompatible with heading frame. |
| CMake `ROSE_USE_ROCC` | `0` | Accelerator controller; incompatible with heading frame. |
| CMake `ROSE_USE_FP16` | `0` | Select the FP16/vector estimator and PID variant. |

## Builds and verification

These commands were run with Zephyr SDK 1.0.0-beta1. Run from the backend
repository root with the west environment initialized. Replace
`<riskybird-dev>` with its absolute checkout path; use absolute sample/model
paths if invoking west from `zephyr_ws`.

Motors-inhibited bench:

```sh
west build -p always -b chipyard_riscv64 samples/rose_fc_dronet \
  --build-dir build_fc_dronet_nav -- \
  -DMB_REPO="$PWD/modelblaster" \
  -DMODEL_DIR="<riskybird-dev>/artifacts/modelblaster/dronet/int8/gemmini_q31_rvv_softfp-gray-nhwc" \
  -DEXTRA_CPPFLAGS="-DROSE_MOTORS_INHIBIT=1 -DROSE_PROFILE=1 -DROSE_TELEM=0 -DDRONET_IRQ_GATE=0" \
  -DROSE_DRONET_NAV=1 -DROSE_FLOW=1 -DROSE_CAMERA_DMA=1
```

Autoflight build form, for use only with a trained model:

```sh
west build -p always -b chipyard_riscv64 samples/rose_fc_dronet \
  --build-dir build_fc_dronet_nav -- \
  -DMB_REPO="$PWD/modelblaster" \
  -DMODEL_DIR="<riskybird-dev>/artifacts/modelblaster/dronet/int8/gemmini_q31_rvv_softfp-gray-nhwc" \
  -DEXTRA_CPPFLAGS="-DROSE_AUTOFLIGHT=1 -DDRONET_MODEL_VALIDATED=1 -DROSE_PROFILE=1 -DROSE_TELEM=0 -DDRONET_IRQ_GATE=0" \
  -DROSE_DRONET_NAV=1 -DROSE_FLOW=1 -DROSE_CAMERA_DMA=1
```

Without `DRONET_MODEL_VALIDATED=1`, the live-motor navigation build fails by
design. Setting the macro does not validate or train the model. The autoflight
compilation check with the existing fixture establishes build compatibility
only; flight requires replacing that fixture with a trained grayscale model.

Host verification:

```sh
python3 -B -m unittest discover -s tests -p 'test_*.py' -v
```

Result: **Ran 6 tests, OK**. The four `test_dronet_nav.py` tests cover yaw and
frame-transform math including pitch 0.2 rad at yaw 1 rad; setpoint mapping,
stop codes, immediate disengagement, stale decay, yaw bounds and the hover
window; PID command invariance across headings with tilt, evolving integrators
and yaw lag; and 15 build-mode guard cases. The two existing
`test_fc_merge.py` tests also pass.

Recorded firmware checks:

| Build | Result / RAM |
| --- | --- |
| Navigation bench, motors inhibited | Passed; 2,151,728 B |
| Navigation autoflight + `DRONET_MODEL_VALIDATED=1` | Passed; 2,154,800 B |
| Navigation autoflight + validated flag + `ROSE_USE_FP16=1` | Passed; 2,149,888 B |
| Navigation bench + `ROSE_UART_TELEM=1` | Passed; 2,154,928 B |
| Navigation autoflight without validated flag | Rejected as intended |
| Navigation + `ROSE_USE_ROCC` | Rejected as intended |
| Navigation off, FC+DroNet command from the integration document | Passed; 1,951,616 B; objcopy loadable image byte-identical to pristine `35bf7c9` (568,520 B) |
| Existing `rb` T21 preset | Passed; 435,872 B; image byte-identical to pristine `35bf7c9` (96,072 B) |
| Existing `rb` photo-bench | Passed; 522,080 B |

Two independent adversarial code reviews were run and their confirmed findings
fixed. Nothing was run on hardware: no FPGA, camera, inference, motors or flight.

## Known limitations and next hardware steps

- **Model:** `modelblaster/models/dronet.py` never loads the checkpoint for
  non-three-channel variants; the committed grayscale DroNet uses seeded
  **RANDOM weights**. The RGB checkpoint trains only steering, with a random
  collision head. Meaningful navigation requires a trained grayscale model,
  followed by updates to `DRONET_STEER_SCALE`, `DRONET_COLL_SCALE` and
  `DRONET_IN_ZERO`. The `ir_gray_nhwc/graph.json` generated by riskybird-dev's
  `tools/modelblaster/dronet_gray_nhwc.py` has scales `x=0.03294586`,
  `linear1=0.0034265661`, `sigmoid1=0.0031727139`, all with zero point 0.
  The largest nonsaturated nonnegative collision output is
  `126 * 0.0031727139 = 0.399762`, about 0.400. Within the valid nonnegative
  output range, only saturation (127) requests a forward stop; negative values
  also trigger the defensive stop decoding.
- **PID units and trim:** Roll/pitch still compare radian targets against
  Gibbs estimates, approximately half physical tilt at small angles. This
  unchanged mismatch can demand about twice the intended physical tilt and
  matters when choosing `DRONET_V_MAX`. The velocity integral lives in the
  heading frame and trims body-fixed offsets; a world-fixed disturbance such
  as wind must be re-learned during turns. The envelope guard's Gibbs tilt
  conversion overestimates tilt at nonzero yaw, conservatively.
- **Corruption checks:** Dual-run and golden checks detect corruption
  probabilistically. Corruption repeated identically in both live runs is not
  caught by their comparison; discrepancies within the tolerance also pass.
  Dual run halves the inference-limited command rate, before capture and
  periodic-check overhead. A live command can be consumed before its following
  golden check fails and withdraws it.
- **Timing:** The scheduler-locked edge spin (about 3 ms expected, bounded by
  `CAM_GRAB_EDGE_SPIN_US`) stalls the control loop. The existing whole-operation
  maxpool IRQ mask is documented in `dronet_thread.c` as about 25 ms per
  inference, so repeating inference repeats that stall. The boot baseline is
  also IRQ-locked. No 200 Hz deadline guarantee has been measured for this path.
- **DMA recovery:** A transfer that times out stays active: the OSPI RTL has
  no abort, and `DMA_CLEAR` clears only done/error flags. Grabs fail while the
  engine remains busy, until it drains.
- **Command uplink:** The `rose_fc_dronet` overlay has no `esp-uart` alias, so
  the uart1 ESTOP/command uplink is absent unless `ROSE_UART_TELEM=1`. That
  enables the raw parser, which has no `PROFILE` command. `telem_uart.c` now
  derives the rounded divisor from the SoC peripheral clock: 35 MHz gives 37;
  50 MHz gives 53. This supersedes the integration document's earlier 50 MHz
  hardcoding note; physical baud and command delivery still need checking.

Next, perform the [integration hardware checks](INTEGRATION_20260928.md#hardware-and-remaining-limits),
then a disarmed bench run logging `DRONET` and `DRONET_NAV` lines, followed by
tethered fixed-forward-speed flight. Put DroNet in the flight loop only after
a trained model exists.
