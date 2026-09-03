# riskybird v3 — flight tuning experiment log

Chronological record of flight-test experiments: what changed, why, and what happened. Newest
entries at the bottom. Append one row + a note per build we fly.

**Platform:** FPGA-laden v3 (~58 g, full-prop 7×20 + HQ ultralight). **Thrust-marginal** — ~71 %
hover duty, raw command saturating 42–65 % of the flight (can't shed weight). Estimator =
complementary (PID + optical flow + ToF/baro alt). Watchdog live (57° tilt / 2 m/s / 0.6 m).

**Reading the logs** (`tools/groundstation/logs/riskybird_<ts>.csv`, one per panel session):
- Frame: **`+vy` / `+roll` = LEFT** (matches the controller's `ROLL_TRIM` comment + the observed drift).
- `u[]` is the controller COMMAND; actual PWM duty = `(u+0.583)·batt_scale`, then a collective
  anti-sat cut at `AUTOFLIGHT_MAX_DUTY=0.95`.
- Attitude `roll/pitch/yaw` in newer logs are proper Euler (+ quaternion); older logs stored the
  raw Gibbs vector (reconstruct with `2·atan`).
- `peak|roll|` ≈ 175° in almost every flight so far = the **crash tumble at the end**, not in-flight.

## Summary table

| # | time | key knobs (Δ) | rationale | result | logs |
|---|------|---------------|-----------|--------|------|
| 1 | 19:14–19:29 | baseline: `IMU_OFFSET_Y=0`, `ATT_GAIN=1.0`, `tau_rollRate=0.025` | starting point | **consistent left drift ~1.8 m/s**, hover 71 % duty saturating, ends in a tumble/crash. Attitude looks fine in flight (flip is the impact). | 191424, 192704, 192937 |
| 2 | 20:00 | `IMU_OFFSET_Y=-0.016` (lever-arm, **wrong sign**) | cancel the off-CoM IMU's rotational accel | **worse** — aggressive left, steady tilt. (This was the sign check.) | 200051 |
| 3 | 20:07 | `IMU_OFFSET_Y=+0.016` (correct sign; silkscreen-confirmed) | flip the sign | rotation-drift **~4× better** (best hover vy 1.8→0.44 m/s, one clean 7 s hover), but **inconsistent** + a residual steady tilt | 200717 |
| 4 | ~20:2x | `ATT_GAIN 1.0→0.7`, `tau_rollRate 0.025→0.04` (softer) | I mis-read the residual as oscillation | **immediate flips** — proved the attitude loop was UNDER-powered, not oscillating | — |
| 5 | ~20:3x | `ATT_GAIN 0.7→1.5`, `tau_rollRate 0.04→0.02` (stronger) | reverse course: give the loop authority | **"significantly more stable"** (user) — closed the steady tilt | — |
| 6 | ~20:4x | `ATT_GAIN 1.5→2.0`, `tau_rollRate 0.02→0.015` (limit test) | push for the stability edge | **most stable initially**, but **oscillates after ~2 s** — found the limit | — |
| 7 | ~20:5x | `ATT_GAIN=2.0`, `tau_rollRate 0.015→0.02` | keep the authority (2.0 closed the steady error), soften just the inner loop to kill the oscillation | **"much more stable flight"** (user) — the oscillation was the inner loop; 2.0/0.02 = keeper | — |
| 8 | ~21:0x | profile: `hover 2→6 s`, `descend 1.5→4 s`, `cap 13.5 s`; **profile now runtime-tunable** (panel + `PROFILE` uplink cmd) | longer hover + slow gentle landing, tunable without reflashing | profile OK (full 13.5 s every flight, RESET doesn't touch it) — **but 3/4 flights the watchdog cut motors mid-descent**: the **baro fusion (`ROSE_BARO=1`, first flight test)** drifted `z` to ~1.2 m (or −0.37 m) while the ToF read 0.08 m *valid* → >0.6 m → estop. The fusion's ToF-reject gate (>0.15 m from baro) locks out the correct ToF → runaway. = the "cutting off" + "spooked landing". | 204716 |
| 9 | ~21:1x | **`ROSE_BARO=0`** (ToF-only altitude, the proven config) | baro fusion not flight-ready (gate runaway near the floor); revert while keeping gains/lever-arm/profile | ToF-only altitude tracks correctly, but the "**first flight good, rest short**" pattern remained (see exp 11) | 205538, 210309 |
| 10 | ~21:2x | **boot-safe arming**: won't auto-arm on power-up; `RESET` cmd enables arming (+ slow-blink "locked" LED) | safety — no takeoff on a stray power-cycle | works (`disarmed on boot -- send RESET` confirmed) | — |
| 11 | ~21:3x | **integrator-reset fix**: moved `alt_int`/`vel_int`/`pos_ref`/tilt-slew from `compute()` statics to controller MEMBER vars, cleared in `ctrl.init()` (which RESET calls) | flights 2+ overshot altitude + tilt grew because integrals carried over between flights | **helped the tilt** (stopped compounding: −2.3/+2.0/+1.9°) but the altitude overshoot + degradation remained → see exp 12 | 211427 |
| 12 | ~21:4x | **altitude overshoot fix**: `T_CLIMB 1.5→3 s` (less climb momentum), `SAFE_MAX_HEIGHT 0.6→0.8 m` (margin), `FLIGHT_MAX 15 s` | Kept as a good change (gentler climb + ceiling margin → no hard height-watchdog crashes at the top of the profile). **BUT the "repeated crashes beat up the drone" diagnosis was WRONG** — user: the drone isn't physically degrading, *the pattern repeats fresh after every re-flash*, which rules out accumulated hardware damage and points to **software state a chip-reset clears but a soft-RESET doesn't.** So the overshoot fix helps the ceiling but is NOT the root of "successively less stable" → see exp 13. | retained | — |
| 13 | ~21:5x | **reuse the boot gyro cal across soft-RESETs** (`ROSE_RECAL_ON_RESET=0`): the soft-RESET no longer calls `gyro_cal_restart()`; every flight keeps the pristine bias measured during the long, untouched boot bringup | **found the accumulator.** Per-flight attitude is not a steady drift but a **~0.5 Hz velocity-loop oscillation whose amplitude grows flight-over-flight** (log 211427 roll swing: flight 1 ±11° → f2 ±12° → f3 ±14°, diverging into the end crash). Ruled out: `batt_scale` (it was *higher* on the good flight 1, 1.107 vs 1.006), all PID integrators + estimator + Mahony (reset by `est/ctrl.init`), flow LP (`TAU=0`, pass-through). The one thing that differs between flight 1 and every later flight = the **gyro cal**: flight 1 uses the clean boot cal; each soft-RESET re-cal'd in a rushed, just-landed/hand-held window → a contaminated rest bias = a residual rate error = phase error in the attitude estimate → erodes the loop's margin → the oscillation grows. Reusing the boot cal makes every flight start bit-identical to flight 1. | **helped ("not as bad")** but not fixed — killed the *contamination* but froze the bias, which then goes stale as the IMU warms → residual persists → see exp 14 | 214029 |
| 14 | ~22:0x | **continuous ground gyro-bias re-tracker** (`GBIAS_TRACK_GAIN=0.0008`, `STILL=0.10 rad/s`): while DISARMED + very still, slow-EMA `g_gyro_bias` toward the live rate; frozen while armed | **the real root, proven by elimination.** New datum: the degradation *persists across a battery unplug* and is cleared *only by a re-flash*. But the flightlog (only flash writer) lives at `storage_partition` 0x3b0000, which `west flash` (erases 0x0–0xbdfff) never touches and which isn't wiped at boot — so it persists across BOTH power-cycle and re-flash, and there's no NVS/settings anywhere ⇒ **no non-volatile state a re-flash clears but a power-cycle doesn't.** The only consistent explanation: the battery unplug isn't resetting the ESP (USB keeps it powered), so it's **ESP-RAM state** — the frozen `g_gyro_bias`, which a chip reset (RTS on flash) re-measures but a soft-RESET/USB-alive-unplug does not. Its stale residual is injected into the rate loop, Mahony, AND the flow gyro-comp → drift grows flight-over-flight; via translation over the floor the altitude wobble grows too. The tracker re-measures the bias to the current temp every time the drone parks → every flight arms fresh, no chip reset needed. | **partial** — still degrades (215805/221242: f1 clean, f2/f3 tilt-std + z-std grow ~2×). vy-std & tilt-std grow in lockstep, meanVy≈0 → growing oscillation, not bias | 215805, 220012, 221242 |
| 15 | ~22:3x (8/30) | **optical flow ENABLED** (`ROSE_FLOW=1`) + gyro tracker retained | **major finding: build_fc_telem was flying with flow OFF** — `ROSE_FLOW=0`, so horizontal velocity was pure accel dead-reckoning (`vx += ax_w*dt`, ZUPT on ground). That inherently drifts (accel bias + tilt→gravity-leak) and likely underlies the baseline drift. Also: the "only a re-flash fixes it, NOT a full power-cycle" datum is **logically inconsistent with any firmware/HW state** (a power-cycle clears RAM+sensors+re-runs the gyro cal; the only flash writer, the flightlog @0x3b0000, is erased by neither a power-cycle nor `west flash`) → suspect a test confound or the evolving code + flight-to-flight variance. Flow gives a real velocity reference; retest for drift + whether the progression changes. **CAUTION:** flow can self-excite a roll↔flow oscillation if gyro-comp/cal is off (rate-gated by `FLOW_GYRO_MAX=1.2`). Built+verified (map: flow_init/flow_thread/pmw3901_init linked); **awaiting board on USB to flash.** | *pending flash* | — |

## Key findings so far

- **IMU lever-arm:** the BMI088 is 16 mm LEFT of the CoM (body +y; `estimator_complementary.cpp`
  subtracts `α×r + ω×(ω×r)`, `r=(0,+0.016,0)`). Cancels the *rotation-correlated* drift.
- **Under-powered attitude loop is the big one:** the controller feeds the estimator's **Gibbs
  vector as Euler** (`state[3,4] ≈ angle/2` near level), so roll/pitch feedback is ~half-scale →
  the loop leaves a steady tilt it can't close. `ATT_GAIN≈2.0` roughly cancels the halving.
- **Softer→flips, stronger→stable** confirms the above (the residual was never oscillation).
- **Stability edge:** `tau_rollRate=0.015` oscillates (inner-loop bandwidth too high); `0.02` is the
  known-stable inner loop. Authority (`ATT_GAIN`) and inner-loop bandwidth (`tau_rollRate`) are
  separable knobs — set authority high, keep the rate loop just under its edge.
- **Principled fix (TODO):** feed **proper roll/pitch** (`2·atan(state[3])` / true Euler) to the
  controller → removes the 2× hack (gains mean what they say) + fixes the yaw-frame coupling; then
  re-tune. An attitude-loop integrator would null any residual steady error outright.
- **Ceiling:** thrust-marginality (71 % hover, saturation) caps how clean the hover can get.
