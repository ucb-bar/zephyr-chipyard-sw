# RiskyBird history-preserving integration, 2026-09-28

This branch merges the original upstream branches first, then the existing
RiskyBird fork. It does not import the previous migration implementation,
relocate firmware/model sources, or create a second flight-controller sample.
Original commits and authors remain in history. Existing development branches
were not advanced. New commits have no AI coauthor trailers.

## Branches and inputs

All software and model branches below are on **lorenshung** forks. The only
upstream publication is the explicitly authorized new **ucb-bar/zephyr** RTOS
branch.

| Repository | New branch | Commit / role |
| --- | --- | --- |
| lorenshung/ModelBlaster | imports/20260928/riskybird-integer-dronet | Exact upstream `10b50ba9df35c8646786e2fa9cb7e16f24d361b7` |
| lorenshung/ModelBlaster | imports/20260928/riskybird-fps-repro | Exact upstream `aaf0d4442b5a889ec7178eadc56947228081ada7` |
| lorenshung/ModelBlaster | imports/20260928/riskybird-fps-placement | Exact upstream `66da197344687b26cbc4ec30ae5399b83ed8f7d8` |
| lorenshung/ModelBlaster | integration/riskybird-20260928-upstream | `c354b9f4a83b6cbcac96fa2b205e3b3aa3ff0ee7`; merges those three upstream branches |
| lorenshung/ModelBlaster | integration/riskybird-20260928 | `80cec139799e6ecf3d8625b14cdaa78ba3bf596a`; additionally merges fork `1293334703cd99fa80871448126bd33b17827551` |
| lorenshung/zephyr-chipyard-sw | imports/20260928/riskybirdv3-bringup | Exact upstream `f33d768e95ba03daed2017244d93e5b103a55963` |
| lorenshung/zephyr-chipyard-sw | imports/20260928/riskybird-fc-dronet | Exact upstream `1e74c5eada69bf6969a2407603fc3d68195d8bbc` |
| lorenshung/zephyr-chipyard-sw | integration/riskybird-20260928-upstream | `a207a8f34057b6757b7761d5ef2f14392168f18f`; upstream merge |
| lorenshung/zephyr-chipyard-sw | integration/riskybird-20260928 | `c64714120d4057ded38303e548a10a4c6bf28987` merges fork `44f79221e3837da79d8a55575303553e5619c80d`; `176ad96` adds the tested command-contract correction |
| ucb-bar/zephyr | integration/riskybird-20260928 | `5241922dd100c98dd953f6a6478f1e2d408de8aa`; merges current `riskybird-v3` (`25945feb`) and `riskybirdv3-bringup` (`bfd185ff`) |

The bring-up software branch is already an ancestor of FC/DroNet. Its explicit
upstream merge therefore has the exact FC/DroNet tree. Likewise ModelBlaster's
integer branch is already an ancestor of FPS-repro. No patches were replayed,
squashed, or cherry-picked. The final model branch has exactly three new merge
commits, and all four original model tips are ancestors.

GitHub SSH access was available, but the GitHub CLI was unauthenticated. These
are ordinary Git merges published to GitHub; no GitHub API PR was created.

## Conflict resolution

ModelBlaster's sole conflict was a comment in `pipeline/backends.py`. Both sides'
functional changes survive, including the automatic merge of ABI and SDK logic
in `pipeline/profile_kernel.py`. Placement remains opt-in.

The software merge had six conflicted paths: `.gitignore`, `modelblaster`,
`flightlog.c`, `flow.c`, `main.cpp`, and `zephyr_ws/zephyr`.

- Keep both ignore lists and the nested ModelBlaster dependency, now pointing at
  the new fork merge. Both RTOS gitlink and the app manifest name the new RTOS
  merge rather than a moving development branch.
- Use the existing fork's complete no-flash logging stubs. The upstream outer
  guard would otherwise remove those functions on boards without flash.
- Retain upstream flow CS=0 on Chipyard, ESP CS=19, and the fork's explicit
  override support. An empty CMake override allows the board defaults to work.
- Combine ToF error diagnostics/channel-get checks with upstream simulation
  stubs, debug counters and pacing. Preserve both periodic summary and UART
  drain blocks. Controller timing still requires hardware measurement.
- Resolve the camera-name collision in the existing implementation: `ROSE_CAMERA`
  retains the fork's still-photo behavior; `ROSE_CAMERA_DMA` selects upstream
  DMA capture. Both enabled is a compile error. Historical upstream commands
  that used `ROSE_CAMERA=1` to mean DMA must now use `ROSE_CAMERA_DMA=1`.
- Preserve upstream raw UART telemetry under `ROSE_UART_TELEM=1`. It disables
  the fork's automatic UART command reader and telemetry mirror. Combining it
  with the ESP motor transport, an explicit driver command reader, or that UART
  as the console is rejected. The existing motor-link path remains available
  with raw telemetry disabled.

The merged panel exposes upstream `SNAP`. A host integration test found that
the motor-link parser did not recognize it. The follow-up commit returns
`FCNAK SNAP` explicitly on that transport; upstream raw-UART/DMA snapshot
handling remains unchanged. It does not map SNAP onto the separate still-photo
workflow or falsely acknowledge successful capture.

The RTOS's only conflict was two independent whole-transaction I2C mutex fixes.
The existing current-driver implementation was retained. Incoming FP16 boot
FCSR guard and fixed-pin pinctrl support merged cleanly. The resulting tree
happens to match the previously audited draft RTOS tree, but was produced by a
fresh merge of the original branches.

## Validation performed

| Check | Result |
| --- | --- |
| ModelBlaster full host suite, original fork vs upstream-only merge vs final merge | Identical: **218 passed, 4 failed, 11 skipped** |
| ModelBlaster real RISC-V compiler checks | **6 passed** |
| ModelBlaster Python/shell syntax | 334 Python files parsed; 118 shell scripts pass `bash -n` |
| Existing riskybird-dev integration tests, selected build/flight/photo/ground/command/model/clock suites | **124 tests, OK, 5 skipped** |
| New `tests/test_fc_merge.py` | **2 passed**; actual command parser compiled/executed on host, eight preprocessor mode cases |
| Existing T21 FullDDR FC preset | **Build passed**, RAM 435,872 B |
| Existing photo-bench preset | **Build passed**, RAM 522,080 B |
| Combined FC+DroNet, raw telemetry/DMA off | **Build passed**, RAM 1,951,616 B; selected FP16-shell instruction scan passed |
| Combined FC+DroNet, DMA/raw telemetry on | **Build passed**, RAM 2051 KiB; selected FP16-shell instruction scan passed |

The ModelBlaster failures are the same four pre-existing `rvv_x60` weight-layout
checks on all three tested inputs. They are not new merge failures. The known
flat Gemmini per-config header lookup defect also remains inherited: some named
DIM32 configurations select fallback DIM16 parameters, whereas the nested
`q31ws_32x32_acc` configuration selects correctly. Neither issue was redesigned
as part of this merge-only task.

The new host tests exercise ESTOP/DISARM/RESET/PING ACKs, HOVER_Z/PROFILE
arguments, malformed-command NAKs, explicit SNAP rejection, fragmented input,
noise recovery, and overlong-line discard. They also test UART ownership and
mutually exclusive camera modes. Run:

```sh
python3 -m unittest discover -s tests -p test_fc_merge.py -v
```

Firmware checks used Zephyr SDK 1.0.0-beta1 and the installed `zephyr-new`
environment. An isolated riskybird-dev `a63a27b` test checkout used these new
backend source trees, with existing top-level firmware libraries/overlays and
committed model fixtures retained in their original locations. No migration
source relocation was used. Installed compiler/Python binaries were reused.

Nested dependencies were cloned into the isolated checkout at their recorded
commits, using local Git object stores. RTOS's original `west-riscv.yml` selected
24 active west projects; TinyMPC is `172926df` and its matlib is `6c5baf9b`.
Two imported projects track moving `zephyr` branches; the exact observed build
dependency revisions are recorded in `integration-20260928-dependencies.tsv`.
This is source/build validation, not a new network-only bootstrap validation.

With that environment initialized, the ordinary integration commands were:

```sh
rb build flight_controller --target arty200t \
  --config RocketArty200TDroneFullDDRConfig --mode t21-hover-1.0m
rb build flight_controller --target arty200t \
  --config RocketArty200TDroneFullDDRConfig --mode photo-bench
```

The combined sample was built directly with west, using this checkout's RTOS
and nested ModelBlaster, and the original committed fixture from riskybird-dev:

```sh
west build -p always -b chipyard_riscv64 samples/rose_fc_dronet \
  --build-dir build_fc_dronet -- \
  -DMB_REPO="$PWD/modelblaster" \
  -DMODEL_DIR="<riskybird-dev>/artifacts/modelblaster/dronet/int8/gemmini_q31_rvv_softfp-gray-nhwc" \
  -DEXTRA_CPPFLAGS="-DROSE_MOTORS_INHIBIT=1 -DROSE_PROFILE=1 -DROSE_TELEM=0 -DDRONET_REPORT_EVERY=1 -DDRONET_IRQ_GATE=0"
# A separate build additionally supplied:
# -DROSE_CAMERA_DMA=1 -DROSE_UART_TELEM=1
```

Use absolute paths for `MODEL_DIR` and the sample when running west from the
existing `zephyr_ws` workspace. The default upstream generated-model directory
is still absent from Git; its generation helper prints a recipe. An explicit
committed fixture proves API/compilation compatibility, not reproduction of
the historical polling/yielding kernel recipe or its concurrency guarantees.

## Hardware and remaining limits

No FPGA programming, inference execution, camera capture, motor test or flight
was performed during these checks. The existing combined application uses a
static input, not live camera frames. Build and instruction-scan success do not
establish numerical correctness, vector context preservation or FC deadlines.

The unchanged upstream `telem_uart.c` assumes a 50 MHz peripheral clock while
the combined FC+DroNet target declares 35 MHz. Its raw UART baud therefore needs
verification/correction before using that combination on hardware. Its raw
command parser also lacks PROFILE support although the panel offers it; this
is inherited upstream behavior. The current motor-link parser supports PROFILE.

Remaining hardware checks: boot/sensors/CS0 flow, standalone numerical inference,
still-photo capture, DMA frame/snapshot integrity and raw UART commands with a
matched clock, then sustained disarmed simultaneous FC/inference timing and
vector-state behavior. Flight/motor qualification is separate. Existing
working trees, default branches and hardware images remain unchanged.
