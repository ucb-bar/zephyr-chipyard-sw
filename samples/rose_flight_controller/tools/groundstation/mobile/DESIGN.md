# riskybird v3 — Mobile Flight-Controller Web Interface (DESIGN)

**Goal.** A phone joins the drone's own WiFi SoftAP and, in a normal mobile browser, gets a
touch-first flight-monitor + control page: at-a-glance flight state, an always-reachable **ESTOP**,
**RESET/ENABLE**, **HOVER_Z ±**, and a designed-in path to **manual velocity control**. No app
install, no internet, no laptop.

This doc: (1) the transport analysis + recommendation, (2) the UI design with wireframes, (3) the
new `VTARGET` uplink command spec, (4) a phased plan, (5) how to test the prototype today.

> Deliverables live in `tools/groundstation/mobile/` (this file + `mobile.html`). **No firmware or
> existing ground-station files are modified.**

---

## 0. What the system actually is (verified against source)

Read from `src/telem_wifi.c`, `src/telem_wifi.h`, `tools/groundstation/riskybird_panel.py`,
`telem.conf`, `prj.conf`, `boards/esp32c6_devkitc_hpcore.conf`, `docs/TELEMETRY_PLAN.md`.

- **Radio / addressing.** The FC (Zephyr on an **ESP32-C6**) hosts an **open 2.4 GHz SoftAP**,
  SSID `riskybird-<id>` (`<id>` = last 2 MAC bytes, e.g. `riskybird-5668`), IP **192.168.4.1**,
  DHCP pool from `192.168.4.10`. A joining client gets a lease.
- **Downlink.** ~**50 Hz plain-text UDP** datagrams, **unicast to each DHCP-leased client** on
  **:14550** (broadcast `192.168.4.255` only as a no-lease fallback). One datagram = the full latest
  state; loss-tolerant.
- **Uplink.** Text UDP commands on **:14551**, one command/datagram, UPPERCASE, ACKed back to sender.
- **Decoupling.** Telemetry TX + command RX run on their own **low-priority** threads
  (`K_PRIO_PREEMPT(10)`), *below* the sensor threads (prio 8) and the ~1 kHz control loop (the
  **main thread**, single-loop `ROSE_THREADED=0`). WiFi TX may block, so it never runs in the loop.

### Telemetry wire format — `telem_format_v1()` (verbatim field order)

```
flight_controller: it=<i> dt=<ms>ms roll=<f> pitch=<f> yaw=<f> z=<f> tofv=<0|1> tofh=<f>
  u=[<f> <f> <f> <f>] vx=<f> vy=<f> vz=<f> zsp=<f> vbat=<f> st=<u> x=<f> y=<f>
```

| field | meaning | notes |
|---|---|---|
| `it` / `dt` | loop iter counter / measured loop period (ms) | `1000/dt` ≈ loop Hz |
| `roll pitch yaw` | attitude as the **Gibbs/Rodrigues vector** (`q_xyz/q_w`) | **must be reconstructed** to a unit quaternion + ZYX Euler (singular near 180°) |
| `z` | estimated height, m | |
| `tofv` / `tofh` | down-ToF valid flag / raw tilt-corrected height, m | |
| `u[0..3]` | motor commands (thrust), 0..1 | order FR/RR/RL/FL = m0/m1/m2/m3 (per panel.html) |
| `vx vy vz` | est velocity, m/s (body: +x fwd, +y right; +z climb) | |
| `zsp` | altitude setpoint, m | |
| `vbat` | smoothed pack voltage, V (0 = batt sense off) | |
| `st` | flag bits (below) | |
| `x y` | dead-reckoned horizontal position, m | |

**Flag bits** (`telem_wifi.h`, emitted as `st=`): `ARMED=0x1`, `ESTOP=0x2`, `ARMING=0x4`,
`CALDONE=0x8`.

The desktop bridge parses this with `CORE_RE`/`TAIL_RE`/`POS_RE` and `gibbs_to_quat_euler()`.
**`mobile.html` ports those exactly** (same regexes, same reconstruction) so it can consume the raw
line directly — see §5.

### Command grammar — `cmd_thread_fn()` (verbatim)

| uplink datagram | firmware hook | ACK | notes |
|---|---|---|---|
| `ESTOP` | `rose_cmd_estop()` | `ACK ESTOP` | **latched** remote kill; clears only on RESET |
| `DISARM` | `rose_cmd_disarm()` | `ACK DISARM` | motors off; autoflight may re-arm via the gate |
| `RESET` | `rose_cmd_reset()` | `ACK RESET` | soft reset: clears estop, disarms, **re-cals gyro, re-enables arming** (no chip reset) |
| `HOVER_Z <mm>` | `rose_cmd_set_hover_z(mm/1000)` | `ACK HOVER_Z <mm>` | **argument is millimetres** (`atoi(rx+7)`); clamped to `[0, SAFE_MAX_HEIGHT]` |
| `PROFILE <c> <h> <d> <max>` | `rose_cmd_set_profile(...)` | `ACK PROFILE …` | autoflight durations, ms; `-1`/absent = keep |
| `PING` | — | `PONG` | liveness |
| *(other)* | — | `ERR unknown` | |

> **No remote force-ARM exists, by design** (arming stays on the on-board gate). The mobile UI must
> not imply it can arm; `RESET/ENABLE` only *permits* the on-board autoflight arm.

### Build/RAM facts that constrain the transport choice

- `telem.conf` enables **UDP only**: `CONFIG_NET_UDP=y`, **`CONFIG_NET_TCP=n`**. Any browser
  transport (WebSocket / SSE / HTTP poll) needs **TCP** → this must be turned on first.
- WiFi/net stack already costs **~104 KB SRAM**; the combined FC+WiFi build sits at **≈71 %** of the
  ~437 KB SRAM region → **~120–130 KB free** headroom (per `docs/TELEMETRY_PLAN.md §5`).
- The project's Zephyr is **v4.2.99**, which ships the full **`CONFIG_HTTP_SERVER`** subsystem
  (`subsys/net/lib/http`, static + **dynamic/streaming** resources, plus a websocket lib and the
  `samples/net/sockets/http_server` reference). So an on-board HTTP server is *available in-tree*.
- The net path has a **history of stack-overflow / lock-inversion crashes** (sys-workq stack,
  net-mgmt event stack, DHCP server-lock inversion — all documented in `telem.conf` comments and
  `docs/TELEMETRY_BRINGUP.md`). Adding TCP + a server thread is exactly the change that can
  reintroduce that risk class, so it must be **bench-measured (INIT_STACKS high-water) before flying**.

---

## 1. The hard problem

**A phone browser cannot open a UDP socket and cannot run the Python bridge.** Even though the
firmware *does* unicast 50 Hz telemetry to the phone's leased IP:14550, JavaScript in the browser
can't read those datagrams, and it can't `sendto` :14551. The page can only speak what browsers
speak: **HTTP / SSE / WebSocket / fetch** — all of which require **TCP** and an **HTTP endpoint on
something the phone can reach** on the SoftAP subnet.

So the question is *where the TCP/HTTP endpoint lives*.

---

## 2. Transport options (evaluated)

| | A. Firmware HTTP **+ WebSocket** | C. Firmware HTTP **+ SSE** (no WS) | B1. Companion **bridge** on the AP | B2. Native app / Termux-python on phone |
|---|---|---|---|---|
| Phone-only? | ✅ | ✅ | ❌ (needs a 2nd device on the AP) | ✅ (but not "just a web page") |
| Firmware change | **large** | **medium** | **none** | none |
| Needs `NET_TCP=y` | yes | yes | no | no |
| Extra SRAM (est.) | ~30–55 KB | ~20–40 KB | 0 | 0 |
| Extra flash (est.) | ~30–60 KB (WS lib + server) | ~20–40 KB (server) | 0 | 0 |
| CPU during flight | net threads only (off the loop); TCP+WS framing | net threads only; TCP; **lighter than WS** | 0 on the drone | 0 on the drone |
| Latency | push, ~1 frame + WiFi | push (SSE) ~1 frame + WiFi; poll ~1 interval | UDP→SSE hop, few ms | direct UDP, minimal |
| Effort | high | **medium** | **trivial (already exists)** | medium (per-platform) |
| Risk to flight timing | medium (new server thread + TCP on the RT chip) | low–medium | **none** | none |

### A — Firmware HTTP + WebSocket
Zephyr 4.2.99 has `CONFIG_WEBSOCKET` + `CONFIG_HTTP_SERVER` WS upgrade. A WS gives a single duplex
low-latency channel: push 50 Hz telemetry frames, receive commands on the same socket. **But** for
this app the channel is essentially **one-way push + occasional command** — WS buys bidirectional
framing we don't need, at the cost of the websocket lib (flash), masking/framing buffers (RAM), and
more moving parts on a real-time flight chip that just fought its way to a stable net stack.
**Not worth it now.**

### C — Firmware HTTP + SSE (recommended firmware path)
Serve three routes from the ESP32-C6 over plain HTTP/1.1:
- `GET /` → the static `mobile.html` (embedded as a flash blob / http_server static resource).
- `GET /stream` → **Server-Sent Events**: a dynamic/streaming resource that writes
  `data: <the raw v1 telemetry line>\n\n` at 50 Hz (or on each snapshot). Browser-native
  (`EventSource`), auto-reconnecting, **no library, no framing** — the cheapest possible push.
- `GET /t` → the single latest snapshot as one text line (poll fallback for flaky SSE).
- `POST /cmd` → command text; the handler calls the **existing** `rose_cmd_*` hooks directly (no UDP
  hop) and returns the ACK as JSON/text.

This is literally *the desktop bridge's data contract moved into the firmware, minus the UDP hop*.
SSE needs TCP but **no** websocket subsystem, so it's the smallest firmware delta that gets a phone
onto the link. It keeps all net work on the net/low-priority threads (off the 1 kHz loop) and is
loss-tolerant by construction (each SSE line is the full state; a dropped/rebuffered TCP segment
just delays the next line — the page keeps showing the last one).

### B1 — Companion bridge on the AP (recommended *today*, zero firmware risk)
Any second device that can join the SoftAP and run Python — a **laptop, a Raspberry Pi, or an old
Android phone in Termux** — runs the existing `riskybird_panel.py` (UDP↔SSE bridge + HTTP). The
**phone** then just browses to `http://<bridge-ip>:8080`. **`mobile.html` already speaks that exact
`/stream` + `/cmd` contract**, so this works with **no new code and no firmware change** — it's the
same thing the desktop dashboard does, with the phone pointed at the bridge instead of `localhost`.
Cost: you carry a second device to the field.

### B2 — Native app / phone-side Python
A native Android/iOS app (or `python3 riskybird_panel.py` inside **Termux on the phone itself**) can
open a real UDP socket → true 50 Hz on the phone, phone-only, no firmware change. But it's not "open
a web page," it's per-platform, and needs an install. Useful power-user path; not the primary UX.

### Rejected
- **WebRTC datachannel / QR-to-hosted-page / cloud tunnel** — all need internet + a signaling/STUN
  server, and the SoftAP is **isolated with no internet**. A local signaling server is just another
  bridge (→ B1). The drone also can't reasonably be a WebRTC peer. **Rejected.**

---

## 3. Recommendation

**Phase the transport; do the risky firmware change last and behind a flag.**

1. **Today — Option B1 (companion bridge).** Zero firmware risk, works immediately, and `mobile.html`
   already targets its contract. This is how you fly a phone *this week*.
2. **Target phone-only — Option C (firmware HTTP + SSE).** Add `CONFIG_NET_TCP=y` +
   `CONFIG_HTTP_SERVER=y` **inside `telem.conf`** (opt-in build, so the flight binary stays
   byte-identical), serve `mobile.html` + `/stream` + `/t` + `/cmd`, and **keep the UDP
   :14550/:14551 path in parallel** (the laptop GCS + logging still use it). Measure combined
   SRAM + **main-thread stack high-water** (`CONFIG_INIT_STACKS`) on the bench before any flight.
3. **Do not** build Option A (WebSocket). SSE + POST covers monitoring and step-commands; revisit WS
   only if the future manual-velocity uplink demands a single duplex socket (it doesn't — see §5,
   VTARGET rides fine on periodic POSTs).

`mobile.html` is built so **all three** (bridge, firmware-SSE, demo) are the same page with a
pluggable adapter — no rewrite between phases.

---

## 4. Mobile UI design

**Principles.** Portrait phone, touch-first, one screen you scroll. Big glanceable numerics + high
contrast (sunlight/OLED). Dark by default (light auto-fallback via `prefers-color-scheme`). Offline:
all CSS/JS inline, no CDNs. Cheap on the phone: render is **rAF-throttled to ~25 fps and decoupled
from the 50 Hz packet rate** (coalesce, never redraw per packet). Tolerant of dropouts (keep last
state; link dot goes stale/red). Auto-reconnecting (`EventSource` reconnects; poll self-heals).
**ESTOP is a fixed bottom bar — always on-screen regardless of scroll.**

### 4.1 Screen map (portrait, single scroll)

```
┌──────────────────────────────────────────────┐  ← sticky header
│ riskybird v3 · mobile FC       50 Hz ● linked │
│ ┌──────────────────────────────────────────┐ │
│ │  ● ARMED — LIVE                   it 1234 │ │  ← big state banner (colour-coded)
│ └──────────────────────────────────────────┘ │
│ [ARMED] [ESTOP] [ARMING] [ CAL ]              │  ← status pills (lit per st= flags)
├──────────────────────────────────────────────┤
│ ┌───────────────┐ ┌───────────────┐          │  ← METRICS (2-col)
│ │ ATTITUDE      │ │ ALTITUDE      │          │
│ │   ( ◔ AH )    │ │  0.30 m       │          │
│ │  roll  pitch  │ │  ▓▓▓▓▓░░│░ ←tgt│          │  (bar + zsp tick)
│ │  yaw          │ │  tgt/ToF/vz   │          │
│ └───────────────┘ └───────────────┘          │
│ ┌───────────────┐ ┌───────────────┐          │
│ │ DRIFT & VEL   │ │ BATTERY       │          │
│ │  (top-down ⊕) │ │  3.95 V       │          │
│ │  vx vy        │ │  ▓▓▓▓▓▓▓░░     │          │
│ │  speed drift  │ │ age/loop/qual │          │
│ └───────────────┘ └───────────────┘          │
│ ┌──────────────────────────────────────────┐ │
│ │ MOTOR u[]   FL m3 ▓▓░  FR m0 ▓▓░          │ │
│ │             RL m2 ▓▓░  RR m1 ▓▓░          │ │
│ └──────────────────────────────────────────┘ │
├──────────────────────────────────────────────┤
│ ALTITUDE SETPOINT · HOVER_Z                   │  ← CONTROL
│  [ − ]  [ 300 mm (0.30 m) ]  [ + ]  [ SET ]   │
│  [ ↺ RESET / ENABLE ]        [ PING ]         │
│  ack: HOVER_Z 300 ⇒ ACK HOVER_Z 300           │
├──────────────────────────────────────────────┤
│ MANUAL VELOCITY TARGET         [COMING SOON]  │  ← SCALE-UP stub (disabled)
│   ┌─────────────┐  ┌──┐                        │
│   │   ⊕ pad     │  │▮ │  ← vz slider           │
│   │  vx / vy    │  │  │                        │
│   └─────────────┘  └──┘                        │
│   → VTARGET vx vy vz (rate-limited, failsafe)  │
├──────────────────────────────────────────────┤
│ src [DEMO][LIVE·SSE][LIVE·POLL]  base URL […]  │  ← transport switch
└──────────────────────────────────────────────┘
┌──────────────────────────────────────────────┐  ← FIXED bottom bar (always visible)
│   ⭘  E S T O P                     50  Hz     │
└──────────────────────────────────────────────┘
```

### 4.2 Phase 1 — Metrics view (implemented in the prototype)
- **State banner** — the machine `stateClass()`: `WAITING` → `CALIBRATING` → `READY` → `ARMING` →
  `ARMED` → `ESTOP`, colour-coded (grey/cyan/green/amber/orange/red), ESTOP pulses.
- **Status pills** — `ARMED / ESTOP / ARMING / CAL`, lit straight off the `st=` bits.
- **Attitude** — compact artificial horizon (canvas) from the reconstructed quaternion/Euler; roll,
  pitch, yaw read-outs.
- **Altitude** — big `z`; bar with a **setpoint tick** at `zsp`; `zsp`, raw `tofh`, `vz`, ToF-valid.
- **Drift & velocity** — top-down puck (canvas) plotting body-frame `vx/vy` with 0.15/0.30/0.50 m/s
  rings, colour by speed; `vx`, `vy`, planar `speed`, and `drift` = `hypot(x,y)`.
- **Battery + link** — big `vbat` with a 3.2–4.2 V bar (red/amber/green); link `age`, `loop` Hz,
  `quality`, `pkts`.
- **Motors** — four `u[]` bars (FL m3 / FR m0 / RL m2 / RR m1), colour by duty.

### 4.3 Phase 2 — Control (implemented)
- **ESTOP** — fixed bottom bar, oversized, haptic (`navigator.vibrate`), flash-confirms → `ESTOP`.
- **RESET / ENABLE** — `RESET` (clears estop, re-cals, re-enables arming). Labeled so it's clear it
  *permits* on-board arming, never force-arms.
- **HOVER_Z ±** — stepper in **mm** (±50, clamp 0–600) with a live `m` readout → `HOVER_Z <mm>`.
- **PING** — liveness. Every command shows the firmware ACK inline.

### 4.4 Phase 3 — Manual velocity (designed, stubbed visible/disabled)
A thumbpad (→ body-frame `vx,vy`) + a vertical slider (→ climb `vz`), laid out now but disabled and
badged **COMING SOON**, because it needs a new firmware uplink command (§5). Wiring it later:
touch → clamp to the envelope → send `VTARGET vx vy vz` at **20 Hz** while touched; on release send
`VTARGET 0 0 0`; show a local "manual link" watchdog that greys out if ACKs stop.

---

## 5. Proposed new uplink command — `VTARGET` (manual velocity target)

The current grammar has **no** velocity-target command. Minimal, consistent addition:

### Grammar
```
VTARGET <vx> <vy> <vz> [<yawrate>]
```
- Signed floats, **m/s**, body frame: **+vx forward, +vy right, +vz climb**. `<yawrate>` (rad/s) is
  **reserved/design-only** — parsed-but-ignored initially (like `PROFILE`'s tolerant trailing args).
- Parsed in `cmd_thread_fn()` exactly like `PROFILE` (walk with `strtod`, missing→0), added to the
  `strncmp` chain (`"VTARGET"` collides with nothing).
- **ACK echoes the *clamped, applied* values** so the operator sees the envelope:
  `ACK VTARGET <vx> <vy> <vz>\n`.

### Firmware hook (new, in `telem_wifi.h` + `main.cpp`)
```c
void rose_cmd_set_vtarget(float vx, float vy, float vz);   /* pokes a shared setpoint + timestamp */
```
Follows the existing shared-flag pattern: stashes `{vx,vy,vz, t_ms=k_uptime_get()}` in a volatile
struct the control loop reads next iteration. Same "no remote arm" rule applies.

### Safety / gating (all in the control loop; tunables via `-DEXTRA_CPPFLAGS` like the other `ROSE_*`)
1. **Authority gate.** Honoured **only when `armed && caldone && !estop`**. Otherwise ignored; the
   horizontal loop keeps its default position/altitude-hold behaviour. Never arms.
2. **Envelope clamp.** `|vx|,|vy| ≤ VTARGET_VXY_MAX` (default **0.5 m/s**); `|vz| ≤ VTARGET_VZ_MAX`
   (default **0.3 m/s**). `vz` integrates onto `zsp`, reusing the existing `[0, SAFE_MAX_HEIGHT]`
   clamp from `HOVER_Z`.
3. **Slew-rate limit.** The *applied* target ramps toward the commanded target at
   `≤ VTARGET_ACC_MAX` (default **0.5 m/s²** xy, **0.4** z) per tick, so a full-stick step can't
   cause a lurch.
4. **Mode capture.** A fresh `VTARGET` switches the horizontal loop from position-hold to
   velocity-target. On expiry (below) it **re-captures the current `(x,y)` as the new hold point** —
   it stops and holds *where it is*, not where it started.
5. **Comms-loss failsafe (the critical one).** A watchdog on the `VTARGET` timestamp: if
   `now − t_ms > VTARGET_TIMEOUT_MS` (default **400 ms**), the applied target **decays to zero
   through the slew limiter** (a smooth stop) and the loop reverts to position/altitude hold at the
   current spot. **It does *not* cut motors** — a dead link mid-air must *stop translating and hold*,
   not fall. This is distinct from `ESTOP` (the deliberate latched kill), and it's the fast inner
   layer beneath `TELEMETRY_PLAN.md §3`'s coarser armed link-loss watchdog (which may later
   auto-hover/auto-land after a longer silence).
6. **Uplink cadence.** The phone sends `VTARGET` at a fixed **~20 Hz** while the pad is touched and
   `VTARGET 0 0 0` on release; the 400 ms firmware timeout is the backstop if the phone or link dies.

### Why not a new port / binary / WS
It fits the existing text-UDP command channel and (in Phase-1 firmware-HTTP) an existing `POST /cmd`.
No new socket, no websocket. Keep it text until a v2 binary telem/command format lands.

**Effort:** small (one parse branch, one hook, ~20 lines in the setpoint path, 4 tunables) — but it
is a **real new flight-control input**, so it lands only after bench validation (props off → tethered
→ free), and stays behind the same opt-in as the rest of the telemetry build.

---

## 6. Phased implementation plan

| Phase | Scope | Firmware? | Status |
|---|---|---|---|
| **P0** | `mobile.html` metrics + ESTOP/RESET/HOVER, demo mode, adapter for bridge/SSE/poll | none | **done (this deliverable)** |
| **P0.5** | Fly a phone **today** via a **companion bridge** (B1): run `riskybird_panel.py` on a laptop/Pi/Termux on the AP, point the phone's browser at it | none | ready now |
| **P1** | **Firmware HTTP + SSE (Option C)** in `telem.conf`: `NET_TCP` + `HTTP_SERVER`; serve `mobile.html` + `/stream` + `/t` + `/cmd`→`rose_cmd_*`; keep UDP in parallel; **measure SRAM + main-stack high-water** | medium | proposed |
| **P2** | **`VTARGET`** uplink (§5) + enable the manual-velocity pad in `mobile.html`; bench-validate props-off → tethered → free | small–medium | proposed |
| **P3** | Polish: PWA/service-worker offline cache, v2 binary telem, RSSI/link-stats, on-phone logging | optional | future |

---

## 7. Testing the prototype today, and what's left for "live on a phone"

**Today, no drone, any desktop browser:**
1. Open `tools/groundstation/mobile/mobile.html` directly (`file://`). It **auto-selects DEMO mode**
   and animates physically-plausible fake telemetry (the exact `telem_format_v1` line shape), so the
   whole metrics view renders and updates.
2. Press **RESET/ENABLE** → watch `CALIBRATING → ARMING → ARMED`; the horizon, drift puck, motors,
   and battery come alive. Press **HOVER_Z ±/SET** → the altitude target tick moves. Press **ESTOP**
   → banner goes red/latched. (The device toolbar's phone emulation shows the portrait layout.)

**Live against the existing desktop bridge (validates the real transport contract):**
1. On a machine joined to the SoftAP, run `python3 tools/groundstation/riskybird_panel.py`.
2. In `mobile.html`, set the **base URL** to that host (e.g. `http://192.168.4.10:8080`) and pick
   **LIVE·SSE**. The page consumes the bridge's `/stream` (JSON snapshot **or** raw line — the
   adapter handles both) and posts commands to `/cmd`, unchanged. From a phone on the same AP,
   browse to that URL → this is already a working phone UI (Option B1).

**What remains to make it phone-only (no second device):** ship **Option C** — add
`CONFIG_NET_TCP` + `CONFIG_HTTP_SERVER` to `telem.conf`, embed/serve `mobile.html`, add the
`/stream` SSE + `/t` + `/cmd`→`rose_cmd_*` routes, and bench-measure the combined SRAM and
main-thread stack high-water before flight. Then the phone browses straight to
`http://192.168.4.1/` and the same page runs with **LIVE·SSE**, base URL blank. Manual velocity
control additionally needs the **`VTARGET`** firmware command from §5.
