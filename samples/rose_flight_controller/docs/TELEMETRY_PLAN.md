# riskybird v3 — Remote Telemetry Integration Plan

Untethered 2.4 GHz link between the flight controller (ESP32‑C6) and a **laptop** host:
a live telemetry **downlink** plus a command/safety **uplink** (remote kill + link‑loss
watchdog), replacing the USB tether for flight monitoring and control.

**Decisions (2026‑08‑12):**
- **Radio:** WiFi (SoftAP + UDP), *assuming RAM permits* — being validated by two parallel spikes.
- **Host:** laptop.
- **Command channel:** yes — remote kill + link‑loss watchdog.

Enabled by the newly‑ordered **Lemon RX I‑PEX MHF3 (2.45 GHz, 10 cm)** antenna for the
ESP32‑C6‑MINI‑1U (which is MHF3 / W.FL — *not* MHF1/U.FL; see the antenna saga).

---

## 1. Radio & link
- **WiFi SoftAP:** the drone hosts an AP (SSID `riskybird-<id>`); the laptop joins it directly.
  No router/infrastructure → a dedicated, low‑latency link. (Station mode — drone joins an
  existing WiFi — is the fallback if the laptop must keep internet during flights.)
- **Downlink (telemetry): UDP**, 50–100 Hz. Loss‑tolerant: each packet carries the latest
  state, so a dropped packet just means "wait for the next one" — no retransmit stalls.
  - **DESIGN CHANGE (2026‑08‑16): unicast, not broadcast.** WiFi broadcast/multicast is buffered at
    the AP and released only at DTIM/beacon boundaries and is never MAC‑ACKed/retried → bursty
    ~37 Hz with 100 ms+ gaps. The firmware now **unicasts the telemetry to each DHCP‑leased client**
    (it runs the DHCP server, so it knows their addresses via `net_dhcpv4_server_foreach_lease`);
    broadcast remains only as a no‑lease fallback. Unicast is ACKed + retried + delivered
    immediately → smooth ~50 Hz, <0.5 % loss. **See `docs/TELEMETRY_BRINGUP.md` for the working
    config + receive procedure + root causes of the earlier connect failure.**
- **Uplink (commands): UDP** with an app‑level ACK for critical commands (or a small TCP
  control socket). Commands: `ARM` / `DISARM` / **`ESTOP`** / `HOVER_Z` / gain tweaks / `DUMP`.
- Suggested ports: `14550` telemetry, `14551` commands (document + keep consistent).

## 2. Firmware architecture (ESP32‑C6 / Zephyr)
- **Telemetry thread** (low priority), **decoupled from the 1 kHz control loop.** The WiFi TX
  can block / has variable latency, so it must NOT run inside the control loop — the same rule
  that put the down‑ToF and optical‑flow reads on their own threads. The control loop fills a
  mutex‑protected `telem_snapshot` each iteration; the telemetry thread copies it and sends a
  UDP packet at a fixed rate.
- **Command handler:** a socket‑RX path that parses uplink commands and sets shared
  flags/setpoints the control loop consumes next iteration (`g_armed`, `g_estop`,
  `g_setpoint[2]`, gains, dump‑request).
- **Data format:**
  - **v1 — reuse the existing text lines** (`it=… roll=… u=[…]`, `flow:`, `walls[]`) over UDP →
    the existing Python parsers work unchanged. Fastest to stand up.
  - **v2 — compact binary struct** (extend the flightlog record: t, attitude, z, vz, vx/vy,
    raw‑flow + est‑vel, u[4], flags, battery, link‑stats) → less bandwidth + parse overhead.

## 3. Safety (why the uplink matters)
- **Remote kill:** an `ESTOP` command → sets `g_estop` → `send_control()` forces all motors to
  0 (latched). This is the manual kill switch we've been missing for untethered flight.
- **Link‑loss watchdog:** while **armed**, if no valid uplink heartbeat for
  `> LINK_TIMEOUT_MS` (configurable) → auto‑cut or auto‑land (policy TBD). Guards against the
  laptop or link dying mid‑flight.
- Both gated on `g_armed` (like the flight watchdog) so pre‑arm link glitches never cut motors.

## 4. Host software (laptop, phased)
1. **Reuse (phase 1):** point `tools/serial_capture.py`, `live_flow_plot.py`, and
   `flow_record.py` at a **UDP socket** instead of the serial port — a one‑function swap
   (`open_noreset()` → a UDP `recvfrom`). Instant wireless telemetry with the same live plots.
2. **Dashboard (phase 2):** a real‑time view — attitude, altitude, velocity, flow (raw vs est),
   motor duties, battery, and **link stats** (rate / loss / RSSI) — plus command buttons
   (arm, **estop**, setpoint, tune). Python GUI or a browser dashboard. Logs to disk (can
   replace the on‑board flightlog for live capture, freeing that RAM).

## 5. RAM — RESOLVED: WiFi fits (2026‑08‑12)
Both spikes are in; on‑chip WiFi telemetry is **viable**.
- **WiFi/net stack cost (spike A):** ~**104 KB** SRAM (mostly a mandatory ~40 KB esp_wifi heap +
  net buffers; Zephyr WiFi builds cleanly for esp32c6 in this fork). **BLE is NOT lighter** —
  ~101 KB (its controller heap is even bigger); BLE only saves *flash*. The radio costs ~100 KB
  either way, so BLE is not a RAM escape hatch.
- **The FC "bloat" is one stale config, not real data (spike B — see `FC_RAM_ANALYSIS.md`):**
  the FC's 96.8 % SRAM is almost entirely a **320 KB main‑thread stack**
  (`CONFIG_MAIN_STACK_SIZE=327680`) — a TinyMPC‑era leftover (TinyMPC's solve needed it; see the
  `ctrl_stack` note at `main.cpp:780`). The PID flight build doesn't link TinyMPC (already
  `--gc-sections`'d out — that ~17 KB win is banked), so the 320 KB is pure waste. Cutting
  `CONFIG_MAIN_STACK_SIZE` to ~32 KB frees **~295 KB** (measured: 96.8 % → 29.4 %), every flight
  feature intact.
- **Net budget:** lean flight build ≈ 128 KB + ~104 KB WiFi ≈ 232 KB of the 437 KB region → fits
  with ~200 KB to spare. **On‑chip WiFi SoftAP is the plan; BLE not needed.**

**Caveats / next steps:**
- The 32 KB main stack is valid **only for `ROSE_USE_PID=1`** (restore the big stack if TinyMPC is
  reselected). The single‑loop control loop runs on the **main thread**, so an undersized stack =
  a crash — **verify the PID loop's stack high‑water at runtime**, then set 24–32 KB.
- Optional extra headroom for a dedicated telemetry build: `CONFIG_VL53L5CX=n` (~20 KB — note
  `ROSE_BUMPER=0` alone does *not* free it; it's driver/DT‑instance data) + drop the flightlog
  (~11 KB, redundant with the live link).
- **Immediate next step:** enable `CONFIG_WIFI` on the FC *with the reduced main stack* and
  re‑measure the real combined FC+WiFi build to confirm the footprint, then start the downlink.

## 6. Roadmap / milestones
1. **RAM spikes (A + B)** → confirm WiFi fits on a lean FC, or pivot to BLE. ✅ *(WiFi fits; combined FC+WiFi build ≈ 71 % SRAM)*
2. **Downlink:** telemetry thread → UDP text @ 50 Hz on a lean FC; adapt one Python tool to
   read UDP → live wireless plots. ✅ **DONE (2026‑08‑16) — reliable ~50 Hz, <0.5 % loss, no crashes,
   reproducible across power cycles. See `docs/TELEMETRY_BRINGUP.md`.**
3. **Uplink + remote kill:** command handler → arm / estop / setpoint; host buttons;
   link‑loss watchdog. *(safety milestone)*
4. **Binary format + dashboard + link stats.**
5. **Range/robustness test** with the Lemon RX MHF3 antenna: indoor latency, packet loss,
   RSSI vs distance.

## 7. Open questions / to confirm
- SoftAP vs station (does the laptop need internet during flights?).
- Telemetry rate (50 vs 100 Hz) and field set (full state vs a curated subset).
- Keep the on‑board flightlog in parallel, or let wireless + host logging replace it (frees RAM)?
- Command reliability: UDP + app‑ACK vs a dedicated TCP control socket.
