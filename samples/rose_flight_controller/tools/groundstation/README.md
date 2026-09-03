# riskybird v3 — ground-station panel

A self-contained **controller / visualizer** for the WiFi telemetry link. Stdlib-only Python
(no pip) + a browser dashboard.

```
   drone SoftAP "riskybird-<id>"                 your laptop
   ├─ UDP :14550  telemetry  (50 Hz)  ─────────▶  riskybird_panel.py ──▶ browser dashboard
   └─ UDP :14551  commands            ◀─────────  (ESTOP / DISARM / HOVER_Z / PING)
```

## Run it

1. **Flash a WiFi build** of the FC (telem.conf + telem.overlay):
   ```
   west build -b esp32c6_devkitc/esp32c6/hpcore samples/rose_flight_controller -d build_fc_telem -- \
     <your flight knobs> -DEXTRA_CONF_FILE=telem.conf -DEXTRA_DTC_OVERLAY_FILE=telem.overlay
   ```
   On boot the console prints `SoftAP ... SSID 'riskybird-<id>'` and `command uplink on UDP :14551`.

2. **Join the drone's AP** on your laptop: SSID `riskybird-<id>` (open network). You'll get a
   `192.168.4.x` DHCP lease; the drone is `192.168.4.1`.
   (For a dedicated adapter, `tools/telem_wifi_recv.sh` associates the ALFA + verifies the link.)

3. **Run the panel and open the URL:**
   ```
   python3 tools/groundstation/riskybird_panel.py
   # -> http://127.0.0.1:8080
   ```
   Options: `--drone 192.168.4.1` `--http 8080` `--telem-port 14550` `--cmd-port 14551`
   `--bind 0.0.0.0` (to view from another device).

## Dashboard

- **3D — estimated state** — the drone rendered at its estimated position (x,y,z) + attitude above a
  checkerboard ground plane, with motor-colored rotors, a heading nose, a velocity arrow, and a
  drop-line/shadow (reads position + altitude at a glance). Drag to orbit, scroll to zoom, `R` to
  reset the view. The whole estimated state in one picture — including the horizontal drift.
- **State banner** — DISARMED/READY · CALIBRATING · ARMING · ARMED · **ESTOP** (color-coded).
- **Attitude** — artificial horizon (roll/pitch) + yaw.
- **Horizontal drift & velocity** — top-down vx/vy vector (rings at 0.2/0.5/1.0 m/s) + vz. The
  fastest way to see the hover drift.
- **Altitude** — estimated z vs target `zsp` vs raw ToF.
- **Motor duty** — the 4 motors in X-layout (FL/FR/RL/RR = m3/m0/m2/m1).
- **Battery** (from AIN5 sense; shows `--` when battery sense is off), **link stats** (rate /
  quality / packets / age), and a **trend** strip (vx/vy/z).

## Commands (uplink → :14551)

| button | wire | effect |
|---|---|---|
| **ESTOP** (or press `E`) | `ESTOP` | latch `g_estop` → all motors off (cleared by **RESET** or a chip reset) |
| DISARM | `DISARM` | clear `g_armed` (autoflight can re-arm via the gate) |
| **↺ RESET** | `RESET` | **soft reset without a chip reset**: clears ESTOP, disarms, re-inits the estimator/controller, and re-runs the gyro-bias cal (drone goes back to CALIBRATING → READY). The recover-and-recalibrate button. |
| HOVER − / + / SET | `HOVER_Z <mm>` | set altitude setpoint (non-autoflight hover builds; autoflight overwrites it from the profile) |
| PING | `PING` | liveness → `PONG` |

Every command shows the firmware's ACK. There is **no remote force-ARM** by design — arming stays
on the on-board gate.

> ⚠️ **Test the uplink on the bench first** (board flipped / props off): join the AP, hit ESTOP,
> confirm the console prints `CMD ESTOP` and motors are cut, before relying on it in flight.
