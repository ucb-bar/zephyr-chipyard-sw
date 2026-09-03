# riskybird v3 — WiFi Telemetry Bring-up (WORKING)

**Status (2026-08-16): RELIABLE end-to-end.** The drone hosts a 2.4 GHz SoftAP; the host ALFA
AWUS036ACS associates, gets a DHCP lease, and receives **~50 Hz UDP telemetry** on port 14550.

Measured on the bench with the Lemon RX MHF3 antenna installed:
- **3-minute soak:** 49.8 Hz, **0.3 % loss, 0 gaps > 100 ms** (max inter-arrival 44.8 ms).
- **2-minute soak:** 49.7 Hz, 0.5 % loss, 0 gaps > 100 ms.
- **Power-cycle reproducibility:** multiple back-to-back drone resets, each: SoftAP appears → ALFA
  associates on the first attempt → DHCP lease → clean ~49.7 Hz stream, 0 gaps > 100 ms, < 0.5 % loss.
- **No firmware crashes** across dozens of resets and client associations.

> ⚠️ **MOTOR SAFETY.** This telemetry build is compiled with **`-DROSE_MOTORS_INHIBIT=1`**. At boot it
> prints `flight_controller: MOTORS INHIBITED ...` and `send_control()` forces all four PWM channels
> to 0 unconditionally (no arming, no estop path, no chirps). Verify that line appears on every flash.
> The `u=[...]` field in the telemetry line is the **controller's commanded thrust**, NOT the motor
> duty — with the inhibit flag the PWM duty is always 0 regardless of `u`.

---

## 1. Root cause(s) of the earlier "couldn't connect / receive" failure

The earlier stall was **not one bug** — it was a stack of them, each of which alone breaks the link.
In order of the path a packet takes:

1. **SoftAP never started.** `telem_wifi_init()` used only `net_if_get_wifi_sap()`. In the lighter
   single-interface SoftAP mode (`CONFIG_ESP32_WIFI_AP_STA_MODE=n`) the esp32 driver registers
   *neither* a `WIFI_TYPE_SAP` nor a `WIFI_TYPE_STA` managed interface, so `net_if_get_wifi_sap()`
   returns **NULL** → the init bailed with "no WiFi interface" and the AP never came up.

2. **Firmware crashed the instant a client associated (NULL deref).** The `AP_STA_CONNECTED`
   handler read the client MAC from `cb->info`, but `CONFIG_NET_MGMT_EVENT_INFO` was not enabled, so
   `cb->info` is never populated (stays NULL) → NULL dereference → fatal → AP torn down. Classic
   "the SSID appears but I can't connect" symptom.

3. **Firmware crashed ~1 s after association (stack overflow — THE main crash).** The DHCPv4 server
   runs an **ICMP address-conflict probe** and a `k_work_delayable` lease timer on the **system
   workqueue**, whose default stack is only **1024 B**. That deep net path (made deeper by this
   board's `CONFIG_CBPRINTF_FP_SUPPORT`) overflowed the sys-workq stack exactly when a client
   completed DHCP → heap/scheduler corruption → `Load access fault` in `z_riscv_switch`. Confirmed by
   resolving the faulting stack pointer to inside `sys_work_q_stack`.

4. **Host firewall silently dropped the telemetry.** `firewalld` is active and put the ALFA
   interface in the default **`public`** zone, which drops unsolicited inbound UDP. ICMP replies
   still returned via conntrack, so **`ping 192.168.4.1` worked while 0 UDP packets arrived** — a
   very misleading symptom.

5. **Broadcast telemetry is inherently unreliable over WiFi.** The original design broadcast to
   `192.168.4.255`. WiFi broadcast/multicast frames are buffered at the AP and released only at
   **DTIM/beacon** boundaries, and are never MAC-ACKed/retried → bursty delivery (~2 ms bursts then
   ~100 ms gaps), effective rate ~37 Hz, high jitter.

6. **Client-side WiFi power-save** added periodic 100–400 ms telemetry gaps (STA sleeps between
   beacons; AP buffers unicast frames for it).

7. **Host association flakiness.** NetworkManager + the rtw88 USB driver occasionally fail the first
   `connect` after a drone reboot, and a **stale association can survive a drone power-cycle** — the
   host keeps its old lease (ping works) but the rebooted drone never allocated it, so the drone has
   no lease to send to and no telemetry flows.

---

## 2. Fixes applied

### Firmware (`src/telem_wifi.c`, `telem.conf`, `src/main.cpp`)

| # | Fix | Where |
|---|-----|-------|
| 1 | Fall back to `net_if_get_first_wifi()` when `net_if_get_wifi_sap()` is NULL (single-iface AP) | `telem_wifi.c` `telem_wifi_init()` |
| 2 | `CONFIG_NET_MGMT_EVENT_INFO=y` + NULL-guard `cb->info` in the event handler | `telem.conf`, `telem_wifi.c` |
| 3 | `CONFIG_SYSTEM_WORKQUEUE_STACK_SIZE=4096` (was 1024) — **the DHCP-time crash fix**; also bumped `NET_MGMT_EVENT_STACK_SIZE`, `NET_TX/RX_STACK_SIZE` to 4096 and the telem thread stack to 6144 | `telem.conf`, `telem_wifi.c` |
| 4 | **Unicast telemetry to each DHCP-leased client** (broadcast only as a no-lease fallback). The firmware runs the DHCP server, so it enumerates leases with `net_dhcpv4_server_foreach_lease()`, copies the addresses **under the server lock**, then `sendto()`s **outside** the lock | `telem_wifi.c` |
| 5 | Fixed-cadence 50 Hz send loop (sleep to next tick, not "work + sleep(period)") | `telem_wifi.c` |
| 6 | `CONFIG_ESP32_WIFI_AP_STA_MODE=n`, `CONFIG_WIFI_ESP32=y`, `CONFIG_NET_IF_MAX_IPV4_COUNT=1` (deterministic single-interface SoftAP) | `telem.conf` |
| 7 | **`ROSE_MOTORS_INHIBIT` compile flag** — hard motor cut (PWM forced 0 in `send_control`; boot/ready/startup chirps skipped) + a boot log line | `main.cpp` |

> Note: sending unicast **inside** the `foreach_lease` callback (an earlier attempt) held the DHCP
> `server_lock` across a blocking net-TX call → priority inversion + corruption. Always copy the
> lease addresses under the lock and send after it returns.

### Host (documented procedure + `tools/telem_wifi_recv.sh`)

- Put the ALFA interface in firewalld's **`trusted`** zone (via the NM connection's `connection.zone`
  — NM applies it as root, no manual `firewall-cmd`/sudo needed).
- **Disable STA power-save** (`802-11-wireless.powersave 2`).
- `ipv4.never-default yes` so the drone link doesn't steal the host's default route (internet stays
  on the built-in `wlan0`).
- Use **one dedicated managed profile** (`riskybird-telem`) — do NOT use ad-hoc `nmcli device wifi
  connect`, which spawns extra unconfigured profiles (public zone + power-save on).
- **Always associate fresh** (down → up) so the host does a fresh DHCP DISCOVER; verify the link with
  a **ping**, not just a lease (rejects zombie associations); scan until the AP is visible before each
  `connection up`; retry.

---

## 3. Final working configuration

**Firmware build (motor-safe telemetry):**
```bash
source /home/cobble/.claude/projects/-home-cobble-Tools-riskybird/esp32env.sh   # before any set -e
cd /home/cobble/Tools/zephyr-chipyard-sw
west build -b esp32c6_devkitc/esp32c6/hpcore samples/rose_flight_controller -d build_telem -p always -- \
  -DROSE_USE_PID=1 -DROSE_USE_EKF=0 -DROSE_FLOW=0 -DROSE_BUMPER=0 \
  -DROSE_ACTUATE_TIMEOUT_MS=0 -DCTRL_ITERS=0 \
  -DEXTRA_CONF_FILE=telem.conf -DEXTRA_DTC_OVERLAY_FILE=telem.overlay \
  -DEXTRA_CPPFLAGS="-DROSE_MOTORS_INHIBIT=1 -DROSE_TELEM=0"
```
- `ROSE_MOTORS_INHIBIT=1` → motors hard-off (required for unattended bench runs with props on).
- `ROSE_TELEM=0` → no ~100 Hz blocking serial telemetry (that saturates the 115200 UART and starves
  the WiFi TX thread → raised UDP loss from ~0.4 % to ~16 %). UDP telemetry is independent of this.

**Flash + boot (no manual power-cycle needed):**
```bash
west flash -d build_telem
esptool --port /dev/ttyACM0 --before default-reset --after hard-reset flash-id   # boots the app
```
`west flash` alone leaves the C6 in ROM download mode; the standalone esptool reset boots the app.

**Link / radio:** SoftAP SSID `riskybird-<MAC tail>` (this board: `riskybird-d500`), **open**,
2.4 GHz **channel 6**, gateway/DHCP-server **192.168.4.1**, DHCP pool from `.11`. Telemetry is
**UDP unicast to each leased client, port 14550, 50 Hz**, payload = the exact `ROSE_TELEM` text line
(`flight_controller: it=... roll=... z=... tofv=... tofh=... u=[...]`).

---

## 4. RECEIVE PROCEDURE (copy-paste)

### Easiest: the helper script
```bash
cd /home/cobble/Tools/zephyr-chipyard-sw
samples/rose_flight_controller/tools/telem_wifi_recv.sh 120        # associate + listen 120 s
samples/rose_flight_controller/tools/telem_wifi_recv.sh 0          # ... or until Ctrl-C
# args: telem_wifi_recv.sh [seconds] [iface=wlan1] [ssid=auto-detect riskybird-*]
```
It auto-detects the SSID, creates/uses the `riskybird-telem` NM profile with the reliability settings,
associates fresh + robustly (ping-verified, retries), then prints the live telemetry lines.

### Manual (equivalent steps)
```bash
IFACE=wlan1          # the ALFA AWUS036ACS (built-in wifi is wlan0 — leave it for internet)
SSID=riskybird-d500  # from: nmcli device wifi list ifname wlan1 | grep riskybird

# one-time: create the managed profile with the reliability settings
nmcli connection add type wifi con-name riskybird-telem ifname "$IFACE" ssid "$SSID" \
  802-11-wireless.powersave 2 connection.zone trusted \
  ipv4.method auto ipv4.never-default yes connection.autoconnect no

# associate fresh (repeat if no lease)
nmcli device wifi rescan ifname "$IFACE"; sleep 3
nmcli connection up riskybird-telem
ip -4 addr show "$IFACE" | grep inet          # expect 192.168.4.x/24
ping -c3 -I "$IFACE" 192.168.4.1              # expect 0% loss

# listen (either works)
python samples/rose_flight_controller/tools/serial_capture.py --udp 14550 120
#   or:  python3 - <<'EOF'
#   import socket; s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM); s.bind(("",14550))
#   while True: print(s.recv(2048).decode().strip())
#   EOF
```

### If nothing arrives (checklist)
1. `nmcli -f GENERAL.STATE,IP4.ADDRESS device show wlan1` — is there a `192.168.4.x` lease?
2. `firewall-cmd --get-zone-of-interface=wlan1` — must be **`trusted`** (public drops the UDP).
3. `nmcli -g 802-11-wireless.powersave connection show riskybird-telem` — must be `2` (disabled).
4. Ping works but no UDP → stale/zombie association: `nmcli connection down riskybird-telem` then up.
5. Confirm the drone is up: reset it and check serial for `SoftAP enabled` (opening `/dev/ttyACM0`
   with `tools/serial_capture.py` resets+boots the C6; it does NOT need a manual power-cycle here).

---

## 5. Verifying motor safety on a flashed build
```bash
python samples/rose_flight_controller/tools/serial_capture.py 5 /dev/ttyACM0   # resets+boots, prints boot log
# MUST show:  flight_controller: MOTORS INHIBITED (ROSE_MOTORS_INHIBIT=1) -- PWM forced to 0, no chirps, no actuation
```
If that line is absent, the build is NOT motor-safe — do not run it with props attached.
