#!/usr/bin/env bash
#
# riskybird GROUND STATION -- one command does it all:
#   1. ensures the drone WiFi auto-connect profile exists + is active on the SECONDARY adapter
#      (the primary adapter / eduroam + the internet default route are left untouched),
#   2. waits for the drone to answer,
#   3. opens the dashboard in your browser,
#   4. runs the telemetry + command bridge (foreground; Ctrl-C stops everything).
#
#   usage: riskybird_gcs.sh [--ssid riskybird-5668] [--http 8080] [--no-browser] [extra panel args...]
#     --ssid        drone SoftAP SSID            (default riskybird-5668)
#     --http        dashboard port              (default 8080)
#     --no-browser  don't auto-open the browser
#     extra args    forwarded to riskybird_panel.py (e.g. --drone 192.168.4.1 --telem-port 14550)
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
SSID="riskybird-5668"
HTTP_PORT=8080
OPEN_BROWSER=1
PANEL_ARGS=()
while [ $# -gt 0 ]; do
  case "$1" in
    --ssid)       SSID="${2:?}"; shift 2;;
    --http)       HTTP_PORT="${2:?}"; shift 2;;
    --no-browser) OPEN_BROWSER=0; shift;;
    *)            PANEL_ARGS+=("$1"); shift;;
  esac
done

echo "== riskybird ground station =="

# 1) WiFi: create the autoconnect profile on first run, else just make sure it's active.
if ! nmcli -t -f NAME connection show 2>/dev/null | grep -qx "$SSID"; then
  echo "[gcs] first run -- creating WiFi profile for '$SSID'"
  "$HERE/riskybird_wifi_setup.sh" "$SSID" || true
elif nmcli -t -f NAME connection show --active 2>/dev/null | grep -qx "$SSID"; then
  echo "[gcs] '$SSID' already connected"
else
  IFACE="$(nmcli -g connection.interface-name connection show "$SSID" 2>/dev/null)"
  echo "[gcs] activating '$SSID'${IFACE:+ on $IFACE} ..."
  nmcli connection up "$SSID" ${IFACE:+ifname "$IFACE"} >/dev/null 2>&1 || true
fi

# 2) wait (non-fatal) for the drone to answer; the panel runs regardless.
printf "[gcs] waiting for drone 192.168.4.1 "
ok=0
for _ in $(seq 1 20); do
  ping -c1 -W1 192.168.4.1 >/dev/null 2>&1 && { ok=1; break; }
  printf "."; sleep 1
done
[ "$ok" = 1 ] && echo " reachable" || echo " (not up yet -- power the drone; the panel will catch it)"

# 3) open the dashboard (best-effort), then run the bridge in the foreground.
URL="http://127.0.0.1:${HTTP_PORT}"
if [ "$OPEN_BROWSER" = 1 ] && command -v xdg-open >/dev/null 2>&1; then
  ( sleep 1.5; xdg-open "$URL" >/dev/null 2>&1 || true ) &
fi
echo "[gcs] dashboard -> $URL   (Ctrl-C to stop)"
exec python3 "$HERE/riskybird_panel.py" --http "$HTTP_PORT" ${PANEL_ARGS[@]+"${PANEL_ARGS[@]}"}
