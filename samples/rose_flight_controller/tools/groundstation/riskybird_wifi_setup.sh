#!/usr/bin/env bash
#
# Set up AUTOMATIC connection to a riskybird drone SoftAP on a SECONDARY WiFi adapter, while the
# PRIMARY adapter keeps its normal network (e.g. eduroam) + the internet default route. Run this
# once per machine; NetworkManager then auto-joins the drone whenever it's powered + in range.
#
# Requires: a 2nd WiFi adapter (the drone can host up to 4 clients, so multiple people can each run
# this on their own laptop + adapter). NetworkManager (nmcli). No root needed.
#
#   usage: riskybird_wifi_setup.sh [ssid] [iface]
#     ssid   drone SoftAP SSID   (default: riskybird-5668  -- the v3/FPGA board)
#     iface  WiFi iface for the drone (default: auto = the WiFi dev NOT holding the default route)
#
# What the profile does (the reliability settings, learned the hard way -- see TELEMETRY_BRINGUP.md):
#   connection.autoconnect yes  : join automatically when the SSID appears
#   connection.interface-name   : PIN to the secondary adapter so it never touches the primary radio
#   ipv4.never-default yes       : the drone advertises itself as gateway; this stops it stealing the
#                                  host default route -> the primary adapter keeps the internet
#   connection.zone trusted     : firewalld's default "public" zone silently DROPS inbound UDP
#   802-11-wireless.powersave 2  : disable STA power-save (else periodic 100-400 ms telemetry gaps)
#   ipv6.method disabled         : the drone is IPv4-only
set -u
SSID="${1:-riskybird-5668}"
IFACE="${2:-}"

DEFDEV=$(ip route show default 2>/dev/null | awk '/default/{print $5; exit}')
if [ -z "${IFACE}" ]; then
  IFACE=$(nmcli -t -f DEVICE,TYPE device status 2>/dev/null | awk -F: -v d="${DEFDEV}" '$2=="wifi" && $1!=d {print $1; exit}')
fi
if [ -z "${IFACE}" ]; then
  echo "ERROR: no secondary WiFi interface found."
  echo "       You need a 2nd WiFi adapter (the primary '${DEFDEV:-?}' stays on your normal network)."
  echo "       Plug it in, then re-run:  $0 ${SSID} <iface>"
  exit 1
fi

echo "== riskybird auto-connect setup =="
echo "drone SSID : ${SSID}"
echo "drone iface: ${IFACE}   (primary '${DEFDEV:-none}' keeps the internet default route)"

CON="${SSID}"   # profile name == SSID, so multiple boards can each have their own profile
nmcli connection delete "${CON}" >/dev/null 2>&1
nmcli connection add type wifi con-name "${CON}" ifname "${IFACE}" ssid "${SSID}" \
  connection.autoconnect yes connection.autoconnect-priority 10 \
  connection.zone trusted \
  802-11-wireless.powersave 2 \
  ipv4.method auto ipv4.never-default yes ipv4.route-metric 700 \
  ipv6.method disabled >/dev/null || { echo "ERROR: nmcli add failed"; exit 1; }
echo "profile '${CON}' created (autoconnect on). It will join ${SSID} on ${IFACE} whenever the drone is up."

echo "-- bringing it up now (ok if the drone is off; autoconnect will catch it later) --"
if timeout 40 nmcli connection up "${CON}" ifname "${IFACE}" >/dev/null 2>&1; then
  ip -4 addr show "${IFACE}" | awk '/inet /{print "  lease : "$2}'
  if ping -c1 -W2 -I "${IFACE}" 192.168.4.1 >/dev/null 2>&1; then
    echo "  drone : 192.168.4.1 reachable ✓"
  else
    echo "  drone : not reachable yet"
  fi
else
  echo "  not up yet (drone off / out of range) -- autoconnect will join it when it appears."
fi
echo "done. Run the dashboard:  python3 $(dirname "$0")/riskybird_panel.py"
