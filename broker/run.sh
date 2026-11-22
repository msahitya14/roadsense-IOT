#!/usr/bin/env bash
# Start the local RoadSense broker on the LAN (Ctrl+C to stop).
#   - listens on every interface: 1883 (MQTT, ESP32) and 9001 (WebSockets, dashboard)
#   - finds the laptop's LAN IP and warns if the network can't carry device-to-device traffic
#   - keeps MQTT_HOST in firmware/include/secrets.h in sync with that IP (reflash if it changes)
#   - self-tests a login on <LAN IP>:1883, the same path the ESP32 uses
set -uo pipefail
cd "$(dirname "$0")"
ROOT="$(cd .. && pwd)"
SECRETS="$ROOT/firmware/include/secrets.h"
[[ -f passwd && -f credentials.txt ]] || { echo "Run ./setup.sh first"; exit 1; }
MOSQ="$(command -v mosquitto || echo /opt/homebrew/sbin/mosquitto)"
SUB="$(command -v mosquitto_sub || echo /opt/homebrew/bin/mosquitto_sub)"
DASH_PASS="$(sed -n 's/^DASH_PASS=//p' credentials.txt)"

red()   { printf '\033[1;31m%s\033[0m\n' "$*"; }
green() { printf '\033[1;32m%s\033[0m\n' "$*"; }
yellow(){ printf '\033[1;33m%s\033[0m\n' "$*"; }

# ---- 1. find a usable LAN IPv4 (skip loopback, link-local 169.254.x, and 192.0.0.x /32 links)
LAN_IP=""; BAD_IP=""
for ifc in $(ifconfig -l); do
  line="$(ifconfig "$ifc" 2>/dev/null | awk '/inet /{print $2, $4; exit}')"
  [[ -z "$line" ]] && continue
  ip="${line%% *}"; mask="${line##* }"
  case "$ip" in 127.*|169.254.*) continue;; esac
  if [[ "$ip" == 192.0.0.* || "$mask" == "0xffffffff" ]]; then BAD_IP="$ip ($ifc)"; continue; fi
  LAN_IP="$ip"; LAN_IF="$ifc"; break
done

AP_MODE=0
[[ "$LAN_IP" == 192.168.4.* ]] && AP_MODE=1

echo "=============================================================="
if [[ $AP_MODE -eq 1 ]]; then
  green "ESP32 AP MODE: laptop joined the car's Wi-Fi (laptop IP $LAN_IP, car at 192.168.4.1)."
  green "The car finds this broker by itself. No reflash needed. No internet: map tiles won't load."
elif [[ -n "$LAN_IP" ]]; then
  green "LAN IP: $LAN_IP on $LAN_IF  ->  ESP32 must use MQTT_HOST \"$LAN_IP\""
else
  red "No usable LAN IPv4 found. ${BAD_IP:+Current address: $BAD_IP}"
  red "This network is point-to-point / client-isolated: the ESP32 cannot reach this laptop."
  yellow "Fix: join a network that gives a normal subnet, e.g."
  yellow "  - iPhone hotspot with 'Maximize Compatibility' ON (laptop gets 172.20.10.x)"
  yellow "  - Android hotspot (192.168.x.x) or any home/travel router"
  yellow "  - OR no hotspot at all: ESP32 AP mode. Power the car on (it falls back to AP after 20 s,"
  yellow "    or hold BTN1 at power-on), join Wi-Fi 'RoadSense-rs01' (pass roadsense24), re-run ./run.sh"
  yellow "Then re-run ./run.sh. Starting the broker anyway (localhost dashboard/simulator still work)."
fi

# ---- 2. keep secrets.h in sync (hotspot mode only; in AP mode the car discovers the laptop)
if [[ -n "$LAN_IP" && $AP_MODE -eq 0 && -f "$SECRETS" ]]; then
  CUR="$(sed -n 's/^#define MQTT_HOST *"\([^"]*\)".*/\1/p' "$SECRETS")"
  if [[ "$CUR" != *.local && "$CUR" != "$LAN_IP" ]]; then
    sed -i '' "s/^#define MQTT_HOST .*/#define MQTT_HOST   \"$LAN_IP\"   \/\/ updated by run.sh/" "$SECRETS"
    yellow "secrets.h MQTT_HOST was \"$CUR\" -> now \"$LAN_IP\".  REFLASH the ESP32:"
    yellow "  cd firmware && pio run -t upload"
  else
    green "secrets.h MQTT_HOST = \"$CUR\" (ok)"
  fi
  SSID="$(sed -n 's/^#define WIFI_SSID *"\([^"]*\)".*/\1/p' "$SECRETS")"
  echo "ESP32 Wi-Fi in secrets.h: \"$SSID\"  (laptop must be on the same network)"
fi

# ---- 3. free the ports from an old broker
if lsof -nP -iTCP:1883 -sTCP:LISTEN >/dev/null 2>&1; then
  yellow "Port 1883 already in use - stopping old broker (brew service or previous run.sh)"
  brew services stop mosquitto >/dev/null 2>&1 || true
  pkill -f "mosquitto -c" 2>/dev/null || true
  sleep 1
fi

# ---- 4. start, self-test on the LAN address, then stay in the foreground
mkdir -p data
"$MOSQ" -c mosquitto.conf &
PID=$!
WEB_PID=""
if ! lsof -nP -iTCP:8000 -sTCP:LISTEN >/dev/null 2>&1; then   # serve the dashboard too
  python3 -m http.server 8000 --bind 0.0.0.0 --directory "$ROOT/dashboard" >/dev/null 2>&1 &
  WEB_PID=$!
fi
trap 'kill $PID $WEB_PID 2>/dev/null; exit 0' INT TERM
sleep 1
TEST_HOST="${LAN_IP:-127.0.0.1}"
if "$SUB" -h "$TEST_HOST" -p 1883 -u dashboard -P "$DASH_PASS" -t 'rs/T24/status' -C 1 -W 2 -i runsh-selftest >/dev/null 2>&1 \
   || [[ $? -eq 27 ]]; then   # 27 = timed out waiting for a message, i.e. login worked
  green "Self-test OK: broker accepts logins on $TEST_HOST:1883"
else
  red "Self-test FAILED on $TEST_HOST:1883 (see broker log above)"
fi
echo "Dashboard: http://localhost:8000  (ws://localhost:9001, or ws://$TEST_HOST:9001 from another device)"
echo "=============================================================="
wait $PID
