# RoadSense

**GPS-free position, road-surface and pothole logger on an RC car, using only an ESP32 and an MPU-6050.**

IoT course project, Project & Lab Group 24.

The car estimates where it is by dead reckoning, using only a 6-axis IMU. There is no wheel encoder and no GPS on the car. It classifies the road surface from vibration and flags potholes from vertical acceleration spikes. **By default it streams to a Mac over BLE**: up to 100 poses per second while driving, one every few seconds when parked. A bridge on the Mac republishes the data to a local MQTT broker, which feeds the dashboard (path on a real map), the logger and the calibration tools. Wi-Fi/MQTT (local or cloud TLS) is still available as an alternative.

```
 MPU-6050 ─I²C─▶ ESP32 (edge processing, 200 Hz) ─BLE─▶ Mac: ble_bridge.py ─▶ local Mosquitto ─▶ dashboard (map + XY)
 (accel+gyro)     • driving detector + speed model     (binary frames,       (localhost)       ─▶ logger (JSONL)
                  • gyro heading, bias tracking          50 Hz driving,                         ─▶ calibration fit
                  • surface class, pothole events        0.2 Hz parked,
                  • offline buffer (1 Hz, ~15 min)       raw IMU 200 Hz opt.)
                  • OLED debug pages, LEDs, buttons     (alternative: Wi-Fi ─MQTT─▶ broker)
```

| Measurement | How it is estimated with an IMU only |
|---|---|
| Driving or parked | Motor/road vibration (std of accel magnitude) above a threshold |
| Distance | Calibrated speed model × driving time. Optional: speed scales with vibration |
| Heading | Gyro yaw rate about the measured gravity axis × scale factor. Bias re-estimated whenever parked |
| Position | Dead reckoning (x, y), placed on the map from a start lat/lon and start bearing |
| Surface | 1-s RMS of vertical acceleration → smooth / medium / rough |
| Pothole | Vertical dip followed by a spike within 250 ms, or one large spike |

The equations, the models we compare, and the calibration procedure are in **[docs/CALIBRATION.md](docs/CALIBRATION.md)**.

---

## Repository layout

```
├── firmware/              PlatformIO project for the ESP32 → see firmware/README.md
├── broker/                local Mosquitto: config, ACL, setup.sh (users), run.sh (start on the LAN)
├── dashboard/             Svelte + Vite dashboard (map, XY, charts, link health, reliability test, controls)
│   ├── src/                   App.svelte, components/, lib/ (MQTT state, link stats, CSV export)
│   ├── dist/                  prebuilt copy served by broker/run.sh (no Node needed to use it)
│   └── legacy/                the original single-file dashboard (fallback)
├── tools/
│   ├── ble_bridge.py          BLE ⇄ MQTT gateway on the Mac (default transport); writes system-timestamped CSVs
│   ├── logger.py              record every MQTT message to logs/*.jsonl (+ the same CSVs; for Wi-Fi mode)
│   ├── csvlog.py              CSV session logs with rx_unix / rx_iso system time on every row
│   ├── reliability_report.py  loss, latency, jitter, outages, clock drift from the CSVs (+ plots)
│   ├── simulate_car.py        fake car for testing the broker and dashboard without hardware
│   ├── fit_calibration.py     fit speed and gyro models from start → end runs (LOO cross-validation, plot)
│   ├── analyze_surface.py     suggest surface thresholds from labelled runs
│   └── mqtt_common.py         shared local/cloud connection options
├── calib/runs_template.csv    truth sheet for calibration runs
└── docs/CALIBRATION.md        models, equations, data-collection protocol, error budget
```

## Hardware

The ESP32 DevKit V1 is wired with the course kit pin map ([firmware/include/iot_pins.h](firmware/include/iot_pins.h)):

| Part | ESP32 pins |
|---|---|
| MPU-6050 (I²C 0x68) | SDA 21, SCL 22, 3V3, GND |
| SSD1306 128×64 OLED (SPI) | CS 5, DC 16, RST 17, SCK 18, MOSI 23 |
| LEDs red / yellow / green | 2 / 4 / 15 |
| Buttons BTN1 / BTN2 | 32 / 33 (to GND) |
| Buzzer | 27 |

Mount the IMU **rigidly** (screws, zip ties or hot glue, not foam). Any orientation works.

## Quick start

### 1. Software

* [PlatformIO](https://platformio.org/) (VS Code extension or CLI)
* Python 3.9+:
  ```bash
  python3 -m venv .venv && .venv/bin/pip install -r tools/requirements.txt
  ```
* For a local broker: Mosquitto (`brew install mosquitto` on macOS)

### 2. Pick a transport

**BLE to the Mac (default, `NET_MODE 3`).** You don't need a Wi-Fi network or an online server. Range is about 10–30 m. If the car drives out of range, it buffers poses at 1 Hz and sends them when it reconnects.
```bash
cd broker && ./setup.sh   # once: broker users (rs01, dashboard)
./run.sh                  # local Mosquitto + dashboard on http://localhost:8000
# second terminal:
.venv/bin/python tools/ble_bridge.py          # finds "RoadSense-rs01", forwards everything to the broker
.venv/bin/python tools/ble_bridge.py --raw    # ... plus the 200 Hz raw IMU stream -> logs/raw-*.csv
```
The first time, macOS asks to allow Bluetooth for Terminal (System Settings → Privacy & Security → Bluetooth). With BLE, `secrets.h` only needs to exist; the Wi-Fi values are unused. To run without any broker, use `ble_bridge.py --no-mqtt`, which writes `logs/ble-*.jsonl` that `fit_calibration.py` reads directly.

**A. Wi-Fi to a cloud broker over TLS (recommended when the laptop and car can't share a network).** Use the HiveMQ Cloud free tier, or any MQTT broker with TLS.
1. Create a cluster and two users:
   * `rs01` for the car, allowed to publish `rs/T24/#` and subscribe to `rs/T24/cmd/rs01`,
   * `dashboard`, allowed to subscribe to `rs/T24/#` and publish to `rs/T24/cmd/#`.
2. In `firmware/include/secrets.h` (copied from [secrets.example.h](firmware/include/secrets.example.h)), set `MQTT_HOST` to the cluster host, `MQTT_PORT 8883`, `MQTT_TLS 1` and the `rs01` credentials. Optionally pin the CA with `MQTT_CA_CERT`.
3. In the dashboard, connect to `wss://<cluster-host>:8884/mqtt` as `dashboard`.
4. Run the tools with `--host <cluster-host> --tls --user dashboard --password …`.

**B. Wi-Fi to a local Mosquitto on the laptop.** The laptop and the car must be on the same 2.4 GHz network.
```bash
cd broker
./setup.sh      # once: creates users rs01/dashboard and firmware/include/secrets.h
./run.sh        # starts MQTT :1883 + WebSocket :9001, serves the dashboard on :8000,
                # finds the LAN IP and keeps MQTT_HOST in secrets.h in sync
```
`run.sh` warns if the network isolates devices from each other. Many phone hotspots and all campus Wi-Fi do this; on those, use option A. As a last resort, the car can make its own Wi-Fi: see *Network modes* in [firmware/README.md](firmware/README.md).

### 3. Flash the car

Create `secrets.h` (copy `secrets.example.h`). For Wi-Fi modes, set `WIFI_SSID` / `WIFI_PASS`; the network must be **2.4 GHz**. Then:
```bash
cd firmware
pio run -t upload && pio device monitor
```
Keep the car still for the first 2 seconds (gyro calibration). The OLED header shows `B+` once the BLE bridge is connected (`W+ M+` in Wi-Fi mode). The NET page shows the live transmit rate.

### 4. Dashboard and logging

* **Dashboard:** `run.sh` serves the prebuilt Svelte dashboard at http://localhost:8000. Enter the broker URL (`ws://localhost:9001`) and the `dashboard` login. The login is remembered, and the page reconnects after a reload. Panels:
  * **Map:** pick the start point and bearing, then drive. The path is coloured by surface. Click the true position to get the error, or to re-anchor the car.
  * **Local XY plot:** with a "follow car" option.
  * **Live tiles:** pose, raw integrals, and distance per surface.
  * **Time series (uPlot):** speed and heading, vibration, x/y, and received frames per second, over a 30 s – 5 min window.
  * **Link health:** receive rate, loss, skipped/late frames, latency and inter-arrival percentiles, outages, and the car's own counters.
  * **Reliability test:** start/stop, live summary, CSV export with system timestamps, copy summary.
  * **Controls:** calibration commands, `txHz`/`parkS`, raw stream toggle, any command, and a parameter table.
  * **Potholes:** table plus ≥3-within-5 m alerts.
  * **Event log:** with filters.
* **Developing the dashboard:**
  ```bash
  cd dashboard && npm install && npm run dev
  ```
  This gives hot reload on http://localhost:5173. After changes, run `npm run build` to refresh `dist/`.
* **Logger:**
  ```bash
  .venv/bin/python tools/logger.py
  ```
  Add `--host … --tls …` for a cloud broker. Logs go to `logs/session-*.jsonl`.
* **No hardware:**
  ```bash
  .venv/bin/python tools/simulate_car.py
  ```

---

## Project stages

| Stage | Goal | Where |
|---|---|---|
| 0 | Wire the kit, build the firmware | [firmware/README.md](firmware/README.md) |
| 1 | Broker and dashboard working with the simulator | Quick start above |
| 2 | Car online: OLED `W+ M+`, dashboard shows "car: online" | Quick start |
| 3 | Driving detector: tune `movTh` on the OLED IMU page (bar vs tick) | firmware README → Tuning |
| 4 | Quick single-run calibration: `dcal` (distance) and `tcal` (turn) | firmware README → Commands |
| 5 | **Proper calibration: repeated start → end runs, model fit, LOO error** | [docs/CALIBRATION.md](docs/CALIBRATION.md) |
| 6 | Map accuracy: drive loops, click the true end on the map → error in m and % | Dashboard |
| 7 | Surface classes: labelled runs, then `tools/analyze_surface.py` | firmware README |
| 8 | Pothole thresholds (`potNeg`, `potPos`, `potBig`) | firmware README |
| 9 | Reliability and power: reconnects, buffering, current vs budget | Status topic, `queued` / `dropped` |

### Calibration in short

Drive from the **same start point** to many end points (straight runs, left and right turns, L-shapes), and measure every true end point (tape measure or Google Maps). Then:

```bash
python tools/fit_calibration.py logs/session-*.jsonl --runs calib/runs.csv --plot
```

The script fits these models with Levenberg–Marquardt and ranks them by leave-one-out error:
* constant speed $L = v_0 T$,
* vibration-linear speed $L = v_0 T + k_v\!\int\!\sigma_z\,dt$,
* gyro scale (and residual bias) $\theta = s_g\Psi - \beta t$.

It prints the `set …` commands for the car and saves `calib/fit.png`. All the details are in [docs/CALIBRATION.md](docs/CALIBRATION.md).

---

## Reliability testing

Every row that the bridge (BLE) or logger (Wi-Fi) writes carries the **Mac's system receive time** (`rx_unix` in epoch seconds, `rx_iso` in local ISO-8601 with ms), next to the car's own clock (`t_ms`):

| File | Rows |
|---|---|
| `logs/pose-<stamp>.csv` | every pose |
| `logs/pothole-<stamp>.csv` | every pothole |
| `logs/link-<stamp>.csv` | connect/disconnect, car status, command and info events |
| `logs/raw-<stamp>.csv` | 200 Hz raw IMU (bridge `--raw`) |

Suggested test procedure:
1. Start `broker/run.sh` and `tools/ble_bridge.py`. In the dashboard, click **Start test**.
2. Drive normally for 10–30 min. Include parked periods, driving away until the link drops, and coming back.
3. Click **Stop test**, then **Export test CSV** (the browser's view), or use the bridge CSVs (the authoritative record):
   ```bash
   .venv/bin/python tools/reliability_report.py logs/pose-<stamp>.csv --plot
   ```
   The report gives:
   * received rate while driving and parked,
   * frames **lost in transit** vs **skipped on purpose** by the car's 1 Hz offline buffer (told apart by the car's clock),
   * duplicates, out-of-order frames and reboots,
   * inter-arrival p50/p95/p99 and jitter,
   * one-way **latency above the best case** (drift-corrected, so the clocks don't need to be synchronised),
   * frames delivered late from the buffer,
   * car-vs-Mac **clock drift** in ppm,
   * a list of **outages**, and the car's queued/dropped/reconnect counters.

   `--plot` saves `logs/reliability-<stamp>.png` with the rate over time (outages shaded), latency, and an inter-arrival histogram. Use `--from HH:MM:SS --to HH:MM:SS` to analyse part of a session.

## MQTT topics and messages

The base topic is `rs/T24`. Every message is JSON. Data messages use QoS 1.

| Topic | When | Payload |
|---|---|---|
| `rs/T24/pose` | `txHz` while driving (default 50), every `parkS` s parked (default 5), and at once on start/stop | `dev, ts, t_ms, seq, x_m, y_m, hdg_deg, spd_mps, surf, conf, dist_m, vib_g, acc_sd, mv, label, drv_s, yaw_raw, vib_int` |
| `rs/T24/pothole` | on event | `dev, ts, seq, x_m, y_m, z_neg_g, z_pos_g, pitch_dps` |
| `rs/T24/status` | retained; every 5 s driving / 30 s parked (BLE), 10 s (Wi-Fi); `offline` on disconnect | `dev, state, link, mtu, up_s, queued, dropped, reconnects, imu_err, ble_err, tx_hz` |
| `rs/T24/info` | command replies, calibration results | text |
| `rs/T24/cmd/rs01` | dashboard → car (development only) | text command, e.g. `reset` |

* `hdg_deg` is counter-clockwise from the start direction.
* `x` points along the start direction and `y` to the left.
* `drv_s`, `yaw_raw` and `vib_int` are the raw integrals the calibration fit uses.
* `ts` is Unix time when NTP (or the `time` sync that the bridge and dashboard send) is available, otherwise seconds since boot. `t_ms` is the car's millisecond clock; use it for timing at high pose rates.
* The BLE binary frame format is documented in [firmware/src/ble_link.cpp](firmware/src/ble_link.cpp) and decoded in [tools/ble_bridge.py](tools/ble_bridge.py).

## Security (STRIDE summary)

* **Spoofing:** per-device MQTT users. **Elevation of privilege:** topic ACLs ([broker/acl](broker/acl)); the car can write only its own topics.
* **Tampering:** TLS to cloud brokers (`MQTT_TLS 1` + `MQTT_CA_CERT`). Every message carries `seq` and `ts`.
* **Information disclosure:** secrets live in `firmware/include/secrets.h` and `broker/credentials.txt`, both git-ignored. Never commit them.
* **Denial of service:** 10-minute on-device buffer, a Last Will for offline detection, and a reconnect counter in the status topic.
* The inbound command topic is for development. Set `ALLOW_MQTT_CMDS 0` in `firmware/include/config.h` for the final build.

## Before pushing to GitHub

* `git status` must not list `secrets.h`, `credentials.txt`, `passwd` or `logs/`. The `.gitignore` covers them.
* If credentials were ever committed, rotate them (`broker/setup.sh --force`, and new users on the cloud broker).
