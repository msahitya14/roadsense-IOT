# RoadSense firmware (ESP32 + MPU-6050)

This is the PlatformIO / Arduino-ESP32 firmware for the RoadSense car. It does all the processing on the ESP32 at 200 Hz. Poses are sent at `txHz` (default 50 per second) while driving and every `parkS` seconds (default 5) while parked. Potholes are sent as they happen. The default transport is **BLE** to a Mac running `tools/ble_bridge.py`; Wi-Fi/MQTT is also supported. Project overview: [../README.md](../README.md). Models and equations: [../docs/CALIBRATION.md](../docs/CALIBRATION.md).

## Build and flash

```bash
cp include/secrets.example.h include/secrets.h   # then edit, or run ../broker/setup.sh
pio run                     # build
pio run -t upload           # flash
pio device monitor          # serial console at 115200; type commands and press Enter
```

`platformio.ini` targets `esp32dev` on `espressif32@6.9.0`, the same as the course labs. Library dependencies:

| Library | Purpose |
|---|---|
| `256dpi/MQTT` | MQTT with **QoS 1 publish** and a Last Will. The labs' PubSubClient can only publish at QoS 0. |
| `adafruit/Adafruit SSD1306` + `Adafruit GFX` | OLED |
| `h2zero/NimBLE-Arduino` | BLE link: a much smaller stack than the built-in Bluedroid |

`board_build.partitions = huge_app.csv` (3 MB app, no OTA, the same as Lab 4) is needed because the firmware includes both Wi-Fi and BLE.

## Source layout

| File | Contents |
|---|---|
| `include/config.h` | device ID, topics, pins (from `iot_pins.h`), sample rate, network mode, feature switches |
| `include/iot_pins.h` | course kit pin map, copied unchanged from the labs |
| `include/secrets.h` | Wi-Fi and MQTT credentials (**git-ignored**); template in `secrets.example.h` |
| `src/imu.cpp` | register-level MPU-6050 driver: 200 Hz, DLPF ~94 Hz, ±8 g, ±500 dps |
| `src/estimator.cpp` | **all the algorithms**: calibration, driving detector, bias tracking, heading, speed models, position, surface, potholes, serial/MQTT commands, NVS parameters |
| `src/main.cpp` | FreeRTOS tasks, transports (BLE / Wi-Fi hotspot / AP), MQTT (plain / TLS), offline queues, JSON messages |
| `src/ble_link.cpp` | NimBLE GATT server (Nordic-UART UUIDs), binary frame encoding, notify flow control |
| `src/debug_ui.cpp` | OLED pages, LEDs, buttons, buzzer |

## Runtime architecture

```
core 1, prio 3  sensorTask   every 5 ms: read IMU → Estimator::update()
                              → PoseMsg (txHz driving / parkS parked) → qPose (900; offline only 1 Hz is kept)
                              → pothole       → qPot
                              → raw samples (200 Hz, `raw on`, BLE) → qRaw
                              ← commands      ← qCmd (serial, BLE, MQTT, buttons)
core 1, prio 2  uiTask       every 20 ms: buttons, LEDs, buzzer; OLED redraw at 5 Hz
core 1, prio 1  loop()       serial input; BLE: drain queues → notifications (potholes > poses > raw > info > status)
                              Wi-Fi: connect, drain queues → MQTT publish
core 0          Wi-Fi / lwIP stack
```

A slow reconnect blocks only `loop()`. The integration never stops. Poses recorded while offline keep their original `ts`/`t_ms`/`seq` and are sent once the connection is back.

**BLE throughput.** The car asks for a 15–30 ms connection interval (15 ms is the macOS minimum) and an MTU of 247. Each pose is a 71-byte binary frame, so 50 poses/s is about 3.6 kB/s, and the 200 Hz raw stream adds about 2.7 kB/s. Both are far below what BLE 4.2 can carry. A frame is taken off its queue only after NimBLE has a free buffer for it (`os_msys_num_free`), so a slow receiver delays poses instead of silently losing them. The live rate is shown on the OLED NET page and in the bridge's `[link]` lines.

## Configuration

`include/config.h`:

| Setting | Default | Meaning |
|---|---|---|
| `DEVICE_ID` / `TOPIC_BASE` | `rs01` / `rs/T24` | MQTT client ID and topics |
| `NET_MODE` | `3` | 3 = BLE to the Mac bridge, 0 = Wi-Fi hotspot, 1 = Wi-Fi access point, 2 = hotspot with AP fallback |
| `USE_OLED`, `USE_LEDS`, `USE_BUTTONS`, `USE_BUZZER` | 1 | disable any kit part that isn't wired |
| `ALLOW_MQTT_CMDS` | 1 | accept commands on `rs/T24/cmd/rs01`; set to 0 for the final build (STRIDE E) |
| `SAMPLE_HZ`, `POSE_PERIOD_S`, `POSE_QUEUE_LEN` | 200, 1, 600 | rates and buffer size |

`include/secrets.h`: `WIFI_SSID`, `WIFI_PASS`, `MQTT_HOST`, `MQTT_PORT`, `MQTT_TLS`, `MQTT_USER`, `MQTT_PASS`, optional `MQTT_CA_CERT` and `AP_PASS`.

### Network modes

* **BLE (`NET_MODE 3`, default).** Wi-Fi is switched off. The car advertises as `RoadSense-rs01`. On the Mac, run `broker/run.sh` and `tools/ble_bridge.py`. The bridge decodes the frames, publishes the usual JSON topics to the local broker, forwards dashboard commands to the car, sends `time` on connect, and writes `logs/raw-*.csv` when started with `--raw`. Range is about 10–30 m, with +9 dBm transmit power.
* **Hotspot / router (`NET_MODE 0`).** It must be **2.4 GHz**. A Windows Mobile Hotspot has to be set to 2.4 GHz explicitly. With a local broker, the laptop must be on the same network, and that network must allow devices to reach each other.
* **Cloud broker over TLS (`MQTT_TLS 1`, port 8883).** It works on any network with internet, which avoids the device-isolation problem. Without `MQTT_CA_CERT` the connection is still encrypted, but the broker's certificate isn't verified, and a warning is printed at boot.
* **Access point (`NET_MODE 1`, or 2 with fallback, or hold BTN1 at power-on in mode 1/2).** The car creates Wi-Fi `RoadSense-rs01` (password `roadsense24` unless `AP_PASS` is set). The laptop joins it and runs `broker/run.sh`, and the car finds the broker at whatever IP its DHCP server gave the laptop. Use this for offline demos. The laptop then has no internet unless it also has a wired or USB connection, and macOS may switch back to a known Wi-Fi network unless Auto-Join is turned off.

## On-car debugging

**OLED.** Press BTN1 to cycle the pages. The header shows `B±` (BLE bridge connected), or `W±`/`A±` (Wi-Fi / AP) plus `M±` (MQTT). The bottom line shows driving state, pothole count and label, or a 2-s banner for events.

| Page | Shows |
|---|---|
| DRIVE | x, y, heading with a compass arrow, distance, speed, surface, vibration |
| IMU | raw accel and gyro, bias, yaw rate, temperature, **accSd bar vs the `movTh` tick** |
| CAL | v0, gyroScale, movTh, `dcal`/`tcal` live state |
| NET | BLE: name, connected/advertising, MTU, **live pose/s**, notify errors. Wi-Fi: SSID, IP, broker, RSSI, last MQTT error. Both: queued/dropped, IMU errors |

**LEDs:**
* green: driving,
* yellow: MQTT down (solid) or sending buffered data (blinking),
* red: pothole flash, or blinking while calibrating.

**Buttons:** BTN2 tap resets the pose; hold it 2 s to recalibrate the gyro. **Buzzer:** a short beep on each pothole.

## Commands

Commands are accepted from the serial monitor, from the dashboard, or on `rs/T24/cmd/rs01`. Type `help` on the car for the list.

| Command | Effect |
|---|---|
| `show` | print all parameters as JSON |
| `calib` | re-measure the gyro bias and gravity; the car must be still for 2 s |
| `reset` | pose, distance, raw integrals and pothole count back to 0 |
| `anchor <x> <y> [hdg]` | re-anchor the pose at a known waypoint |
| `dcal start` / `dcal end <m>` | quick distance calibration from one straight run → `v0`, `vibRef` |
| `tcal start` / `tcal end <deg>` | quick turn calibration → `gyroScale` |
| `set <key> <value>` | change a parameter; it's saved to flash (NVS) |
| `label <name>` | tag the following poses (`run01`, `asphalt`, …) for calibration or data collection |
| `stream <hz>` | raw IMU CSV on serial: `R,ms,ax,ay,az,gx,gy,gz` (max 100 Hz) |
| `raw on` / `raw off` | 200 Hz raw IMU over BLE, saved by the bridge to `logs/raw-*.csv` |
| `time <unix>` | set the clock (the bridge and dashboard do this automatically) |
| `defaults` | restore the default parameters |

### Parameters (`set <key> <value>`)

| Key | Default | Meaning |
|---|---|---|
| `v0` | 0.80 | m/s. Cruise speed (S0), or the intercept of the vibration model (S2) |
| `spdMode` | 0 | 0: $v=v_0$ · 1: $v=v_0\,\sigma_z/\sigma_{ref}$ · 2: $v=v_0+k_v\sigma_z$ |
| `vibRef`, `kVib` | 0.15, 0 | parameters of the vibration speed models |
| `gyroScale` | 1.0 | heading = gyroScale × raw yaw integral |
| `txHz` | 50 | pose messages per second while driving (1–100) |
| `parkS` | 5 | seconds between pose messages while parked |
| `movTh` | 0.035 g | driving threshold on the accel-magnitude std |
| `surfT1`, `surfT2` | 0.08, 0.20 g | surface class boundaries on the 1-s vertical RMS |
| `potNeg`, `potPos`, `potBig` | −0.6, 0.9, 1.8 g | pothole dip, spike and single-spike thresholds |

## Tuning order

1. **`movTh`:** on the IMU page, the accSd bar must stay left of the tick when parked with the motor off, and go past it when driving.
2. **Quick calibration:** run `dcal` on a taped straight line and `tcal` with 2 full circles, to get reasonable starting values.
3. **Proper calibration:** do repeated start → end runs and run `tools/fit_calibration.py` ([docs/CALIBRATION.md](../docs/CALIBRATION.md)). Send the `set …` lines it prints.
4. **Surface:** use `label asphalt|gravel|grass`, driving about 1 min each, then run `tools/analyze_surface.py` and `set surfT1/surfT2`. Rename the classes in `SURF_NAMES` in `estimator.cpp` if you like.
5. **Potholes:** drive over a 1–2 cm board and a dip, and adjust `potNeg`/`potPos`/`potBig`. Use `stream 100` to record raw data.

## Troubleshooting

| Serial / OLED shows | Cause and fix |
|---|---|
| `MPU-6050 NOT FOUND` | Read the `bus:` line printed under it: `after SDA=0` means a line is held low (unplug USB for 5 s; if that doesn't help, the MPU is unpowered or shorted). `none` with both lines high means bad jumpers or swapped SDA/SCL. Address 0x69 and 100 kHz are tried automatically |
| Worked before, now not found after flashing | The ESP32 reset mid-read and the MPU kept holding SDA. The boot-time `I2C bus recovery` (16 clock pulses + STOP) normally fixes this; otherwise power-cycle |
| `OLED init failed` | Check the SPI pins. Everything else keeps working |
| `BLE: advertising …, waiting for the Mac bridge` | Start `tools/ble_bridge.py`. Allow Bluetooth for Terminal in macOS Privacy settings. Quit other apps connected to the car (nRF Connect, a second bridge) |
| Bridge connects but no poses arrive | Check the OLED NET page (`connected`, MTU should be 247 or at least 74). Restart the bridge |
| `Wi-Fi: connecting to '…'` never ends | The network is 5 GHz, or the SSID/password is wrong |
| `MQTT connect failed (lastError -3)` | TCP can't reach the broker: wrong host, broker not running, devices isolated, or a VPN/firewall on the laptop |
| `MQTT connect failed (… returnCode 5)` | Wrong MQTT username or password, or not allowed by the ACL |
| Heading drifts while parked | Run `calib` with the car still. Park for at least 2 s so bias tracking runs |
| Distance grows while parked | Raise `movTh` |
