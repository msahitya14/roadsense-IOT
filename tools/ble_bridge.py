#!/usr/bin/env python3
"""BLE -> MQTT bridge: the Mac is the car's gateway (no Wi-Fi network or online broker needed).

Connects to the car ("RoadSense-rs01", NET_MODE 3), decodes its binary BLE frames and republishes
them as the usual JSON topics on the LOCAL broker, so the dashboard, logger and calibration tools
work unchanged. Commands published on rs/T24/cmd/rs01 (dashboard buttons) are written to the car.

    broker/run.sh                                   # local Mosquitto (+ dashboard on :8000)
    .venv/bin/python tools/ble_bridge.py            # bridge; add --raw for the 200 Hz IMU stream
    .venv/bin/python tools/ble_bridge.py --no-mqtt  # no broker at all: just log to logs/ble-*.jsonl

First run: macOS asks to allow Bluetooth for Terminal/Python (System Settings > Privacy > Bluetooth).
"""
import argparse, asyncio, json, struct, sys, time

from bleak import BleakClient, BleakScanner

import mqtt_common as mc
from csvlog import SessionCSV, stamp_now

SVC_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
RX_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  # write: commands to the car
TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # notify: frames from the car

# must match firmware/src/ble_link.cpp
POSE = struct.Struct("<BIII11fBB12s")  # 71 bytes
POTHOLE = struct.Struct("<BIII5f")     # 33 bytes
RAW_HDR = struct.Struct("<BIBH")       # 8 bytes, then n * <6h>
RAW_SAMPLE = struct.Struct("<6h")
F_POSE, F_POTHOLE, F_STATUS, F_INFO, F_RAW = 1, 2, 3, 4, 5
SURF = ["idle", "smooth", "medium", "rough"]


def decode_pose(b, dev):
    (_, seq, t_ms, ts, x, y, hdg, spd, dist, vib, acc_sd, conf, drv, yaw_raw, vib_int, surf, mv, label) = POSE.unpack(b)
    return {"dev": dev, "ts": ts, "seq": seq, "t_ms": t_ms, "x_m": round(x, 3), "y_m": round(y, 3),
            "hdg_deg": round(hdg, 2), "spd_mps": round(spd, 3), "surf": SURF[surf] if surf < 4 else str(surf),
            "conf": round(conf, 2), "dist_m": round(dist, 3), "vib_g": round(vib, 4), "acc_sd": round(acc_sd, 4),
            "mv": mv, "label": label.split(b"\0", 1)[0].decode(errors="replace"),
            "drv_s": round(drv, 3), "yaw_raw": round(yaw_raw, 3), "vib_int": round(vib_int, 5)}


def decode_pothole(b, dev):
    _, seq, t_ms, ts, x, y, zneg, zpos, pitch = POTHOLE.unpack(b)
    return {"dev": dev, "ts": ts, "seq": seq, "t_ms": t_ms, "x_m": round(x, 2), "y_m": round(y, 2),
            "z_neg_g": round(zneg, 2), "z_pos_g": round(zpos, 2), "pitch_dps": round(pitch)}


def decode_raw(b):
    _, t0, n, period_us = RAW_HDR.unpack_from(b)
    out = []
    for i in range(n):
        ax, ay, az, gx, gy, gz = RAW_SAMPLE.unpack_from(b, RAW_HDR.size + i * RAW_SAMPLE.size)
        out.append((t0 + i * period_us / 1000.0, ax / 4096, ay / 4096, az / 4096, gx / 65.5, gy / 65.5, gz / 65.5))
    return out


class Bridge:
    def __init__(self, a):
        self.a, self.dev = a, a.name.split("-")[-1]
        self.base = mc.BASE
        self.loop = None
        self.client = None
        self.mq = None
        self.jsonl = None
        self.csv = None
        self.info_buf = b""
        self.last_seq = None
        self.stats = dict(pose=0, pot=0, raw=0, bytes=0, gaps=0, t0=time.time())

    # ---------------------------------------------------------------- outputs
    def out(self, topic, data, retain=False, qos=1):
        full = f"{self.base}/{topic}"
        if self.mq:
            payload = data if isinstance(data, str) else json.dumps(data, separators=(",", ":"))
            self.mq.publish(full, payload, qos=qos, retain=retain)
        if self.jsonl:
            self.jsonl.write(json.dumps({"rx": time.time(), "topic": full, "data": data}) + "\n")

    def offline(self):
        self.out("status", {"dev": self.dev, "state": "offline", "link": "ble"}, retain=True)
        if self.csv:
            self.csv.link("disconnected")

    def open_logs(self):
        (mc.ROOT / "logs").mkdir(exist_ok=True)
        stamp = time.strftime("%Y%m%d-%H%M%S")
        if self.a.log:
            p = mc.ROOT / "logs" / f"ble-{stamp}.jsonl"
            self.jsonl = p.open("a", buffering=1)
            print(f"logging to {p}")
        if not self.a.no_csv:  # CSVs with the Mac's system receive time (reliability tests)
            self.csv = SessionCSV(mc.ROOT / "logs", stamp)

    # ---------------------------------------------------------------- BLE frames
    def on_frame(self, _char, data: bytearray):
        b = bytes(data)
        rx = stamp_now()  # Mac system time this frame arrived
        self.stats["bytes"] += len(b)
        t = b[0] if b else 0
        try:
            if t == F_POSE and len(b) == POSE.size:
                d = decode_pose(b, self.dev)
                self.count_seq(d["seq"])
                self.stats["pose"] += 1
                self.out("pose", d, qos=0)
                if self.csv:
                    self.csv.pose(d, rx)
            elif t == F_POTHOLE and len(b) == POTHOLE.size:
                d = decode_pothole(b, self.dev)
                self.count_seq(d["seq"])
                self.stats["pot"] += 1
                print(f"POTHOLE at ({d['x_m']}, {d['y_m']})  z {d['z_neg_g']}/{d['z_pos_g']} g", flush=True)
                self.out("pothole", d)
                if self.csv:
                    self.csv.pothole(d, rx)
            elif t == F_STATUS:
                d = json.loads(b[1:].decode())
                d["bridge"] = "mac-ble"
                self.out("status", d, retain=True)
                if self.csv:
                    self.csv.link("status", d, rx)
            elif t == F_INFO and len(b) >= 2:
                self.info_buf += b[2:]
                if not b[1]:  # last chunk
                    txt = self.info_buf.decode(errors="replace")
                    self.info_buf = b""
                    print(f"car: {txt}", flush=True)
                    self.out("info", txt, qos=0)
                    if self.csv:
                        self.csv.link("info", txt, rx)
            elif t == F_RAW and len(b) >= RAW_HDR.size:
                rows = decode_raw(b)
                self.stats["raw"] += len(rows)
                if self.csv:
                    self.csv.raw(rows, rx)
            else:
                print(f"? unknown frame type {t} len {len(b)}")
        except Exception as e:  # never let one bad frame kill the bridge
            print(f"! frame decode error ({e}) type {t} len {len(b)}")

    def count_seq(self, seq):
        if self.last_seq is not None and seq > self.last_seq + 1:
            self.stats["gaps"] += seq - self.last_seq - 1
        self.last_seq = seq if self.last_seq is None or seq > self.last_seq else self.last_seq

    # ---------------------------------------------------------------- commands
    async def send_cmd(self, cmd: str):
        if self.client and self.client.is_connected:
            await self.client.write_gatt_char(RX_UUID, cmd.encode(), response=True)
            print(f"> {cmd}", flush=True)
            if self.csv:
                self.csv.link("command", cmd)

    def on_mqtt_cmd(self, _cl, _u, msg):
        cmd = msg.payload.decode(errors="replace").strip()
        if cmd and self.loop:
            asyncio.run_coroutine_threadsafe(self.send_cmd(cmd), self.loop)

    # ---------------------------------------------------------------- main loop
    async def stats_task(self):
        while True:
            await asyncio.sleep(5)
            s, dt = self.stats, time.time() - self.stats["t0"]
            if self.client and self.client.is_connected:
                print(f"[link] {s['pose']/dt:5.1f} pose/s  {s['raw']/dt:6.1f} raw/s  {s['bytes']/dt/1000:5.2f} kB/s  "
                      f"potholes {s['pot']}  seq gaps {s['gaps']}", flush=True)
            self.stats.update(pose=0, pot=0, raw=0, bytes=0, t0=time.time())

    async def run(self):
        self.loop = asyncio.get_running_loop()
        self.open_logs()
        if not self.a.no_mqtt:
            self.mq = mc.connect(self.a, f"{self.dev}-blebridge",
                                 will=(f"{self.base}/status", json.dumps({"dev": self.dev, "state": "offline"}), 1, True))
            self.mq.on_message = self.on_mqtt_cmd
            self.mq.on_connect = lambda cl, *_: cl.subscribe(f"{self.base}/cmd/{self.dev}", qos=1)
            self.mq.loop_start()
        asyncio.create_task(self.stats_task())
        while True:
            print(f"scanning for {self.a.name} ...", flush=True)
            dev = await BleakScanner.find_device_by_name(self.a.name, timeout=10)
            if not dev:
                continue
            gone = asyncio.Event()
            try:
                async with BleakClient(dev, disconnected_callback=lambda _c: self.loop.call_soon_threadsafe(gone.set)) as c:
                    self.client = c
                    print(f"connected to {self.a.name} (MTU {c.mtu_size})", flush=True)
                    if self.csv:
                        self.csv.link("connected", {"mtu": c.mtu_size})
                    await c.start_notify(TX_UUID, self.on_frame)
                    await self.send_cmd(f"time {int(time.time())}")
                    if self.a.raw:
                        await self.send_cmd("raw on")
                    for cmd in self.a.cmd:
                        await self.send_cmd(cmd)
                    await gone.wait()
            except Exception as e:
                print(f"BLE error: {e}")
            self.client = None
            print("disconnected - the car buffers at 1 Hz until we reconnect", flush=True)
            self.offline()
            await asyncio.sleep(1)


def main():
    ap = mc.add_args(argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter),
                     role="device")
    ap.add_argument("--name", default="RoadSense-rs01", help="BLE name of the car")
    ap.add_argument("--raw", action="store_true", help="ask the car for the 200 Hz raw IMU stream (logs/raw-*.csv)")
    ap.add_argument("--cmd", action="append", default=[], help="command to send on every connect (repeatable)")
    ap.add_argument("--no-mqtt", action="store_true", help="don't use a broker; implies --log")
    ap.add_argument("--log", action="store_true", help="also write logs/ble-*.jsonl (same format as logger.py)")
    ap.add_argument("--no-csv", action="store_true", help="don't write logs/pose|pothole|link|raw-*.csv")
    a = ap.parse_args()
    a.log = a.log or a.no_mqtt
    try:
        asyncio.run(Bridge(a).run())
    except KeyboardInterrupt:
        print("\nbye")


if __name__ == "__main__":
    main()
