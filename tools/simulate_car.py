#!/usr/bin/env python3
"""Fake car: publishes pose/pothole/status like the ESP32 (as user rs01) so the broker,
dashboard and logger can be tested without hardware. Drives a 10 m x 6 m rectangle.

    python tools/simulate_car.py                      # local broker
    python tools/simulate_car.py --host ... --tls --user rs01 --password ...   # cloud broker
"""
import argparse, json, math, random, time
import mqtt_common as mc

ap = mc.add_args(argparse.ArgumentParser(description=__doc__), role="device")
ap.add_argument("--hz", type=float, default=1.0, help="poses per second (the BLE car sends 50)")
ap.add_argument("--loss", type=float, default=0.0, help="fraction of poses to drop, to test loss statistics")
a = ap.parse_args()
t_start = time.time()
BASE, DEV = mc.BASE, "rs01"
cl = mc.connect(a, DEV + "-sim", will=(f"{BASE}/status", json.dumps({"dev": DEV, "state": "offline"}), 1, True))
cl.loop_start()
cl.publish(f"{BASE}/status", json.dumps({"dev": DEV, "state": "online", "ip": "sim", "rssi": -50, "up_s": 0,
                                         "queued": 0, "dropped": 0, "reconnects": 1}), qos=1, retain=True)

legs = [(10, 0), (6, 90), (10, 180), (6, 270)]  # (length m, heading deg)
x = y = dist = drv = yaw = vint = 0.0
seq, v, hdg_prev = 0, 0.8, 0
dt = 1.0 / a.hz
try:
    while True:
        for length, hdg in legs:
            yaw += (hdg - hdg_prev) % 360
            hdg_prev = hdg
            for _ in range(int(length / v / dt)):
                x += v * dt * math.cos(math.radians(hdg)); y += v * dt * math.sin(math.radians(hdg))
                dist += v * dt; drv += dt
                seq += 1
                surf = "smooth" if hdg in (0, 180) else random.choice(["medium", "rough"])
                vib = {"smooth": 0.05, "medium": 0.13, "rough": 0.27}[surf] + random.uniform(-0.01, 0.01)
                vint += vib * dt
                pose = {"dev": DEV, "ts": int(time.time()), "seq": seq, "x_m": round(x, 2), "y_m": round(y, 2),
                        "hdg_deg": hdg, "spd_mps": v, "surf": surf, "conf": 0.8, "dist_m": round(dist, 2),
                        "vib_g": round(vib, 4), "acc_sd": 0.08, "mv": 1, "label": "sim", "t_ms": int((time.time() - t_start) * 1000),
                        "drv_s": drv, "yaw_raw": round(yaw, 1), "vib_int": round(vint, 3)}
                if random.random() >= a.loss:
                    cl.publish(f"{BASE}/pose", json.dumps(pose), qos=0 if a.hz > 5 else 1)
                if hdg == 90 and random.random() < 0.4 * dt:
                    seq += 1
                    cl.publish(f"{BASE}/pothole", json.dumps({"dev": DEV, "ts": int(time.time()), "seq": seq,
                               "t_ms": int((time.time() - t_start) * 1000),
                               "x_m": round(x, 2), "y_m": round(y, 2), "z_neg_g": -0.9, "z_pos_g": 1.6, "pitch_dps": 120}), qos=1)
                if seq % max(1, int(a.hz)) == 0:
                    print(pose, flush=True)
                time.sleep(dt)
except KeyboardInterrupt:
    pass
