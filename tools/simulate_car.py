#!/usr/bin/env python3
"""Fake car: publishes pose/pothole/status like the ESP32 (as user rs01) so the broker,
dashboard and logger can be tested without hardware. Drives a 10 m x 6 m rectangle.

    python tools/simulate_car.py                      # local broker
    python tools/simulate_car.py --host ... --tls --user rs01 --password ...   # cloud broker
"""
import argparse, json, math, random, time
import mqtt_common as mc

ap = mc.add_args(argparse.ArgumentParser(description=__doc__), role="device")
a = ap.parse_args()
BASE, DEV = mc.BASE, "rs01"
cl = mc.connect(a, DEV + "-sim", will=(f"{BASE}/status", json.dumps({"dev": DEV, "state": "offline"}), 1, True))
cl.loop_start()
cl.publish(f"{BASE}/status", json.dumps({"dev": DEV, "state": "online", "ip": "sim", "rssi": -50, "up_s": 0,
                                         "queued": 0, "dropped": 0, "reconnects": 1}), qos=1, retain=True)

legs = [(10, 0), (6, 90), (10, 180), (6, 270)]  # (length m, heading deg)
x = y = dist = drv = yaw = vint = 0.0
seq, v, hdg_prev = 0, 0.8, 0
try:
    while True:
        for length, hdg in legs:
            yaw += (hdg - hdg_prev) % 360
            hdg_prev = hdg
            for _ in range(int(length / v)):
                x += v * math.cos(math.radians(hdg)); y += v * math.sin(math.radians(hdg)); dist += v; drv += 1
                seq += 1
                surf = "smooth" if hdg in (0, 180) else random.choice(["medium", "rough"])
                vib = {"smooth": 0.05, "medium": 0.13, "rough": 0.27}[surf] + random.uniform(-0.01, 0.01)
                vint += vib
                pose = {"dev": DEV, "ts": int(time.time()), "seq": seq, "x_m": round(x, 2), "y_m": round(y, 2),
                        "hdg_deg": hdg, "spd_mps": v, "surf": surf, "conf": 0.8, "dist_m": round(dist, 2),
                        "vib_g": round(vib, 4), "acc_sd": 0.08, "mv": 1, "label": "sim",
                        "drv_s": drv, "yaw_raw": round(yaw, 1), "vib_int": round(vint, 3)}
                cl.publish(f"{BASE}/pose", json.dumps(pose), qos=1)
                if hdg == 90 and random.random() < 0.4:
                    seq += 1
                    cl.publish(f"{BASE}/pothole", json.dumps({"dev": DEV, "ts": int(time.time()), "seq": seq,
                               "x_m": round(x, 2), "y_m": round(y, 2), "z_neg_g": -0.9, "z_pos_g": 1.6, "pitch_dps": 120}), qos=1)
                print(pose, flush=True)
                time.sleep(1)
except KeyboardInterrupt:
    pass
