"""CSV session logs with the receiving computer's system timestamp on every row.

Used by ble_bridge.py and logger.py. One session = files sharing a timestamp stamp:
  logs/pose-<stamp>.csv      every pose        (rx_unix, rx_iso, then the car's fields)
  logs/pothole-<stamp>.csv   every pothole
  logs/link-<stamp>.csv      connects, disconnects, status messages, command/info lines
  logs/raw-<stamp>.csv       200 Hz raw IMU (BLE bridge with --raw)

rx_unix = system time when the message reached this computer (seconds, float, UTC epoch)
rx_iso  = the same time as local ISO-8601 with milliseconds, for humans and spreadsheets
t_ms    = the car's own millisecond clock (millis()), ts = car Unix time if synced
"""
import csv, json, time
from datetime import datetime

POSE_COLS = ["seq", "t_ms", "ts", "x_m", "y_m", "hdg_deg", "spd_mps", "dist_m", "surf", "conf", "vib_g", "acc_sd",
             "mv", "label", "drv_s", "yaw_raw", "vib_int"]
POT_COLS = ["seq", "t_ms", "ts", "x_m", "y_m", "z_neg_g", "z_pos_g", "pitch_dps"]
RAW_COLS = ["t_ms", "ax_g", "ay_g", "az_g", "gx_dps", "gy_dps", "gz_dps"]


def stamp_now():
    t = round(time.time(), 3)  # round once so rx_unix and rx_iso show the same millisecond
    return t, datetime.fromtimestamp(t).isoformat(timespec="milliseconds")


class SessionCSV:
    def __init__(self, logdir, stamp=None):
        logdir.mkdir(exist_ok=True)
        self.dir, self.stamp = logdir, stamp or time.strftime("%Y%m%d-%H%M%S")
        self._w = {}

    def _writer(self, kind, cols):
        if kind not in self._w:
            fh = (self.dir / f"{kind}-{self.stamp}.csv").open("w", newline="", buffering=1)
            w = csv.writer(fh)
            w.writerow(["rx_unix", "rx_iso"] + cols)
            self._w[kind] = (fh, w)
            print(f"{kind} CSV -> {fh.name}", flush=True)
        return self._w[kind][1]

    def pose(self, d, rx=None):
        t, iso = rx or stamp_now()
        self._writer("pose", POSE_COLS).writerow([f"{t:.3f}", iso] + [d.get(k, "") for k in POSE_COLS])

    def pothole(self, d, rx=None):
        t, iso = rx or stamp_now()
        self._writer("pothole", POT_COLS).writerow([f"{t:.3f}", iso] + [d.get(k, "") for k in POT_COLS])

    def link(self, event, detail="", rx=None):
        t, iso = rx or stamp_now()
        if not isinstance(detail, str):
            detail = json.dumps(detail, separators=(",", ":"))
        self._writer("link", ["event", "detail"]).writerow([f"{t:.3f}", iso, event, detail])

    def raw(self, rows, rx=None):
        t, iso = rx or stamp_now()
        self._writer("raw", RAW_COLS).writerows(
            [[f"{t:.3f}", iso, f"{r[0]:.1f}"] + [f"{v:.4f}" for v in r[1:]] for r in rows])

    def close(self):
        for fh, _ in self._w.values():
            fh.close()
