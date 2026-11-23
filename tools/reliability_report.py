#!/usr/bin/env python3
"""Reliability report for a RoadSense session, from the system-timestamped CSVs.

    python tools/reliability_report.py logs/pose-20261122-170000.csv            # finds pothole/link CSVs of the session
    python tools/reliability_report.py logs/pose-*.csv --plot                   # several sessions, saves PNGs
    python tools/reliability_report.py logs/pose-X.csv --from 17:05:00 --to 17:15:00   # analyse a time window

What it measures (rx = Mac system time of arrival, t_ms = car clock):
  delivery     frames received, rate driving / parked, sequence gaps = loss, duplicates, out-of-order
  timing       inter-arrival time at the Mac (jitter) vs. the car's own send interval
  latency      one-way delay above the best case: (rx - t_ms) - min(rx - t_ms). Absolute latency needs
               synchronised clocks; the minimum is the best-case path, so this is the extra queueing delay.
  outages      periods with no data while the car should be sending (gap > --outage s), and link events
  buffering    frames delivered much later than normal = sent from the car's offline buffer after a
               reconnect. Seq gaps spanning ~1 s of car time (or next to late frames) are the buffer's
               1 Hz decimation, not radio loss.
  clock drift  slope of (rx - t_ms) over time, in ppm (car crystal vs Mac clock)
"""
import argparse, csv, json, math, pathlib, sys
from datetime import datetime

import numpy as np


def read_csv(path):
    if not path.exists():
        return []
    with path.open() as fh:
        return list(csv.DictReader(fh))


def pct(a, q):
    return float(np.percentile(a, q)) if len(a) else float("nan")


def hms(t):
    return datetime.fromtimestamp(t).strftime("%H:%M:%S.%f")[:-3]


def window(rows, lo, hi):
    if lo is None and hi is None:
        return rows
    out = []
    for r in rows:
        tod = datetime.fromtimestamp(float(r["rx_unix"])).strftime("%H:%M:%S")
        if (lo is None or tod >= lo) and (hi is None or tod <= hi):
            out.append(r)
    return out


def analyse(pose_path, a):
    stamp = pose_path.stem.split("pose-", 1)[-1]
    d = pose_path.parent
    poses = window(read_csv(pose_path), a.frm, a.to)
    pots = window(read_csv(d / f"pothole-{stamp}.csv"), a.frm, a.to)
    links = window(read_csv(d / f"link-{stamp}.csv"), a.frm, a.to)
    if len(poses) < 3:
        print(f"{pose_path}: fewer than 3 poses, skipped")
        return None

    rx = np.array([float(r["rx_unix"]) for r in poses])
    tms = np.array([float(r["t_ms"]) for r in poses])
    seq = np.array([int(r["seq"]) for r in poses])
    mv = np.array([r["mv"] in ("1", "True", "true") for r in poses])
    dur = rx[-1] - rx[0]

    # ---- car reboots: seq AND t_ms restart (buffered frames are older but not restarted)
    reboot = np.r_[False, (np.diff(tms) < -1000) & (np.diff(seq) < 0) & (tms[1:] < 0.5 * tms[:-1])]
    seg = np.cumsum(reboot)

    # ---- latency relative to the best case, and clock drift (per boot)
    off = rx * 1000.0 - tms                       # ms; = clock offset + one-way delay
    lat = np.empty_like(off)
    drift_ppm = float("nan")
    for s in np.unique(seg):
        m = seg == s
        lat[m] = off[m] - off[m].min()
        if m.sum() > 20 and (rx[m][-1] - rx[m][0]) > 60:
            # drift: line through the minimum delay of each time bin (best-case frames, no queueing bias)
            t_rel, o = rx[m] - rx[m][0], off[m]
            bins = np.minimum((t_rel / t_rel[-1] * 20).astype(int), 19)
            bt = [t_rel[bins == b][np.argmin(o[bins == b])] for b in range(20) if (bins == b).any()]
            bo = [o[bins == b].min() for b in range(20) if (bins == b).any()]
            if len(bt) > 5:
                coef = np.polyfit(bt, bo, 1)
                drift_ppm = float(coef[0] / 1000.0 * 1e6)
                lat[m] = o - np.polyval(coef, t_rel)  # remove clock drift, keep only queueing delay
                lat[m] -= lat[m].min()
    live_lat = lat[lat < np.percentile(lat, 90) * 3 + 50]
    late_thr = max(300.0, 5 * pct(live_lat, 99))
    late = lat > late_thr  # sent from the car's offline buffer, delivered after a reconnect

    # ---- sequence analysis in SEND order (seq), poses + potholes share one counter
    pot_seq = [int(r["seq"]) for r in pots]
    pot_t = [float(r.get("t_ms") or "nan") for r in pots]
    frames = sorted(zip(np.r_[seg, np.full(len(pots), -1)], np.r_[seq, pot_seq], np.r_[tms, pot_t],
                        np.r_[late, np.zeros(len(pots), bool)]), key=lambda f: (f[0] if f[0] >= 0 else 0, f[1]))
    dups = missing_loss = missing_buffer = 0
    for (b1, s1, t1, l1), (b2, s2, t2, l2) in zip(frames, frames[1:]):
        if b1 != b2 and b1 >= 0 and b2 >= 0:
            continue  # reboot boundary
        if s2 == s1:
            dups += 1
            continue
        g = int(s2 - s1 - 1)
        if g <= 0:
            continue
        # the offline buffer keeps ~1 frame/s: a skip spans ~1 s of car time, or touches a late frame
        if (not np.isnan(t1) and not np.isnan(t2) and t2 - t1 >= 900) or l1 or l2:
            missing_buffer += g
        else:
            missing_loss += g
    missing_total = missing_loss + missing_buffer
    expected_live = len(poses) + len(pots) - dups + missing_loss
    loss_pct = 100.0 * missing_loss / expected_live if expected_live else 0.0
    live_seq = seq[~late]
    ooo = int((np.diff(live_seq) < 0).sum() - reboot[~late].sum())

    # ---- inter-arrival (Mac) vs send interval (car), live frames, excluding outages
    live = ~late
    ia = np.diff(rx[live]) * 1000.0
    send = np.diff(tms[live])
    keep = (send > 0) & (ia < a.outage * 1000)
    ia_all_max = float(ia.max()) if len(ia) else float("nan")
    ia, send = ia[keep], send[keep]
    jitter = ia - send

    # ---- rates
    def rate(mask):
        r_, idx = rx[mask], np.where(mask)[0]
        if len(r_) < 2:
            return float("nan")
        # time spent in this state = sum of intervals that start in it
        dt = np.diff(rx)[idx[idx < len(rx) - 1]]
        dt = dt[dt < 10]
        return len(r_) / dt.sum() if dt.sum() > 0 else float("nan")

    # ---- outages: no data for longer than expected
    gaps_rx = np.diff(rx)
    out_idx = np.where(gaps_rx > np.maximum(a.outage, np.where(mv[:-1], a.outage, a.park_outage)))[0]
    outages = [(rx[i], rx[i + 1], gaps_rx[i]) for i in out_idx]

    ev = {}
    for r in links:
        ev[r["event"]] = ev.get(r["event"], 0) + 1
    statuses = [json.loads(r["detail"]) for r in links if r["event"] == "status" and r["detail"].startswith("{")]

    # ---- print
    print(f"\n=== {pose_path.name}   {hms(rx[0])} -> {hms(rx[-1])}   ({dur/60:.1f} min) ===")
    print(f"frames            {len(poses)} poses, {len(pots)} potholes   driving {mv.mean()*100:.0f}% of poses")
    print(f"rate              driving {rate(mv):.1f}/s   parked {rate(~mv):.2f}/s   overall {len(poses)/dur:.1f}/s")
    print(f"sequence          {missing_total} missing seq: {missing_loss} lost in transit, {missing_buffer} skipped by "
          f"the offline buffer (1 Hz)")
    print(f"loss (live link)  {loss_pct:.3f} %   duplicates {dups}   out-of-order {max(ooo, 0)}   car reboots {int(reboot.sum())}")
    if len(ia):
        print(f"inter-arrival ms  p50 {pct(ia,50):.1f}  p95 {pct(ia,95):.1f}  p99 {pct(ia,99):.1f}  max {ia.max():.0f}"
              f"   (car send interval p50 {pct(send,50):.1f}; outages excluded, longest gap {ia_all_max:.0f})")
        print(f"jitter ms         (arrival - send interval) p95 |j| {pct(np.abs(jitter),95):.1f}   std {jitter.std():.1f}")
    print(f"latency ms        above best case: p50 {pct(lat[live],50):.0f}  p95 {pct(lat[live],95):.0f}  "
          f"p99 {pct(lat[live],99):.0f}  max {lat[live].max():.0f}")
    print(f"buffered frames   {int(late.sum())} delivered > {late_thr:.0f} ms late (offline buffer after reconnect)")
    print(f"clock drift       {drift_ppm:+.0f} ppm (car vs Mac)" if not math.isnan(drift_ppm) else "clock drift       n/a (< 60 s)")
    print(f"outages           {len(outages)} gaps > {a.outage:g} s driving / {a.park_outage:g} s parked"
          + (f", longest {max(o[2] for o in outages):.1f} s" if outages else ""))
    for s, e, g in outages[:10]:
        print(f"                  {hms(s)} -> {hms(e)}  {g:6.1f} s")
    if ev:
        print("link events       " + ", ".join(f"{k} {v}" for k, v in sorted(ev.items())))
    if statuses:
        last = statuses[-1]
        print("car counters      " + ", ".join(f"{k} {last[k]}" for k in
                                             ("queued", "dropped", "reconnects", "ble_err", "imu_err", "tx_hz") if k in last))

    if a.plot:
        plot(pose_path, rx, lat, ia, mv, outages)
    return dict(name=pose_path.name, minutes=dur / 60, poses=len(poses), loss_pct=loss_pct, lat_p95=pct(lat[live], 95),
                ia_p95=pct(ia, 95), outages=len(outages))


def plot(pose_path, rx, lat, ia, mv, outages):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        sys.exit("pip install matplotlib for --plot")
    t = (rx - rx[0]) / 60.0
    fig, ax = plt.subplots(3, 1, figsize=(10, 8), sharex=False)
    # rate per second
    sec = np.floor(rx - rx[0]).astype(int)
    cnt = np.bincount(sec)
    ax[0].plot(np.arange(len(cnt)) / 60.0, cnt, lw=0.8)
    for s, e, g in outages:
        ax[0].axvspan((s - rx[0]) / 60, (e - rx[0]) / 60, color="red", alpha=0.2)
    ax[0].set_ylabel("frames / s"); ax[0].set_title("Received rate (red = outage)")
    ax[1].plot(t, lat, ".", ms=2)
    ax[1].set_ylabel("latency above best (ms)"); ax[1].set_yscale("symlog", linthresh=100)
    ax[1].set_xlabel("minutes")
    ax[2].hist(np.clip(ia, 0, 200), bins=100)
    ax[2].set_xlabel("inter-arrival at Mac (ms, clipped at 200)"); ax[2].set_ylabel("count")
    fig.tight_layout()
    out = pose_path.with_name(pose_path.stem.replace("pose-", "reliability-") + ".png")
    fig.savefig(out, dpi=120)
    print(f"plot              {out}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("pose_csv", nargs="+", type=pathlib.Path)
    ap.add_argument("--outage", type=float, default=1.0, help="gap (s) counted as an outage while driving")
    ap.add_argument("--park-outage", type=float, default=15.0, help="gap (s) counted as an outage while parked")
    ap.add_argument("--from", dest="frm", help="start time HH:MM:SS (local)")
    ap.add_argument("--to", help="end time HH:MM:SS (local)")
    ap.add_argument("--plot", action="store_true", help="save logs/reliability-<stamp>.png")
    a = ap.parse_args()
    res = [r for p in a.pose_csv if (r := analyse(p, a))]
    if len(res) > 1:
        print("\n=== summary ===")
        print(f"{'session':34s} {'min':>6s} {'poses':>7s} {'loss%':>7s} {'lat95':>6s} {'ia95':>6s} {'outages':>7s}")
        for r in res:
            print(f"{r['name']:34s} {r['minutes']:6.1f} {r['poses']:7d} {r['loss_pct']:7.3f} {r['lat_p95']:6.0f} "
                  f"{r['ia_p95']:6.1f} {r['outages']:7d}")


if __name__ == "__main__":
    main()
