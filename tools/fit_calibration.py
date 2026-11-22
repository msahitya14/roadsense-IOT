#!/usr/bin/env python3
"""Fit RoadSense dead-reckoning parameters from repeated start -> end runs.

Protocol (see docs/CALIBRATION.md):
  * every run starts at the same marked start point, car facing the same direction
  * dashboard/serial:  reset  ->  label run01  ->  drive to an end point, stop  ->  label none
  * write the measured truth for each run into a CSV (template: calib/runs_template.csv)
  * tools/logger.py must be running during the runs (it writes logs/session-*.jsonl)

    python tools/fit_calibration.py logs/session-*.jsonl --runs calib/runs.csv
    python tools/fit_calibration.py logs/*.jsonl --runs calib/runs.csv --origin 33.6430,-117.8417,90 --plot

runs.csv columns (leave a cell empty if you did not measure it):
  run            label used on the car (run01 ...)
  end_x_m,end_y_m   end point in the start frame: x = metres along the start direction, y = metres to the LEFT
  end_lat,end_lon   ...or the end point from Google Maps (needs --origin start_lat,start_lon,start_bearing_deg)
  path_len_m     distance actually driven along the path (tape / map measure), optional
  end_hdg_deg    final heading relative to the start direction, CCW positive (left turn = +90), optional

Firmware quantities used (parameter-free integrals since `reset`, logged in every pose message):
  drv_s   = T     time spent driving            [s]
  vib_int = V     integral of vibration RMS dt  [g s]
  yaw_raw = psi   unscaled yaw integral         [deg]
"""
import argparse, csv, json, math, sys
from collections import defaultdict

import numpy as np

# ----------------------------------------------------------------------------- data loading


def load_runs(log_paths):
    """label -> list of pose dicts in receive order."""
    runs = defaultdict(list)
    for p in log_paths:
        with open(p) as fh:
            for line in fh:
                try:
                    m = json.loads(line)
                except ValueError:
                    continue
                d = m.get("data")
                if m.get("topic", "").endswith("/pose") and isinstance(d, dict) and d.get("label", "none") != "none":
                    if "drv_s" not in d:
                        sys.exit("pose messages lack drv_s/yaw_raw/vib_int: flash the current firmware and record again")
                    d["_rx"] = m.get("rx", 0)
                    runs[d["label"]].append(d)
    for lab in runs:
        runs[lab].sort(key=lambda d: d["_rx"])
    return runs


def run_series(poses):
    """Arrays of cumulative T, V, psi and elapsed time t, starting at the run start (zeros)."""
    T = np.array([p["drv_s"] for p in poses], float)
    V = np.array([p["vib_int"] for p in poses], float)
    P = np.array([p["yaw_raw"] for p in poses], float)
    t = np.arange(1, len(poses) + 1, dtype=float)  # poses arrive at 1 Hz
    warn = None
    if T[0] > 1.5:  # label was set long after reset: measure from the first labelled pose instead
        warn = f"first pose already has drv_s={T[0]:.1f}s (reset before labelling?) -> using it as start"
        T, V, P, t = T - T[0], V - V[0], P - P[0], t - t[0]
    z = np.zeros(1)
    return dict(T=np.r_[z, T], V=np.r_[z, V], P=np.r_[z, P], t=np.r_[z, t], warn=warn)


def load_truth(path, origin):
    rows = []
    with open(path) as fh:
        for r in csv.DictReader(fh):
            if not r.get("run") or r["run"].startswith("#"):
                continue
            f = lambda k: float(r[k]) if r.get(k, "").strip() not in ("", None) else None
            x, y = f("end_x_m"), f("end_y_m")
            if x is None and f("end_lat") is not None:
                if not origin:
                    sys.exit(f"{r['run']}: end_lat/end_lon given but no --origin lat,lon,bearing")
                x, y = latlon_to_local(f("end_lat"), f("end_lon"), *origin)
            rows.append(dict(run=r["run"].strip(), x=x, y=y, L=f("path_len_m"), hdg=f("end_hdg_deg")))
    return rows


def latlon_to_local(lat, lon, lat0, lon0, brg_deg):
    """Same convention as the dashboard: x along the start bearing, y to the left."""
    b = math.radians(brg_deg)
    N = (lat - lat0) * 111320.0
    E = (lon - lon0) * 111320.0 * math.cos(math.radians(lat0))
    return E * math.sin(b) + N * math.cos(b), -E * math.cos(b) + N * math.sin(b)


# ----------------------------------------------------------------------------- models
# Each model maps parameters -> per-interval distance increments and headings.
#   speed:   v = v0                (const)        |  v = v0 + kv * vib   (vib, firmware spdMode 2)
#   heading: theta = s * psi       (scale)        |  theta = s * psi - beta * t   (scale + residual gyro bias)

MODELS = {
    "M1 const speed + gyro scale":       dict(p=["v0", "s"],             speed="const", bias=False),
    "M2 vib-linear speed + gyro scale":  dict(p=["v0", "kv", "s"],       speed="vib",   bias=False),
    "M3 const speed + scale + bias":     dict(p=["v0", "s", "beta"],     speed="const", bias=True),
}


def simulate(params, model, S):
    """Re-integrate one run with new parameters. Returns end x, y, heading(deg), path length."""
    q = dict(zip(model["p"], params))
    dT, dV = np.diff(S["T"]), np.diff(S["V"])
    dd = q["v0"] * dT + (q.get("kv", 0.0) * dV if model["speed"] == "vib" else 0.0)
    dd = np.maximum(dd, 0.0)
    th = q["s"] * S["P"] - (q.get("beta", 0.0) * S["t"] if model["bias"] else 0.0)
    th_mid = np.radians(0.5 * (th[1:] + th[:-1]))  # midpoint rule over each 1-s interval
    return dd @ np.cos(th_mid), dd @ np.sin(th_mid), th[-1], dd.sum()


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


def residuals(params, model, data, w_hdg=0.05):
    """Stacked residuals over runs: end position (m), heading (deg * w_hdg m/deg), path length (m)."""
    r = []
    for S, tr in data:
        x, y, h, L = simulate(params, model, S)
        if tr["x"] is not None:
            r += [x - tr["x"], y - tr["y"]]
        if tr["hdg"] is not None:
            r.append(w_hdg * wrap180(h - tr["hdg"]))
        if tr["L"] is not None:
            r.append(L - tr["L"])
    return np.array(r)


def fit(model, data, x0):
    """Levenberg-Marquardt with a numeric Jacobian (few parameters, few runs)."""
    x = np.array(x0, float)
    lam = 1e-2
    r = residuals(x, model, data)
    for _ in range(200):
        J = np.empty((len(r), len(x)))
        for j in range(len(x)):
            h = 1e-6 * max(1.0, abs(x[j]))
            xp = x.copy(); xp[j] += h
            J[:, j] = (residuals(xp, model, data) - r) / h
        A, g = J.T @ J, J.T @ r
        step = np.linalg.solve(A + lam * np.diag(np.diag(A) + 1e-12), -g)
        xn = x + step
        rn = residuals(xn, model, data)
        if rn @ rn < r @ r:
            x, r, lam = xn, rn, lam * 0.3
            if np.linalg.norm(step) < 1e-9 * (1 + np.linalg.norm(x)):
                break
        else:
            lam *= 10
            if lam > 1e8:
                break
    return x


def initial_guess(model, data):
    num = den = 0.0
    for S, tr in data:
        L = tr["L"] if tr["L"] is not None else (math.hypot(tr["x"], tr["y"]) if tr["x"] is not None else None)
        if L is not None:
            num += L * S["T"][-1]; den += S["T"][-1] ** 2
    v0 = num / den if den else 0.8
    guess = dict(v0=v0, kv=0.0, s=1.0, beta=0.0)
    return [guess[k] for k in model["p"]]


def end_errors(params, model, data):
    out = []
    for S, tr in data:
        x, y, h, L = simulate(params, model, S)
        e = math.hypot(x - tr["x"], y - tr["y"]) if tr["x"] is not None else float("nan")
        eh = wrap180(h - tr["hdg"]) if tr["hdg"] is not None else float("nan")
        Lref = tr["L"] if tr["L"] is not None else L
        out.append(dict(run=tr["run"], e=e, eh=eh, pct=100 * e / Lref if Lref > 0 else float("nan"), pred=(x, y, h, L)))
    return out


def rms(v):
    v = [a for a in v if not math.isnan(a)]
    return math.sqrt(sum(a * a for a in v) / len(v)) if v else float("nan")


# ----------------------------------------------------------------------------- closed-form path-length fits


def length_fits(data):
    rows = [(S["T"][-1], S["V"][-1], tr["L"]) for S, tr in data if tr["L"] is not None]
    if len(rows) < 2:
        return
    T, V, L = map(np.array, zip(*rows))
    print("\nPath-length only (closed-form least squares, uses path_len_m):")
    designs = {
        "D1  L = v0*T":            np.c_[T],
        "D2  L = v0*T + c":        np.c_[T, np.ones_like(T)],
        "D3  L = v0*T + kv*V":     np.c_[T, V],
    }
    for name, X in designs.items():
        if len(L) <= X.shape[1]:
            continue
        beta, *_ = np.linalg.lstsq(X, L, rcond=None)
        res = L - X @ beta
        # leave-one-out via the hat matrix: e_i / (1 - h_ii)
        H = X @ np.linalg.pinv(X.T @ X) @ X.T
        loo = res / np.clip(1 - np.diag(H), 1e-6, None)
        coef = ", ".join(f"{b:.4f}" for b in beta)
        print(f"  {name:22s} coef [{coef}]   RMSE {rms(res):.3f} m   LOO-RMSE {rms(loo):.3f} m")
    print("  (D2's c < 0 means the car moves a bit before 'driving' is detected: lag = -c/v0 seconds)")


# ----------------------------------------------------------------------------- main


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("logs", nargs="+", help="logs/session-*.jsonl from tools/logger.py")
    ap.add_argument("--runs", required=True, help="truth CSV (see calib/runs_template.csv)")
    ap.add_argument("--origin", help="start_lat,start_lon,start_bearing_deg (needed for end_lat/end_lon)")
    ap.add_argument("--plot", action="store_true", help="save calib/fit.png (needs matplotlib)")
    a = ap.parse_args()

    origin = tuple(float(v) for v in a.origin.split(",")) if a.origin else None
    logs = load_runs(a.logs)
    truth = load_truth(a.runs, origin)
    data = []
    for tr in truth:
        if tr["run"] not in logs:
            print(f"  ! {tr['run']}: no labelled poses in the logs, skipped")
            continue
        S = run_series(logs[tr["run"]])
        if S["warn"]:
            print(f"  ! {tr['run']}: {S['warn']}")
        data.append((S, tr))
    if len(data) < 2:
        sys.exit("need at least 2 runs with logs + truth")

    print(f"{len(data)} runs:")
    for S, tr in data:
        print(f"  {tr['run']:8s} T={S['T'][-1]:6.1f}s  V={S['V'][-1]:6.2f}gs  psi={S['P'][-1]:7.1f}deg  "
              f"truth end=({tr['x']}, {tr['y']}) L={tr['L']} hdg={tr['hdg']}")

    # heading scale alone, closed form: theta_true = s * psi
    hd = [(S["P"][-1], tr["hdg"]) for S, tr in data if tr["hdg"] is not None and abs(S["P"][-1]) > 20]
    if hd:
        P, H = map(np.array, zip(*hd))
        # unwrap the truth to the turn the gyro saw (e.g. 350 vs -10)
        H = P + wrap180(H - P)
        s = float(P @ H / (P @ P))
        print(f"\nGyro scale from end headings only: s = {s:.4f}  (firmware: set gyroScale {s:.4f})")

    length_fits(data)

    have_xy = sum(tr["x"] is not None for _, tr in data)
    print("\nJoint end-point fits (re-integrates every run; LOO = fit without that run, then predict it):")
    best = None
    for name, model in MODELS.items():
        n_obs = sum(2 * (tr["x"] is not None) + (tr["hdg"] is not None) + (tr["L"] is not None) for _, tr in data)
        if n_obs <= len(model["p"]):
            continue
        x = fit(model, data, initial_guess(model, data))
        fit_err = end_errors(x, model, data)
        loo = []
        for i in range(len(data)):
            rest = data[:i] + data[i + 1:]
            xi = fit(model, rest, x)
            loo += end_errors(xi, model, [data[i]])
        params = ", ".join(f"{k}={v:.4f}" for k, v in zip(model["p"], x))
        score = rms([e["e"] for e in loo]) if have_xy else rms([e["eh"] for e in loo])
        print(f"  {name:34s} {params}")
        print(f"      end error RMSE {rms([e['e'] for e in fit_err]):.2f} m ({rms([e['pct'] for e in fit_err]):.1f}% of path)"
              f"   LOO {rms([e['e'] for e in loo]):.2f} m ({rms([e['pct'] for e in loo]):.1f}%)"
              f"   heading RMSE {rms([e['eh'] for e in fit_err]):.1f} deg")
        if best is None or score < best[0] - 1e-9:
            best = (score, name, model, x, fit_err)

    score, name, model, x, fit_err = best
    q = dict(zip(model["p"], x))
    print(f"\nBest by leave-one-out error: {name}")
    print("Per-run end errors:")
    for e in fit_err:
        px, py, ph, pL = e["pred"]
        print(f"  {e['run']:8s} predicted ({px:6.2f}, {py:6.2f}) hdg {ph:6.1f}  path {pL:6.1f} m   error {e['e']:.2f} m")
    print("\nSend to the car (serial or dashboard):")
    if model["speed"] == "vib":
        print(f"  set spdMode 2\n  set v0 {q['v0']:.4f}\n  set kVib {q['kv']:.4f}")
    else:
        print(f"  set spdMode 0\n  set v0 {q['v0']:.4f}")
    print(f"  set gyroScale {q['s']:.4f}")
    if model["bias"]:
        print(f"  # residual gyro drift beta = {q['beta']:.4f} deg/s is not a firmware parameter: if it is large,\n"
              f"  # the car is not parked long enough for bias tracking (park 2-3 s before each run).")

    if a.plot:
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except ImportError:
            sys.exit("pip install matplotlib for --plot")
        fig, ax = plt.subplots(figsize=(7, 7))
        for (S, tr), e in zip(data, fit_err):
            dT, dV = np.diff(S["T"]), np.diff(S["V"])
            dd = np.maximum(q["v0"] * dT + (q.get("kv", 0) * dV if model["speed"] == "vib" else 0), 0)
            th = np.radians(q["s"] * S["P"] - (q.get("beta", 0) * S["t"] if model["bias"] else 0))
            thm = 0.5 * (th[1:] + th[:-1])
            px, py = np.r_[0, np.cumsum(dd * np.cos(thm))], np.r_[0, np.cumsum(dd * np.sin(thm))]
            ln, = ax.plot(-py, px, lw=1.5, label=tr["run"])  # plot with x forward = up, y left = left
            if tr["x"] is not None:
                ax.plot(-tr["y"], tr["x"], "x", ms=10, mew=2, color=ln.get_color())
        ax.plot(0, 0, "ko")
        ax.set_aspect("equal"); ax.grid(alpha=.3); ax.legend(fontsize=8)
        ax.set_xlabel("right  [m]"); ax.set_ylabel("forward from start  [m]")
        ax.set_title(f"{name}\nlines = re-integrated runs, x = measured end points", fontsize=10)
        out = "calib/fit.png"
        import os; os.makedirs("calib", exist_ok=True)
        fig.tight_layout(); fig.savefig(out, dpi=130)
        print(f"\nsaved {out}")


if __name__ == "__main__":
    main()
