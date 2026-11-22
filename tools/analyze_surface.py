#!/usr/bin/env python3
"""Suggest surface thresholds from labelled runs.

Drive each surface with `label asphalt`, `label gravel`, `label grass` ... while
tools/logger.py is running, then:

    python tools/analyze_surface.py logs/session-*.jsonl
"""
import json, statistics, sys
from collections import defaultdict

vib = defaultdict(list)
for path in sys.argv[1:]:
    for line in open(path):
        m = json.loads(line)
        d = m["data"]
        if m["topic"].endswith("/pose") and isinstance(d, dict) and d.get("mv") and d.get("label", "none") != "none":
            vib[d["label"]].append(d["vib_g"])

if not vib:
    sys.exit("no labelled driving poses found")

rows = []
for lab, xs in vib.items():
    xs.sort()
    q = lambda p: xs[min(len(xs) - 1, int(p * len(xs)))]
    rows.append((statistics.median(xs), lab, len(xs), q(0.1), q(0.9)))
rows.sort()
print(f"{'label':12s} {'n':>4s} {'p10':>7s} {'median':>7s} {'p90':>7s}   (vertical vib RMS, g)")
for med, lab, n, p10, p90 in rows:
    print(f"{lab:12s} {n:4d} {p10:7.3f} {med:7.3f} {p90:7.3f}")
print("\nsuggested boundaries (midpoint between neighbouring medians):")
names = ["surfT1", "surfT2"]
for i in range(min(2, len(rows) - 1)):
    t = (rows[i][0] + rows[i + 1][0]) / 2
    print(f"  set {names[i]} {t:.3f}    # {rows[i][1]} | {rows[i + 1][1]}")
print("classes are reported as smooth < surfT1 <= medium < surfT2 <= rough")
