// Link-quality statistics, computed per received frame. Same rules as tools/reliability_report.py:
//  - seq gaps spanning >= 900 ms of car time (or next to a late frame) = skipped by the car's 1 Hz offline
//    buffer; other gaps = lost in transit
//  - latency = (rx - t_ms) - min(rx - t_ms): delay above the best case seen (clocks need not be synced)
//  - outage = no frame for > 1 s while driving / > 15 s while parked

const RING = 2000;

function pct(arr, q) {
  if (!arr.length) return NaN;
  const s = [...arr].sort((a, b) => a - b);
  return s[Math.min(s.length - 1, Math.floor((q / 100) * s.length))];
}

export class LinkStats {
  constructor() { this.reset(); }

  reset() {
    this.t0 = Date.now();
    this.frames = this.poses = this.pots = 0;
    this.lost = this.skipped = this.dups = this.ooo = this.reboots = this.late = 0;
    this.lastSeq = null; this.lastTms = null; this.lastRx = null; this.lastMoving = false;
    this.offMin = Infinity;
    this.lat = []; this.ia = []; this.win = [];
    this.outages = [];
    this.perSec = new Map();  // second -> frames, for the rate chart
  }

  onFrame(d, rxMs, isPose) {
    this.frames++;
    if (isPose) this.poses++; else this.pots++;
    const tms = d.t_ms;
    // car reboot: seq and t_ms both restart
    if (this.lastSeq != null && d.seq < this.lastSeq && tms != null && this.lastTms != null &&
        tms < this.lastTms - 1000 && tms < 0.5 * this.lastTms) {
      this.reboots++; this.lastSeq = null; this.offMin = Infinity;
    }
    let lat = 0;
    if (tms != null) {
      const off = rxMs - tms;
      if (off < this.offMin) this.offMin = off;
      lat = off - this.offMin;
    }
    const isLate = lat > 500;
    if (isLate) this.late++;
    else { this.lat.push(lat); if (this.lat.length > RING) this.lat.shift(); }

    if (this.lastSeq != null) {
      if (d.seq === this.lastSeq) this.dups++;
      else if (d.seq < this.lastSeq) { if (!isLate) this.ooo++; }
      else {
        const g = d.seq - this.lastSeq - 1;
        if (g > 0) {
          const dt = tms != null && this.lastTms != null ? tms - this.lastTms : 0;
          if (dt >= 900 || isLate) this.skipped += g; else this.lost += g;
        }
      }
    }
    if (this.lastSeq == null || d.seq > this.lastSeq) { this.lastSeq = d.seq; this.lastTms = tms; }

    if (this.lastRx != null) {
      const gap = rxMs - this.lastRx;
      const limit = this.lastMoving ? 1000 : 15000;
      if (gap > limit) this.outages.push({ start: this.lastRx, end: rxMs, ms: gap });
      else if (!isLate) { this.ia.push(gap); if (this.ia.length > RING) this.ia.shift(); }
    }
    this.lastRx = rxMs;
    if (isPose) this.lastMoving = !!d.mv;
    this.win.push(rxMs);
    const sec = Math.floor(rxMs / 1000);
    this.perSec.set(sec, (this.perSec.get(sec) || 0) + 1);
    if (this.perSec.size > 600) this.perSec.delete(this.perSec.keys().next().value);
  }

  summary(now = Date.now()) {
    while (this.win.length && this.win[0] < now - 1000) this.win.shift();
    const expected = this.frames - this.dups + this.lost;
    const longest = this.outages.reduce((m, o) => Math.max(m, o.ms), 0);
    return {
      duration_s: (now - this.t0) / 1000,
      frames: this.frames, poses: this.poses, potholes: this.pots,
      rate_now: this.win.length,
      lost: this.lost, skipped: this.skipped, dups: this.dups, out_of_order: this.ooo, reboots: this.reboots,
      late: this.late,
      loss_pct: expected ? (100 * this.lost) / expected : 0,
      lat_p50: pct(this.lat, 50), lat_p95: pct(this.lat, 95), lat_p99: pct(this.lat, 99),
      lat_max: this.lat.length ? Math.max(...this.lat) : NaN,
      ia_p50: pct(this.ia, 50), ia_p95: pct(this.ia, 95), ia_max: this.ia.length ? Math.max(...this.ia) : NaN,
      outages: this.outages.length, outage_longest_s: longest / 1000,
      last_age_ms: this.lastRx ? now - this.lastRx : NaN,
    };
  }
}
