<script>
  import { app, data, live } from '../lib/state.svelte.js';
  import TimeChart from './TimeChart.svelte';

  // time window of poses (by browser receive time)
  function tail() {
    const P = data.poses, t0 = Date.now() - app.chartWindow * 1000;
    let i = P.length;
    while (i > 0 && P[i - 1]._rx >= t0) i--;
    return P.slice(i);
  }
  const col = (rows, k) => rows.map((p) => p[k] ?? null);
  const xs = (rows) => rows.map((p) => p._rx / 1000);

  function motion() { const r = tail(); return [xs(r), col(r, 'spd_mps'), col(r, 'hdg_deg')]; }
  function vib() { const r = tail(); return [xs(r), col(r, 'vib_g'), col(r, 'acc_sd')]; }
  function linkData() {
    // frames per second (bar-like line) and per-pose latency above best case
    const now = Math.floor(Date.now() / 1000), n = app.chartWindow, x = [], rate = [];
    for (let s = now - n; s <= now; s++) { x.push(s); rate.push(live.perSec.get(s) || 0); }
    return [x, rate];
  }
  function pos() { const r = tail(); return [xs(r), col(r, 'x_m'), col(r, 'y_m')]; }
</script>

<div class="card">
  <h2>Time series
    <select bind:value={app.chartWindow}>
      <option value={30}>30 s</option><option value={60}>60 s</option><option value={120}>2 min</option>
      <option value={300}>5 min</option>
    </select>
  </h2>
  <div class="grid">
    <TimeChart title="Speed (m/s) and heading (°)" tick={app.tick} getData={motion}
      series={[{ label: 'speed m/s', color: '#4aa3ff' }, { label: 'heading °', color: '#f5a623', scale: 'r' }]}
      scales={{ r: { range: [0, 360] } }} />
    <TimeChart title="Vibration (g)" tick={app.tick} getData={vib}
      series={[{ label: 'vib RMS', color: '#2ecc71' }, { label: 'accel sd', color: '#c38cff' }]} />
    <TimeChart title="Position (m)" tick={app.tick} getData={pos}
      series={[{ label: 'x', color: '#4aa3ff' }, { label: 'y', color: '#ff5a5f' }]} />
    <TimeChart title="Received frames per second" tick={app.tick} getData={linkData}
      series={[{ label: 'frames/s', color: '#8ba0b3' }]} />
  </div>
</div>

<style>
  .grid { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 12px; }
  @media (max-width: 800px) { .grid { grid-template-columns: minmax(0, 1fr); } }
</style>
