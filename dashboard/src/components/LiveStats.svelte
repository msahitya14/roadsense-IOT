<script>
  import { app, data } from '../lib/state.svelte.js';
  import { SURF_COL } from '../lib/geo.js';
  import { f } from '../lib/fmt.js';

  let p = $derived(app.lastPose);
  // distance driven per surface class (from consecutive dist_m differences), updated incrementally
  let acc = {}, done = 1;
  let bySurf = $derived.by(() => {
    app.tick;
    const P = data.poses;
    if (P.length < done) { acc = {}; done = 1; }
    for (; done < P.length; done++) {
      const d = P[done].dist_m - P[done - 1].dist_m;
      if (d > 0 && d < 50) acc[P[done].surf] = (acc[P[done].surf] || 0) + d;
    }
    return { ...acc };
  });
  let total = $derived(Object.values(bySurf).reduce((a, b) => a + b, 0));
</script>

<div class="card">
  <h2>Live <span class="pill {p?.mv ? 'ok' : ''}">{p ? (p.mv ? 'DRIVING' : 'parked') : '–'}</span>
    {#if p?.label && p.label !== 'none'}<span class="pill warn">label {p.label}</span>{/if}</h2>
  <div class="tiles">
    <div><b>{f(p?.x_m)}</b><span>x (m)</span></div>
    <div><b>{f(p?.y_m)}</b><span>y (m)</span></div>
    <div><b>{f(p?.hdg_deg, 1)}</b><span>heading (°)</span></div>
    <div><b>{f(p?.spd_mps)}</b><span>speed (m/s)</span></div>
    <div><b>{f(p?.dist_m)}</b><span>distance (m)</span></div>
    <div><b>{(app.tick, data.potholes.length)}</b><span>potholes</span></div>
    <div><b style="color:{SURF_COL[p?.surf] || 'inherit'}">{p?.surf ?? '–'}</b><span>surface ({f(p?.conf)})</span></div>
    <div><b>{f(p?.vib_g, 3)}</b><span>vib RMS (g)</span></div>
    <div><b>{f(p?.acc_sd, 3)}</b><span>accel sd (g)</span></div>
    <div><b>{f(p?.drv_s, 1)}</b><span>driving (s)</span></div>
    <div><b>{f(p?.yaw_raw, 1)}</b><span>raw yaw (°)</span></div>
    <div><b>{p?.seq ?? '–'}</b><span>seq</span></div>
  </div>
  {#if total > 0}
    <div class="bar" title="distance per surface">
      {#each Object.entries(bySurf) as [s, d]}
        <div style="width:{(100 * d) / total}%; background:{SURF_COL[s] || '#aaa'}" title="{s}: {f(d, 1)} m"></div>
      {/each}
    </div>
    <div class="mut small">{#each Object.entries(bySurf) as [s, d]}{s} {f(d, 1)} m · {/each}</div>
  {/if}
</div>

<style>
  .tiles { display: grid; grid-template-columns: repeat(3, 1fr); gap: 6px; }
  .tiles div { background: var(--card2); border-radius: 8px; padding: 6px 8px; }
  .tiles b { display: block; font-size: 18px; font-variant-numeric: tabular-nums; }
  .tiles span { color: var(--mut); font-size: 11px; }
  .bar { display: flex; height: 8px; border-radius: 4px; overflow: hidden; margin-top: 10px; }
  .small { font-size: 11px; margin-top: 4px; }
</style>
