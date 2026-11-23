<script>
  import { app, data, clearPath } from '../lib/state.svelte.js';
  import { SURF_COL } from '../lib/geo.js';

  let cv;
  let follow = $state(false);

  function draw() {
    if (!cv) return;
    const ctx = cv.getContext('2d');
    const dpr = devicePixelRatio || 1, W = cv.clientWidth, H = cv.clientHeight;
    if (cv.width !== W * dpr || cv.height !== H * dpr) { cv.width = W * dpr; cv.height = H * dpr; }
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, W, H);
    const P = data.poses, Hs = data.potholes;
    // bounds: whole path incl. the start point, or (follow) only the last 500 poses
    const src = follow && P.length ? P.slice(-500) : P;
    let minX = follow && P.length ? Infinity : 0, maxX = -minX, minY = minX, maxY = -minX;
    for (const p of src) { minX = Math.min(minX, p.x_m); maxX = Math.max(maxX, p.x_m); minY = Math.min(minY, p.y_m); maxY = Math.max(maxY, p.y_m); }
    const span = Math.max(maxX - minX, maxY - minY, 4) * 1.15, cx = (minX + maxX) / 2, cy = (minY + maxY) / 2;
    const s = Math.min(W, H) / span;
    const T = (x, y) => [W / 2 - (y - cy) * s, H / 2 - (x - cx) * s];  // x forward = up, y left = left
    const step = span > 200 ? 50 : span > 60 ? 10 : span > 20 ? 5 : 1;
    ctx.strokeStyle = '#1c2834'; ctx.lineWidth = 1; ctx.fillStyle = '#4b5d6e'; ctx.font = '10px system-ui';
    for (let g = Math.floor((cx - span) / step) * step; g <= cx + span; g += step) {
      const [, yy] = T(g, 0); ctx.beginPath(); ctx.moveTo(0, yy); ctx.lineTo(W, yy); ctx.stroke(); ctx.fillText(g + ' m', 2, yy - 2);
    }
    for (let g = Math.floor((cy - span) / step) * step; g <= cy + span; g += step) {
      const [xx] = T(0, g); ctx.beginPath(); ctx.moveTo(xx, 0); ctx.lineTo(xx, H); ctx.stroke();
    }
    ctx.lineWidth = 3;
    for (let i = 1; i < P.length; ) {
      const surf = P[i].surf;
      ctx.strokeStyle = SURF_COL[surf] || '#aaa'; ctx.beginPath(); ctx.moveTo(...T(P[i - 1].x_m, P[i - 1].y_m));
      for (; i < P.length && P[i].surf === surf; i++) ctx.lineTo(...T(P[i].x_m, P[i].y_m));
      ctx.stroke();
    }
    ctx.fillStyle = '#4aa3ff'; ctx.beginPath(); ctx.arc(...T(0, 0), 5, 0, 7); ctx.fill();
    ctx.fillStyle = '#ff5a5f';
    for (const h of Hs) { ctx.beginPath(); ctx.arc(...T(h.x_m, h.y_m), 5, 0, 7); ctx.fill(); }
    if (P.length) {
      const l = P.at(-1), [px, py] = T(l.x_m, l.y_m), a = (l.hdg_deg * Math.PI) / 180;
      ctx.save(); ctx.translate(px, py); ctx.rotate(-a); ctx.fillStyle = '#fff';
      ctx.beginPath(); ctx.moveTo(0, -12); ctx.lineTo(7, 8); ctx.lineTo(-7, 8); ctx.closePath(); ctx.fill(); ctx.restore();
    }
  }

  $effect(() => { app.tick; follow; draw(); });
</script>

<div class="card">
  <h2>Local frame <span class="mut">x forward at start, y left · metres</span></h2>
  <canvas bind:this={cv}></canvas>
  <div class="row">
    <label><input type="checkbox" bind:checked={follow} /> follow car</label>
    <button onclick={clearPath}>Clear path</button>
    <span class="mut">{(app.tick, data.poses.length)} poses · {data.potholes.length} potholes</span>
  </div>
</div>

<style>
  canvas { width: 100%; flex: 1; min-height: 440px; background: var(--card2); border-radius: 8px; display: block; }
</style>
