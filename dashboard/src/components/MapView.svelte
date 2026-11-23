<script>
  import { onMount } from 'svelte';
  import L from 'leaflet';
  import { app, data, send, save, log } from '../lib/state.svelte.js';
  import { toLatLon, toLocal, SURF_COL, alertClusters } from '../lib/geo.js';
  import { f } from '../lib/fmt.js';

  let el;
  let map, pathLayer, potLayer, originMk, carMk;
  let drawn = 0, curLine = null, curSurf = null, potDrawn = 0;
  let mode = $state(null);   // 'origin' | 'true' | 'anchor'
  let errOut = $state('');
  let tilesOk = $state(true);

  const ll = (p) => toLatLon(p.x_m, p.y_m, app.origin);

  function extend(p, prev) {
    if (!curLine || p.surf !== curSurf) {
      curLine = L.polyline(prev ? [ll(prev), ll(p)] : [ll(p)], { color: SURF_COL[p.surf] || '#aaa', weight: 5 }).addTo(pathLayer);
      curSurf = p.surf;
    } else curLine.addLatLng(ll(p));
  }

  function redrawAll() {
    if (!map) return;
    pathLayer.clearLayers(); potLayer.clearLayers();
    curLine = null; curSurf = null; drawn = 0; potDrawn = 0;
    originMk.setLatLng([app.origin.lat, app.origin.lon]);
    sync();
  }

  function sync() {
    const P = data.poses;
    if (P.length < drawn) { redrawAll(); return; }  // path was cleared
    for (; drawn < P.length; drawn++) extend(P[drawn], drawn ? P[drawn - 1] : null);
    const H = data.potholes;
    if (H.length < potDrawn) { redrawAll(); return; }
    if (H.length > potDrawn) {
      for (; potDrawn < H.length; potDrawn++) {
        const h = H[potDrawn];
        L.circleMarker(ll(h), { radius: 5, color: '#ff5a5f', fillOpacity: 1 })
          .bindTooltip(`pothole ${f(h.z_neg_g)}/${f(h.z_pos_g)} g`).addTo(potLayer);
      }
      for (const c of alertClusters(H))
        L.circle(toLatLon(c.x, c.y, app.origin), { radius: 5, color: '#ff5a5f', fill: false, weight: 3 })
          .bindTooltip(`ALERT: ${c.n} potholes within 5 m`).addTo(potLayer);
    }
    if (P.length) carMk.setLatLng(ll(P.at(-1))).addTo(map);
  }

  onMount(() => {
    map = L.map(el, { zoomControl: true }).setView([app.origin.lat, app.origin.lon], 19);
    const tiles = L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',
      { maxZoom: 22, maxNativeZoom: 19, attribution: '© OpenStreetMap' }).addTo(map);
    tiles.on('tileerror', () => { tilesOk = false; });
    pathLayer = L.layerGroup().addTo(map);
    potLayer = L.layerGroup().addTo(map);
    originMk = L.circleMarker([app.origin.lat, app.origin.lon], { radius: 6, color: '#4aa3ff' }).addTo(map).bindTooltip('start');
    carMk = L.circleMarker([0, 0], { radius: 7, color: '#fff', fillColor: '#4aa3ff', fillOpacity: 1 });
    map.on('click', onClick);
    const ro = new ResizeObserver(() => map.invalidateSize());
    ro.observe(el);
    return () => { ro.disconnect(); map.remove(); };
  });

  $effect(() => { app.tick; sync(); });

  function setOrigin() { save('origin', app.origin); redrawAll(); }

  function onClick(e) {
    const { lat, lng } = e.latlng;
    if (mode === 'origin') {
      app.origin.lat = +lat.toFixed(7); app.origin.lon = +lng.toFixed(7);
      setOrigin();
      log(`start point set to ${lat.toFixed(6)}, ${lng.toFixed(6)}`);
    } else if ((mode === 'true' || mode === 'anchor') && app.lastPose) {
      const [tx, ty] = toLocal(lat, lng, app.origin), p = app.lastPose;
      const err = Math.hypot(tx - p.x_m, ty - p.y_m), d = p.dist_m || 0;
      errOut = `error ${f(err)} m${d ? ` (${f((100 * err) / d, 1)}% of ${f(d, 1)} m driven)` : ''}`;
      log(`true position (${f(tx)}, ${f(ty)}) vs estimate (${f(p.x_m)}, ${f(p.y_m)}): ${errOut}`);
      if (mode === 'anchor') send(`anchor ${tx.toFixed(2)} ${ty.toFixed(2)}`);
    }
    mode = null;
  }
</script>

<div class="card">
  <h2>Map <span class="mut">dead-reckoned path on real coordinates</span></h2>
  <div class="row">
    start lat <input type="number" step="any" bind:value={app.origin.lat} onchange={setOrigin} />
    lon <input type="number" step="any" bind:value={app.origin.lon} onchange={setOrigin} />
    bearing° <input type="number" step="any" bind:value={app.origin.brg} onchange={setOrigin}
      title="compass direction the car faces at reset (0 = N, 90 = E)" />
    <button class:primary={mode === 'origin'} onclick={() => (mode = 'origin')}>Pick start on map</button>
  </div>
  <div class="map" bind:this={el} class:picking={mode}></div>
  <div class="row">
    <button class:primary={mode === 'true'} onclick={() => (mode = 'true')}>Click true position → error</button>
    <button class:primary={mode === 'anchor'} onclick={() => (mode = 'anchor')}>Click true position → re-anchor</button>
    <span class="mut">{errOut}</span>
  </div>
  <div class="legend mut">
    <span style="background:#2ecc71"></span>smooth <span style="background:#f5a623"></span>medium
    <span style="background:#ff5a5f"></span>rough <span style="background:#7f8c8d"></span>idle · ● pothole · ◯ alert (≥3 within 5 m)
    {#if !tilesOk}· <b class="warnc">no map tiles (offline)</b>{/if}
  </div>
</div>

<style>
  .map { flex: 1; min-height: 440px; border-radius: 8px; }
  .picking { cursor: crosshair; outline: 2px dashed var(--acc); }
  .legend span { display: inline-block; width: 10px; height: 10px; border-radius: 2px; margin: 0 3px 0 8px; }
</style>
