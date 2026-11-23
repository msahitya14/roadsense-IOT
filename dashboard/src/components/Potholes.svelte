<script>
  import { app, data } from '../lib/state.svelte.js';
  import { alertClusters } from '../lib/geo.js';
  import { f, clock } from '../lib/fmt.js';

  let pots = $derived((app.tick, data.potholes.slice(-50).reverse()));
  let clusters = $derived((app.tick, alertClusters(data.potholes)));
</script>

<div class="card">
  <h2>Potholes <span class="pill {clusters.length ? 'bad' : ''}">{clusters.length} alert{clusters.length === 1 ? '' : 's'}</span></h2>
  {#if clusters.length}
    <div class="alerts">
      {#each clusters as c}<div class="bad">⚠ {c.n} potholes within 5 m of ({f(c.x, 1)}, {f(c.y, 1)})</div>{/each}
    </div>
  {/if}
  <div class="scroll">
    <table class="kv">
      <thead><tr><td>time</td><td>x, y (m)</td><td>dip / spike (g)</td><td>pitch °/s</td></tr></thead>
      <tbody>
        {#each pots as p (p.seq + '-' + p._rx)}
          <tr><td class="mono">{clock(p._rx)}</td><td>{f(p.x_m, 1)}, {f(p.y_m, 1)}</td>
            <td>{f(p.z_neg_g)} / {f(p.z_pos_g)}</td><td>{f(p.pitch_dps, 0)}</td></tr>
        {:else}
          <tr><td colspan="4" class="mut" style="text-align:left">none yet</td></tr>
        {/each}
      </tbody>
    </table>
  </div>
</div>

<style>
  .scroll { max-height: 300px; overflow: auto; }
  thead td { color: var(--mut); font-size: 11px; text-transform: uppercase; }
  .alerts { margin-bottom: 8px; font-size: 13px; }
</style>
