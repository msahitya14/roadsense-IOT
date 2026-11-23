<script>
  import { app, data, test, startTest, stopTest, exportTest, exportAll, log } from '../lib/state.svelte.js';
  import { f, dur, clock } from '../lib/fmt.js';

  let s = $derived((app.tick, test.summary(app.test.running ? Date.now() : app.test.stop ?? Date.now())));
  let has = $derived(app.test.start != null);

  function summaryText() {
    return [
      `RoadSense reliability test ${new Date(app.test.start).toISOString()} -> ${app.test.stop ? new Date(app.test.stop).toISOString() : 'running'}`,
      `duration ${dur(s.duration_s)}, ${s.poses} poses, ${s.potholes} potholes`,
      `loss ${s.lost} (${f(s.loss_pct, 3)} %), skipped by offline buffer ${s.skipped}, duplicates ${s.dups}, out-of-order ${s.out_of_order}`,
      `latency above best p50/p95/p99/max ${f(s.lat_p50, 0)}/${f(s.lat_p95, 0)}/${f(s.lat_p99, 0)}/${f(s.lat_max, 0)} ms`,
      `inter-arrival p50/p95/max ${f(s.ia_p50, 0)}/${f(s.ia_p95, 0)}/${f(s.ia_max, 0)} ms`,
      `outages ${s.outages} (longest ${f(s.outage_longest_s, 1)} s), car reboots ${s.reboots}`,
    ].join('\n');
  }
  async function copy() {
    try { await navigator.clipboard.writeText(summaryText()); log('test summary copied to clipboard', 'ok'); }
    catch { log(summaryText()); }
  }
</script>

<div class="card">
  <h2>Reliability test</h2>
  <div class="row">
    {#if app.test.running}
      <button class="danger" onclick={stopTest}>■ Stop test</button>
    {:else}
      <button class="primary" onclick={startTest}>● Start test</button>
    {/if}
    <button disabled={!has} onclick={exportTest}>Export test CSV</button>
    <button disabled={!has} onclick={copy}>Copy summary</button>
  </div>
  <p class="mut hint">Counts every pose and pothole received while the test runs. The CSV has the system receive
    time (<span class="mono">rx_unix</span>, <span class="mono">rx_iso</span>) and the car's clock
    (<span class="mono">t_ms</span>) on every row. For the full analysis, use the bridge's
    <span class="mono">logs/pose-*.csv</span> with <span class="mono">tools/reliability_report.py</span>.</p>
  {#if has}
    <table class="kv">
      <tbody>
        <tr><td>Window</td><td>{clock(app.test.start)} → {app.test.running ? 'now' : clock(app.test.stop)} ({dur(s.duration_s)})</td></tr>
        <tr><td>Received</td><td>{s.poses} poses · {s.potholes} potholes · {f(s.frames / Math.max(s.duration_s, 1e-3), 1)}/s</td></tr>
        <tr><td>Lost in transit</td><td class={s.loss_pct > 2 ? 'bad' : s.loss_pct > 0.5 ? 'warnc' : 'good'}>{s.lost} ({f(s.loss_pct, 3)} %)</td></tr>
        <tr><td>Skipped by offline buffer</td><td>{s.skipped}</td></tr>
        <tr><td>Duplicates · out of order · reboots</td><td>{s.dups} · {s.out_of_order} · {s.reboots}</td></tr>
        <tr><td>Latency p50 / p95 / p99 / max</td><td>{f(s.lat_p50, 0)} / {f(s.lat_p95, 0)} / {f(s.lat_p99, 0)} / {f(s.lat_max, 0)} ms</td></tr>
        <tr><td>Inter-arrival p50 / p95 / max</td><td>{f(s.ia_p50, 0)} / {f(s.ia_p95, 0)} / {f(s.ia_max, 0)} ms</td></tr>
        <tr><td>Outages · longest</td><td>{s.outages} · {f(s.outage_longest_s, 1)} s</td></tr>
      </tbody>
    </table>
    {#if test.outages.length}
      <div class="mut small">Outages:
        {#each test.outages.slice(-6) as o}<div class="mono">{clock(o.start)} → {clock(o.end)} ({f(o.ms / 1000, 1)} s)</div>{/each}
      </div>
    {/if}
  {/if}
  <div class="row"><button onclick={exportAll}>Export whole session CSV ({(app.tick, data.allRows.length)} rows)</button></div>
</div>

<style>
  .hint { font-size: 12px; margin: 4px 0 8px; }
  .small { font-size: 12px; margin-top: 6px; }
</style>
