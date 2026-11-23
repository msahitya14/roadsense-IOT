<script>
  import { app, live } from '../lib/state.svelte.js';
  import { f, dur } from '../lib/fmt.js';

  let s = $derived((app.tick, live.summary()));
  let st = $derived(app.status);
  const cls = (v, warn, bad) => (Number.isNaN(v) ? '' : v >= bad ? 'bad' : v >= warn ? 'warnc' : 'good');
</script>

<div class="card">
  <h2>Link health <span class="mut">since dashboard connected · {dur(s.duration_s)}</span></h2>
  <table class="kv">
    <tbody>
      <tr><td>Receive rate now</td><td>{s.rate_now} /s {#if st?.tx_hz != null}<span class="mut">(car sent {f(st.tx_hz, 1)}/s)</span>{/if}</td></tr>
      <tr><td>Last frame</td><td class={cls(s.last_age_ms, 1000, 5000)}>{Number.isNaN(s.last_age_ms) ? '–' : f(s.last_age_ms / 1000, 1) + ' s ago'}</td></tr>
      <tr><td>Frames received</td><td>{s.poses} poses · {s.potholes} potholes</td></tr>
      <tr><td>Lost in transit</td><td class={cls(s.loss_pct, 0.5, 2)}>{s.lost} ({f(s.loss_pct, 3)} %)</td></tr>
      <tr><td>Skipped by offline buffer</td><td>{s.skipped} <span class="mut">· {s.late} delivered late</span></td></tr>
      <tr><td>Duplicates · out of order</td><td>{s.dups} · {s.out_of_order}</td></tr>
      <tr><td>Latency above best p50 / p95 / max</td><td class={cls(s.lat_p95, 100, 500)}>{f(s.lat_p50, 0)} / {f(s.lat_p95, 0)} / {f(s.lat_max, 0)} ms</td></tr>
      <tr><td>Inter-arrival p50 / p95 / max</td><td>{f(s.ia_p50, 0)} / {f(s.ia_p95, 0)} / {f(s.ia_max, 0)} ms</td></tr>
      <tr><td>Outages (&gt;1 s driving)</td><td class={cls(s.outages, 1, 5)}>{s.outages}{s.outages ? ` · longest ${f(s.outage_longest_s, 1)} s` : ''}</td></tr>
      {#if st}
        <tr><td colspan="2" class="sub mut">car status ({st.link || 'wifi'}) · {new Date(st._rx).toLocaleTimeString([], { hour12: false })}</td></tr>
        {#if st.mtu}<tr><td>BLE MTU</td><td>{st.mtu}</td></tr>{/if}
        {#if st.rssi}<tr><td>Wi-Fi RSSI</td><td>{st.rssi} dBm</td></tr>{/if}
        <tr><td>Uptime</td><td>{dur(st.up_s)}</td></tr>
        <tr><td>Queued on car · dropped</td><td class={cls(st.dropped ?? 0, 1, 100)}>{st.queued ?? '–'} · {st.dropped ?? '–'}</td></tr>
        <tr><td>Reconnects · BLE errors · IMU errors</td><td>{st.reconnects ?? '–'} · {st.ble_err ?? '–'} · {st.imu_err ?? '–'}</td></tr>
      {/if}
    </tbody>
  </table>
</div>

<style>
  .sub { text-align: left !important; padding-top: 8px; font-size: 11px; text-transform: uppercase; letter-spacing: 0.05em; }
</style>
