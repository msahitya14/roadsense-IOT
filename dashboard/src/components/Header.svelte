<script>
  import { app, connect, live } from '../lib/state.svelte.js';
  import { f } from '../lib/fmt.js';

  let s = $derived((app.tick, live.summary()));
  let brokerCls = $derived(app.broker === 'connected' ? 'ok' : app.broker === 'connecting' ? 'warn' : 'bad');
  let age = $derived(s.last_age_ms);
  let stale = $derived(app.carOnline && age > (app.lastPose?.mv ? 2000 : 15000));
  let carCls = $derived(!app.status ? '' : !app.carOnline ? 'bad' : stale ? 'warn' : 'ok');
  let carTxt = $derived(!app.status ? 'car: –' : !app.carOnline ? 'car: offline'
    : `car: ${app.status.link === 'ble' ? 'BLE' : 'Wi-Fi'}${stale ? ' (no data ' + f(age / 1000, 0) + ' s)' : ''}`);
</script>

<header>
  <h1>RoadSense</h1>
  <form class="row" onsubmit={(e) => { e.preventDefault(); connect(); }}>
    <input class="url" bind:value={app.conn.url} title="broker WebSocket URL" />
    <input class="usr" bind:value={app.conn.user} title="user" />
    <input class="pw" type="password" bind:value={app.conn.pass} placeholder="password (broker/credentials.txt)" />
    <button class="primary">Connect</button>
  </form>
  <span class="pill {brokerCls}">broker: {app.broker}</span>
  <span class="pill {carCls}">{carTxt}</span>
  <span class="pill">{s.rate_now} frames/s</span>
  {#if app.test.running}<span class="pill warn">● test running</span>{/if}
</header>

<style>
  header { display: flex; gap: 10px; align-items: center; padding: 10px 16px; border-bottom: 1px solid var(--line); flex-wrap: wrap; }
  h1 { font-size: 18px; margin: 0 8px 0 0; }
  form { margin: 0; }
  .url { width: 200px; } .usr { width: 100px; } .pw { width: 230px; }
</style>
