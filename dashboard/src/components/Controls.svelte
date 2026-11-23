<script>
  import { app, send } from '../lib/state.svelte.js';

  let meters = $state('');
  let degrees = $state(360);
  let label = $state('none');
  let txHz = $state(50);
  let parkS = $state(5);
  let raw = $state('');
  let rawOn = $state(false);

  $effect(() => {  // follow the car's saved parameters when `show` replies
    if (app.params?.txHz != null) txHz = app.params.txHz;
    if (app.params?.parkS != null) parkS = app.params.parkS;
  });
  const PARAM_ORDER = ['v0', 'spdMode', 'kVib', 'vibRef', 'gyroScale', 'movTh', 'txHz', 'parkS', 'surfT1', 'surfT2',
    'potNeg', 'potPos', 'potBig', 'label'];
</script>

<div class="card">
  <h2>Car control &amp; calibration</h2>
  <div class="row">
    <button onclick={() => send('calib')} title="car must be still for 2 s">calib (still)</button>
    <button onclick={() => send('reset')}>reset pose</button>
    <button onclick={() => send('show')}>show params</button>
  </div>
  <div class="row">
    <button onclick={() => send('dcal start')}>dcal start</button>
    <input type="number" step="any" placeholder="metres" bind:value={meters} />
    <button disabled={!meters} onclick={() => send('dcal end ' + meters)}>dcal end</button>
  </div>
  <div class="row">
    <button onclick={() => send('tcal start')}>tcal start</button>
    <input type="number" step="any" bind:value={degrees} />
    <button onclick={() => send('tcal end ' + degrees)}>tcal end (°)</button>
  </div>
  <div class="row">
    label
    <input list="labels" bind:value={label} style="width:120px" />
    <datalist id="labels"><option>none</option><option>run01</option><option>asphalt</option><option>concrete</option>
      <option>gravel</option><option>grass</option><option>pothole_run</option></datalist>
    <button onclick={() => send('label ' + (label || 'none'))}>set</button>
  </div>
  <div class="row">
    rate driving <input type="number" min="1" max="100" bind:value={txHz} /> Hz
    <button onclick={() => send('set txHz ' + txHz)}>set</button>
    parked every <input type="number" min="0.2" step="0.5" bind:value={parkS} /> s
    <button onclick={() => send('set parkS ' + parkS)}>set</button>
  </div>
  <div class="row">
    <button class:primary={rawOn} onclick={() => { rawOn = !rawOn; send(rawOn ? 'raw on' : 'raw off'); }}>
      raw IMU 200 Hz: {rawOn ? 'ON' : 'off'}</button>
    <span class="mut">saved by the bridge to logs/raw-*.csv</span>
  </div>
  <form class="row" onsubmit={(e) => { e.preventDefault(); if (raw) { send(raw); raw = ''; } }}>
    <input bind:value={raw} placeholder="any command, e.g. set movTh 0.04" style="flex:1" />
    <button>send</button>
  </form>
  {#if app.params}
    <table class="kv params">
      <tbody>
        {#each PARAM_ORDER.filter((k) => app.params[k] != null) as k}
          <tr><td class="mono">{k}</td><td>{app.params[k]}</td></tr>
        {/each}
      </tbody>
    </table>
  {/if}
</div>

<style>
  .params { margin-top: 8px; columns: 2; }
</style>
