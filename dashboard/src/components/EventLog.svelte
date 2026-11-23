<script>
  import { app } from '../lib/state.svelte.js';
  import { clock } from '../lib/fmt.js';

  let el;
  let filter = $state('all');
  let rows = $derived(filter === 'all' ? app.log : app.log.filter((l) => l.kind === filter));
  $effect(() => { rows.length; if (el) queueMicrotask(() => (el.scrollTop = el.scrollHeight)); });
</script>

<div class="card">
  <h2>Event log
    <select bind:value={filter}>
      <option value="all">all</option><option value="car">car replies</option><option value="cmd">commands</option>
      <option value="warn">potholes</option><option value="err">errors</option>
    </select>
    <button onclick={() => (app.log = [])}>clear</button>
  </h2>
  <div class="log" bind:this={el}>
    {#each rows as l}
      <div class={l.kind}><span class="mut">{clock(l.t)}</span> {l.text}</div>
    {/each}
  </div>
</div>

<style>
  .log { height: 300px; overflow: auto; background: var(--card2); border-radius: 8px; padding: 8px;
    font: 12px ui-monospace, Menlo, monospace; white-space: pre-wrap; }
  .ok { color: var(--ok); } .err { color: var(--bad); } .warn { color: var(--warn); } .cmd { color: var(--acc); }
</style>
