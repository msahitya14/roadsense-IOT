<script>
  // Thin uPlot wrapper: `series` = [{label, color, scale?}], `getData()` returns [xs, ...ys] (xs in unix seconds)
  import { onMount } from 'svelte';
  import uPlot from 'uplot';

  let { title = '', series = [], getData, height = 160, scales = {}, tick = 0 } = $props();
  let el, plot;

  onMount(() => {
    const opts = {
      width: el.clientWidth || 400, height,
      legend: { show: true },
      cursor: { drag: { x: false, y: false } },
      scales: { x: { time: true }, ...scales },
      axes: [
        { stroke: '#8ba0b3', grid: { stroke: '#1c2834' }, ticks: { stroke: '#2a3846' } },
        { stroke: '#8ba0b3', grid: { stroke: '#1c2834' }, ticks: { stroke: '#2a3846' }, scale: series[0]?.scale || 'y' },
        ...(series.some((s) => s.scale === 'r')
          ? [{ side: 1, scale: 'r', stroke: '#8ba0b3', grid: { show: false }, ticks: { stroke: '#2a3846' } }] : []),
      ],
      series: [{}, ...series.map((s) => ({ label: s.label, stroke: s.color, width: 1.5, scale: s.scale || 'y',
        points: { show: s.points ?? false, size: 3, fill: s.color }, paths: s.points ? () => null : undefined }))],
    };
    plot = new uPlot(opts, getData(), el);
    const ro = new ResizeObserver(() => plot.setSize({ width: el.clientWidth, height }));
    ro.observe(el);
    return () => { ro.disconnect(); plot.destroy(); };
  });

  $effect(() => { tick; if (plot) plot.setData(getData()); });
</script>

<div class="wrap">
  {#if title}<div class="t mut">{title}</div>{/if}
  <div bind:this={el}></div>
</div>

<style>
  .wrap { min-width: 0; }
  .t { font-size: 12px; margin-bottom: 2px; }
  :global(.uplot) { font-family: inherit; }
  :global(.u-legend) { color: var(--fg); font-size: 12px; }
</style>
