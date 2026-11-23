<script>
  import Header from './components/Header.svelte';
  import MapView from './components/MapView.svelte';
  import XYPlot from './components/XYPlot.svelte';
  import LiveStats from './components/LiveStats.svelte';
  import Charts from './components/Charts.svelte';
  import LinkHealth from './components/LinkHealth.svelte';
  import Reliability from './components/Reliability.svelte';
  import Controls from './components/Controls.svelte';
  import Potholes from './components/Potholes.svelte';
  import EventLog from './components/EventLog.svelte';
</script>

<Header />
<main>
  <section class="map"><MapView /></section>
  <section class="xy"><XYPlot /></section>
  <aside class="side">
    <LiveStats />
    <LinkHealth />
  </aside>
  <section class="charts"><Charts /></section>
  <aside class="side2">
    <Reliability />
  </aside>
  <section class="ctrl"><Controls /></section>
  <section class="pots"><Potholes /></section>
  <section class="log"><EventLog /></section>
</main>

<style>
  main {
    display: grid; gap: 12px; padding: 12px;
    grid-template-columns: minmax(0, 1.2fr) minmax(0, 1fr) 380px;
    grid-template-areas:
      'map xy side'
      'charts charts side2'
      'ctrl pots log';
  }
  .map { grid-area: map; } .xy { grid-area: xy; } .side { grid-area: side; display: grid; gap: 12px; align-content: start; }
  .charts { grid-area: charts; } .side2 { grid-area: side2; }
  .ctrl { grid-area: ctrl; } .pots { grid-area: pots; } .log { grid-area: log; }
  section, aside { min-width: 0; }
  .map, .xy { display: flex; flex-direction: column; }
  .map > :global(.card), .xy > :global(.card) { flex: 1; display: flex; flex-direction: column; }
  @media (max-width: 1300px) {
    main { grid-template-columns: minmax(0, 1fr) minmax(0, 1fr);
      grid-template-areas: 'map xy' 'side side2' 'charts charts' 'ctrl pots' 'log log'; }
  }
  @media (max-width: 800px) {
    main { grid-template-columns: minmax(0, 1fr);
      grid-template-areas: 'map' 'side' 'side2' 'xy' 'charts' 'ctrl' 'pots' 'log'; padding: 8px 16px; }
  }
</style>
