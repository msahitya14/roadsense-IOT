// Shared dashboard state.
// Bulk data (thousands of poses) lives in plain arrays in `data`; the reactive `app.tick` is bumped at most
// 10x per second so the UI keeps up at 50-100 poses/s without re-rendering on every message.
import mqtt from 'mqtt';
import { LinkStats } from './stats.js';
import { rowsToCsv, download, stamp } from './csv.js';

export const BASE = 'rs/T24';
export const DEV = 'rs01';

export const data = { poses: [], potholes: [], testRows: [], allRows: [] };
export const live = new LinkStats();   // since the dashboard connected
export const test = new LinkStats();   // reliability test window

function load(key, dflt) {
  try { const v = localStorage.getItem('rs2_' + key); return v == null ? dflt : JSON.parse(v); } catch { return dflt; }
}
export function save(key, val) {
  try { localStorage.setItem('rs2_' + key, JSON.stringify(val)); } catch { /* private mode: ignore */ }
}

export const app = $state({
  tick: 0,
  broker: 'idle',            // idle | connecting | connected | error | closed
  conn: load('conn', { url: 'ws://localhost:9001', user: 'dashboard', pass: '' }),
  origin: load('origin', { lat: 33.643, lon: -117.8417, brg: 0 }),
  status: null,              // last rs/T24/status from the car (via the bridge)
  carOnline: false,
  lastPose: null,
  params: null,              // parsed from the car's `show` reply
  log: [],
  test: { running: false, start: null, stop: null },
  chartWindow: 60,
});

let client = null;
let dirty = false;
setInterval(() => {
  if (dirty) {
    dirty = false;
    app.lastPose = data.poses.at(-1) ?? null;
  }
  app.tick++;  // also drives "last packet age" while nothing arrives
}, 100);

export function log(text, kind = 'info') {
  app.log.push({ t: Date.now(), text, kind });
  if (app.log.length > 400) app.log.splice(0, app.log.length - 400);
}

export function connect() {
  save('conn', app.conn);
  if (client) client.end(true);
  app.broker = 'connecting';
  client = mqtt.connect(app.conn.url, {
    username: app.conn.user, password: app.conn.pass,
    clientId: 'dash-' + Math.random().toString(16).slice(2, 8), reconnectPeriod: 2000,
  });
  client.on('connect', () => {
    app.broker = 'connected';
    client.subscribe(BASE + '/#', { qos: 1 });
    log('connected to broker ' + app.conn.url, 'ok');
  });
  client.on('close', () => { if (app.broker !== 'error') app.broker = 'closed'; });
  client.on('error', (e) => { app.broker = 'error'; log('broker error: ' + e.message, 'err'); });
  client.on('message', onMessage);
}

function onMessage(topic, buf) {
  const rx = Date.now();
  const txt = buf.toString();
  let m = null;
  try { m = JSON.parse(txt); } catch { /* info is plain text */ }
  const kind = topic.slice(topic.lastIndexOf('/') + 1);

  if (kind === 'pose' && m) {
    m._rx = rx; m._topic = 'pose';
    data.poses.push(m);
    data.allRows.push(m);
    live.onFrame(m, rx, true);
    if (app.test.running) { test.onFrame(m, rx, true); data.testRows.push(m); }
    dirty = true;
  } else if (kind === 'pothole' && m) {
    m._rx = rx; m._topic = 'pothole';
    data.potholes.push(m);
    data.allRows.push(m);
    live.onFrame(m, rx, false);
    if (app.test.running) { test.onFrame(m, rx, false); data.testRows.push(m); }
    log(`POTHOLE at (${m.x_m}, ${m.y_m})  z ${m.z_neg_g}/${m.z_pos_g} g`, 'warn');
    dirty = true;
  } else if (kind === 'status' && m) {
    const wasOnline = app.carOnline;
    app.status = { ...m, _rx: rx };
    app.carOnline = m.state === 'online';
    if (app.carOnline && !wasOnline) {
      log(`car online (${m.link || 'wifi'}${m.mtu ? ', MTU ' + m.mtu : ''})`, 'ok');
      send('time ' + Math.floor(rx / 1000), true);
      send('show', true);
    } else if (!app.carOnline && wasOnline) log('car offline', 'err');
  } else if (kind === 'info') {
    if (txt.startsWith('params ')) {
      try { app.params = JSON.parse(txt.slice(7)); } catch { /* ignore */ }
    }
    log('car: ' + txt, 'car');
  }
}

export function send(cmd, quiet = false) {
  if (!client || !client.connected) { log('not connected to broker', 'err'); return; }
  client.publish(`${BASE}/cmd/${DEV}`, cmd, { qos: 1 });
  if (!quiet) log('> ' + cmd, 'cmd');
  if (cmd === 'reset') clearPath();
}

export function clearPath() {
  data.poses = [];
  data.potholes = [];
  dirty = true;
  app.tick++;
}

// ---------------------------------------------------------------- reliability test
export function startTest() {
  test.reset();
  data.testRows = [];
  app.test = { running: true, start: Date.now(), stop: null };
  log('reliability test started', 'ok');
}
export function stopTest() {
  app.test = { ...app.test, running: false, stop: Date.now() };
  log('reliability test stopped', 'ok');
}
export function exportTest() {
  download(`reliability-${stamp()}.csv`, rowsToCsv(data.testRows));
}
export function exportAll() {
  download(`roadsense-${stamp()}.csv`, rowsToCsv(data.allRows));
}

// reconnect automatically after a page reload when a login was saved
if (app.conn.pass) setTimeout(connect, 0);
