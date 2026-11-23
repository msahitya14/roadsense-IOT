// CSV export with the browser's system receive time on every row.
export function isoLocal(ms) {
  const d = new Date(ms), z = (n, w = 2) => String(n).padStart(w, '0');
  const off = -d.getTimezoneOffset(), sign = off >= 0 ? '+' : '-';
  return `${d.getFullYear()}-${z(d.getMonth() + 1)}-${z(d.getDate())}T${z(d.getHours())}:${z(d.getMinutes())}:` +
    `${z(d.getSeconds())}.${z(d.getMilliseconds(), 3)}${sign}${z(Math.floor(Math.abs(off) / 60))}:${z(Math.abs(off) % 60)}`;
}

const COLS = ['seq', 't_ms', 'ts', 'x_m', 'y_m', 'hdg_deg', 'spd_mps', 'dist_m', 'surf', 'conf', 'vib_g', 'acc_sd', 'mv',
  'label', 'drv_s', 'yaw_raw', 'vib_int', 'z_neg_g', 'z_pos_g', 'pitch_dps'];

export function rowsToCsv(rows) {
  const esc = (v) => (v == null ? '' : /[",\n]/.test(String(v)) ? `"${String(v).replace(/"/g, '""')}"` : String(v));
  const lines = [['rx_unix', 'rx_iso', 'topic', ...COLS].join(',')];
  for (const r of rows)
    lines.push([(r._rx / 1000).toFixed(3), isoLocal(r._rx), r._topic, ...COLS.map((c) => esc(r[c]))].join(','));
  return lines.join('\n');
}

export function download(name, text, type = 'text/csv') {
  const a = document.createElement('a');
  a.href = URL.createObjectURL(new Blob([text], { type }));
  a.download = name;
  a.click();
  setTimeout(() => URL.revokeObjectURL(a.href), 1000);
}

export function stamp() {
  const d = new Date(), z = (n) => String(n).padStart(2, '0');
  return `${d.getFullYear()}${z(d.getMonth() + 1)}${z(d.getDate())}-${z(d.getHours())}${z(d.getMinutes())}${z(d.getSeconds())}`;
}
