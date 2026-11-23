export const f = (v, d = 2) => (v == null || Number.isNaN(v) ? '–' : Number(v).toFixed(d));
export const clock = (ms) => (ms ? new Date(ms).toLocaleTimeString([], { hour12: false }) : '–');
export function dur(s) {
  if (s == null || Number.isNaN(s)) return '–';
  const h = Math.floor(s / 3600), m = Math.floor((s % 3600) / 60), r = Math.floor(s % 60);
  return h ? `${h}h ${m}m ${r}s` : m ? `${m}m ${r}s` : `${r}s`;
}
