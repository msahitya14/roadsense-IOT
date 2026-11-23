// Local frame: x = metres along the start bearing, y = metres to the left. Same as tools/fit_calibration.py.
export function toLatLon(x, y, o) {
  const b = (o.brg * Math.PI) / 180;
  const E = x * Math.sin(b) - y * Math.cos(b), N = x * Math.cos(b) + y * Math.sin(b);
  return [o.lat + N / 111320, o.lon + E / (111320 * Math.cos((o.lat * Math.PI) / 180))];
}

export function toLocal(lat, lon, o) {
  const b = (o.brg * Math.PI) / 180;
  const N = (lat - o.lat) * 111320, E = (lon - o.lon) * 111320 * Math.cos((o.lat * Math.PI) / 180);
  return [E * Math.sin(b) + N * Math.cos(b), -E * Math.cos(b) + N * Math.sin(b)];
}

export const SURF_COL = { smooth: '#2ecc71', medium: '#f5a623', rough: '#ff5a5f', idle: '#7f8c8d' };

// pothole alert rule from the progress report: >= 3 potholes within 5 m
export function alertClusters(pots) {
  const out = [];
  for (const p of pots) {
    const near = pots.filter((q) => Math.hypot(q.x_m - p.x_m, q.y_m - p.y_m) <= 5);
    if (near.length >= 3 && !out.some((c) => Math.hypot(c.x - p.x_m, c.y - p.y_m) <= 5))
      out.push({ x: p.x_m, y: p.y_m, n: near.length });
  }
  return out;
}
