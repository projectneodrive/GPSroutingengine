// Fetch elevations for every node of a .osm extract and write
// elevations.csv (node_id,elevation_m) which the backend loads automatically.
//
// Uses the free OpenTopoData SRTM 30 m API : 100 locations per
// request, 1 request per second. A city extract (~20k nodes) takes ~3-4 min.
//
// Usage:  node fetch_elevation.mjs [map.osm] [elevations.csv]

import { readFileSync, writeFileSync, existsSync, appendFileSync } from 'node:fs';

const mapPath = process.argv[2] ?? new URL('./map.osm', import.meta.url).pathname.replace(/^\/([A-Za-z]:)/, '$1');
const outPath = process.argv[3] ?? mapPath.replace(/map\.osm$/, 'elevations.csv');

console.log(`Reading ${mapPath} ...`);
const xml = readFileSync(mapPath, 'utf8');

// Only nodes referenced by ways matter for routing.
const referenced = new Set();
for (const m of xml.matchAll(/<nd ref="(\d+)"/g)) referenced.add(m[1]);

const nodes = [];
for (const m of xml.matchAll(/<node id="(\d+)"[^>]*? lat="([0-9.\-]+)" lon="([0-9.\-]+)"/g)) {
  if (referenced.has(m[1])) nodes.push({ id: m[1], lat: +m[2], lon: +m[3] });
}
console.log(`${nodes.length} routing nodes need elevation.`);

// Resume support: skip ids already present in the output file.
const done = new Set();
if (existsSync(outPath)) {
  for (const line of readFileSync(outPath, 'utf8').split('\n')) {
    const c = line.indexOf(',');
    if (c > 0) done.add(line.slice(0, c));
  }
  console.log(`${done.size} already fetched - resuming.`);
} else {
  writeFileSync(outPath, '');
}

const todo = nodes.filter(n => !done.has(n.id));
const BATCH = 100;
const sleep = ms => new Promise(r => setTimeout(r, ms));

for (let i = 0; i < todo.length; i += BATCH) {
  const batch = todo.slice(i, i + BATCH);
  const locations = batch.map(n => `${n.lat},${n.lon}`).join('|');
  let ok = false;
  for (let attempt = 0; attempt < 4 && !ok; attempt++) {
    try {
      const res = await fetch('https://api.opentopodata.org/v1/srtm30m', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ locations }),
      });
      if (res.status === 429) { await sleep(5000); continue; }
      if (!res.ok) throw new Error(`HTTP ${res.status}`);
      const json = await res.json();
      const lines = [];
      json.results.forEach((r, j) => {
        if (r.elevation != null) lines.push(`${batch[j].id},${r.elevation.toFixed(1)}`);
      });
      appendFileSync(outPath, lines.join('\n') + '\n');
      ok = true;
    } catch (e) {
      console.warn(`  batch ${i / BATCH}: ${e.message} - retrying`);
      await sleep(4000);
    }
  }
  if (!ok) { console.error('Giving up on this batch; rerun later to resume.'); }
  process.stdout.write(`\r${Math.min(i + BATCH, todo.length)}/${todo.length}`);
  await sleep(1100); // 1 req/s API etiquette
}
console.log(`\nDone. Elevations written to ${outPath}`);
