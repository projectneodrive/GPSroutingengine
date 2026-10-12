# Véli

**Route planning for vehicles that are neither bicycles nor cars.**

Véli is the GPS and route-planning system of [**NeoDrive**](https://projectneodrive.ch/),
a student-led project building a light hybrid human–electric vehicle. It is a dependency-free
C++17 routing engine plus a Leaflet web app, built around the constraints that
actually decide where such a vehicle can go - and that no existing router models.

```
            width - barriers, gates, chicanes, bollard spacing
   slope asymmetry - capped uphill by regulation, free downhill
     effort model - pedal-by-wire assistance is a parameter, not a constant
```

---

## Table of contents

- [Why this exists](#why-this-exists)
- [Quick start](#quick-start)
- [Requirements](#requirements)
- [How to use](#how-to-use)
- [Running](#running)
- [Repository layout](#repository-layout)
- [How it works](#how-it-works)
- [Vehicle profile reference](#vehicle-profile-reference)
- [HTTP API](#http-api)
- [CLI](#cli)
- [How to integrate Véli](#how-to-integrate-véli)
- [Map data](#map-data)
- [Forking and adapting](#forking-and-adapting)
- [Performance](#performance)
- [Limitations](#limitations)
- [Contributing](#contributing)
- [Credits](#credits)
- [Licence and attribution](#licence-and-attribution)

---

## Why this exists

Every routing engine assumes you are one of three things: a car, a pedestrian,
or a bicycle. An intermediate vehicle is none of them.

| Problem | What goes wrong with a bicycle router | What Véli does |
|---|---|---|
| **Width** | It routes you through anti-motorcycle gates, bollard pairs and chicanes. A bicycle fits; a vehicle 80–120 cm wide discovers it does not, several kilometres in, at the gate. | Barrier nodes carry their opening width. Anything narrower than the vehicle + 6 cm margin is impassable and pruned from the search. |
| **Slope asymmetry** | It treats gradient roughly symmetrically. | Speed is solved from a power balance: capped by regulation on the flat and uphill, uncapped downhill. A steeper, longer route can genuinely be faster. |
| **Effort model** | Assumes a fixed human power output. | Mass, drag area, rolling resistance and assistance power are profile parameters, so the ETA reflects *your* pedal-by-wire configuration. |

The result is measurable. On a 5 km test across Lausanne with +261 m of climb,
Véli's route is **longer than the bicycle route yet 8 minutes faster**
(5.39 km / 25 min vs. 4.93 km / 33 min), and shares only 47 % of its geometry.
The built-in *Comparer à un vélo* button reproduces this for any trip.

---

## Quick start

```bash
git clone https://github.com/projectneodrive/VeliRoutingEngine.git && cd veli
```

The repository holds **source only**: no compiled binaries and no map data
(both are gitignored). You build the backend once with any C++17 compiler -
it takes a few seconds and needs no libraries - and the map data downloads
itself the first time you route somewhere.

### Windows: one double-click

Build the launcher once, then start everything by double-clicking it:

```bash
g++ -O2 -std=c++17 -o Veli.exe backend/src/launcher.cpp -lws2_32 -lshell32 -static
```

`Veli.exe` compiles the backend if it is missing, preloads `data/map.osm` when
present, picks the first free port in 8080-8099, starts the server and opens
your browser on it. Closing its window stops the server, so nothing is left
holding the port. Everything below is the manual equivalent.

```bash
# Windows (MinGW-w64 g++)
g++ -O2 -std=c++17 -o backend/veli.exe backend/src/main.cpp -lws2_32 -static
```

```bash
# Linux / macOS
g++ -O2 -std=c++17 -o backend/veli backend/src/main.cpp -pthread
```

```bash
# Run - no map data needed, it downloads what a route requires
./backend/veli serve --web frontend --port 8080
```

Then open **<http://localhost:8080/>**, type a start and destination (or click
the map), and the network for that area is fetched automatically.

> **First route into a new area takes 20–90 s** while the road network and
> elevation grid download. Everything after that is milliseconds. The loading
> animation shows a cyclist crossing the screen, then a Swiss flag when the
> zone is ready.

---

## Requirements

| | Needed for | Notes |
|---|---|---|
| **C++17 compiler** | the backend | GCC 13 / Clang 15 / MSVC 2019+. Developed with MinGW-w64 GCC 13.2. |
| **`curl` on PATH** | on-demand map fetching | Ships with Windows 10+, macOS and most Linux. The backend shells out to it instead of linking libcurl. |
| **A browser** | the web UI | Any modern one. Leaflet is loaded from a CDN. |
| **Node 18+** *(optional)* | `data/fetch_elevation.mjs` | Only for high-precision per-node elevations. |
| **PowerShell** *(optional)* | `data/fetch_osm.ps1` | Only for pre-downloading an area. |

There are **no build-time libraries to install** - no XML parser, no HTTP
framework, no TLS stack. That is a deliberate constraint: the same source is
meant to compile for the vehicle's onboard computer.

> **No `g++` on Windows?** Install MinGW-w64, for example with
> `winget install BrechtSanders.WinLibs.POSIX.UCRT`, then open a new terminal
> so `g++` is on your PATH.

---

## How to use

The interface is in French. Once the server is running and
<http://localhost:8080/> is open:

1. **Set the start and destination.** Type an address in *Départ* and
   *Arrivée* (suggestions appear as you type), or click the map: each click
   fills the next empty field, and once all are set a click moves the
   destination. Markers can be dragged.
2. **Add stops** with **＋ Étape** to force the route through a place, for
   example a charging point or a barrier you have checked on the ground.
   **⇅ Inverser** swaps start and destination.
3. **Read the result.** The panel shows distance, duration, climb and descent,
   metres on car roads, unpaved ground and pushing, and an elevation profile.
   Yellow dots on the map are barriers whose width is unknown - check them
   before relying on the route. Blue dots mark stretches where you push.
4. **Describe your vehicle** in the **Réglages** tab: width, total mass,
   speed class (25 km/h pedelec or 45 km/h S-class), downhill limit, and road
   constraints (car roads, footways, gravel, untagged bollards). The route
   recalculates with the new profile.
5. **Compare** with **Comparer à un vélo**, which overlays the route an
   ordinary bicycle router would choose for the same trip.
6. **Export** with **⬇ Télécharger le GPX** to load the route on a GPS or
   phone. Uncertain barriers are included as waypoints.
7. **Effacer** clears everything and starts over.

The first route into an area that is not downloaded yet takes 20–90 s; the
loading animation shows progress. Routes in that area are instant afterwards,
and the download is cached in `data/` for the next launch.

---

## Running

### Serve the web app

```bash
./backend/veli serve --web frontend --port 8080
```

Optional flags:

| Flag | Default | Meaning |
|---|---|---|
| `--map <file.osm>` | *none* | Preload an area so the first route is instant. Without it, everything is fetched on demand. |
| `--ele <file.csv>` | `<map dir>/elevations.csv` | Per-node elevations (`node_id,metres`). Falls back to the coarse SRTM grid. |
| `--web <dir>` | `../frontend` | Directory of static files to serve. |
| `--port <n>` | `8080` | Listens on loopback only. |
| `--data <dir>` | `data` *(or the map's directory)* | Where downloaded areas are cached. |

With the Lausanne extract preloaded:

```bash
./backend/veli serve --map data/map.osm --ele data/elevations.csv --web frontend --port 8080
```

> **Keep it in its own terminal.** The server runs in the foreground until you
> stop it with Ctrl-C. Launching it from something that owns the process (an IDE
> preview pane, a background task runner) means it dies whenever that owner
> closes. On Windows, `Veli.exe` handles this for you: it runs the server in a
> job object, so the two always live and die together.

### Route from the command line

```bash
./backend/veli route --map data/map.osm --from 46.5095,6.6250 --to 46.5386,6.6283 --via "46.5220,6.6330" --width 0.95 --power 350 --car_roads 0 --gpx route.gpx
```

```text
Itinéraire calculé en 203.3 ms :
  distance : 5.67 km
  durée    : 29 min (moy. 11.9 km/h)
  dénivelé : +271 m / -66 m
  à pied : 415 m
  ! Potelet sans largeur renseignée sur l'itinéraire - supposé franchissable [...]
  ! 160 m à pied - cheminement piéton, mettre pied à terre et pousser.
GPX écrit dans route.gpx
```

---

## Repository layout

```text
backend/src/
  util.hpp      geodesy (haversine, bearings), URL/JSON/XML helpers      179 loc
  osm.hpp       streaming OSM XML parser, barrier classification         399 loc
  graph.hpp     routing graph, highway/surface/maxspeed classes,
                connected-component analysis                             468 loc
  vehicle.hpp   vehicle profile + physics speed model                    162 loc
  router.hpp    A* over the constraint-aware cost model, multi-leg       413 loc
  gpx.hpp       GPX 1.1 export with barrier waypoints                     55 loc
  net.hpp       on-demand Overpass + SRTM fetching via curl              215 loc
  main.cpp      CLI, HTTP server, JSON serialisation                     600 loc
  launcher.cpp  double-click launcher: build, serve, open browser        246 loc
frontend/
  index.html    the whole web app: Leaflet map, itinerary panel,
                settings tab, elevation profile                         1632 loc
data/
  fetch_osm.ps1        pre-download a bbox from Overpass (optional)
  fetch_elevation.mjs  per-node SRTM elevations, resumable (optional)
.gitignore             binaries and downloaded map data stay out of git
```

Roughly **4 400 lines** total. Every module is a header; `main.cpp` is the
backend's only translation unit (`launcher.cpp` builds the separate Windows
launcher).

---

## How it works

A route request travels through eight stages. Each hands a specific artefact to
the next - those hand-offs are where the pieces meet.

| # | Stage | Emits |
|---|---|---|
| 1 | **Input** - Leaflet map, Nominatim autocomplete | `from` / `to` / `via` coordinates |
| 2 | **Transport** - hand-written HTTP server (winsock ⇄ POSIX) | parsed query map |
| 3 | **Coverage** - Overpass + OpenTopoData via `curl`, padded bbox cache | `map_auto.osm` + elevation grid |
| 4 | **Parse** - scanning XML reader, routing tags only | `OsmData { nodes, ways }` |
| 5 | **Graph** - classification, elevation interpolation, union-find | `Graph { vertices, adj, in_main }` |
| 6 | **Speed** - power-balance solve per edge | v(gradient, surface) in m/s |
| 7 | **Search** - A* on "perceived seconds" | `RouteResult { path, time, stats, warnings }` |
| 8 | **Output** - JSON + GPX 1.1 + SVG profile | polyline, elevation chart, `itineraire-veli.gpx` |

### The speed model

Each edge's steady-state speed solves the power balance

```text
P = v·m·g·(crr_eff + grade) + ½·ρ·CdA·v³
```

by bisection (50 iterations, ρ = 1.226 kg/m³). The result is then capped:

- **flat and uphill** → `vflat` (25 km/h pedelec or 45 km/h S-class),
- **downhill** (grade < −0.5 %) → `vdown`, the rider's chosen safe maximum.

Below ~2 km/h the rider is assumed to dismount and push at 4 km/h. Surface
affects the model twice: a rolling-resistance multiplier (asphalt 1.0 →
gravel 2.2 → mud 8.0) and a separate comfort/vibration speed factor
(cobblestone 0.5, sand 0.3) - a velomobile shakes apart long before rolling
resistance stops it.

### The cost function

A* minimises **perceived seconds**: real travel time × preference multipliers,
plus turn and barrier penalties.

| Term | Effect |
|---|---|
| Traffic stress | ×1.8 on primary roads without cycle infrastructure, ×1.05 residential |
| Designated cycle infrastructure | ×0.92 |
| Unpaved | ×(1 + 2.5 · `avoid_unpaved`) |
| Unlit at night | ×1.7 when `lit=no`, ×1.4 on unlit tracks |
| Scenic | ×(1 − 0.35 · `scenic`) on green ways, penalty on roads |
| Pushing | ×1.3 on top of walking pace |
| Turn penalty | 0 below 30°, up to ~24 s for a U-turn, scaled by `agility` |
| Barrier penalty | chicane ~33 s, gate 15 s, bollard ~6 s, scaled by `agility` |

The reported ETA is the **unweighted physical time**; multipliers only steer the
search. Because a multiplier can dip below 1.0, the A* heuristic is scaled by the
smallest possible multiplier (`0.55 / v_max`) so it stays admissible.

### Hard constraints (pruned before costing)

- **Width vs. barriers.** Tagged opening < `width` + 6 cm ⇒ impassable.
  Untagged openings follow `barriers_passable`; per-type defaults are
  bollard 1.35 m, chicane 1.10 m, motorcycle barrier 0.85 m, gate 3.0 m.
  Kissing gates and turnstiles are always blocked.
- **Car roads.** Excluded unless `car_roads=1`, and then only where the posted
  `maxspeed` ≤ `max_road_speed`. Untagged roads assume primary 80, secondary 60,
  tertiary 50 km/h. Roads with dedicated cycle infrastructure are always allowed.
- **Foot infrastructure.** Usable at pushing pace unless `footways=0`.
- **Legal access.** `access=no|private`, `vehicle=no`, `bicycle=no` on
  non-foot ways, motorways and steps are never routed.
- **Connectivity.** Endpoints snap to network *this vehicle* can use, not just
  the nearest road. The nearest vertex is often a motorway, a private drive, or
  a stretch cut off by a narrow barrier, so the router tries the ~8 start
  points closest to the click, flood-fills what the profile can reach from
  each, and snaps the destination to the closest reachable vertex. A route is
  therefore found whenever one exists nearby, and an endpoint moved by more
  than 75 m is reported as a `snapped` warning.

---

## Vehicle profile reference

Every parameter is a CLI flag (`--width`) *and* an API query parameter
(`?width=`). Defaults describe a typical velomobile-class NeoDrive vehicle.

| Parameter | Default | Range | Meaning |
|---|---|---|---|
| `width` | `0.95` | 0.4–2.5 m | Physical width including mirrors. Drives every barrier decision. |
| `mass` | `140` | 40–600 kg | Vehicle + rider + cargo. |
| `power` | `350` | 50–4000 W | Sustained human + motor power at the wheel. |
| `cda` | `0.35` | 0.05–1.5 m² | Drag area. Velomobiles are slippery. |
| `crr` | `0.008` | 0.002–0.05 | Rolling resistance on smooth asphalt. |
| `vflat` | `45` | 6–80 km/h | Assisted cap on flat and uphill. |
| `vdown` | `70` | ≥ vflat, ≤ 100 | Rider's safe downhill maximum. |
| `agility` | `0.5` | 0–1 | 0 = barge (hates chicanes), 1 = nimble. |
| `car_roads` | `0` | 0/1 | May share the carriageway with cars. |
| `max_road_speed` | `50` | 20–120 km/h | ...but only where cars are limited to this. |
| `footways` | `1` | 0/1 | May use pedestrian paths by pushing. `0` = avoid. |
| `avoid_unpaved` | `0.5` | 0–1 | Aversion to gravel and dirt. |
| `night` | `0` | 0/1 | Prefer lit ways. |
| `scenic` | `0` | 0–1 | Prefer green, quiet ways over speed. |
| `barriers_passable` | `1` | 0/1 | Assumption for untagged barrier widths. |
| `skip_untagged_bollards` | `0` | 0/1 | Stricter: avoid bollards with no width tag entirely. |
| `bicycle_baseline` | `0` | 0/1 | Route as a conventional bicycle, for comparison. |

---

## HTTP API

All endpoints are `GET`, return `Access-Control-Allow-Origin: *`, and are served
by the same process as the frontend.

### `GET /api/info`

```json
{ "vertices": 62226, "edges": 133722,
  "bounds": [[46.5061496, 6.5749608], [46.5506138, 6.6542608]] }
```

`bounds` is `null` when no area is loaded yet.

### `GET /api/route`

```text
/api/route?from=46.5095,6.6250&to=46.5386,6.6283&via=46.52,6.63|46.53,6.64&width=0.95&car_roads=1&max_road_speed=30
```

`from` and `to` are required; `via` takes any number of `lat,lon` stops
separated by `|` or `;`. All [profile parameters](#vehicle-profile-reference)
are accepted. If the points fall outside the loaded network, the area is
downloaded first and the response carries the new `network` bounds.

```json
{
  "ok": true,
  "distance_m": 5393.2, "time_s": 1489.7,
  "ascent_m": 261.4, "descent_m": 54.1,
  "on_car_roads_m": 162.0, "on_unpaved_m": 15.3,
  "on_unlit_m": 0.0, "pushing_m": 218.4,
  "compute_ms": 365.2,
  "geometry":   [[46.5094777, 6.6249993]],
  "elevations": [393.0],
  "warnings": [
    { "lat": 46.5278724, "lon": 6.6296680, "kind": "barrier_assumed",
      "text": "Potelet sans largeur renseignée..." }
  ]
}
```

`elevations` is index-aligned with `geometry` (`null` where unknown) and drives
the profile chart. Warning `kind` is one of `barrier_assumed` (untagged width,
verify on the ground), `barrier_ok` (tagged and fits), `pushing` (one entry
per contiguous foot-path segment), or `snapped` (the start or destination was
moved more than 75 m to reach network the vehicle can use). On failure: `{"ok": false, "error": "..."}`
with status 404, or 503 if the area could not be downloaded.

**Error and warning messages are in French**, matching the UI.

### `GET /api/route.gpx`

Identical parameters; returns a GPX 1.1 file
(`Content-Disposition: attachment; filename="itineraire-veli.gpx"`) with the
track plus one `<wpt>` per uncertain barrier - the observations worth checking
on the ground and feeding back to OpenStreetMap.

---

## CLI

```text
veli serve [--map <file.osm>] [--ele <elevations.csv>]
           [--web <frontend dir>] [--port 8080] [--data <dir>]

veli route --map <file.osm> [--ele <elevations.csv>]
           --from <lat,lon> --to <lat,lon> [--via "lat,lon|lat,lon"]
           [--gpx <out.gpx>] [vehicle profile flags]
```

`route` exits `1` on bad arguments, `2` when no route exists.

---

## How to integrate Véli

Véli is meant to be embedded: in the vehicle's dashboard, a companion app, or
another planning tool. There are three ways in, from loosest to tightest.

### 1. Call the HTTP API (any language)

Run `veli serve` next to your application and call
[`/api/route`](#get-apiroute). Everything is a plain `GET` returning JSON.

```bash
curl "http://localhost:8080/api/route?from=46.5095,6.6250&to=46.5386,6.6283&width=0.95"
```

```js
// JavaScript (browser or Node 18+)
const params = new URLSearchParams({
  from: '46.5095,6.6250',
  to: '46.5386,6.6283',
  width: 0.95,     // any vehicle profile parameter
  car_roads: 1,
  max_road_speed: 30,
});
const res = await fetch(`http://localhost:8080/api/route?${params}`);
const route = await res.json();
if (!route.ok) throw new Error(route.error);
console.log(`${(route.distance_m / 1000).toFixed(2)} km, ${Math.round(route.time_s / 60)} min`);
// route.geometry is [[lat, lon], ...] - draw it with Leaflet, MapLibre, etc.
// route.warnings lists barriers to check, with their coordinates.
```

```python
# Python
import requests
r = requests.get("http://localhost:8080/api/route", params={
    "from": "46.5095,6.6250", "to": "46.5386,6.6283", "width": 0.95,
}).json()
print(r["distance_m"], r["time_s"], len(r["warnings"]))
```

Fetch `/api/route.gpx` with the same parameters to get a file for a GPS unit.
The server sends `Access-Control-Allow-Origin: *`, so a web page on another
origin can call it directly.

For **live re-routing**, call `/api/route` again with the current GPS fix as
`from`. Requests are stateless, so there is no session to manage.

### 2. Run the CLI from a script

For batch jobs or tools that only need a GPX file:

```bash
./backend/veli route --map data/map.osm --from 46.5095,6.6250 --to 46.5386,6.6283 --gpx out.gpx
```

Check the exit code: `0` success, `1` bad arguments, `2` no route.

### 3. Link the engine into your C++ program

The engine is header-only, so you can include it directly and skip HTTP. A
minimal program follows the same steps `main.cpp` does in `route` mode: parse
an OSM file (`osm.hpp`), build the graph (`graph.hpp`), fill a
`VehicleProfile` (`vehicle.hpp`, or `VehicleProfile::from_query` from a
key/value map), then call `route_multi(graph, profile, points)`
(`router.hpp`) and read its `RouteResult`; `route_to_gpx` (`gpx.hpp`) turns
it into a GPX file. The `route` branch of `main()` in `backend/src/main.cpp`
is a complete working example. Compile with the same flags as the backend.

---

## Map data

**By default you do not prepare any data.** The server downloads the road
network (Overpass) and a ~120 m SRTM elevation grid (OpenTopoData) for whatever
area a route needs, pads the bounding box by 50 % so nearby follow-up routes hit
the cache, and interpolates the grid onto the graph. Areas larger than about
40 × 40 km are refused.

### Which services are used, and fallbacks

Public instances are rate-limited and go down independently, so each download
tries several sources in order and only replaces the cache on success. Only
servers run by OpenStreetMap community non-profits are used: the Swiss
OpenStreetMap Association (SOSM), FOSSGIS e.V. (Germany) and Private.coffee
(Austria). All of them serve the same OpenStreetMap data.

| Data | Tried in order | Notes |
|---|---|---|
| Road network | `$VELI_OVERPASS_URL` (if set) → overpass.osm.ch (areas in and around Switzerland only) → overpass-api.de → z.overpass-api.de → overpass.private.coffee | Each server gets two attempts; after a full pass the server waits 15 s and goes round again, up to three passes. A reply with no roads counts as a miss, so a server that does not cover the area is skipped. |
| Elevation grid | OpenTopoData (SRTM 30 m) → Open-Meteo (Copernicus 90 m) | OpenTopoData allows 1 request/s and 1 000/day (about 90 new areas a day). Open-Meteo counts each point as a call (600/min), so it serves as a backup only. If both fail the route still works, just without gradients. |

To use your own Overpass server (self-hosted, or a mirror not in the list), set
an environment variable before starting the server:

```powershell
$env:VELI_OVERPASS_URL = "https://my-overpass.example.org/api/interpreter"
```

```bash
VELI_OVERPASS_URL=https://my-overpass.example.org/api/interpreter ./backend/veli serve --web frontend
```

Every request identifies itself with a `Veli/1.0` User-Agent, as the services'
usage policies ask; overpass-api.de rejects curl's default one.

To pre-download instead - useful for demos, offline work, or a sharper elevation
model:

```powershell
# 1. Road network for a bbox (default: central Lausanne)
.\data\fetch_osm.ps1 -South 46.508 -West 6.58 -North 46.545 -East 6.65
```

```bash
# 2. Per-node elevations - higher fidelity than the grid.
#    ~10 min for a city, resumable, polite 1 req/s.
node data/fetch_elevation.mjs data/map.osm data/elevations.csv
```

Both outputs are gitignored; they are large and fully regenerable.

---

## Forking and adapting

### Point it at a different city

Nothing to configure - just route there. Any bundled `data/map.osm` is only a
convenience preload; delete it and the server fetches whatever you ask for.

### Model a different vehicle

Everything about the vehicle lives in `backend/src/vehicle.hpp`. Change the
defaults in `VehicleProfile`, or just pass flags. Two worked examples:

```bash
# A wide cargo velomobile that will not touch traffic
--width 1.25 --mass 200 --power 500 --vflat 25 --car_roads 0 --agility 0.2
```

```bash
# A narrow, nimble prototype allowed in 30 km/h zones
--width 0.75 --mass 90 --power 250 --car_roads 1 --max_road_speed 30 --agility 0.9
```

### Add a new constraint end to end

The five places to touch, in order:

1. **`osm.hpp`** - capture the tag in `OsmWay` / `OsmNode` and the tag switch.
2. **`graph.hpp`** - fold it into `Edge` (there are spare bytes) or a new
   `EdgeFlags` bit, and set it in `build_graph`.
3. **`vehicle.hpp`** - add the field to `VehicleProfile` and read it in
   `from_query` (this alone makes it a working CLI flag *and* API parameter).
4. **`router.hpp`** - use it in `evaluate_edge`: `return ev` early for a hard
   constraint, or multiply into `mult` for a preference.
5. **`frontend/index.html`** - add the control in the *Réglages* tab and one
   line in `params()`.

### Port to the vehicle's onboard computer

The POSIX branch already exists; `#ifdef _WIN32` in `main.cpp` is the only
platform split. Build with `-pthread`, keep `curl` available (or replace
`run_cmd` in `net.hpp` with your own transport), and preload a regional extract
so the device never needs Overpass in the field. The server is stateless per
request, so live re-routing from a GPS fix is just another `/api/route` call.

### Translate the UI

All user-facing strings are French and live in two places: `frontend/index.html`,
and the warning/error strings in `router.hpp`, `main.cpp` and `osm.hpp`
(`barrier_name`).

---

## Performance

Measured on the default profile, a consumer laptop, single-threaded.

| | |
|---|---|
| Lausanne centre graph | 62 226 vertices / 133 722 directed edges |
| Largest auto-fetched area | 487 377 vertices / 1 003 189 edges |
| Route query (133 k edges) | ~200 ms |
| Route query (1 M edges) | 0.8–1.1 s |
| OSM parse (11 MB extract) | ~3 s |
| First-time area download | 21–87 s (Overpass + elevation grid) |
| Binary size | ~2 MB, statically linked |

---

## Limitations

Since this is a prototype:

- **The HTTP server is single-threaded.** An area download blocks the queue for
  its duration. Fine for one rider and one onboard computer; not a public service.
  Idle connections (browser preconnects) are dropped after 2 s so they cannot
  stall it.
- **Bound to loopback**, no authentication, no TLS. Do not expose it directly.
- **Coarse elevations by default.** The auto-fetched grid is ~120 m - right for
  route-scale hills, wrong for a short steep ramp. Use `fetch_elevation.mjs`
  where precision matters.
- **Nearest-vertex snapping is a linear scan.** Fine to ~500 k vertices; a
  spatial index is the obvious next optimisation.
- **Barrier data is only as good as OpenStreetMap**, which usually omits widths
  - hence the explicit assumption switch and the verification waypoints.
- **No turn restrictions, no traffic-signal delays, no live traffic.**
- Public Overpass, elevation and Nominatim instances are rate-limited. The
  server falls back across mirrors (see [Map data](#which-services-are-used-and-fallbacks)),
  but heavy forks should self-host Overpass and point `VELI_OVERPASS_URL` at it.

---

## Contributing

1. Fork the repository and create a branch: `git checkout -b my-change`.
2. Build the backend (see [Quick start](#quick-start)) and check your change
   in the web app and with a CLI route, e.g. the Lausanne example in
   [Running](#route-from-the-command-line).
3. Keep the zero-dependency rule: standard library only in `backend/src`, and
   code that compiles on both Windows (MinGW) and Linux.
4. Never commit binaries or downloaded map data. `.gitignore` already
   excludes them.
5. Open a pull request explaining what changed and, for routing changes, how
   a test route behaved before and after.

Wrong or missing barrier widths are best fixed **in OpenStreetMap itself**
(tag `maxwidth:physical` on the barrier node), so every router benefits.

---

## Credits

**Routing engine, physics model, backend and web app** -
[Giovanni Y.](https://www.linkedin.com/in/giovanniyazbeck/)

**Vehicle, requirements and project** - the
[NeoDrive](https://projectneodrive.ch/) team, who defined the vehicle class this
router exists to serve and supplied the constraints that shape every routing
decision here.

Véli stands on shared, open infrastructure.

| Project | Role in Véli |
|---|---|
| [OpenStreetMap](https://www.openstreetmap.org/) | Every road, path, barrier, surface and speed limit. |
| [Overpass API](https://overpass-api.de/) | Extracts the road network for a bounding box on demand. |
| [Nominatim](https://nominatim.org/) | Forward and reverse geocoding for the itinerary fields. |
| [OpenTopoData](https://www.opentopodata.org/) + NASA [SRTM](https://www.earthdata.nasa.gov/data/instruments/srtm) | Elevations, without which the slope model has nothing to work with. |
| [Open-Meteo](https://open-meteo.com/) + Copernicus DEM | Backup elevation source. |
| Overpass servers of the [Swiss OpenStreetMap Association](https://sosm.ch/) (overpass.osm.ch), [FOSSGIS](https://www.fossgis.de/) (overpass-api.de) and [Private.coffee](https://private.coffee/) | Road network downloads, with fallback between them. |
| [Leaflet](https://leafletjs.com/) | Map rendering, markers and polylines. |
| [OSM tile servers](https://operations.osmfoundation.org/policies/tiles/) | Base map imagery. |
| [curl](https://curl.se/) | The backend's only outbound network channel. |

---

## Licence and attribution

- Véli's source code is licensed under the **Mozilla Public License 2.0** -
  see [LICENSE](LICENSE). You may use it in larger, even proprietary,
  projects; changes to the MPL-covered files themselves must stay open.
- Map data © [OpenStreetMap](https://www.openstreetmap.org/copyright)
  contributors, available under the **Open Database Licence (ODbL)**. Any
  derived map data you publish must carry the same attribution.
- Overpass, Nominatim, OpenTopoData and the OSM tile servers are shared
  community resources - respect their usage policies, and self-host if you
  intend heavy use.
- Leaflet is BSD-2-Clause.

Part of the **NeoDrive** project - <https://projectneodrive.ch/>
