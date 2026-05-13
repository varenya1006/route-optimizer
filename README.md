# Route Optimizer — Nagpur

A full-stack, deployment-ready route-optimization app that merges:

- **Backend** — Route Optimizer graph engine (Dijkstra, A*, traffic simulation, REST API)
- **Frontend** — Interactive Leaflet map (geocoding, click/search routing, live stats)
- **Graph** — Real OSMnx street network for Nagpur (5 km radius, ~30 000 nodes)

---

## Quick Start (Docker — recommended)

```bash
docker-compose up --build
```

Open **http://localhost:5000** in your browser.  
The first start downloads the OSM graph (~30–60 s). Subsequent restarts are instant because the graph is cached to disk (`./cache/nagpur_graph.graphml`).

---

## Quick Start (Local Python)

```bash
pip install -r requirements.txt
python app.py
```

Open **http://localhost:5000**.

---

## What changed (deployment-ready fixes)

| Fix | Detail |
|---|---|
| **A* heuristic** | Now uses haversine formula in **meters** (previously Euclidean in degrees, which overestimated and broke admissibility). |
| **Traffic-aware routing** | Congestion & delay sliders now **actually influence** the path chosen by Dijkstra / A*. The backend builds a traffic-weighted graph copy per request so concurrent calls are safe. |
| **Nominatim proxy** | Frontend no longer calls Nominatim directly. All geocoding goes through `/geocode?q=…` with a proper `User-Agent` header, respecting OSM ToS. |
| **OSM graph cache** | Graph is saved to `GRAPH_CACHE_DIR` after first download. Restarts load instantly from `.graphml`. |
| **Dockerfile** | Added production-ready image with all geospatial system libraries pre-installed. |
| **Mobile responsive** | Sidebar collapses to a bottom panel on screens < 768 px. |
| **Healthcheck** | `docker-compose` includes a healthcheck endpoint so orchestrators know when the graph is ready. |

---

## How to use the map

| Action | How |
|---|---|
| Search-based routing | Type From / To in the sidebar and click **Find Route** |
| Click-based routing  | Click the map to drop Start, then End |
| Drag to reroute      | Drag either marker to a new position |
| Change algorithm     | Click **Both / Dijkstra / A*** before searching |
| Simulate congestion  | Move sliders and click **Apply** — the next route will avoid heavily penalized edges |
| Reset                | Click **↺ Reset** |

---

## REST API

All endpoints accept/return JSON.

### `GET /geocode?q={query}`
Proxy for Nominatim with proper `User-Agent`. Used by the frontend.
```json
[{"lat":"21.14","lon":"79.08","display_name":"Sitabuldi, Nagpur"}]
```

### `POST /route`
Used by the Leaflet UI. Runs Dijkstra + A* on the real OSM graph with traffic weights applied.
```json
{ "start": [21.145, 79.088], "end": [21.155, 79.098], "algorithm": "both" }
```

### `POST /graph`
Create a custom in-memory directed graph.
```json
{ "graph_id": "g1", "nodes": ["A","B","C"], "edges": [{"source":"A","target":"B","weight":4}] }
```

### `POST /shortest-path`
Run Dijkstra or A* on a custom graph.
```json
{ "graph_id": "g1", "source": "A", "target": "C", "algorithm": "astar" }
```

### `POST /delay-analysis`
Analyse delay patterns across all edges of a custom graph.

### `POST /route-insights`
Betweenness centrality, bottleneck nodes, degree stats.

### `POST /simulate-congestion`
Update global traffic parameters.
```json
{ "congestion_factor": 1.5, "delay_per_edge": 2.0 }
```

### `GET /health`
```json
{ "status": "healthy", "graphs_loaded": 1, "osm_graph_ready": true }
```

---

## Project structure

```
route_optimizer_app/
├── app.py               # Flask backend (merged Route Optimizer + Pathfinding logic)
├── requirements.txt
├── Dockerfile           # Production image with geospatial libs
├── docker-compose.yml   # Includes healthcheck & persistent cache volume
└── templates/
    └── index.html       # Leaflet frontend (mobile-responsive, proxied geocoding)
```

---

## Environment variables

| Variable | Default | Purpose |
|---|---|---|
| `GRAPH_CACHE_DIR` | `./cache` | Where the OSM `.graphml` cache is stored |
| `FLASK_ENV` | `production` | Flask environment mode |
