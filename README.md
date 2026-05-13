# Route Optimizer — Nagpur

A full-stack route-optimization app that merges:

- **Backend** — Route Optimizer graph engine (Dijkstra, A*, traffic simulation, REST API)
- **Frontend** — Interactive Leaflet map (geocoding, click/search routing, live stats)
- **Graph** — Real OSMnx street network for Nagpur (5 km radius, ~30 000 nodes)

---

## Quick Start (Docker — recommended)

```bash
docker-compose up --build
```

Open **http://localhost:5000** in your browser.  
The first start takes ~30–60 s while the OSM graph is downloaded.

---

## Quick Start (Local Python)

```bash
pip install -r requirements.txt
python app.py
```

Open **http://localhost:5000**.

---

## How to use the map

| Action | How |
|---|---|
| Search-based routing | Type From / To in the sidebar and click **Find Route** |
| Click-based routing  | Click the map to drop Start, then End |
| Drag to reroute      | Drag either marker to a new position |
| Change algorithm     | Click **Both / Dijkstra / A*** before searching |
| Simulate congestion  | Move sliders and click **Apply** |
| Reset                | Click **↺ Reset** |

---

## REST API

All endpoints accept/return JSON.

### `POST /route`
Used by the Leaflet UI. Runs Dijkstra + A* on the real OSM graph.
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
├── Dockerfile
├── docker-compose.yml
└── templates/
    └── index.html       # Leaflet frontend
```
