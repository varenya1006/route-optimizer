"""
Route Optimization & Workflow Mapper
=====================================
Merged app: Route Optimizer backend (graph engine, Dijkstra/A*, traffic simulation)
+ Pathfinding frontend (interactive Leaflet map, geocoding, real OSM graph).

Tech Stack: Python | Flask | NetworkX | OSMnx | Dijkstra | A* | Leaflet | Docker
"""

from flask import Flask, request, jsonify, render_template
from flask_cors import CORS
import networkx as nx
import osmnx as ox
import json
import time
import math
from datetime import datetime
from typing import Dict, List, Tuple, Optional
from geopy.distance import geodesic

app = Flask(__name__)
CORS(app)

# ── In-memory graph store ──────────────────────────────────────────────────────
_graph_db: Dict[str, nx.MultiDiGraph] = {}
_simulation_config = {"congestion_factor": 1.0, "delay_per_edge": 0.0}

# ── OSM graph constants ────────────────────────────────────────────────────────
OSM_GRAPH_ID = "nagpur_osm"
OSM_CENTER = (21.1458, 79.0882)   # Nagpur, Maharashtra
OSM_DIST   = 5000                  # metres radius


# ═══════════════════════════════════════════════════════════════════════════════
# Startup: load real-world OSM graph
# ═══════════════════════════════════════════════════════════════════════════════

def load_osm_graph():
    print(f"[startup] Loading OSM graph for Nagpur ({OSM_DIST}m radius)…")
    G = ox.graph_from_point(OSM_CENTER, dist=OSM_DIST, network_type="drive")

    # Traffic simulation: inflate first 5000 edge weights by 1.3×
    for u, v, data in list(G.edges(data=True))[:5000]:
        if "length" in data:
            data["length"] *= 1.3

    _graph_db[OSM_GRAPH_ID] = G
    print(f"[startup] OSM graph loaded — {G.number_of_nodes()} nodes, {G.number_of_edges()} edges")


# ═══════════════════════════════════════════════════════════════════════════════
# Helpers
# ═══════════════════════════════════════════════════════════════════════════════

def _get_nearest_node(G, lat: float, lon: float) -> int:
    """Snap a lat/lon coordinate to the nearest drivable OSM node."""
    u, v, _ = ox.distance.nearest_edges(G, lon, lat)
    u_pt = (G.nodes[u]["y"], G.nodes[u]["x"])
    v_pt = (G.nodes[v]["y"], G.nodes[v]["x"])
    return u if geodesic((lat, lon), u_pt).meters < geodesic((lat, lon), v_pt).meters else v


def _nodes_to_coords(G, route: List) -> List[List[float]]:
    return [[G.nodes[n]["y"], G.nodes[n]["x"]] for n in route]


def _route_length(G, route: List) -> float:
    length = 0.0
    for a, b in zip(route, route[1:]):
        edges = G.get_edge_data(a, b)
        if edges:
            best = min(edges.values(), key=lambda d: d.get("length", 0))
            length += best.get("length", 0)
    return length


def _heuristic_osm(G, u, v) -> float:
    x1, y1 = G.nodes[u]["x"], G.nodes[u]["y"]
    x2, y2 = G.nodes[v]["x"], G.nodes[v]["y"]
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


def _heuristic_generic(u: str, v: str, G: nx.DiGraph) -> float:
    pos_u = G.nodes[u].get("pos", (0, 0))
    pos_v = G.nodes[v].get("pos", (0, 0))
    return math.sqrt((pos_u[0] - pos_v[0]) ** 2 + (pos_u[1] - pos_v[1]) ** 2)


def _apply_traffic_delay(G, path: List) -> Tuple[List, float, Dict]:
    weight_key = "length" if OSM_GRAPH_ID in _graph_db and G is _graph_db.get(OSM_GRAPH_ID) else "weight"

    def edge_w(u, v):
        ed = G.get_edge_data(u, v)
        if isinstance(ed, dict) and 0 in ed:      # MultiDiGraph
            return ed[0].get(weight_key, 1.0)
        if isinstance(ed, dict):
            return ed.get(weight_key, 1.0)
        return 1.0

    base_weight  = sum(edge_w(u, v) for u, v in zip(path, path[1:]))
    congestion   = _simulation_config["congestion_factor"]
    delay_per_e  = _simulation_config["delay_per_edge"]
    total_delay  = delay_per_e * max(0, len(path) - 1) * congestion
    adjusted     = base_weight * congestion + total_delay

    return path, round(adjusted, 2), {
        "base_weight":        round(base_weight, 2),
        "congestion_factor":  congestion,
        "total_delay":        round(total_delay, 2),
        "adjusted_weight":    round(adjusted, 2),
        "edge_count":         len(path) - 1,
    }


# ═══════════════════════════════════════════════════════════════════════════════
# Frontend route
# ═══════════════════════════════════════════════════════════════════════════════

@app.route("/")
def index():
    return render_template("index.html")


# ═══════════════════════════════════════════════════════════════════════════════
# Map routing endpoint (used by Leaflet UI)
# ═══════════════════════════════════════════════════════════════════════════════

@app.route("/route", methods=["POST"])
def map_route():
    """
    POST /route
    Used by the Leaflet frontend.
    Payload: {"start": [lat, lon], "end": [lat, lon], "algorithm": "both"|"dijkstra"|"astar"}
    Returns dijkstra + astar coords, lengths, times, and traffic metrics.
    """
    data      = request.get_json(force=True)
    start_lat, start_lon = data["start"]
    end_lat,   end_lon   = data["end"]
    algo      = data.get("algorithm", "both")

    G = _graph_db.get(OSM_GRAPH_ID)
    if G is None:
        return jsonify({"error": "OSM graph not loaded yet — try again in a few seconds"}), 503

    try:
        source = _get_nearest_node(G, start_lat, start_lon)
        target = _get_nearest_node(G, end_lat,   end_lon)
    except Exception as e:
        return jsonify({"error": f"Node snap failed: {e}"}), 400

    result = {}

    if algo in ("both", "dijkstra"):
        t0 = time.perf_counter()
        try:
            d_path = nx.shortest_path(G, source, target, weight="length")
        except nx.NetworkXNoPath:
            return jsonify({"error": "No Dijkstra path found between these points"}), 404
        d_time = time.perf_counter() - t0
        _, d_adj, d_metrics = _apply_traffic_delay(G, d_path)
        result.update({
            "dijkstra":  _nodes_to_coords(G, d_path),
            "d_len":     _route_length(G, d_path),
            "d_time":    d_time,
            "d_metrics": d_metrics,
        })

    if algo in ("both", "astar"):
        t0 = time.perf_counter()
        try:
            a_path = nx.astar_path(
                G, source, target,
                heuristic=lambda u, v: _heuristic_osm(G, u, v),
                weight="length"
            )
        except nx.NetworkXNoPath:
            return jsonify({"error": "No A* path found between these points"}), 404
        a_time = time.perf_counter() - t0
        _, a_adj, a_metrics = _apply_traffic_delay(G, a_path)
        result.update({
            "astar":    _nodes_to_coords(G, a_path),
            "a_len":    _route_length(G, a_path),
            "a_time":   a_time,
            "a_metrics": a_metrics,
        })

    return jsonify(result), 200


# ═══════════════════════════════════════════════════════════════════════════════
# Generic graph REST API (from Route Optimizer)
# ═══════════════════════════════════════════════════════════════════════════════

@app.route("/graph", methods=["POST"])
def create_graph():
    data     = request.get_json(force=True)
    graph_id = data.get("graph_id", "default")
    G = nx.DiGraph()
    for node in data.get("nodes", []):
        nid   = node["id"] if isinstance(node, dict) else node
        attrs = {k: v for k, v in node.items() if k != "id"} if isinstance(node, dict) else {}
        G.add_node(nid, **attrs)
    for edge in data.get("edges", []):
        w = edge.get("weight", 1.0)
        G.add_edge(edge["source"], edge["target"], weight=w, base_weight=w)
    _graph_db[graph_id] = G
    return jsonify({"status": "graph_created", "graph_id": graph_id,
                    "node_count": G.number_of_nodes(), "edge_count": G.number_of_edges()}), 201


@app.route("/shortest-path", methods=["POST"])
def shortest_path():
    data      = request.get_json(force=True)
    graph_id  = data.get("graph_id", "default")
    source    = data.get("source")
    target    = data.get("target")
    algorithm = data.get("algorithm", "dijkstra").lower()

    if graph_id not in _graph_db:
        return jsonify({"error": f"Graph '{graph_id}' not found"}), 404
    G = _graph_db[graph_id]
    if source not in G or target not in G:
        return jsonify({"error": "Source or target node not in graph"}), 400

    t0 = time.perf_counter()
    try:
        if algorithm == "dijkstra":
            path = nx.dijkstra_path(G, source, target, weight="weight")
        elif algorithm == "astar":
            path = nx.astar_path(G, source, target,
                                 heuristic=lambda u, v: _heuristic_generic(u, v, G),
                                 weight="weight")
        else:
            return jsonify({"error": f"Unknown algorithm '{algorithm}'"}), 400
    except nx.NetworkXNoPath:
        return jsonify({"error": f"No path from '{source}' to '{target}'"}), 404

    compute_ms = round((time.perf_counter() - t0) * 1000, 2)
    path, adjusted_weight, sim_metrics = _apply_traffic_delay(G, path)
    return jsonify({
        "graph_id": graph_id, "algorithm": algorithm,
        "source": source, "target": target,
        "optimal_path": path, "compute_time_ms": compute_ms,
        "traffic_simulation": sim_metrics,
        "timestamp": datetime.utcnow().isoformat() + "Z"
    }), 200


@app.route("/delay-analysis", methods=["POST"])
def delay_analysis():
    data     = request.get_json(force=True)
    graph_id = data.get("graph_id", "default")
    if graph_id not in _graph_db:
        return jsonify({"error": f"Graph '{graph_id}' not found"}), 404
    G = _graph_db[graph_id]
    edge_delays = []
    for u, v, attrs in G.edges(data=True):
        base    = attrs.get("base_weight", 1.0)
        current = attrs.get("weight", 1.0)
        edge_delays.append({
            "edge": f"{u} -> {v}", "base_weight": base, "current_weight": current,
            "delay_added": round(current - base, 2),
            "congestion_ratio": round(current / base, 2) if base > 0 else None
        })
    edge_delays.sort(key=lambda x: x["delay_added"], reverse=True)
    return jsonify({
        "graph_id": graph_id, "total_edges": len(edge_delays),
        "edges_with_delay": len([e for e in edge_delays if e["delay_added"] > 0]),
        "top_delay_edges": edge_delays[:10],
        "timestamp": datetime.utcnow().isoformat() + "Z"
    }), 200


@app.route("/route-insights", methods=["POST"])
def route_insights():
    data     = request.get_json(force=True)
    graph_id = data.get("graph_id", "default")
    if graph_id not in _graph_db:
        return jsonify({"error": f"Graph '{graph_id}' not found"}), 404
    G = _graph_db[graph_id]
    centrality = nx.betweenness_centrality(G, weight="weight")
    top_bottlenecks = sorted(centrality.items(), key=lambda x: x[1], reverse=True)[:5]
    try:
        avg_path_length = nx.average_shortest_path_length(G, weight="weight")
    except nx.NetworkXError:
        avg_path_length = None
    in_degrees  = [d for _, d in G.in_degree()]
    out_degrees = [d for _, d in G.out_degree()]
    return jsonify({
        "graph_id": graph_id,
        "node_count": G.number_of_nodes(), "edge_count": G.number_of_edges(),
        "average_path_length": round(avg_path_length, 2) if avg_path_length else None,
        "top_bottlenecks": [{"node": n, "centrality": round(c, 4)} for n, c in top_bottlenecks],
        "degree_stats": {
            "avg_in_degree":  round(sum(in_degrees)  / len(in_degrees),  2) if in_degrees  else 0,
            "avg_out_degree": round(sum(out_degrees) / len(out_degrees), 2) if out_degrees else 0,
            "max_in_degree":  max(in_degrees)  if in_degrees  else 0,
            "max_out_degree": max(out_degrees) if out_degrees else 0,
        },
        "timestamp": datetime.utcnow().isoformat() + "Z"
    }), 200


@app.route("/simulate-congestion", methods=["POST"])
def simulate_congestion():
    data = request.get_json(force=True)
    _simulation_config["congestion_factor"] = data.get("congestion_factor", 1.0)
    _simulation_config["delay_per_edge"]    = data.get("delay_per_edge",    0.0)
    return jsonify({"status": "simulation_config_updated", "config": _simulation_config}), 200


@app.route("/health", methods=["GET"])
def health():
    return jsonify({
        "status": "healthy",
        "graphs_loaded": len(_graph_db),
        "osm_graph_ready": OSM_GRAPH_ID in _graph_db
    }), 200


# ═══════════════════════════════════════════════════════════════════════════════
# Entry point
# ═══════════════════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    load_osm_graph()
    app.run(host="0.0.0.0", port=5000, debug=False)
