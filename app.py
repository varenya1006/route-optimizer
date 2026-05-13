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
import os
import requests
from datetime import datetime
from typing import Dict, List, Tuple
from geopy.distance import geodesic
from dotenv import load_dotenv

load_dotenv()

app = Flask(__name__)
CORS(app)

_graph_db: Dict[str, nx.MultiDiGraph] = {}
_simulation_config = {"congestion_factor": 1.0, "delay_per_edge": 0.0, "hour_of_day": datetime.now().hour}

OSM_GRAPH_ID = "nagpur_osm"
OSM_CENTER = (21.1458, 79.0882)
OSM_DIST = 5000

GRAPH_CACHE_DIR = os.environ.get("GRAPH_CACHE_DIR", os.path.join(os.path.dirname(__file__), "cache"))
GRAPHML_PATH = os.path.join(GRAPH_CACHE_DIR, "nagpur_graph.graphml")
os.makedirs(GRAPH_CACHE_DIR, exist_ok=True)

SPEED_PROFILES = {
    "motorway":     {"00-05": 90, "05-08": 70, "08-11": 55, "11-14": 65, "14-17": 50, "17-20": 45, "20-24": 75},
    "trunk":        {"00-05": 80, "05-08": 60, "08-11": 45, "11-14": 55, "14-17": 40, "17-20": 35, "20-24": 65},
    "primary":      {"00-05": 70, "05-08": 50, "08-11": 35, "11-14": 45, "14-17": 30, "17-20": 28, "20-24": 55},
    "secondary":    {"00-05": 60, "05-08": 45, "08-11": 30, "11-14": 40, "14-17": 28, "17-20": 25, "20-24": 50},
    "tertiary":     {"00-05": 50, "05-08": 40, "08-11": 28, "11-14": 35, "14-17": 25, "17-20": 22, "20-24": 42},
    "residential":  {"00-05": 40, "05-08": 35, "08-11": 25, "11-14": 30, "14-17": 22, "17-20": 20, "20-24": 35},
    "unclassified": {"00-05": 45, "05-08": 38, "08-11": 28, "11-14": 33, "14-17": 25, "17-20": 22, "20-24": 38},
    "service":      {"00-05": 30, "05-08": 25, "08-11": 20, "11-14": 22, "14-17": 18, "17-20": 15, "20-24": 25},
}


def _get_speed_for_road_class(road_class: str, hour: int) -> float:
    profile = SPEED_PROFILES.get(road_class, SPEED_PROFILES["unclassified"])
    buckets = [
        (0, 5, "00-05"), (5, 8, "05-08"), (8, 11, "08-11"), (11, 14, "11-14"),
        (14, 17, "14-17"), (17, 20, "17-20"), (20, 24, "20-24")
    ]
    for start, end, key in buckets:
        if start <= hour < end:
            return profile[key]
    return profile["00-05"]


def load_osm_graph():
    if os.path.exists(GRAPHML_PATH):
        print(f"[startup] Loading cached OSM graph from {GRAPHML_PATH}…")
        G = ox.load_graphml(GRAPHML_PATH)
        _graph_db[OSM_GRAPH_ID] = G
        print(f"[startup] Cached graph loaded — {G.number_of_nodes()} nodes, {G.number_of_edges()} edges")
        return
    print(f"[startup] Downloading OSM graph for Nagpur ({OSM_DIST}m radius)…")
    G = ox.graph_from_point(OSM_CENTER, dist=OSM_DIST, network_type="drive")
    ox.save_graphml(G, GRAPHML_PATH)
    _graph_db[OSM_GRAPH_ID] = G
    print(f"[startup] OSM graph downloaded & cached — {G.number_of_nodes()} nodes, {G.number_of_edges()} edges")


def _get_nearest_node(G, lat: float, lon: float) -> int:
    u, v, _ = ox.distance.nearest_edges(G, lon, lat)
    u_pt = (G.nodes[u]["y"], G.nodes[u]["x"])
    v_pt = (G.nodes[v]["y"], G.nodes[v]["x"])
    return u if geodesic((lat, lon), u_pt).meters < geodesic((lat, lon), v_pt).meters else v


def _route_length(G, route: List) -> float:
    length = 0.0
    for a, b in zip(route, route[1:]):
        edges = G.get_edge_data(a, b)
        if edges:
            best = min(edges.values(), key=lambda d: d.get("length", 0))
            length += best.get("length", 0)
    return length


def _route_to_coords_lightweight(G, route: List, max_points_per_edge: int = 8, min_edge_meters: float = 80.0) -> List[List[float]]:
    coords = []
    for u, v in zip(route, route[1:]):
        edge_data = min(G.get_edge_data(u, v).values(), key=lambda d: d.get("length", 0))
        length = edge_data.get("length", 0)
        if "geometry" in edge_data and length > min_edge_meters:
            geom = edge_data["geometry"]
            pts = list(geom.coords)
            if len(pts) > max_points_per_edge:
                step = max(1, len(pts) // max_points_per_edge)
                pts = pts[::step]
                last_pt = list(geom.coords)[-1]
                if tuple(pts[-1]) != tuple(last_pt):
                    pts.append(last_pt)
        else:
            pts = [
                (G.nodes[u]["x"], G.nodes[u]["y"]),
                (G.nodes[v]["x"], G.nodes[v]["y"])
            ]
        for lon, lat in pts:
            pt = [lat, lon]
            if not coords or pt != coords[-1]:
                coords.append(pt)
    return coords


def _heuristic_osm(G, u, v) -> float:
    y1, x1 = G.nodes[u]["y"], G.nodes[u]["x"]
    y2, x2 = G.nodes[v]["y"], G.nodes[v]["x"]
    lat1, lon1 = math.radians(y1), math.radians(x1)
    lat2, lon2 = math.radians(y2), math.radians(x2)
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    c = 2 * math.asin(math.sqrt(a))
    return 6371000 * c


def _heuristic_generic(u: str, v: str, G: nx.DiGraph) -> float:
    pos_u = G.nodes[u].get("pos", (0, 0))
    pos_v = G.nodes[v].get("pos", (0, 0))
    return math.sqrt((pos_u[0] - pos_v[0]) ** 2 + (pos_u[1] - pos_v[1]) ** 2)


def _get_routing_graph(G, use_time_traffic: bool = False):
    cf = _simulation_config["congestion_factor"]
    dp = _simulation_config["delay_per_edge"]
    hour = _simulation_config.get("hour_of_day", datetime.now().hour)
    if not use_time_traffic and abs(cf - 1.0) < 1e-9 and abs(dp) < 1e-9:
        return G, "length"
    H = G.copy()
    for u, v, key, data in H.edges(keys=True, data=True):
        base = data.get("length", 1.0)
        if use_time_traffic:
            road_class = data.get("highway", "unclassified")
            if isinstance(road_class, list):
                road_class = road_class[0]
            speed_kmh = _get_speed_for_road_class(road_class, hour)
            speed_ms = speed_kmh / 3.6
            data["weight"] = base / speed_ms if speed_ms > 0 else base
        else:
            delay = dp * cf
            data["weight"] = base * cf + delay
    return H, "weight"


def _apply_traffic_delay(G, path: List) -> Tuple[List, float, Dict]:
    def edge_w(u, v):
        ed = G.get_edge_data(u, v)
        if isinstance(ed, dict) and 0 in ed:
            return ed[0].get("length", 1.0)
        if isinstance(ed, dict):
            return ed.get("length", 1.0)
        return 1.0
    base_weight = sum(edge_w(u, v) for u, v in zip(path, path[1:]))
    congestion = _simulation_config["congestion_factor"]
    delay_per_e = _simulation_config["delay_per_edge"]
    total_delay = delay_per_e * max(0, len(path) - 1) * congestion
    adjusted = base_weight * congestion + total_delay
    return path, round(adjusted, 2), {
        "base_weight": round(base_weight, 2),
        "congestion_factor": congestion,
        "total_delay": round(total_delay, 2),
        "adjusted_weight": round(adjusted, 2),
        "edge_count": len(path) - 1,
    }


def _estimate_travel_time(G, path: List, hour: int) -> float:
    total_time = 0.0
    for a, b in zip(path, path[1:]):
        edges = G.get_edge_data(a, b)
        if edges:
            best = min(edges.values(), key=lambda d: d.get("length", 0))
            length = best.get("length", 0)
            road_class = best.get("highway", "unclassified")
            if isinstance(road_class, list):
                road_class = road_class[0]
            speed_kmh = _get_speed_for_road_class(road_class, hour)
            speed_ms = speed_kmh / 3.6
            total_time += length / speed_ms if speed_ms > 0 else length / 8.33
    return total_time


def _find_alternative_paths(G, source, target, num_paths=3, max_factor=1.3, penalty=0.5, similarity_threshold=0.65):
    weight_key = "length"
    try:
        p1 = nx.shortest_path(G, source, target, weight=weight_key)
    except nx.NetworkXNoPath:
        return []
    shortest_len = _route_length(G, p1)
    max_allowed = shortest_len * max_factor
    results = [(p1, shortest_len)]
    penalized_edges = set()
    for u, v in zip(p1, p1[1:]):
        penalized_edges.add((min(u, v), max(u, v)))
    current_penalty = penalty
    for _ in range(num_paths - 1):
        def make_weight_fn(pen_edges, pen_mult):
            def weight_fn(u, v, d):
                base = d.get(weight_key, 1.0)
                if (min(u, v), max(u, v)) in pen_edges:
                    return base * (1 + pen_mult)
                return base
            return weight_fn
        wfn = make_weight_fn(penalized_edges, current_penalty)
        try:
            alt = nx.shortest_path(G, source, target, weight=wfn)
            alt_len = _route_length(G, alt)
            if alt_len > max_allowed:
                if current_penalty > 0.2:
                    current_penalty = max(0.1, current_penalty - 0.2)
                    wfn = make_weight_fn(penalized_edges, current_penalty)
                    alt = nx.shortest_path(G, source, target, weight=wfn)
                    alt_len = _route_length(G, alt)
                    if alt_len > max_allowed:
                        break
                else:
                    break
            alt_edge_set = set((min(u, v), max(u, v)) for u, v in zip(alt, alt[1:]))
            is_diverse = True
            for existing_path, _ in results:
                existing_edge_set = set((min(u, v), max(u, v)) for u, v in zip(existing_path, existing_path[1:]))
                jaccard = len(alt_edge_set & existing_edge_set) / len(alt_edge_set | existing_edge_set) if len(alt_edge_set | existing_edge_set) > 0 else 0
                if jaccard > similarity_threshold:
                    is_diverse = False
                    break
            if not is_diverse:
                current_penalty += 0.3
                wfn = make_weight_fn(penalized_edges, current_penalty)
                try:
                    alt = nx.shortest_path(G, source, target, weight=wfn)
                    alt_len = _route_length(G, alt)
                    if alt_len > max_allowed:
                        break
                    alt_edge_set = set((min(u, v), max(u, v)) for u, v in zip(alt, alt[1:]))
                    is_diverse = True
                    for existing_path, _ in results:
                        existing_edge_set = set((min(u, v), max(u, v)) for u, v in zip(existing_path, existing_path[1:]))
                        jaccard = len(alt_edge_set & existing_edge_set) / len(alt_edge_set | existing_edge_set) if len(alt_edge_set | existing_edge_set) > 0 else 0
                        if jaccard > similarity_threshold:
                            is_diverse = False
                            break
                    if not is_diverse:
                        break
                except nx.NetworkXNoPath:
                    break
            results.append((alt, alt_len))
            for u, v in zip(alt, alt[1:]):
                penalized_edges.add((min(u, v), max(u, v)))
            current_penalty = penalty
        except nx.NetworkXNoPath:
            break
    return results


@app.route("/")
def index():
    return render_template("index.html")


@app.route("/geocode", methods=["GET"])
def geocode_proxy():
    q = request.args.get("q", "").strip()
    if not q:
        return jsonify({"error": "Missing query parameter 'q'"}), 400
    try:
        headers = {"User-Agent": "RouteOptimizer-Nagpur/1.0"}
        url = "https://nominatim.openstreetmap.org/search"
        params = {"format": "json", "q": q, "limit": 1}
        r = requests.get(url, headers=headers, params=params, timeout=10)
        return jsonify(r.json()), r.status_code
    except Exception as e:
        return jsonify({"error": str(e)}), 500


@app.route("/reverse-geocode", methods=["GET"])
def reverse_geocode_proxy():
    lat = request.args.get("lat", "").strip()
    lon = request.args.get("lon", "").strip()
    if not lat or not lon:
        return jsonify({"error": "Missing lat/lon parameters"}), 400
    try:
        headers = {"User-Agent": "RouteOptimizer-Nagpur/1.0"}
        url = "https://nominatim.openstreetmap.org/reverse"
        params = {"format": "json", "lat": lat, "lon": lon, "zoom": 18}
        r = requests.get(url, headers=headers, params=params, timeout=10)
        return jsonify(r.json()), r.status_code
    except Exception as e:
        return jsonify({"error": str(e)}), 500


@app.route("/route", methods=["POST"])
def map_route():
    data = request.get_json(force=True)
    start_lat, start_lon = data["start"]
    end_lat, end_lon = data["end"]
    algo = data.get("algorithm", "both")
    use_time_traffic = data.get("use_time_traffic", False)
    hour = data.get("hour", datetime.now().hour)

    G = _graph_db.get(OSM_GRAPH_ID)
    if G is None:
        return jsonify({"error": "OSM graph not loaded yet — try again in a few seconds"}), 503

    try:
        source = _get_nearest_node(G, start_lat, start_lon)
        target = _get_nearest_node(G, end_lat, end_lon)
    except Exception as e:
        return jsonify({"error": f"Node snap failed: {e}"}), 400

    _simulation_config["hour_of_day"] = hour
    RG, weight_key = _get_routing_graph(G, use_time_traffic=use_time_traffic)
    result = {}

    if algo in ("both", "dijkstra"):
        t0 = time.perf_counter()
        try:
            d_path = nx.shortest_path(RG, source, target, weight=weight_key)
        except nx.NetworkXNoPath:
            return jsonify({"error": "No Dijkstra path found between these points"}), 404
        d_time = time.perf_counter() - t0
        _, d_adj, d_metrics = _apply_traffic_delay(G, d_path)
        d_estimated_duration = _estimate_travel_time(G, d_path, hour)
        result.update({
            "dijkstra": _route_to_coords_lightweight(G, d_path),
            "d_len": _route_length(G, d_path),
            "d_time": d_time,
            "d_metrics": d_metrics,
            "d_estimated_duration": round(d_estimated_duration, 1),
        })

    if algo in ("both", "astar"):
        t0 = time.perf_counter()
        try:
            a_path = nx.astar_path(
                RG, source, target,
                heuristic=lambda u, v: _heuristic_osm(G, u, v),
                weight=weight_key
            )
        except nx.NetworkXNoPath:
            return jsonify({"error": "No A* path found between these points"}), 404
        a_time = time.perf_counter() - t0
        _, a_adj, a_metrics = _apply_traffic_delay(G, a_path)
        a_estimated_duration = _estimate_travel_time(G, a_path, hour)
        result.update({
            "astar": _route_to_coords_lightweight(G, a_path),
            "a_len": _route_length(G, a_path),
            "a_time": a_time,
            "a_metrics": a_metrics,
            "a_estimated_duration": round(a_estimated_duration, 1),
        })

    result["hour"] = hour
    result["time_aware"] = use_time_traffic
    return jsonify(result), 200


@app.route("/route-alternatives", methods=["POST"])
def map_route_alternatives():
    data = request.get_json(force=True)
    start_lat, start_lon = data["start"]
    end_lat, end_lon = data["end"]
    algo = data.get("algorithm", "both")
    num_alternatives = min(data.get("num_alternatives", 2), 4)
    max_factor = data.get("max_length_factor", 1.3)
    use_time_traffic = data.get("use_time_traffic", False)
    hour = data.get("hour", datetime.now().hour)

    G = _graph_db.get(OSM_GRAPH_ID)
    if G is None:
        return jsonify({"error": "OSM graph not loaded yet — try again in a few seconds"}), 503

    try:
        source = _get_nearest_node(G, start_lat, start_lon)
        target = _get_nearest_node(G, end_lat, end_lon)
    except Exception as e:
        return jsonify({"error": f"Node snap failed: {e}"}), 400

    _simulation_config["hour_of_day"] = hour
    RG, weight_key = _get_routing_graph(G, use_time_traffic=use_time_traffic)
    paths_result = []

    def build_path_entry(path, algo_name, is_primary, shortest_len):
        t0 = time.perf_counter()
        path_coords = _route_to_coords_lightweight(G, path)
        compute_time = time.perf_counter() - t0
        length = _route_length(G, path)
        _, adj, metrics = _apply_traffic_delay(G, path)
        estimated_duration = _estimate_travel_time(G, path, hour)
        delta_pct = round(((length / shortest_len) - 1) * 100, 1) if shortest_len > 0 else 0
        return {
            "algorithm": algo_name,
            "primary": is_primary,
            "coords": path_coords,
            "length": round(length, 1),
            "time": round(compute_time, 4),
            "metrics": metrics,
            "delta_pct": delta_pct,
            "estimated_duration": round(estimated_duration, 1),
        }

    if algo in ("both", "dijkstra"):
        alt_paths = _find_alternative_paths(RG, source, target, num_paths=num_alternatives, max_factor=max_factor)
        if alt_paths:
            shortest_len = alt_paths[0][1]
            for i, (path, _) in enumerate(alt_paths):
                paths_result.append(build_path_entry(path, "dijkstra", i == 0, shortest_len))

    if algo in ("both", "astar"):
        try:
            a_primary = nx.astar_path(
                RG, source, target,
                heuristic=lambda u, v: _heuristic_osm(G, u, v),
                weight=weight_key
            )
        except nx.NetworkXNoPath:
            pass
        else:
            shortest_len = _route_length(G, a_primary)
            paths_result.append(build_path_entry(a_primary, "astar", True, shortest_len))
            penalized_edges = set()
            for u, v in zip(a_primary, a_primary[1:]):
                penalized_edges.add((min(u, v), max(u, v)))
            current_penalty = 0.5
            found = 1
            while found < num_alternatives:
                def make_weight_fn(pen_edges, pen_mult):
                    def weight_fn(u, v, d):
                        base = d.get(weight_key, 1.0)
                        if (min(u, v), max(u, v)) in pen_edges:
                            return base * (1 + pen_mult)
                        return base
                    return weight_fn
                wfn = make_weight_fn(penalized_edges, current_penalty)
                try:
                    alt = nx.astar_path(
                        RG, source, target,
                        heuristic=lambda u, v: _heuristic_osm(G, u, v),
                        weight=wfn
                    )
                    alt_len = _route_length(G, alt)
                    max_allowed = shortest_len * max_factor
                    if alt_len <= max_allowed:
                        alt_edge_set = set((min(u, v), max(u, v)) for u, v in zip(alt, alt[1:]))
                        is_diverse = True
                        for existing in [p for p in paths_result if p["algorithm"] == "astar"]:
                            primary_edges = set((min(u, v), max(u, v)) for u, v in zip(a_primary, a_primary[1:]))
                            jaccard = len(alt_edge_set & primary_edges) / len(alt_edge_set | primary_edges) if len(alt_edge_set | primary_edges) > 0 else 0
                            if jaccard > 0.65:
                                is_diverse = False
                                break
                        if is_diverse:
                            paths_result.append(build_path_entry(alt, "astar", False, shortest_len))
                            found += 1
                            for u, v in zip(alt, alt[1:]):
                                penalized_edges.add((min(u, v), max(u, v)))
                            current_penalty = 0.5
                        else:
                            current_penalty += 0.3
                    else:
                        break
                except nx.NetworkXNoPath:
                    break

    if not paths_result:
        return jsonify({"error": "No paths found between these points"}), 404

    return jsonify({"paths": paths_result, "hour": hour, "time_aware": use_time_traffic}), 200


@app.route("/graph", methods=["POST"])
def create_graph():
    data = request.get_json(force=True)
    graph_id = data.get("graph_id", "default")
    G = nx.DiGraph()
    for node in data.get("nodes", []):
        nid = node["id"] if isinstance(node, dict) else node
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
    data = request.get_json(force=True)
    graph_id = data.get("graph_id", "default")
    source = data.get("source")
    target = data.get("target")
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
    data = request.get_json(force=True)
    graph_id = data.get("graph_id", "default")
    if graph_id not in _graph_db:
        return jsonify({"error": f"Graph '{graph_id}' not found"}), 404
    G = _graph_db[graph_id]
    edge_delays = []
    for u, v, attrs in G.edges(data=True):
        base = attrs.get("base_weight", 1.0)
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
    data = request.get_json(force=True)
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
    in_degrees = [d for _, d in G.in_degree()]
    out_degrees = [d for _, d in G.out_degree()]
    return jsonify({
        "graph_id": graph_id,
        "node_count": G.number_of_nodes(), "edge_count": G.number_of_edges(),
        "average_path_length": round(avg_path_length, 2) if avg_path_length else None,
        "top_bottlenecks": [{"node": n, "centrality": round(c, 4)} for n, c in top_bottlenecks],
        "degree_stats": {
            "avg_in_degree": round(sum(in_degrees) / len(in_degrees), 2) if in_degrees else 0,
            "avg_out_degree": round(sum(out_degrees) / len(out_degrees), 2) if out_degrees else 0,
            "max_in_degree": max(in_degrees) if in_degrees else 0,
            "max_out_degree": max(out_degrees) if out_degrees else 0,
        },
        "timestamp": datetime.utcnow().isoformat() + "Z"
    }), 200


@app.route("/simulate-congestion", methods=["POST"])
def simulate_congestion():
    data = request.get_json(force=True)
    _simulation_config["congestion_factor"] = data.get("congestion_factor", 1.0)
    _simulation_config["delay_per_edge"] = data.get("delay_per_edge", 0.0)
    _simulation_config["hour_of_day"] = data.get("hour", datetime.now().hour)
    return jsonify({"status": "simulation_config_updated", "config": _simulation_config}), 200


@app.route("/health", methods=["GET"])
def health():
    return jsonify({
        "status": "healthy",
        "graphs_loaded": len(_graph_db),
        "osm_graph_ready": OSM_GRAPH_ID in _graph_db
    }), 200


if __name__ == "__main__":
    load_osm_graph()
    app.run(host="0.0.0.0", port=5000, debug=False)