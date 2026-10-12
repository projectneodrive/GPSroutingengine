#pragma once

// A* routing over the constraint-aware cost model.
//
// Cost unit is "perceived seconds": real travel time from the physics model,
// multiplied by preference factors (car traffic discomfort, unpaved aversion,
// unlit ways at night, scenic bonus) plus turn penalties scaled by the
// vehicle's maneuvrability. Hard constraints (width vs. barriers, legal
// access) prune edges/nodes outright.

#include <algorithm>
#include <vector>
#include <queue>
#include <cmath>
#include <string>
#include <unordered_set>
#include "graph.hpp"
#include "vehicle.hpp"

namespace nd {

struct RouteWarning {
    double lat, lon;
    std::string kind; // "barrier_unknown", "barrier_assumed", "pushing", ...
    std::string text;
};

struct RouteResult {
    bool ok = false;
    std::string error;
    std::vector<int32_t> path; // vertex indices
    double distance_m = 0;
    double time_s = 0; // real (un-weighted) travel time
    double ascent_m = 0, descent_m = 0;
    double on_car_roads_m = 0, on_unpaved_m = 0, on_unlit_m = 0, pushing_m = 0;
    std::vector<RouteWarning> warnings;
};

// Width margin: a 0.95 m vehicle does not thread a 1.00 m gate at speed.
constexpr double kWidthMargin = 0.06;

// Typical physical openings for untagged barriers (metres). Used only to
// decide plausibility when the user opts to assume untagged = passable.
inline double barrier_default_opening(uint8_t b) {
    switch (b) {
        case B_BOLLARD: return 1.35; // common bollard spacing
        case B_GATE:
        case B_SWING_GATE:
        case B_LIFT_GATE: return 3.0;     // opens
        case B_CYCLE_BARRIER: return 1.1; // chicane: tight AND needs maneuvring
        case B_BLOCK: return 1.3;
        case B_KISSING_GATE: return 0.5; // never fits anything with a body
        case B_TURNSTILE: return 0.5;
        case B_MOTORCYCLE: return 0.85; // designed to stop wide two-tracks
        case B_CHAIN: return 1.2;
        default: return 1.2;
    }
}

// Can this vehicle pass the barrier at vertex v? Returns:
//  0 = no barrier, 1 = passes (tagged), 2 = passes (assumed), -1 = blocked.
inline int barrier_check(const Vertex& v, const VehicleProfile& p) {
    if (v.barrier == B_NONE)
        return 0;
    if (v.barrier_locked)
        return -1;
    if (v.barrier == B_KISSING_GATE || v.barrier == B_TURNSTILE)
        return -1;
    double need = p.width_m + kWidthMargin;
    if (v.barrier_width >= 0)
        return v.barrier_width >= need ? 1 : -1;
    // Stricter per-type rule: bollards with no width tag are avoided outright.
    if (p.skip_untagged_bollards && v.barrier == B_BOLLARD)
        return -1;
    // Untagged width - the project's open question #1, resolved by a user switch:
    if (!p.assume_untagged_barrier_passable) {
        // conservative: only barriers that open (gates) are assumed passable
        if (v.barrier == B_GATE || v.barrier == B_SWING_GATE ||
            v.barrier == B_LIFT_GATE)
            return 2;
        return -1;
    }
    return barrier_default_opening(v.barrier) >= need ? 2 : -1;
}

struct EdgeEval {
    bool allowed;
    double time_s; // physical traversal time
    double cost_s; // perceived cost (>= 0)
    bool pushing;  // rider must dismount
};

inline EdgeEval evaluate_edge(const Edge& e, const VehicleProfile& p) {
    EdgeEval ev{false, 0, 0, false};

    // hard constraints
    if (e.maxwidth >= 0 && e.maxwidth < p.width_m + kWidthMargin)
        return ev;
    bool has_cycle_infra = (e.flags & EF_CYCLE_INFRA) != 0;

    // Sharing the carriageway with cars: only if enabled, and even then only
    // on roads where car traffic is limited to the user's tolerated speed.
    // (A road with dedicated cycle infrastructure is always fine.)

    if (e.hw <= HW_TERTIARY && !has_cycle_infra) {
        if (!p.allow_car_roads)
            return ev;
        int limit = e.maxspeed > 0 ? e.maxspeed
                                   : (e.hw == HW_PRIMARY     ? 80
                                      : e.hw == HW_SECONDARY ? 60
                                                             : 50);
        if (limit > p.max_road_speed + 0.5)
            return ev;
    }
    // Foot infrastructure: allowed only if the profile permits pushing.
    bool foot_infra = (e.flags & EF_FOOT_INFRA) != 0;
    if (foot_infra && !p.allow_footways)
        return ev;
    // bicycle=no on a non-foot way means vehicles like us are banned outright.
    if ((e.flags & EF_BIKE_NO) && !foot_infra && e.hw >= HW_TRACK)
        return ev;

    //  speed & time
    double v = p.edge_speed(
        e.grade, surface_crr_mult(e.surface), surface_speed_factor(e.surface));
    if (foot_infra) {
        v = std::min(v, 1.4);
        ev.pushing = true;
    } // pushing pace
    double t = e.length / v;
    ev.time_s = t;

    // soft preferences -> perceived cost
    double mult = 1.0;
    switch (e.hw) { // traffic stress, even when legally allowed
        case HW_PRIMARY: mult *= has_cycle_infra ? 1.15 : 1.8; break;
        case HW_SECONDARY: mult *= has_cycle_infra ? 1.10 : 1.5; break;
        case HW_TERTIARY: mult *= has_cycle_infra ? 1.05 : 1.25; break;
        case HW_UNCLASSIFIED: mult *= 1.10; break;
        case HW_RESIDENTIAL: mult *= 1.05; break;
        default: break;
    }
    if (e.flags & EF_BIKE_DESIGNATED)
        mult *= 0.92; // prefer infrastructure built for us
    if (surface_is_unpaved(e.surface))
        mult *= 1.0 + 2.5 * p.avoid_unpaved;
    if (e.surface == SURF_UNKNOWN)
        mult *= 1.0 + 0.3 * p.avoid_unpaved;
    if (p.night) {
        if (e.flags & EF_LIT_NO)
            mult *= 1.7;
        else if (!(e.flags & EF_LIT_YES))
            mult *= (e.hw >= HW_TRACK ? 1.4 : 1.1);
    }
    if (p.scenic > 0) {
        bool green =
            (e.hw == HW_TRACK || e.hw == HW_PATH || e.hw == HW_BRIDLEWAY ||
             (e.hw == HW_CYCLEWAY && !(e.flags & EF_CYCLE_INFRA)));
        if (green)
            mult *= 1.0 - 0.35 * p.scenic;
        else if (e.hw <= HW_TERTIARY)
            mult *= 1.0 + 0.5 * p.scenic;
    }
    if (ev.pushing)
        mult *= 1.3; // pushing is worse than the time alone suggests

    ev.allowed = true;
    ev.cost_s = t * mult;
    return ev;
}

// Turn penalty in seconds, given the direction change in degrees.
inline double turn_penalty(double angle_deg, const VehicleProfile& p) {
    if (angle_deg < 30)
        return 0;
    double stiffness = 1.0 - 0.8 * p.agility; // nimble vehicles barely care
    if (angle_deg > 150)
        return 20.0 * stiffness + 4.0; // near U-turn
    double x = (angle_deg - 30) / 120.0;
    return (1.5 + 8.0 * x * x) * stiffness;
}

// Extra seconds to negotiate a passable barrier (slow down, aim, squeeze).
inline double barrier_penalty(uint8_t b, const VehicleProfile& p) {
    double stiffness = 1.0 - 0.6 * p.agility;
    switch (b) {
        case B_CYCLE_BARRIER: return 25.0 * stiffness + 8.0; // chicane maneuvre
        case B_GATE:
        case B_SWING_GATE: return 15.0; // stop & open
        case B_LIFT_GATE: return 10.0;
        case B_BOLLARD:
        case B_BLOCK:
        case B_CHAIN: return 4.0 * stiffness + 2.0;
        default: return 5.0;
    }
}

// Endpoint snapping for a given vehicle profile.
//
// The plain nearest vertex is often useless for this vehicle: a motorway, a
// private drive, steps, or a stretch walled off by a too-narrow barrier. The
// raw connectivity in Graph::in_main cannot see that, because it ignores the
// profile. So: try the start candidates nearest the click, flood-fill what
// this vehicle can reach from each, and snap the destination to the closest
// reachable vertex. The A* that follows is then guaranteed to succeed.
struct Snap {
    int32_t src = -1, dst = -1;
    double src_off_m = 0, dst_off_m = 0; // distance from click to snapped vertex
};

// Every vertex reachable from `src` under the same rules the A* applies.
inline std::vector<uint8_t>
reachable_from(const Graph& g, const VehicleProfile& p, int32_t src) {
    std::vector<uint8_t> seen(g.vertices.size(), 0);
    std::vector<int32_t> stack{src};
    seen[src] = 1;
    while (!stack.empty()) {
        int32_t u = stack.back();
        stack.pop_back();
        if (u != src && barrier_check(g.vertices[u], p) < 0)
            continue;
        for (const Edge& e : g.adj[u]) {
            if (seen[e.to] || !evaluate_edge(e, p).allowed)
                continue;
            seen[e.to] = 1;
            stack.push_back(e.to);
        }
    }
    return seen;
}

inline Snap snap_endpoints(const Graph& g,
                           const VehicleProfile& p,
                           double from_lat,
                           double from_lon,
                           double to_lat,
                           double to_lon) {
    const size_t N = g.vertices.size();
    // A vertex is a sensible start only if this vehicle can leave it.
    auto usable = [&](size_t i) {
        if (barrier_check(g.vertices[i], p) < 0)
            return false;
        for (const Edge& e : g.adj[i])
            if (evaluate_edge(e, p).allowed)
                return true;
        return false;
    };
    std::vector<std::pair<double, int32_t>> cands; // (distance, vertex)
    double dst_any = 1e18; // nearest usable vertex to the destination
    for (size_t i = 0; i < N; ++i) {
        if (!usable(i))
            continue;
        const Vertex& v = g.vertices[i];
        cands.push_back({haversine(from_lat, from_lon, v.lat, v.lon), (int32_t)i});
        dst_any = std::min(dst_any, haversine(to_lat, to_lon, v.lat, v.lon));
    }
    const size_t K = std::min<size_t>(8, cands.size());
    std::partial_sort(cands.begin(), cands.begin() + K, cands.end());

    // Accept a destination snap that is close in absolute terms, or not much
    // worse than the best the network offers there at all.
    const double dst_limit = std::max(500.0, dst_any + 300.0);
    Snap best;
    double best_total = 1e18;
    int32_t last_root = -1;
    std::vector<uint8_t> seen;
    for (size_t k = 0; k < K; ++k) {
        auto [src_off, src] = cands[k];
        // Candidates already inside the last flood-fill reach the same set.
        if (last_root >= 0 && seen[src])
            continue;
        seen = reachable_from(g, p, src);
        last_root = src;
        int32_t dst = -1;
        double dst_off = 1e18;
        for (size_t i = 0; i < N; ++i) {
            if (!seen[i] || (int32_t)i == src)
                continue;
            double d = haversine(
                to_lat, to_lon, g.vertices[i].lat, g.vertices[i].lon);
            if (d < dst_off) {
                dst_off = d;
                dst = (int32_t)i;
            }
        }
        if (dst < 0 || dst_off > dst_limit)
            continue;
        if (src_off + dst_off < best_total) {
            best_total = src_off + dst_off;
            best = {src, dst, src_off, dst_off};
        }
        if (dst_off <= dst_any + 1.0)
            break; // cannot do better for the destination
    }
    return best;
}

inline RouteResult route(const Graph& g,
                         const VehicleProfile& p,
                         double from_lat,
                         double from_lon,
                         double to_lat,
                         double to_lon) {
    RouteResult res;
    if (g.nearest_vertex(from_lat, from_lon) < 0) {
        res.error =
            "aucun réseau routier chargé à proximité des points demandés";
        return res;
    }
    Snap snap = snap_endpoints(g, p, from_lat, from_lon, to_lat, to_lon);
    int32_t src = snap.src, dst = snap.dst;
    if (src < 0 || dst < 0) {
        res.error = "aucun itinéraire trouvé - les contraintes du véhicule "
                    "(largeur, accès ou "
                    "barrières) déconnectent ces points sur le réseau chargé";
        return res;
    }
    // Aim the heuristic at the snapped vertex, not the raw click.
    to_lat = g.vertices[dst].lat;
    to_lon = g.vertices[dst].lon;
    if (src == dst) {
        res.error =
            "les deux points correspondent au même endroit sur le réseau";
        return res;
    }

    const double vmax = std::max(p.v_cap_down_kmh, p.v_cap_flat_kmh) / 3.6;
    // Perceived cost can dip below real time (scenic bonus, designated ways);
    // scale the heuristic by the smallest possible multiplier to stay admissible.
    const double h_scale = 0.55 / vmax;
    auto heur = [&](int32_t v) {
        return haversine(g.vertices[v].lat, g.vertices[v].lon, to_lat, to_lon) *
               h_scale;
    };

    const size_t N = g.vertices.size();
    std::vector<double> cost(N, 1e18), time_at(N, 0);
    std::vector<int32_t> parent(N, -1);
    std::vector<float> in_bearing(N, -1.0f);
    using QItem = std::pair<double, int32_t>; // (f, vertex)
    std::priority_queue<QItem, std::vector<QItem>, std::greater<QItem>> pq;

    cost[src] = 0;
    pq.push({heur(src), src});

    while (!pq.empty()) {
        auto [f, u] = pq.top();
        pq.pop();
        if (u == dst)
            break;
        if (f - heur(u) > cost[u] + 1e-9)
            continue; // stale entry
        const Vertex& vu = g.vertices[u];
        // A blocked barrier vertex can be a route endpoint but not a through-point.
        if (u != src && barrier_check(vu, p) < 0)
            continue;

        for (const Edge& e : g.adj[u]) {
            EdgeEval ev = evaluate_edge(e, p);
            if (!ev.allowed)
                continue;
            const Vertex& vt = g.vertices[e.to];
            if (e.to != dst && !g.adj[e.to].empty() &&
                barrier_check(vt, p) < 0 && e.to != src) {
                // still allow reaching it as dead end? No: skip early for speed
                // unless it's the destination.
                continue;
            }
            double b_out = bearing(vu.lat, vu.lon, vt.lat, vt.lon);
            double extra = 0;
            if (in_bearing[u] >= 0)
                extra += turn_penalty(turn_angle(in_bearing[u], b_out), p);
            int bc = barrier_check(vt, p);
            if (bc > 0)
                extra += barrier_penalty(vt.barrier, p);
            double nc = cost[u] + ev.cost_s + extra;
            if (nc < cost[e.to]) {
                cost[e.to] = nc;
                time_at[e.to] = time_at[u] + ev.time_s + extra;
                parent[e.to] = u;
                in_bearing[e.to] = (float)b_out;
                pq.push({nc + heur(e.to), e.to});
            }
        }
    }

    if (parent[dst] < 0) {
        res.error = "aucun itinéraire trouvé - les contraintes du véhicule "
                    "(largeur, accès ou "
                    "barrières) déconnectent ces points sur le réseau chargé";
        return res;
    }

    // Reconstruct
    for (int32_t v = dst; v >= 0; v = parent[v]) {
        res.path.push_back(v);
        if (v == src)
            break;
    }
    std::reverse(res.path.begin(), res.path.end());
    res.ok = true;
    res.time_s = time_at[dst];

    // Stats + warnings along the path. Pushing stretches are reported per
    // contiguous segment so each one can be marked on the map.
    std::unordered_set<int32_t> warned;
    double push_seg_len = 0;
    int32_t push_seg_start = -1;
    auto flush_push_segment = [&]() {
        if (push_seg_start >= 0 && push_seg_len > 10) {
            const Vertex& vs = g.vertices[push_seg_start];
            res.warnings.push_back({vs.lat,
                                    vs.lon,
                                    "pushing",
                                    std::to_string((int)push_seg_len) +
                                        " m à pied - cheminement piéton, "
                                        "mettre pied à terre et pousser."});
        }
        push_seg_len = 0;
        push_seg_start = -1;
    };
    for (size_t i = 0; i + 1 < res.path.size(); ++i) {
        int32_t a = res.path[i], b = res.path[i + 1];
        const Edge* used = nullptr;
        for (const Edge& e : g.adj[a])
            if (e.to == b) {
                used = &e;
                break;
            }
        if (!used)
            continue;
        res.distance_m += used->length;
        if (used->grade > 0)
            res.ascent_m += used->grade * used->length;
        else
            res.descent_m -= used->grade * used->length;
        if (used->hw <= HW_TERTIARY && !(used->flags & EF_CYCLE_INFRA))
            res.on_car_roads_m += used->length;
        if (surface_is_unpaved(used->surface))
            res.on_unpaved_m += used->length;
        if (p.night && !(used->flags & EF_LIT_YES))
            res.on_unlit_m += used->length;
        EdgeEval ev = evaluate_edge(*used, p);
        if (ev.pushing) {
            res.pushing_m += used->length;
            if (push_seg_start < 0)
                push_seg_start = a;
            push_seg_len += used->length;
        } else {
            flush_push_segment();
        }

        const Vertex& vb = g.vertices[b];
        int bc = barrier_check(vb, p);
        if (bc == 2 && warned.insert(b).second) {
            res.warnings.push_back(
                {vb.lat,
                 vb.lon,
                 "barrier_assumed",
                 std::string(barrier_name(vb.barrier)) +
                     " sans largeur renseignée sur l'itinéraire - supposé "
                     "franchissable pour un véhicule de " +
                     std::to_string((int)std::round(p.width_m * 100)) +
                     " cm de large. À vérifier sur le terrain."});
        } else if (bc == 1 && warned.insert(b).second && vb.barrier != B_GATE) {
            res.warnings.push_back(
                {vb.lat,
                 vb.lon,
                 "barrier_ok",
                 std::string(barrier_name(vb.barrier)) +
                     " (ouverture renseignée suffisante pour le véhicule)"});
        }
    }
    flush_push_segment();

    // Tell the rider when an endpoint had to move to reach usable network.
    auto snap_warning = [&](int32_t v, double off_m, const char* what) {
        if (off_m < 75)
            return;
        const Vertex& vv = g.vertices[v];
        res.warnings.push_back(
            {vv.lat,
             vv.lon,
             "snapped",
             std::string(what) + " déplacé de " +
                 std::to_string((int)std::round(off_m)) +
                 " m vers le point accessible le plus proche pour ce "
                 "véhicule."});
    };
    snap_warning(src, snap.src_off_m, "Départ");
    snap_warning(dst, snap.dst_off_m, "Arrivée");
    return res;
}

// Multi-leg routing through intermediate stops (étapes): routes each pair of
// consecutive points and concatenates the legs into one RouteResult.
inline RouteResult
route_multi(const Graph& g,
            const VehicleProfile& p,
            const std::vector<std::pair<double, double>>& pts) {
    RouteResult total;
    if (pts.size() < 2) {
        total.error = "au moins deux points sont nécessaires";
        return total;
    }
    for (size_t i = 0; i + 1 < pts.size(); ++i) {
        RouteResult leg = route(g,
                                p,
                                pts[i].first,
                                pts[i].second,
                                pts[i + 1].first,
                                pts[i + 1].second);
        if (!leg.ok) {
            total.ok = false;
            total.error = pts.size() > 2 ? "tronçon " + std::to_string(i + 1) +
                                               " : " + leg.error
                                         : leg.error;
            return total;
        }
        size_t skip = (!total.path.empty() && !leg.path.empty() &&
                       total.path.back() == leg.path.front())
                          ? 1
                          : 0;
        total.path.insert(
            total.path.end(), leg.path.begin() + skip, leg.path.end());
        total.distance_m += leg.distance_m;
        total.time_s += leg.time_s;
        total.ascent_m += leg.ascent_m;
        total.descent_m += leg.descent_m;
        total.on_car_roads_m += leg.on_car_roads_m;
        total.on_unpaved_m += leg.on_unpaved_m;
        total.on_unlit_m += leg.on_unlit_m;
        total.pushing_m += leg.pushing_m;
        for (auto& w : leg.warnings)
            total.warnings.push_back(w);
    }
    total.ok = true;
    return total;
}

}