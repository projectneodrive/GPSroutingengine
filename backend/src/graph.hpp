#pragma once
// NeoDrive routing graph built from parsed OSM data.
// Every way node becomes a vertex (geometry is preserved for free); each
// consecutive node pair becomes one or two directed edges carrying the
// attributes the cost model needs.
#include <vector>
#include <unordered_map>
#include <string>
#include <cstdint>
#include <cmath>
#include "osm.hpp"
#include "util.hpp"

namespace nd {

enum HighwayClass : uint8_t {
    HW_PRIMARY = 0,
    HW_SECONDARY,
    HW_TERTIARY,
    HW_UNCLASSIFIED,
    HW_RESIDENTIAL,
    HW_LIVING_STREET,
    HW_SERVICE,
    HW_TRACK,
    HW_CYCLEWAY,
    HW_PATH,
    HW_FOOTWAY,
    HW_PEDESTRIAN,
    HW_BRIDLEWAY,
    HW_OTHER,
    HW_COUNT
};

inline const char* highway_name(uint8_t h) {
    static const char* names[] = {"primary road",
                                  "secondary road",
                                  "tertiary road",
                                  "minor road",
                                  "residential street",
                                  "living street",
                                  "service way",
                                  "track",
                                  "cycleway",
                                  "path",
                                  "footway",
                                  "pedestrian zone",
                                  "bridleway",
                                  "way"};
    return h < HW_COUNT ? names[h] : "way";
}

enum SurfaceClass : uint8_t {
    SURF_UNKNOWN = 0,
    SURF_ASPHALT,
    SURF_PAVED,
    SURF_CONCRETE,
    SURF_PAVING_STONES,
    SURF_SETT,
    SURF_COBBLE,
    SURF_COMPACTED,
    SURF_FINE_GRAVEL,
    SURF_GRAVEL,
    SURF_GROUND,
    SURF_DIRT,
    SURF_GRASS,
    SURF_SAND,
    SURF_MUD,
    SURF_WOOD
};

inline const char* surface_name(uint8_t s) {
    static const char* names[] = {"unknown",
                                  "asphalt",
                                  "paved",
                                  "concrete",
                                  "paving stones",
                                  "sett",
                                  "cobblestone",
                                  "compacted",
                                  "fine gravel",
                                  "gravel",
                                  "ground",
                                  "dirt",
                                  "grass",
                                  "sand",
                                  "mud",
                                  "wood"};
    return s <= SURF_WOOD ? names[s] : "unknown";
}

inline bool surface_is_unpaved(uint8_t s) {
    return s >= SURF_COMPACTED;
}

// Rolling-resistance multiplier vs. smooth asphalt.
inline double surface_crr_mult(uint8_t s) {
    switch (s) {
        case SURF_ASPHALT:
        case SURF_PAVED:
        case SURF_CONCRETE: return 1.0;
        case SURF_PAVING_STONES: return 1.15;
        case SURF_SETT: return 1.5;
        case SURF_COBBLE: return 2.0;
        case SURF_COMPACTED: return 1.3;
        case SURF_FINE_GRAVEL: return 1.6;
        case SURF_GRAVEL: return 2.2;
        case SURF_GROUND:
        case SURF_DIRT: return 2.5;
        case SURF_GRASS: return 4.0;
        case SURF_SAND:
        case SURF_MUD: return 8.0;
        case SURF_WOOD: return 1.1;
        default: return 1.2; // unknown: mildly pessimistic
    }
}

// Comfort/vibration speed factor (a velomobile shakes apart on cobbles long
// before rolling resistance stops it).
inline double surface_speed_factor(uint8_t s) {
    switch (s) {
        case SURF_SETT: return 0.7;
        case SURF_COBBLE: return 0.5;
        case SURF_GRAVEL: return 0.75;
        case SURF_GROUND:
        case SURF_DIRT: return 0.7;
        case SURF_GRASS: return 0.5;
        case SURF_SAND:
        case SURF_MUD: return 0.3;
        default: return 1.0;
    }
}

enum EdgeFlags : uint8_t {
    EF_LIT_YES = 1,
    EF_LIT_NO = 2,
    EF_CYCLE_INFRA = 4, // dedicated lane/track alongside a car road
    EF_BIKE_NO = 8,     // bicycle=no|use_sidepath
    EF_BIKE_DESIGNATED = 16,
    EF_FOOT_INFRA = 32, // footway/pedestrian without bicycle permission
};

struct Edge {
    int32_t to;
    float length;      // metres
    float grade;       // signed rise/run in direction of travel (0 if unknown)
    float maxwidth;    // way-level width restriction, <0 = none known
    uint8_t hw;        // HighwayClass
    uint8_t surface;   // SurfaceClass
    uint8_t flags;     // EdgeFlags
    uint8_t maxspeed;  // posted car speed limit km/h, 0 = untagged
    uint32_t name_idx; // index into Graph::names ("" = 0)
};

struct Vertex {
    double lat, lon;
    float ele;           // NAN if unknown
    uint8_t barrier;     // BarrierType
    float barrier_width; // <0 unknown
    bool barrier_locked;
};

struct Graph {
    std::vector<Vertex> vertices;
    std::vector<std::vector<Edge>> adj;
    std::vector<std::string> names;                  // way names, names[0] = ""
    std::unordered_map<int64_t, int32_t> osm_to_idx; // OSM node id -> vertex
    std::vector<uint8_t> in_main; // part of the main connected network
    double min_lat = 90, max_lat = -90, min_lon = 180, max_lon = -180;
    size_t edge_count = 0;

    // Nearest vertex of the MAIN connected component: snapping to an isolated
    // fragment (a castle courtyard, a park island) would make every route
    // from it fail even though the surrounding network is fine.
    int32_t nearest_vertex(double lat, double lon) const {
        int32_t best = -1;
        double best_d = 1e18;
        for (size_t i = 0; i < vertices.size(); ++i) {
            if (adj[i].empty())
                continue;
            if (!in_main.empty() && !in_main[i])
                continue;
            double d = haversine(lat, lon, vertices[i].lat, vertices[i].lon);
            if (d < best_d) {
                best_d = d;
                best = (int32_t)i;
            }
        }
        return best;
    }
};

namespace detail {

inline uint8_t classify_highway(const std::string& h) {
    if (h == "primary" || h == "primary_link" || h == "trunk" ||
        h == "trunk_link")
        return HW_PRIMARY;
    if (h == "secondary" || h == "secondary_link")
        return HW_SECONDARY;
    if (h == "tertiary" || h == "tertiary_link")
        return HW_TERTIARY;
    if (h == "unclassified" || h == "road")
        return HW_UNCLASSIFIED;
    if (h == "residential")
        return HW_RESIDENTIAL;
    if (h == "living_street")
        return HW_LIVING_STREET;
    if (h == "service")
        return HW_SERVICE;
    if (h == "track")
        return HW_TRACK;
    if (h == "cycleway")
        return HW_CYCLEWAY;
    if (h == "path")
        return HW_PATH;
    if (h == "footway" || h == "steps")
        return HW_FOOTWAY; // steps filtered earlier
    if (h == "pedestrian")
        return HW_PEDESTRIAN;
    if (h == "bridleway")
        return HW_BRIDLEWAY;
    return HW_OTHER;
}

// Posted speed limit in km/h (0 if untagged/unparseable).
inline uint8_t parse_maxspeed(const std::string& s) {
    if (s.empty())
        return 0;
    if (s == "walk")
        return 6;
    if (s == "none")
        return 200;
    try {
        int v = std::stoi(s);
        if (s.find("mph") != std::string::npos)
            v = (int)(v * 1.609 + 0.5);
        return (uint8_t)std::clamp(v, 1, 200);
    } catch (...) {
        return 0;
    }
}

inline uint8_t classify_surface(const OsmWay& w, uint8_t hw) {
    const std::string& s = w.surface;
    if (s == "asphalt" || s == "chipseal")
        return SURF_ASPHALT;
    if (s == "paved")
        return SURF_PAVED;
    if (s == "concrete" || s == "concrete:plates" || s == "concrete:lanes")
        return SURF_CONCRETE;
    if (s == "paving_stones")
        return SURF_PAVING_STONES;
    if (s == "sett" || s == "unhewn_cobblestone:sett")
        return SURF_SETT;
    if (s == "cobblestone" || s == "unhewn_cobblestone")
        return SURF_COBBLE;
    if (s == "compacted")
        return SURF_COMPACTED;
    if (s == "fine_gravel" || s == "pebblestone")
        return SURF_FINE_GRAVEL;
    if (s == "gravel" || s == "rock")
        return SURF_GRAVEL;
    if (s == "ground" || s == "earth" || s == "unpaved")
        return SURF_GROUND;
    if (s == "dirt")
        return SURF_DIRT;
    if (s == "grass" || s == "grass_paver")
        return SURF_GRASS;
    if (s == "sand")
        return SURF_SAND;
    if (s == "mud")
        return SURF_MUD;
    if (s == "wood" || s == "woodchips")
        return SURF_WOOD;
    if (!s.empty())
        return SURF_UNKNOWN;
    // No surface tag: infer from way type / tracktype.
    if (hw == HW_TRACK) {
        if (w.tracktype == "grade1")
            return SURF_COMPACTED;
        if (w.tracktype == "grade2")
            return SURF_GRAVEL;
        if (w.tracktype == "grade3")
            return SURF_DIRT;
        if (w.tracktype == "grade4" || w.tracktype == "grade5")
            return SURF_GRASS;
        return SURF_GRAVEL;
    }
    if (hw == HW_PATH || hw == HW_BRIDLEWAY)
        return SURF_GROUND;
    if (hw <= HW_SERVICE || hw == HW_CYCLEWAY || hw == HW_FOOTWAY ||
        hw == HW_PEDESTRIAN)
        return SURF_ASPHALT; // roads & urban infra default to paved
    return SURF_UNKNOWN;
}

}

inline Graph build_graph(const OsmData& data) {
    Graph g;
    g.names.push_back("");
    std::unordered_map<std::string, uint32_t> name_idx;

    auto vertex_for = [&](int64_t osm_id, const OsmNode& n) -> int32_t {
        auto it = g.osm_to_idx.find(osm_id);
        if (it != g.osm_to_idx.end())
            return it->second;
        int32_t idx = (int32_t)g.vertices.size();
        g.vertices.push_back({n.lat,
                              n.lon,
                              n.ele,
                              n.barrier,
                              n.barrier_width,
                              n.barrier_locked});
        g.adj.emplace_back();
        g.osm_to_idx[osm_id] = idx;
        g.min_lat = std::min(g.min_lat, n.lat);
        g.max_lat = std::max(g.max_lat, n.lat);
        g.min_lon = std::min(g.min_lon, n.lon);
        g.max_lon = std::max(g.max_lon, n.lon);
        return idx;
    };

    for (const auto& w : data.ways) {
        const std::string& h = w.highway;
        // Never usable by any vehicle in this category:
        if (h == "motorway" || h == "motorway_link" || h == "steps" ||
            h == "construction" || h == "proposed" || h == "raceway" ||
            h == "corridor" || h == "elevator" || h == "platform" ||
            h == "bus_stop")
            continue;
        if (w.area == "yes")
            continue; // polygons, not linear ways
        // Legal access:
        bool bike_yes = (w.bicycle == "yes" || w.bicycle == "designated" ||
                         w.bicycle == "permissive");
        if ((w.access == "no" || w.access == "private") && !bike_yes)
            continue;
        // (footway/pedestrian/path with bicycle=no are kept: the rider may
        //  dismount and push - they get flagged as foot infrastructure below)
        if (w.vehicle_tag == "no" && !bike_yes)
            continue;

        uint8_t hw = detail::classify_highway(h);
        uint8_t surf = detail::classify_surface(w, hw);
        uint8_t mspeed = detail::parse_maxspeed(w.maxspeed);

        uint8_t flags = 0;
        if (w.lit == "yes")
            flags |= EF_LIT_YES;
        else if (w.lit == "no")
            flags |= EF_LIT_NO;
        if (!w.cycleway.empty() && w.cycleway != "no" && w.cycleway != "none")
            flags |= EF_CYCLE_INFRA;
        if (w.bicycle == "no" || w.bicycle == "use_sidepath")
            flags |= EF_BIKE_NO;
        if (w.bicycle == "designated" || hw == HW_CYCLEWAY)
            flags |= EF_BIKE_DESIGNATED;
        if ((hw == HW_FOOTWAY || hw == HW_PEDESTRIAN) && !bike_yes)
            flags |= EF_FOOT_INFRA;

        float maxw = -1;
        {
            double mw = parse_meters(w.maxwidth);
            if (mw > 0)
                maxw = (float)mw;
            // A very narrow way is itself a width constraint:
            double ww = parse_meters(w.width);
            if (ww > 0 && (maxw < 0 || ww < maxw) && ww < 3.0 &&
                (hw == HW_PATH || hw == HW_FOOTWAY || hw == HW_CYCLEWAY ||
                 hw == HW_TRACK))
                maxw = (float)ww;
        }

        uint32_t nidx = 0;
        if (!w.name.empty()) {
            auto it = name_idx.find(w.name);
            if (it == name_idx.end()) {
                nidx = (uint32_t)g.names.size();
                g.names.push_back(w.name);
                name_idx[w.name] = nidx;
            } else
                nidx = it->second;
        }

        bool oneway_fwd = false, oneway_rev = false;
        if (w.oneway == "yes" || w.oneway == "1" || w.oneway == "true")
            oneway_fwd = true;
        else if (w.oneway == "-1")
            oneway_rev = true;
        if (w.oneway_bicycle == "no") {
            oneway_fwd = oneway_rev = false;
        }
        // On car roads a oneway binds us like everyone else; on paths it rarely applies.
        if (hw >= HW_TRACK && w.oneway_bicycle.empty() && w.oneway != "yes") {
            oneway_fwd = oneway_rev = false;
        }

        for (size_t i = 0; i + 1 < w.refs.size(); ++i) {
            auto a_it = data.nodes.find(w.refs[i]);
            auto b_it = data.nodes.find(w.refs[i + 1]);
            if (a_it == data.nodes.end() || b_it == data.nodes.end())
                continue;
            int32_t a = vertex_for(w.refs[i], a_it->second);
            int32_t b = vertex_for(w.refs[i + 1], b_it->second);
            if (a == b)
                continue;
            const OsmNode& na = a_it->second;
            const OsmNode& nb = b_it->second;
            float len = (float)haversine(na.lat, na.lon, nb.lat, nb.lon);
            if (len < 0.01f)
                continue;
            float grade = 0.0f;
            if (!std::isnan(na.ele) && !std::isnan(nb.ele))
                grade = std::clamp((nb.ele - na.ele) / len, -0.30f, 0.30f);
            if (!oneway_rev) {
                g.adj[a].push_back(
                    {b, len, grade, maxw, hw, surf, flags, mspeed, nidx});
                g.edge_count++;
            }
            if (!oneway_fwd) {
                g.adj[b].push_back(
                    {a, len, -grade, maxw, hw, surf, flags, mspeed, nidx});
                g.edge_count++;
            }
        }
    }

    // Connected components (undirected) via union-find; mark the largest one
    // as the "main" network for endpoint snapping.
    {
        const size_t N = g.vertices.size();
        std::vector<int32_t> parent(N);
        for (size_t i = 0; i < N; ++i)
            parent[i] = (int32_t)i;
        auto find = [&](int32_t x) {
            while (parent[x] != x) {
                parent[x] = parent[parent[x]];
                x = parent[x];
            }
            return x;
        };
        for (size_t u = 0; u < N; ++u)
            for (const Edge& e : g.adj[u]) {
                int32_t ru = find((int32_t)u), rv = find(e.to);
                if (ru != rv)
                    parent[ru] = rv;
            }
        std::unordered_map<int32_t, size_t> comp_size;
        for (size_t i = 0; i < N; ++i)
            if (!g.adj[i].empty())
                comp_size[find((int32_t)i)]++;
        int32_t main_root = -1;
        size_t main_size = 0;
        for (auto& [root, size] : comp_size)
            if (size > main_size) {
                main_size = size;
                main_root = root;
            }
        g.in_main.assign(N, 0);
        for (size_t i = 0; i < N; ++i)
            if (!g.adj[i].empty() && find((int32_t)i) == main_root)
                g.in_main[i] = 1;
    }
    return g;
}

}
