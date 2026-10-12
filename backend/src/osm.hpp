#pragma once

// Lightweight streaming parser for OSM XML (.osm) files.
// No external XML library: OSM planet dumps are regular enough that a
// hand-rolled element scanner is safe, fast, and dependency-free (a hard
// requirement for the vehicle's built-in computer later on).

#include <string>
#include <vector>
#include <unordered_map>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <cstdint>
#include "util.hpp"

namespace nd {

struct OsmNode {
    double lat = 0, lon = 0;
    float ele = NAN;          // from "ele" tag or external elevation file
    uint8_t barrier = 0;      // BarrierType
    float barrier_width = -1; // usable opening in metres, <0 = untagged
    bool barrier_locked = false;
};

enum BarrierType : uint8_t {
    B_NONE = 0,
    B_BOLLARD,
    B_GATE,
    B_CYCLE_BARRIER,
    B_BLOCK,
    B_KISSING_GATE,
    B_TURNSTILE,
    B_MOTORCYCLE,
    B_SWING_GATE,
    B_LIFT_GATE,
    B_CHAIN,
    B_OTHER
};

inline const char* barrier_name(uint8_t b) {
    switch (b) {
        case B_BOLLARD: return "Potelet";
        case B_GATE: return "Portail";
        case B_CYCLE_BARRIER: return "Chicane vélo";
        case B_BLOCK: return "Bloc";
        case B_KISSING_GATE: return "Portillon en chicane";
        case B_TURNSTILE: return "Tourniquet";
        case B_MOTORCYCLE: return "Barrière anti-motos";
        case B_SWING_GATE: return "Portail pivotant";
        case B_LIFT_GATE: return "Barrière levante";
        case B_CHAIN: return "Chaîne";
        case B_OTHER: return "Barrière";
        default: return "aucune";
    }
}

struct OsmWay {
    int64_t id = 0;
    std::vector<int64_t> refs;
    std::string highway, surface, oneway, oneway_bicycle, bicycle, foot, access,
        vehicle_tag, motor_vehicle, lit, cycleway, tracktype, smoothness, name,
        maxwidth, width, area, segregated, maxspeed;
};

struct OsmData {
    std::unordered_map<int64_t, OsmNode> nodes;
    std::vector<OsmWay> ways;
};

namespace detail {

inline std::string decode_entities(const std::string& s) {
    if (s.find('&') == std::string::npos)
        return s;
    std::string out;
    out.reserve(s.size());
    for (size_t i = 0; i < s.size(); ++i) {
        if (s[i] != '&') {
            out += s[i];
            continue;
        }
        size_t semi = s.find(';', i);
        if (semi == std::string::npos || semi - i > 8) {
            out += s[i];
            continue;
        }
        std::string ent = s.substr(i + 1, semi - i - 1);
        if (ent == "amp")
            out += '&';
        else if (ent == "lt")
            out += '<';
        else if (ent == "gt")
            out += '>';
        else if (ent == "quot")
            out += '"';
        else if (ent == "apos")
            out += '\'';
        else if (!ent.empty() && ent[0] == '#') {
            try {
                long code = (ent.size() > 1 && (ent[1] == 'x' || ent[1] == 'X'))
                                ? std::stol(ent.substr(2), nullptr, 16)
                                : std::stol(ent.substr(1));
                if (code > 0 && code < 128)
                    out += char(code); 
            } catch (...) {
            }
        }
        i = semi;
    }
    return out;
}

struct Attr {
    std::string k, v;
};

inline size_t parse_attrs(const std::string& s,
                          size_t pos,
                          std::vector<Attr>& out,
                          bool& self_closing) {
    out.clear();
    self_closing = false;
    size_t n = s.size();
    while (pos < n) {
        while (pos < n && (s[pos] == ' ' || s[pos] == '\t' || s[pos] == '\n' ||
                           s[pos] == '\r'))
            pos++;
        if (pos >= n)
            break;
        if (s[pos] == '/') {
            self_closing = true;
            pos++;
            continue;
        }
        if (s[pos] == '>') {
            pos++;
            break;
        }
        size_t ks = pos;
        while (pos < n && s[pos] != '=' && s[pos] != '>' && s[pos] != ' ')
            pos++;
        if (pos >= n || s[pos] != '=')
            continue;
        std::string key = s.substr(ks, pos - ks);
        pos++; 
        if (pos >= n)
            break;
        char quote = s[pos];
        if (quote != '"' && quote != '\'')
            continue;
        pos++;
        size_t vs = pos;
        while (pos < n && s[pos] != quote)
            pos++;
        out.push_back(
            {std::move(key), decode_entities(s.substr(vs, pos - vs))});
        if (pos < n)
            pos++; 
    }
    return pos;
}

} 

// Parse a .osm XML file. Keeps every node (geometry) and every way that has a
// highway tag. Node tags are folded into the compact OsmNode representation.

inline OsmData parse_osm_file(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f)
        throw std::runtime_error("cannot open OSM file: " + path);
    std::stringstream ss;
    ss << f.rdbuf();
    std::string s = ss.str();

    OsmData data;
    size_t pos = 0, n = s.size();

    enum Ctx { NONE, NODE, WAY } ctx = NONE;
    int64_t cur_node_id = 0;
    OsmNode cur_node;
    bool cur_node_keep_extra = false; // has barrier/ele info
    OsmWay cur_way;
    std::vector<detail::Attr> attrs;

    auto flush_node = [&]() {
        data.nodes[cur_node_id] = cur_node;
        ctx = NONE;
    };
    auto flush_way = [&]() {
        if (!cur_way.highway.empty() && cur_way.refs.size() >= 2)
            data.ways.push_back(std::move(cur_way));
        cur_way = OsmWay{};
        ctx = NONE;
    };

    while (pos < n) {
        size_t lt = s.find('<', pos);
        if (lt == std::string::npos)
            break;
        pos = lt + 1;
        if (pos >= n)
            break;
        char c = s[pos];
        if (c == '?' || c == '!') {
            pos = s.find('>', pos);
            if (pos == std::string::npos)
                break;
            pos++;
            continue;
        }
        if (c == '/') { 
            size_t gt = s.find('>', pos);
            std::string name = s.substr(pos + 1, gt - pos - 1);
            if (name == "node" && ctx == NODE)
                flush_node();
            else if (name == "way" && ctx == WAY)
                flush_way();
            pos = gt + 1;
            continue;
        }
        size_t ns = pos;
        while (pos < n && s[pos] != ' ' && s[pos] != '>' && s[pos] != '/' &&
               s[pos] != '\t' && s[pos] != '\n' && s[pos] != '\r')
            pos++;
        std::string name = s.substr(ns, pos - ns);
        bool self_closing = false;
        pos = detail::parse_attrs(s, pos, attrs, self_closing);

        if (name == "node") {
            cur_node = OsmNode{};
            cur_node_id = 0;
            cur_node_keep_extra = false;
            for (auto& a : attrs) {
                if (a.k == "id")
                    cur_node_id = std::stoll(a.v);
                else if (a.k == "lat")
                    cur_node.lat = std::stod(a.v);
                else if (a.k == "lon")
                    cur_node.lon = std::stod(a.v);
            }
            if (self_closing) {
                data.nodes[cur_node_id] = cur_node;
            } else
                ctx = NODE;
        } else if (name == "way") {
            cur_way = OsmWay{};
            for (auto& a : attrs)
                if (a.k == "id")
                    cur_way.id = std::stoll(a.v);
            if (self_closing) { /* empty way, ignore */
            } else
                ctx = WAY;
        } else if (name == "nd" && ctx == WAY) {
            for (auto& a : attrs)
                if (a.k == "ref")
                    cur_way.refs.push_back(std::stoll(a.v));
        } else if (name == "tag") {
            std::string k, v;
            for (auto& a : attrs) {
                if (a.k == "k")
                    k = a.v;
                else if (a.k == "v")
                    v = a.v;
            }
            if (ctx == NODE) {
                if (k == "barrier") {
                    cur_node_keep_extra = true;
                    if (v == "bollard")
                        cur_node.barrier = B_BOLLARD;
                    else if (v == "gate")
                        cur_node.barrier = B_GATE;
                    else if (v == "cycle_barrier")
                        cur_node.barrier = B_CYCLE_BARRIER;
                    else if (v == "block")
                        cur_node.barrier = B_BLOCK;
                    else if (v == "kissing_gate")
                        cur_node.barrier = B_KISSING_GATE;
                    else if (v == "turnstile" || v == "full-height_turnstile")
                        cur_node.barrier = B_TURNSTILE;
                    else if (v == "motorcycle_barrier")
                        cur_node.barrier = B_MOTORCYCLE;
                    else if (v == "swing_gate")
                        cur_node.barrier = B_SWING_GATE;
                    else if (v == "lift_gate")
                        cur_node.barrier = B_LIFT_GATE;
                    else if (v == "chain")
                        cur_node.barrier = B_CHAIN;
                    else if (v == "kerb" || v == "entrance" ||
                             v == "toll_booth" || v == "border_control")
                        cur_node.barrier = B_NONE;
                    else
                        cur_node.barrier = B_OTHER;
                } else if (k == "maxwidth" || k == "maxwidth:physical" ||
                           k == "opening" || k == "width") {
                    double w = parse_meters(v);
                    if (w > 0 && (cur_node.barrier_width < 0 ||
                                  w < cur_node.barrier_width))
                        cur_node.barrier_width = (float)w;
                } else if (k == "ele") {
                    try {
                        cur_node.ele = std::stof(v);
                    } catch (...) {
                    }
                } else if (k == "locked" && v == "yes") {
                    cur_node.barrier_locked = true;
                } else if ((k == "access" || k == "bicycle") &&
                           (v == "no" || v == "private")) {
                    // an impassable barrier regardless of width
                    cur_node.barrier_locked = true;
                } else if ((k == "access" || k == "bicycle") &&
                           (v == "yes" || v == "permissive" ||
                            v == "designated")) {
                    // explicitly passable: treat as generous opening if untagged
                    if (cur_node.barrier_width < 0)
                        cur_node.barrier_width = 9.0f;
                }
            } else if (ctx == WAY) {
                if (k == "highway")
                    cur_way.highway = v;
                else if (k == "surface")
                    cur_way.surface = v;
                else if (k == "oneway")
                    cur_way.oneway = v;
                else if (k == "oneway:bicycle")
                    cur_way.oneway_bicycle = v;
                else if (k == "bicycle")
                    cur_way.bicycle = v;
                else if (k == "foot")
                    cur_way.foot = v;
                else if (k == "access")
                    cur_way.access = v;
                else if (k == "vehicle")
                    cur_way.vehicle_tag = v;
                else if (k == "motor_vehicle")
                    cur_way.motor_vehicle = v;
                else if (k == "lit")
                    cur_way.lit = v;
                else if (k == "cycleway" || k == "cycleway:left" ||
                         k == "cycleway:right" || k == "cycleway:both") {
                    if (cur_way.cycleway.empty() || cur_way.cycleway == "no")
                        cur_way.cycleway = v;
                } else if (k == "tracktype")
                    cur_way.tracktype = v;
                else if (k == "smoothness")
                    cur_way.smoothness = v;
                else if (k == "name")
                    cur_way.name = v;
                else if (k == "maxwidth" || k == "maxwidth:physical")
                    cur_way.maxwidth = v;
                else if (k == "width")
                    cur_way.width = v;
                else if (k == "area")
                    cur_way.area = v;
                else if (k == "segregated")
                    cur_way.segregated = v;
                else if (k == "maxspeed")
                    cur_way.maxspeed = v;
            }
        }
        (void)cur_node_keep_extra;
    }
    return data;
}

// Optional external elevation file: lines of "node_id,elevation_m".
// Returns number of nodes updated.

inline size_t load_elevations(OsmData& data, const std::string& path) {
    std::ifstream f(path);
    if (!f)
        return 0;
    size_t count = 0;
    std::string line;
    while (std::getline(f, line)) {
        size_t comma = line.find(',');
        if (comma == std::string::npos)
            continue;
        try {
            int64_t id = std::stoll(line.substr(0, comma));
            float ele = std::stof(line.substr(comma + 1));
            auto it = data.nodes.find(id);
            if (it != data.nodes.end()) {
                it->second.ele = ele;
                count++;
            }
        } catch (...) {
        }
    }
    return count;
}

}