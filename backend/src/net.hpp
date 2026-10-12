#pragma once
// On-demand network data: downloads the OSM road network and an
// elevation grid for the area a route actually needs, so the router works
// anywhere without pre-downloading extracts.
//
// External transfers go through the system `curl` binary (present on
// Windows 10+, macOS, and virtually every Linux) - the backend itself stays
// free of HTTP/TLS library dependencies for the vehicle's built-in computer.

#include <algorithm>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <thread>
#include <chrono>
#include "graph.hpp"

namespace nd {

struct BBox {
    double s = 90, w = 180, n = -90, e = -180;
    void expand(double lat, double lon) {
        s = std::min(s, lat);
        n = std::max(n, lat);
        w = std::min(w, lon);
        e = std::max(e, lon);
    }
    bool contains(const BBox& o) const {
        return o.s >= s && o.n <= n && o.w >= w && o.e <= e;
    }
    bool valid() const {
        return n >= s && e >= w;
    }
};

// Public services (Overpass in particular) reject curl's default User-Agent
// with HTTP 406, and their usage policies ask clients to identify themselves.
#define ND_CURL "curl -s -A \"Veli/1.0 (NeoDrive route planner; https://projectneodrive.ch/)\" "

inline bool run_cmd(const std::string& cmd) {
    return std::system(cmd.c_str()) == 0;
}

inline std::string read_file(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f)
        return "";
    std::stringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

//Overpass: road network for a bbox

// Downloads all highway ways in `b` to <dir>/map_auto.osm. Returns the path,
// or "" on failure.
inline std::string fetch_overpass(const BBox& b, const std::string& dir) {
    std::string qpath = dir + "/overpass_query.txt";
    std::string opath = dir + "/map_auto.osm";
    {
        char buf[512];
        snprintf(buf,
                 sizeof buf,
                 "[out:xml][timeout:120];\n"
                 "(way[\"highway\"](%.6f,%.6f,%.6f,%.6f););\n"
                 "(._;>;);\nout body;",
                 b.s,
                 b.w,
                 b.n,
                 b.e);
        std::ofstream q(qpath, std::ios::binary);
        q << buf;
    }
    // Download beside the cache and only replace it on success, so a failed
    // request never destroys the last good area.
    std::string tpath = opath + ".part";
    // Public Overpass instances all serve the same OpenStreetMap data but
    // rate-limit per IP and go down independently, so try each in turn.
    // Only instances run by OpenStreetMap community non-profits are listed.
    // VELI_OVERPASS_URL puts your own (e.g. self-hosted) instance first.
    std::vector<std::string> endpoints;
    auto add = [&](const std::string& u) {
        if (std::find(endpoints.begin(), endpoints.end(), u) == endpoints.end())
            endpoints.push_back(u);
    };
    if (const char* own = std::getenv("VELI_OVERPASS_URL"); own && *own)
        add(own);
    // Swiss OpenStreetMap Association (SOSM): Switzerland plus a border
    // margin only, so it goes first just for areas around Switzerland.
    if (b.s > 45.6 && b.n < 48.0 && b.w > 5.7 && b.e < 10.7)
        add("https://overpass.osm.ch/api/interpreter");
    add("https://overpass-api.de/api/interpreter");   // FOSSGIS e.V. (DE)
    add("https://z.overpass-api.de/api/interpreter"); // FOSSGIS, 2nd server
    add("https://overpass.private.coffee/api/interpreter"); // Private.coffee (AT)
    // Sanity: must be OSM XML, not an error page. A rate-limited or timed-out
    // query can also come back as valid XML carrying only a runtime-error
    // remark and truncated data, which must not be cached either. A result
    // with no ways at all means the instance does not cover this area.
    auto valid_osm = [&]() {
        std::string body = read_file(tpath);
        if (body.substr(0, 4096).find("<osm") == std::string::npos ||
            body.find("<way ") == std::string::npos)
            return false;
        size_t r = body.find("<remark>");
        if (r == std::string::npos)
            return true;
        std::string remark = body.substr(r, body.find("</remark>", r) - r);
        return remark.find("error") == std::string::npos;
    };
    bool ok = false;
    // Slots on a busy instance free up within a minute: after a full pass
    // over every mirror, wait and go round again (up to three passes).
    for (int round = 0; round < 3 && !ok; ++round) {
        if (round > 0)
            std::this_thread::sleep_for(std::chrono::seconds(15));
        for (size_t k = 0; k < endpoints.size() && !ok; ++k) {
            const std::string& url = endpoints[k];
            std::string cmd = ND_CURL "--max-time 180 --data-urlencode \"data@" +
                              qpath + "\" \"" + url + "\" -o \"" + tpath + "\"";
            // Mirrors behind a load balancer often bounce one request with an
            // instant 502/504 and accept the next, so give each a second try.
            for (int attempt = 0; attempt < 2 && !ok; ++attempt) {
                if (attempt > 0)
                    std::this_thread::sleep_for(std::chrono::seconds(5));
                fprintf(stderr, "  Overpass: %s\n", url.c_str());
                ok = run_cmd(cmd) && valid_osm();
            }
        }
    }
    if (!ok) {
        std::remove(tpath.c_str());
        return "";
    }
    std::remove(opath.c_str());
    if (std::rename(tpath.c_str(), opath.c_str()) != 0)
        return "";
    return opath;
}

// OpenTopoData: coarse elevation grid
// A full per-node elevation fetch takes minutes; a ~100 m grid takes seconds
// and is bilinearly interpolated onto the graph - plenty for route-scale
// gradients (the slope-asymmetry model cares about hills, not curbstones).

struct EleGrid {
    BBox b;
    int nlat = 0, nlon = 0;
    std::vector<float> z; // row-major, rows south->north

    bool valid() const {
        return nlat >= 2 && nlon >= 2 && (int)z.size() == nlat * nlon;
    }

    float sample(double lat, double lon) const {
        double fy = (lat - b.s) / (b.n - b.s) * (nlat - 1);
        double fx = (lon - b.w) / (b.e - b.w) * (nlon - 1);
        fy = std::clamp(fy, 0.0, nlat - 1.000001);
        fx = std::clamp(fx, 0.0, nlon - 1.000001);
        int y0 = (int)fy, x0 = (int)fx;
        int y1 = std::min(y0 + 1, nlat - 1), x1 = std::min(x0 + 1, nlon - 1);
        float z00 = z[y0 * nlon + x0], z01 = z[y0 * nlon + x1];
        float z10 = z[y1 * nlon + x0], z11 = z[y1 * nlon + x1];
        if (std::isnan(z00) || std::isnan(z01) || std::isnan(z10) ||
            std::isnan(z11))
            return NAN;
        double ty = fy - y0, tx = fx - x0;
        return (float)((z00 * (1 - tx) + z01 * tx) * (1 - ty) +
                       (z10 * (1 - tx) + z11 * tx) * ty);
    }
};

// Parses every `"elevation": <number|null>` occurrence, in order.
inline void parse_elevations_json(const std::string& json,
                                  std::vector<float>& out) {
    size_t pos = 0;
    const std::string key = "\"elevation\":";
    while ((pos = json.find(key, pos)) != std::string::npos) {
        pos += key.size();
        while (pos < json.size() && (json[pos] == ' ' || json[pos] == '\t'))
            pos++;
        if (json.compare(pos, 4, "null") == 0)
            out.push_back(NAN);
        else
            out.push_back((float)strtod(json.c_str() + pos, nullptr));
    }
}

// Parses Open-Meteo's `{"elevation":[n, n, ...]}`.
inline void parse_elevation_array(const std::string& json,
                                  std::vector<float>& out) {
    size_t pos = json.find("\"elevation\"");
    if (pos == std::string::npos || (pos = json.find('[', pos)) == std::string::npos)
        return;
    const char* p = json.c_str() + pos + 1;
    while (*p && *p != ']') {
        while (*p == ' ' || *p == ',')
            p++;
        if (strncmp(p, "null", 4) == 0) {
            out.push_back(NAN);
            p += 4;
            continue;
        }
        char* end;
        double v = strtod(p, &end);
        if (end == p)
            break;
        out.push_back((float)v);
        p = end;
    }
}

inline bool
fetch_elevation_grid(const BBox& b, const std::string& dir, EleGrid& grid) {
    double mid = deg2rad((b.s + b.n) / 2);
    double m_lat = (b.n - b.s) * 111320.0;
    double m_lon = (b.e - b.w) * 111320.0 * std::cos(mid);
    // target ~120 m spacing, capped so a fetch stays a handful of requests
    grid.b = b;
    grid.nlat = std::clamp((int)(m_lat / 120.0) + 2, 8, 33);
    grid.nlon = std::clamp((int)(m_lon / 120.0) + 2, 8, 33);
    std::vector<std::pair<double, double>> pts;
    pts.reserve((size_t)grid.nlat * grid.nlon);
    for (int y = 0; y < grid.nlat; ++y)
        for (int x = 0; x < grid.nlon; ++x)
            pts.push_back({b.s + (b.n - b.s) * y / (grid.nlat - 1),
                           b.w + (b.e - b.w) * x / (grid.nlon - 1)});

    grid.z.clear();
    grid.z.reserve(pts.size());
    std::string reqp = dir + "/ele_req.json", resp = dir + "/ele_resp.json";
    const size_t BATCH = 100; // both services accept at most 100 points
    for (size_t i = 0; i < pts.size(); i += BATCH) {
        const size_t n = std::min(BATCH, pts.size() - i);
        std::ostringstream loc, lats, lons;
        for (auto* s : {&loc, &lats, &lons})
            *s << std::setprecision(6) << std::fixed;
        for (size_t j = i; j < i + n; ++j) {
            if (j > i) {
                loc << "|";
                lats << ",";
                lons << ",";
            }
            loc << pts[j].first << "," << pts[j].second;
            lats << pts[j].first;
            lons << pts[j].second;
        }

        // Primary: OpenTopoData (SRTM 30 m). Asks for 1 request/s and allows
        // 1000 requests/day, i.e. ~90 areas.
        bool got = false;
        {
            std::ofstream f(reqp, std::ios::binary);
            f << "{\"locations\":\"" << loc.str() << "\"}";
        }
        for (int attempt = 0; attempt < 2 && !got; ++attempt) {
            if (attempt > 0)
                std::this_thread::sleep_for(std::chrono::milliseconds(3000));
            std::string cmd =
                ND_CURL "--max-time 30 -H \"Content-Type: application/json\" "
                "--data-binary \"@" +
                reqp +
                "\" "
                "https://api.opentopodata.org/v1/srtm30m -o \"" +
                resp + "\"";
            if (run_cmd(cmd)) {
                std::vector<float> vals;
                parse_elevations_json(read_file(resp), vals);
                if (vals.size() == n) {
                    grid.z.insert(grid.z.end(), vals.begin(), vals.end());
                    got = true;
                }
            }
        }

        // Fallback: Open-Meteo (Copernicus 90 m DEM). Its free tier counts
        // every point as a call (600/min), so it can only cover a few
        // batches a minute - fine as a backup, too tight to be primary.
        if (!got) {
            std::string cmd = ND_CURL "--max-time 30 \"https://api.open-meteo.com/"
                                      "v1/elevation?latitude=" +
                              lats.str() + "&longitude=" + lons.str() +
                              "\" -o \"" + resp + "\"";
            if (run_cmd(cmd)) {
                std::vector<float> vals;
                parse_elevation_array(read_file(resp), vals);
                if (vals.size() == n) {
                    grid.z.insert(grid.z.end(), vals.begin(), vals.end());
                    got = true;
                }
            }
        }
        if (!got)
            return false; // give up: routing still works, just flat
        std::this_thread::sleep_for(
            std::chrono::milliseconds(1100)); // API etiquette: 1 req/s
    }
    std::remove(reqp.c_str());
    std::remove(resp.c_str());
    return grid.valid();
}

// Fills missing vertex elevations from the grid, then recomputes every edge
// gradient consistently.
inline void apply_elevation(Graph& g, const EleGrid& grid) {
    for (auto& v : g.vertices)
        if (std::isnan(v.ele))
            v.ele = grid.sample(v.lat, v.lon);
    for (size_t u = 0; u < g.adj.size(); ++u) {
        float eu = g.vertices[u].ele;
        if (std::isnan(eu))
            continue;
        for (auto& e : g.adj[u]) {
            float et = g.vertices[e.to].ele;
            if (std::isnan(et) || e.length <= 0)
                continue;
            e.grade = std::clamp((et - eu) / e.length, -0.30f, 0.30f);
        }
    }
}

}
