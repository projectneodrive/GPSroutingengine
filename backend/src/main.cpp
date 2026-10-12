// Véli: routing backend for the NeoDrive light hybrid human-electric vehicle.
//
//   veli serve [--map data/map.osm] [--ele data/elevations.csv]
//                  [--web ../frontend] [--port 8080] [--data data]
//   veli route --map data/map.osm --from 46.52,6.63 --to 46.53,6.60
//                  [--via "lat,lon|lat,lon"] [--gpx out.gpx] [--width 0.95] ...
//
// The server exposes:
//   GET /api/info                     network bounds & stats
//   GET /api/route?from=..&to=..&via=lat,lon|lat,lon&...   route as JSON
//   GET /api/route.gpx?...            same route as a downloadable GPX file
//   GET /...                          static frontend files
//
// If a requested route falls outside the loaded network, the server fetches
// the needed area automatically (Overpass road data + SRTM elevation grid)
// and swaps in the new graph before routing.

#include <cstdio>
#include <cstring>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <chrono>
#include <iomanip>

#include "util.hpp"
#include "osm.hpp"
#include "graph.hpp"
#include "vehicle.hpp"
#include "router.hpp"
#include "gpx.hpp"
#include "net.hpp"

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>
typedef SOCKET sock_t;
#define CLOSESOCK closesocket
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/time.h>
#include <unistd.h>
typedef int sock_t;
#define INVALID_SOCKET (-1)
#define CLOSESOCK close
#endif

using namespace nd;

struct ServerState {
    Graph g;
    BBox loaded; // area the current graph covers
    bool has_graph = false;
    std::string data_dir = "data";
    std::string ele_csv; // optional per-node elevation file (initial map)
};

static std::string network_json(const Graph& g) {
    std::ostringstream o;
    o << std::setprecision(7) << std::fixed
      << "{\"vertices\":" << g.vertices.size() << ",\"edges\":" << g.edge_count
      << ",\"bounds\":[[" << g.min_lat << "," << g.min_lon << "],[" << g.max_lat
      << "," << g.max_lon << "]]}";
    return o.str();
}

static std::string route_to_json(const Graph& g,
                                 const RouteResult& r,
                                 double elapsed_ms,
                                 bool reloaded) {
    std::ostringstream o;
    o << std::setprecision(7) << std::fixed;
    if (!r.ok) {
        o << "{\"ok\":false,\"error\":\"" << json_escape(r.error) << "\"";
        if (reloaded)
            o << ",\"network\":" << network_json(g);
        o << "}";
        return o.str();
    }
    o << "{\"ok\":true"
      << ",\"distance_m\":" << std::setprecision(1) << r.distance_m
      << ",\"time_s\":" << r.time_s << ",\"ascent_m\":" << r.ascent_m
      << ",\"descent_m\":" << r.descent_m
      << ",\"on_car_roads_m\":" << r.on_car_roads_m
      << ",\"on_unpaved_m\":" << r.on_unpaved_m
      << ",\"on_unlit_m\":" << r.on_unlit_m << ",\"pushing_m\":" << r.pushing_m
      << ",\"compute_ms\":" << elapsed_ms << std::setprecision(7);
    if (reloaded)
        o << ",\"network\":" << network_json(g);
    o << ",\"geometry\":[";
    for (size_t i = 0; i < r.path.size(); ++i) {
        const Vertex& v = g.vertices[r.path[i]];
        if (i)
            o << ",";
        o << "[" << v.lat << "," << v.lon << "]";
    }
    // per-point elevations for the profile chart (null where unknown)
    o << "],\"elevations\":[";
    for (size_t i = 0; i < r.path.size(); ++i) {
        if (i)
            o << ",";
        float ele = g.vertices[r.path[i]].ele;
        if (std::isnan(ele))
            o << "null";
        else {
            char buf[16];
            snprintf(buf, sizeof buf, "%.1f", ele);
            o << buf;
        }
    }
    o << "],\"warnings\":[";
    for (size_t i = 0; i < r.warnings.size(); ++i) {
        const auto& w = r.warnings[i];
        if (i)
            o << ",";
        o << "{\"lat\":" << w.lat << ",\"lon\":" << w.lon << ",\"kind\":\""
          << json_escape(w.kind) << "\""
          << ",\"text\":\"" << json_escape(w.text) << "\"}";
    }
    o << "]}";
    return o.str();
}

static bool parse_latlon(const std::string& s, double& lat, double& lon) {
    size_t comma = s.find(',');
    if (comma == std::string::npos)
        return false;
    try {
        lat = std::stod(s.substr(0, comma));
        lon = std::stod(s.substr(comma + 1));
        return lat >= -90 && lat <= 90 && lon >= -180 && lon <= 180;
    } catch (...) {
        return false;
    }
}

// from + via (pipe- or semicolon-separated lat,lon) + to. Empty on error.
static std::vector<std::pair<double, double>>
parse_points(const std::map<std::string, std::string>& q) {
    std::vector<std::pair<double, double>> pts;
    auto fit = q.find("from"), tit = q.find("to");
    double lat, lon;
    if (fit == q.end() || !parse_latlon(fit->second, lat, lon))
        return {};
    pts.push_back({lat, lon});
    auto vit = q.find("via");
    if (vit != q.end() && !vit->second.empty()) {
        std::string s = vit->second;
        for (auto& c : s)
            if (c == ';')
                c = '|';
        size_t pos = 0;
        while (pos <= s.size()) {
            size_t bar = s.find('|', pos);
            if (bar == std::string::npos)
                bar = s.size();
            std::string tok = s.substr(pos, bar - pos);
            if (!tok.empty()) {
                if (!parse_latlon(tok, lat, lon))
                    return {};
                pts.push_back({lat, lon});
            }
            pos = bar + 1;
        }
    }
    if (tit == q.end() || !parse_latlon(tit->second, lat, lon))
        return {};
    pts.push_back({lat, lon});
    return pts;
}

// Makes sure the loaded graph covers all points; fetches the area from
// Overpass (+ elevation grid).
static std::string
ensure_coverage(ServerState& st,
                const std::vector<std::pair<double, double>>& pts,
                bool& reloaded) {
    reloaded = false;
    BBox need;
    for (auto& p : pts)
        need.expand(p.first, p.second);
    // margin: room for detours around the straight-line envelope
    double mlat = std::max(0.25 * (need.n - need.s), 0.012);
    double mlon = std::max(0.25 * (need.e - need.w), 0.017);
    need.s -= mlat;
    need.n += mlat;
    need.w -= mlon;
    need.e += mlon;

    if (st.has_graph && st.loaded.contains(need))
        return "";

    if (need.n - need.s > 0.40 || need.e - need.w > 0.55)
        return "zone trop vaste pour le prototype, gardez les points à moins "
               "de ~40 km les uns des autres";
    {
        double plat = std::max(0.5 * (need.n - need.s), 0.020);
        double plon = std::max(0.5 * (need.e - need.w), 0.028);
        need.s -= plat;
        need.n += plat;
        need.w -= plon;
        need.e += plon;
        // stay under the size cap, shrinking symmetrically around the centre
        double clat = (need.s + need.n) / 2, clon = (need.w + need.e) / 2;
        if (need.n - need.s > 0.40) {
            need.s = clat - 0.20;
            need.n = clat + 0.20;
        }
        if (need.e - need.w > 0.55) {
            need.w = clon - 0.275;
            need.e = clon + 0.275;
        }
    }

    printf("Fetching road network for bbox %.4f,%.4f,%.4f,%.4f ...\n",
           need.s,
           need.w,
           need.n,
           need.e);
    fflush(stdout);
    auto t0 = std::chrono::steady_clock::now();
    std::string osm_path = fetch_overpass(need, st.data_dir);
    if (osm_path.empty())
        return "échec du téléchargement des données OpenStreetMap (service "
               "Overpass indisponible ?)";

    OsmData data;
    try {
        data = parse_osm_file(osm_path);
    } catch (...) {
        return "données OpenStreetMap illisibles";
    }
    if (data.ways.empty())
        return "aucun réseau routier dans cette zone";
    if (!st.ele_csv.empty())
        load_elevations(data, st.ele_csv);

    Graph ng = build_graph(data);
    if (ng.vertices.empty())
        return "aucun réseau routier exploitable dans cette zone";
    double dl_s =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - t0)
            .count();
    printf("  %zu vertices, %zu edges (%.1f s). Fetching elevation grid ...\n",
           ng.vertices.size(),
           ng.edge_count,
           dl_s);
    fflush(stdout);

    EleGrid grid;
    if (fetch_elevation_grid(need, st.data_dir, grid)) {
        apply_elevation(ng, grid);
        printf("  elevation grid %dx%d applied\n", grid.nlat, grid.nlon);
    } else {
        printf("  elevation grid unavailable - gradients flat for this area\n");
    }
    fflush(stdout);

    st.g = std::move(ng);
    st.loaded = need;
    st.has_graph = true;
    reloaded = true;
    return "";
}

// Minimal HTTP server (GET only).

static void send_response(sock_t c,
                          const std::string& status,
                          const std::string& ctype,
                          const std::string& body,
                          const std::string& extra_headers = "") {
    std::ostringstream h;
    h << "HTTP/1.1 " << status << "\r\n"
      << "Content-Type: " << ctype << "\r\n"
      << "Content-Length: " << body.size() << "\r\n"
      << "Access-Control-Allow-Origin: *\r\n"
      << "Cache-Control: no-store\r\n"
      << extra_headers << "Connection: close\r\n\r\n";
    std::string head = h.str();
    send(c, head.data(), (int)head.size(), 0);
    size_t sent = 0;
    while (sent < body.size()) {
        int n = send(c,
                     body.data() + sent,
                     (int)std::min<size_t>(body.size() - sent, 1 << 16),
                     0);
        if (n <= 0)
            break;
        sent += (size_t)n;
    }
}

static std::string guess_mime(const std::string& path) {
    auto ends = [&](const char* e) {
        size_t l = strlen(e);
        return path.size() >= l && path.compare(path.size() - l, l, e) == 0;
    };
    if (ends(".html"))
        return "text/html; charset=utf-8";
    if (ends(".js"))
        return "text/javascript; charset=utf-8";
    if (ends(".css"))
        return "text/css; charset=utf-8";
    if (ends(".json"))
        return "application/json";
    if (ends(".png"))
        return "image/png";
    if (ends(".svg"))
        return "image/svg+xml";
    if (ends(".ico"))
        return "image/x-icon";
    return "application/octet-stream";
}

static void serve(ServerState& st, const std::string& webroot, int port) {
#ifdef _WIN32
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
        fprintf(stderr, "WSAStartup failed\n");
        return;
    }
#endif
    sock_t srv = socket(AF_INET, SOCK_STREAM, 0);
    if (srv == INVALID_SOCKET) {
        fprintf(stderr, "socket() failed\n");
        return;
    }
    int opt = 1;
    setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, (const char*)&opt, sizeof opt);
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK); // dev server: localhost only
    addr.sin_port = htons((unsigned short)port);
    if (bind(srv, (sockaddr*)&addr, sizeof addr) != 0) {
        fprintf(stderr, "bind() failed - is port %d already in use?\n", port);
        return;
    }
    listen(srv, 16);
    printf("Véli server listening on http://localhost:%d/\n", port);
    fflush(stdout);

    for (;;) {
        sock_t c = accept(srv, nullptr, nullptr);
        if (c == INVALID_SOCKET)
            continue;
        // Browsers open spare "preconnect" sockets that may never send a
        // request. On a single-threaded server a blocking recv() on one of
        // them would freeze every other client, so give up after 2 s.
#ifdef _WIN32
        DWORD rcv_timeout = 2000;
#else
        timeval rcv_timeout{2, 0};
#endif
        setsockopt(c,
                   SOL_SOCKET,
                   SO_RCVTIMEO,
                   (const char*)&rcv_timeout,
                   sizeof rcv_timeout);
        std::string req;
        char buf[8192];
        for (;;) {
            int n = recv(c, buf, sizeof buf, 0);
            if (n <= 0)
                break;
            req.append(buf, (size_t)n);
            if (req.find("\r\n\r\n") != std::string::npos || req.size() > 65536)
                break;
        }
        if (req.empty()) { // idle or timed-out connection: nothing to answer
            CLOSESOCK(c);
            continue;
        }
        size_t sp1 = req.find(' ');
        size_t sp2 = (sp1 == std::string::npos) ? std::string::npos
                                                : req.find(' ', sp1 + 1);
        if (sp1 == std::string::npos || sp2 == std::string::npos ||
            req.compare(0, 3, "GET") != 0) {
            send_response(
                c, "405 Method Not Allowed", "text/plain", "GET only");
            CLOSESOCK(c);
            continue;
        }
        std::string target = req.substr(sp1 + 1, sp2 - sp1 - 1);
        size_t qpos = target.find('?');
        std::string path = url_decode(
            qpos == std::string::npos ? target : target.substr(0, qpos));
        std::map<std::string, std::string> query;
        if (qpos != std::string::npos)
            query = parse_query(target.substr(qpos + 1));

        if (path == "/api/info") {
            std::string body =
                st.has_graph ? network_json(st.g)
                             : "{\"vertices\":0,\"edges\":0,\"bounds\":null}";
            send_response(c, "200 OK", "application/json", body);
        } else if (path == "/api/route" || path == "/api/route.gpx") {
            auto pts = parse_points(query);
            if (pts.size() < 2) {
                send_response(
                    c,
                    "400 Bad Request",
                    "application/json",
                    "{\"ok\":false,\"error\":\"paramètres from/to/via "
                    "manquants ou invalides (attendu : lat,lon)\"}");
                CLOSESOCK(c);
                continue;
            }
            bool reloaded = false;
            std::string cov_err = ensure_coverage(st, pts, reloaded);
            if (!cov_err.empty()) {
                send_response(c,
                              "503 Service Unavailable",
                              "application/json",
                              "{\"ok\":false,\"error\":\"" +
                                  json_escape(cov_err) + "\"}");
                CLOSESOCK(c);
                continue;
            }
            VehicleProfile p = get_bool(query, "bicycle_baseline", false)
                                   ? VehicleProfile::bicycle_baseline()
                                   : VehicleProfile::from_query(query);
            auto t0 = std::chrono::steady_clock::now();
            RouteResult res = route_multi(st.g, p, pts);
            double ms = std::chrono::duration<double, std::milli>(
                            std::chrono::steady_clock::now() - t0)
                            .count();
            if (path == "/api/route.gpx") {
                if (!res.ok)
                    send_response(c,
                                  "404 Not Found",
                                  "text/plain",
                                  "aucun itinéraire : " + res.error);
                else
                    send_response(c,
                                  "200 OK",
                                  "application/gpx+xml",
                                  route_to_gpx(st.g, res),
                                  "Content-Disposition: attachment; "
                                  "filename=\"itineraire-veli.gpx\"\r\n");
            } else {
                send_response(c,
                              res.ok ? "200 OK" : "404 Not Found",
                              "application/json",
                              route_to_json(st.g, res, ms, reloaded));
            }
            printf("route %s (%zu pts%s) -> %.0f m, %.0f s, %.1f ms\n",
                   res.ok ? "OK " : "FAIL",
                   pts.size(),
                   reloaded ? ", area reloaded" : "",
                   res.distance_m,
                   res.time_s,
                   ms);
            fflush(stdout);
        } else {
            std::string rel = (path == "/") ? "/index.html" : path;
            if (rel.find("..") != std::string::npos || webroot.empty()) {
                send_response(c, "404 Not Found", "text/plain", "not found");
            } else {
                std::ifstream f(webroot + rel, std::ios::binary);
                if (!f)
                    send_response(
                        c, "404 Not Found", "text/plain", "not found");
                else {
                    std::stringstream ss;
                    ss << f.rdbuf();
                    send_response(c, "200 OK", guess_mime(rel), ss.str());
                }
            }
        }
        CLOSESOCK(c);
    }
}

// ---------------------------------------------------------------------------

static void usage() {
    printf("Véli - routing backend for intermediate human-electric vehicles\n\n"
           "  veli serve [--map <file.osm>] [--ele <elevations.csv>]\n"
           "             [--web <frontend dir>] [--port 8080] [--data <dir>]\n"
           "     (without --map, the network is fetched on the first route "
           "request)\n\n"
           "  veli route --map <file.osm> [--ele <elevations.csv>]\n"
           "             --from <lat,lon> --to <lat,lon> [--via "
           "\"lat,lon|lat,lon\"]\n"
           "             [--gpx <out.gpx>] [vehicle profile flags]\n\n"
           "  vehicle profile flags (also accepted as /api/route query "
           "parameters):\n"
           "             [--width 0.95] [--mass 140] [--power 350] [--cda "
           "0.35] [--crr 0.008]\n"
           "             [--vflat 45] [--vdown 70] [--agility 0..1]\n"
           "             [--car_roads 0|1] [--max_road_speed 50]\n"
           "             [--footways 0|1] [--avoid_unpaved 0..1] [--night 0|1] "
           "[--scenic 0..1]\n"
           "             [--barriers_passable 0|1] [--skip_untagged_bollards "
           "0|1]\n");
}

int main(int argc, char** argv) {
    if (argc < 2) {
        usage();
        return 1;
    }
    std::string mode = argv[1];
    std::map<std::string, std::string> args;
    for (int i = 2; i + 1 < argc; i += 2) {
        std::string k = argv[i];
        if (k.rfind("--", 0) == 0)
            args[k.substr(2)] = argv[i + 1];
    }

    ServerState st;
    if (args.count("data"))
        st.data_dir = args["data"];

    if (args.count("map")) {
        printf("Loading OSM data from %s ...\n", args["map"].c_str());
        fflush(stdout);
        OsmData data;
        try {
            data = parse_osm_file(args["map"]);
        } catch (const std::exception& e) {
            fprintf(stderr, "error: %s\n", e.what());
            return 1;
        }
        printf("  %zu nodes, %zu highway ways\n",
               data.nodes.size(),
               data.ways.size());

        st.ele_csv = args.count("ele") ? args["ele"] : "";
        if (st.ele_csv.empty()) {
            std::string p = args["map"];
            size_t slash = p.find_last_of("/\\");
            st.ele_csv =
                (slash == std::string::npos ? "" : p.substr(0, slash + 1)) +
                "elevations.csv";
        }
        size_t ne = load_elevations(data, st.ele_csv);
        if (ne)
            printf("  elevations for %zu nodes loaded from %s\n",
                   ne,
                   st.ele_csv.c_str());
        else
            printf("  no elevation data (%s not found)\n", st.ele_csv.c_str());

        st.g = build_graph(data);
        st.has_graph = !st.g.vertices.empty();
        st.loaded = {st.g.min_lat, st.g.min_lon, st.g.max_lat, st.g.max_lon};
        printf("  graph: %zu vertices, %zu directed edges\n",
               st.g.vertices.size(),
               st.g.edge_count);
        // derive data dir from the map path if not given explicitly
        if (!args.count("data")) {
            size_t slash = args["map"].find_last_of("/\\");
            if (slash != std::string::npos)
                st.data_dir = args["map"].substr(0, slash);
        }
        fflush(stdout);
    } else if (mode == "route") {
        fprintf(stderr, "error: --map is required in route mode\n");
        return 1;
    } else {
        printf("No --map given: the network will be fetched for the first "
               "route request.\n");
    }

    if (mode == "serve") {
        int port = (int)get_num(args, "port", 8080);
        std::string web = args.count("web") ? args["web"] : "../frontend";
        serve(st, web, port);
        return 0;
    }
    if (mode == "route") {
        std::map<std::string, std::string> q = args;
        auto pts = parse_points(q);
        if (pts.size() < 2) {
            fprintf(stderr,
                    "error: --from et --to doivent être au format lat,lon\n");
            return 1;
        }
        VehicleProfile p = VehicleProfile::from_query(args);
        auto t0 = std::chrono::steady_clock::now();
        RouteResult res = route_multi(st.g, p, pts);
        double ms = std::chrono::duration<double, std::milli>(
                        std::chrono::steady_clock::now() - t0)
                        .count();
        if (!res.ok) {
            fprintf(stderr, "aucun itinéraire : %s\n", res.error.c_str());
            return 2;
        }
        printf("Itinéraire calculé en %.1f ms :\n", ms);
        printf("  distance : %.2f km\n", res.distance_m / 1000.0);
        printf("  durée    : %.0f min (moy. %.1f km/h)\n",
               res.time_s / 60.0,
               res.distance_m / std::max(res.time_s, 1.0) * 3.6);
        printf("  dénivelé : +%.0f m / -%.0f m\n", res.ascent_m, res.descent_m);
        if (res.on_car_roads_m > 0)
            printf("  partagé avec les voitures : %.0f m\n",
                   res.on_car_roads_m);
        if (res.on_unpaved_m > 0)
            printf("  non revêtu : %.0f m\n", res.on_unpaved_m);
        if (res.pushing_m > 0)
            printf("  à pied : %.0f m\n", res.pushing_m);
        for (auto& w : res.warnings)
            printf("  ! %s\n", w.text.c_str());
        if (args.count("gpx")) {
            std::ofstream f(args["gpx"], std::ios::binary);
            f << route_to_gpx(st.g, res);
            printf("GPX écrit dans %s\n", args["gpx"].c_str());
        }
        return 0;
    }
    usage();
    return 1;
}
