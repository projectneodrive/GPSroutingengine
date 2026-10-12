#pragma once
// GPX 1.1 export of a computed route.
#include <string>
#include <sstream>
#include <iomanip>
#include <cmath>
#include "graph.hpp"
#include "router.hpp"

namespace nd {

inline std::string route_to_gpx(const Graph& g,
                                const RouteResult& r,
                                const std::string& name = "Itinéraire Véli") {
    std::ostringstream out;
    out << std::setprecision(7) << std::fixed;
    out << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
        << "<gpx version=\"1.1\" creator=\"Véli\" "
           "xmlns=\"http://www.topografix.com/GPX/1/1\">\n"
        << "  <metadata>\n"
        << "    <name>" << xml_escape(name) << "</name>\n"
        << "    <desc>Itinéraire pour véhicule intermédiaire "
           "humain-électrique. Distance "
        << (int)std::round(r.distance_m) << " m, durée estimée "
        << (int)std::round(r.time_s / 60.0) << " min.</desc>\n"
        << "  </metadata>\n";
    // Waypoints for barriers worth checking on the ground
    for (const auto& w : r.warnings) {
        if (w.kind != "barrier_assumed" && w.kind != "barrier_ok")
            continue;
        out << "  <wpt lat=\"" << w.lat << "\" lon=\"" << w.lon << "\">\n"
            << "    <name>"
            << xml_escape(w.kind == "barrier_assumed"
                              ? "Barrière - vérifier la largeur"
                              : "Barrière")
            << "</name>\n"
            << "    <desc>" << xml_escape(w.text) << "</desc>\n"
            << "  </wpt>\n";
    }
    out << "  <trk>\n    <name>" << xml_escape(name)
        << "</name>\n    <trkseg>\n";
    for (int32_t v : r.path) {
        const Vertex& vx = g.vertices[v];
        out << "      <trkpt lat=\"" << vx.lat << "\" lon=\"" << vx.lon
            << "\">";
        if (!std::isnan(vx.ele))
            out << "<ele>" << std::setprecision(1) << vx.ele
                << std::setprecision(7) << "</ele>";
        out << "</trkpt>\n";
    }
    out << "    </trkseg>\n  </trk>\n</gpx>\n";
    return out.str();
}

}
