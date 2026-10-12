#pragma once

#include <string>
#include <cmath>
#include <vector>
#include <map>
#include <cstdint>
#include <algorithm>

namespace nd {

constexpr double kPi = 3.14159265358979323846;
constexpr double kEarthR = 6371000.0; 

inline double deg2rad(double d) {
    return d * kPi / 180.0;
}
inline double rad2deg(double r) {
    return r * 180.0 / kPi;
}

inline double haversine(double lat1, double lon1, double lat2, double lon2) {
    double dlat = deg2rad(lat2 - lat1);
    double dlon = deg2rad(lon2 - lon1);
    double a = std::sin(dlat / 2) * std::sin(dlat / 2) +
               std::cos(deg2rad(lat1)) * std::cos(deg2rad(lat2)) *
                   std::sin(dlon / 2) * std::sin(dlon / 2);
    return 2.0 * kEarthR * std::asin(std::min(1.0, std::sqrt(a)));
}

inline double bearing(double lat1, double lon1, double lat2, double lon2) {
    double p1 = deg2rad(lat1), p2 = deg2rad(lat2), dl = deg2rad(lon2 - lon1);
    double y = std::sin(dl) * std::cos(p2);
    double x = std::cos(p1) * std::sin(p2) -
               std::sin(p1) * std::cos(p2) * std::cos(dl);
    double b = rad2deg(std::atan2(y, x));
    return b < 0 ? b + 360.0 : b;
}

inline double turn_angle(double b_in, double b_out) {
    double d = std::fabs(b_out - b_in);
    if (d > 180.0)
        d = 360.0 - d;
    return d;
}

inline double parse_meters(const std::string& s) {
    if (s.empty())
        return -1.0;
    std::string t = s;
    for (auto& c : t)
        if (c == ',')
            c = '.';
    try {
        size_t idx = 0;
        double v = std::stod(t, &idx);
        while (idx < t.size() && t[idx] == ' ')
            idx++;
        std::string unit = t.substr(idx);
        if (unit == "cm")
            v /= 100.0;
        else if (unit == "mm")
            v /= 1000.0;
        else if (!unit.empty() && unit != "m" && unit != "meter" &&
                 unit != "metre")
            return -1.0;
        return v > 0 ? v : -1.0;
    } catch (...) {
        return -1.0;
    }
}

inline std::string url_decode(const std::string& in) {
    std::string out;
    out.reserve(in.size());
    for (size_t i = 0; i < in.size(); ++i) {
        if (in[i] == '+')
            out += ' ';
        else if (in[i] == '%' && i + 2 < in.size()) {
            auto hex = [](char c) -> int {
                if (c >= '0' && c <= '9')
                    return c - '0';
                if (c >= 'a' && c <= 'f')
                    return c - 'a' + 10;
                if (c >= 'A' && c <= 'F')
                    return c - 'A' + 10;
                return -1;
            };
            int h = hex(in[i + 1]), l = hex(in[i + 2]);
            if (h >= 0 && l >= 0) {
                out += char(h * 16 + l);
                i += 2;
            } else
                out += in[i];
        } else
            out += in[i];
    }
    return out;
}

inline std::map<std::string, std::string> parse_query(const std::string& q) {
    std::map<std::string, std::string> out;
    size_t pos = 0;
    while (pos < q.size()) {
        size_t amp = q.find('&', pos);
        if (amp == std::string::npos)
            amp = q.size();
        std::string kv = q.substr(pos, amp - pos);
        size_t eq = kv.find('=');
        if (eq != std::string::npos)
            out[url_decode(kv.substr(0, eq))] = url_decode(kv.substr(eq + 1));
        else if (!kv.empty())
            out[url_decode(kv)] = "";
        pos = amp + 1;
    }
    return out;
}

inline std::string xml_escape(const std::string& s) {
    std::string out;
    out.reserve(s.size());
    for (char c : s) {
        switch (c) {
            case '&': out += "&amp;"; break;
            case '<': out += "&lt;"; break;
            case '>': out += "&gt;"; break;
            case '"': out += "&quot;"; break;
            case '\'': out += "&apos;"; break;
            default: out += c;
        }
    }
    return out;
}

inline std::string json_escape(const std::string& s) {
    std::string out;
    out.reserve(s.size());
    for (char c : s) {
        switch (c) {
            case '"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default:
                if ((unsigned char)c < 0x20) {
                    char buf[8];
                    snprintf(buf, sizeof buf, "\\u%04x", c);
                    out += buf;
                } else
                    out += c;
        }
    }
    return out;
}

inline double get_num(const std::map<std::string, std::string>& q,
                      const std::string& k,
                      double def) {
    auto it = q.find(k);
    if (it == q.end() || it->second.empty())
        return def;
    try {
        return std::stod(it->second);
    } catch (...) {
        return def;
    }
}

inline bool get_bool(const std::map<std::string, std::string>& q,
                     const std::string& k,
                     bool def) {
    auto it = q.find(k);
    if (it == q.end())
        return def;
    return it->second == "1" || it->second == "true" || it->second == "yes";
}

}
