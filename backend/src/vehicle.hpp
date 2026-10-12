#pragma once
// Vehicle profile and physics-based speed model.
//
// The whole point of this router: an intermediate vehicle is neither a bicycle
// nor a car. Its speed is NOT a fixed per-road-class table; it comes from a
// power balance that is asymmetric in gradient:
//   - flat & uphill: capped by regulation (assistance cut-off) and by
//     available power (human + motor, i.e. the pedal-by-wire configuration),
//   - downhill: not power-capped; low CdA and mass work in its favour, capped
//     only by the rider's chosen safe maximum.
// So a steeper, longer route can genuinely be faster than a flatter, shorter
// one - which is exactly what bicycle routers never suggest.

#include <map>
#include <string>
#include <cmath>
#include "util.hpp"

namespace nd {

struct VehicleProfile {
    // geometry
    // physical width incl. mirrors; gates narrower than this block us
    double width_m = 0.95;

    // mass & power (pedal-by-wire: assistance is a parameter, not a fact)
    double mass_kg = 140.0; // vehicle + rider + cargo
    double power_w = 350.0; // sustained human + motor power at the wheel
    double cda = 0.35;      // drag area (m^2); velomobiles are slippery
    double crr = 0.008;     // rolling resistance on smooth asphalt

    // regulation / safety caps
    // assisted cap on flat & uphill (25 pedelec / 45 S-class)
    double v_cap_flat_kmh = 45.0;
    double v_cap_down_kmh = 70.0; // rider's chosen safe downhill max

    // network access preferences
    // may it share primary/secondary roads with cars?
    bool allow_car_roads = false;
    // ...but only roads where cars are limited to this (km/h)
    double max_road_speed = 50;
    // 0 = don't care .. 1 = strongly avoid gravel/dirt
    double avoid_unpaved = 0.5;
    bool night = false;  // prefer lit ways
    double scenic = 0.0; // 0 = fastest .. 1 = prefer green/quiet ways
    // the open question, as a switch
    bool assume_untagged_barrier_passable = true;
    // stricter: bollards w/o width = blocked
    bool skip_untagged_bollards = false;
    double agility = 0.5; // 0 = barge (hates turns/chicanes) .. 1 = nimble
    bool allow_footways = true; // may push/ride slowly on foot infrastructure

    static VehicleProfile
    from_query(const std::map<std::string, std::string>& q);

    // A conventional bicycle, for the "how different is it really?" comparison

    static VehicleProfile bicycle_baseline();

    // Steady-state speed (m/s) on a given gradient with a surface-dependent
    // rolling-resistance multiplier. Solves the power balance
    //   P = v * m * g * (crr_eff + grade)  +  0.5 * rho * CdA * v^3
    // by bisection. On descents the gravity term goes negative and the
    // equilibrium ("terminal") speed can be high; the caller caps it.

    double raw_speed(double grade, double crr_mult) const;

    // Speed on an edge, with the regulation/safety caps applied.
    // surface_speed_factor <= 1 additionally slows rough surfaces
    // (vibration/comfort limit, distinct from rolling resistance).

    double edge_speed(double grade,
                      double crr_mult,
                      double surface_speed_factor) const;
};

inline VehicleProfile
VehicleProfile::from_query(const std::map<std::string, std::string>& q) {
    VehicleProfile p;
    p.width_m = get_num(q, "width", p.width_m);
    p.mass_kg = get_num(q, "mass", p.mass_kg);
    p.power_w = get_num(q, "power", p.power_w);
    p.cda = get_num(q, "cda", p.cda);
    p.crr = get_num(q, "crr", p.crr);
    p.v_cap_flat_kmh = get_num(q, "vflat", p.v_cap_flat_kmh);
    p.v_cap_down_kmh = get_num(q, "vdown", p.v_cap_down_kmh);
    p.allow_car_roads = get_bool(q, "car_roads", p.allow_car_roads);
    p.max_road_speed = std::clamp(
        get_num(q, "max_road_speed", p.max_road_speed), 20.0, 120.0);
    p.avoid_unpaved = get_num(q, "avoid_unpaved", p.avoid_unpaved);
    p.night = get_bool(q, "night", p.night);
    p.scenic = get_num(q, "scenic", p.scenic);
    p.assume_untagged_barrier_passable = get_bool(
        q, "barriers_passable", p.assume_untagged_barrier_passable);
    p.skip_untagged_bollards =
        get_bool(q, "skip_untagged_bollards", p.skip_untagged_bollards);
    p.agility = get_num(q, "agility", p.agility);
    p.allow_footways = get_bool(q, "footways", p.allow_footways);
    // sanity clamps
    p.width_m = std::clamp(p.width_m, 0.4, 2.5);
    p.mass_kg = std::clamp(p.mass_kg, 40.0, 600.0);
    p.power_w = std::clamp(p.power_w, 50.0, 4000.0);
    p.cda = std::clamp(p.cda, 0.05, 1.5);
    p.crr = std::clamp(p.crr, 0.002, 0.05);
    p.v_cap_flat_kmh = std::clamp(p.v_cap_flat_kmh, 6.0, 80.0);
    p.v_cap_down_kmh =
        std::clamp(p.v_cap_down_kmh, p.v_cap_flat_kmh, 100.0);
    p.avoid_unpaved = std::clamp(p.avoid_unpaved, 0.0, 1.0);
    p.scenic = std::clamp(p.scenic, 0.0, 1.0);
    p.agility = std::clamp(p.agility, 0.0, 1.0);
    return p;
}

inline VehicleProfile VehicleProfile::bicycle_baseline() {
    VehicleProfile p;
    p.width_m = 0.6;
    p.mass_kg = 90.0;
    p.power_w = 150.0;
    p.cda = 0.6;
    p.crr = 0.006;
    p.v_cap_flat_kmh = 30.0;
    p.v_cap_down_kmh = 45.0;
    p.allow_car_roads = false;
    p.avoid_unpaved = 0.2;
    p.agility = 1.0; // a bike fits through everything and turns on a dime
    p.assume_untagged_barrier_passable = true;
    return p;
}

inline double VehicleProfile::raw_speed(double grade, double crr_mult) const {
    const double rho = 1.226, g = 9.81;
    double crr_eff = crr * crr_mult;
    auto f = [&](double v) {
        return power_w - v * mass_kg * g * (crr_eff + grade) -
               0.5 * rho * cda * v * v * v;
    };
    double hi = 45.0;
    if (f(hi) >= 0)
        return hi;
    double lo = 0.0;
    for (int i = 0; i < 50; ++i) {
        double mid = 0.5 * (lo + hi);
        (f(mid) > 0 ? lo : hi) = mid;
    }
    return 0.5 * (lo + hi);
}

inline double VehicleProfile::edge_speed(double grade,
                                         double crr_mult,
                                         double surface_speed_factor) const {
    double v = raw_speed(grade, crr_mult);
    double cap_ms =
        (grade < -0.005 ? v_cap_down_kmh : v_cap_flat_kmh) / 3.6;
    v = std::min(v, cap_ms) * surface_speed_factor;
    // Below ~2.2 km/h the rider dismounts and pushes at walking pace.
    const double push = 1.1; // m/s ≈ 4 km/h pushing
    if (v < 0.6)
        v = std::min(push, surface_speed_factor * push);
    return std::max(v, 0.3);
}

}
