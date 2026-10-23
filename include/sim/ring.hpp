#pragma once
#include <cmath>

#include "sim/portable.hpp"
#include "sim/types.hpp"

namespace sim {

// Geometry of the circular test track. Vehicles are physically integrated
// in Cartesian (x, y, yaw, v) by the bicycle model; a lane index and an arc
// length are projections of that Cartesian state onto the ring, recomputed
// every tick rather than carried as separate ground truth, so they can
// never drift out of sync with where the vehicle actually is.

constexpr double kPi = 3.14159265358979323846;

SIM_HD inline double wrap_pi(double a) {
    while (a > kPi) a -= 2.0 * kPi;
    while (a < -kPi) a += 2.0 * kPi;
    return a;
}

SIM_HD inline double lane_radius(int lane) {
    return kBaseRadius + lane * kLaneWidth;
}

SIM_HD inline double radius_of(double x, double y) {
    return sqrt(x * x + y * y);
}

SIM_HD inline double theta_of(double x, double y) {
    return atan2(y, x);
}

// Nearest lane index to a given radius, clamped to the valid range.
SIM_HD inline int nearest_lane(double r) {
    double cont = (r - kBaseRadius) / kLaneWidth;
    int lane = (int)(cont + 0.5);
    if (lane < 0) lane = 0;
    if (lane > kLanes - 1) lane = kLanes - 1;
    return lane;
}

// Signed angular gap from theta_a to theta_b going in the direction of
// increasing theta (counter-clockwise travel), always in [0, 2*pi).
SIM_HD inline double angular_gap(double theta_a, double theta_b) {
    const double two_pi = 6.283185307179586476925286766559;
    double d = theta_b - theta_a;
    while (d < 0.0) d += two_pi;
    while (d >= two_pi) d -= two_pi;
    return d;
}

// Arc-length gap (meters) from vehicle a to vehicle b around a lane of the
// given radius, traveling in the direction of motion.
SIM_HD inline double arc_gap(double radius, double theta_a, double theta_b) {
    return radius * angular_gap(theta_a, theta_b);
}

} // namespace sim
