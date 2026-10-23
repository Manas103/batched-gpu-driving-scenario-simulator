#pragma once
#include <cstdint>

#include "sim/portable.hpp"

namespace sim {

// Fixed per-run shape. Changing these changes the binary layout of a trace
// file, which is why they are compile-time constants rather than CLI flags:
// a seed manifest and a trace file always agree on how to read each other.
constexpr int kVehiclesPerScenario = 8;
constexpr int kLanes = 3;
constexpr double kLaneWidth = 3.6;      // meters
constexpr double kBaseRadius = 160.0;   // meters, innermost lane centerline
constexpr double kVehicleLength = 4.5;  // meters
constexpr double kWheelbase = 2.7;      // meters
constexpr double kMaxSteer = 0.5;       // radians
constexpr double kDt = 1.0 / 20.0;      // seconds, fixed 20Hz step
constexpr int kMinChangeInterval = 20;  // ticks (1s at 20Hz); MOBIL hysteresis

// IDM parameters (Treiber, Hennecke & Helbing 2000 ring-road values, lightly
// adapted). This is "IDM-inspired", not a literal reproduction of the
// published calibration: v0 and a few others are per-vehicle, not global.
struct IdmParams {
    double v0 = 20.0;     // desired speed, m/s
    double T = 1.5;       // desired time headway, s
    double a_max = 1.0;   // max acceleration, m/s^2
    double b = 1.5;       // comfortable deceleration, m/s^2
    double s0 = 2.0;      // minimum gap, m
    double delta = 4.0;   // acceleration exponent
};

// MOBIL parameters (Kesting, Treiber & Helbing 2007).
struct MobilParams {
    double politeness = 0.2;
    double change_threshold = 0.2;  // m/s^2, minimum net advantage to switch
    double safe_decel = 3.0;        // m/s^2, max deceleration imposed on the new follower
};

// Bicycle-model kinematic state about the rear axle: x,y in meters, yaw in
// radians, v in m/s. This is the only state that actually gets integrated;
// everything else (lane, arc length) is read back out of it each tick.
struct VehicleState {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double v = 0.0;
    int target_lane = 0;   // the lane MOBIL last committed to
    int since_change = 0;  // ticks since the last lane change (MOBIL hysteresis)
};

struct VehicleParams {
    IdmParams idm;
    double length = kVehicleLength;
};

// One scenario: kVehiclesPerScenario vehicles on a 3-lane ring road.
struct ScenarioState {
    VehicleState vehicles[kVehiclesPerScenario];
};

struct ScenarioParams {
    VehicleParams vehicles[kVehiclesPerScenario];
    MobilParams mobil;
};

} // namespace sim
