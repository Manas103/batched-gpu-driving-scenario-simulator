#pragma once
#include <cmath>
#include <cstdint>

#include "sim/ring.hpp"
#include "sim/types.hpp"

namespace sim {

// Counter-based deterministic PRNG: a SplitMix64 hash of (seed, stream),
// turned into a uniform double in [0, 1). No RNG state is carried between
// calls, so scenario N's initial condition depends only on the manifest
// seed and N, never on generation order. This is what makes "replay from a
// seed manifest" meaningful: the manifest is the seed list, nothing else.
inline uint64_t splitmix64(uint64_t x) {
    x += 0x9E3779B97f4A7C15ULL;
    uint64_t z = x;
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
    return z ^ (z >> 31);
}

inline double uniform01(uint64_t seed, uint64_t stream) {
    uint64_t h = splitmix64(seed ^ (stream * 0xD1B54A32D192ED03ULL));
    return (double)(h >> 11) * (1.0 / 9007199254740992.0); // 53-bit mantissa
}

// Builds one scenario's initial state and per-vehicle parameters from a
// single 64-bit seed. Vehicles start evenly spread around the ring (so a
// fresh scenario never starts with two vehicles overlapping), each assigned
// a starting lane round-robin, with a small per-vehicle jitter on desired
// speed (v0) and starting speed so the 8 vehicles in a scenario are not
// identical copies of one driver.
inline void make_scenario(uint64_t seed, ScenarioState& state, ScenarioParams& params) {
    const double two_pi = 6.283185307179586476925286766559;
    for (int i = 0; i < kVehiclesPerScenario; ++i) {
        int lane = i % kLanes;
        double theta = two_pi * ((double)i / (double)kVehiclesPerScenario) +
                        0.05 * (uniform01(seed, 100 + i) - 0.5);
        double r = lane_radius(lane);
        VehicleState& v = state.vehicles[i];
        v.x = r * cos(theta);
        v.y = r * sin(theta);
        v.yaw = wrap_pi(theta + 1.5707963267948966);
        double v0_jitter = 2.0 * (uniform01(seed, 200 + i) - 0.5); // +/- 1 m/s
        v.v = 14.0 + v0_jitter;
        v.target_lane = lane;
        v.since_change = kMinChangeInterval; // free to change from tick 0

        IdmParams idm;
        idm.v0 = 20.0 + 3.0 * (uniform01(seed, 300 + i) - 0.5);
        params.vehicles[i].idm = idm;
        params.vehicles[i].length = kVehicleLength;
    }
    params.mobil = MobilParams{};
}

} // namespace sim
