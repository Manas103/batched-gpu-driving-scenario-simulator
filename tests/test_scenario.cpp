#include <cmath>

#include "check.hpp"
#include "sim/cpu_reference.hpp"
#include "sim/ring.hpp"
#include "sim/scenario_gen.hpp"

int main() {
    // Same seed, built twice, must be byte-identical: no hidden RNG state.
    {
        sim::ScenarioState s1, s2;
        sim::ScenarioParams p1, p2;
        sim::make_scenario(42, s1, p1);
        sim::make_scenario(42, s2, p2);
        for (int i = 0; i < sim::kVehiclesPerScenario; ++i) {
            CHECK(s1.vehicles[i].x == s2.vehicles[i].x);
            CHECK(s1.vehicles[i].y == s2.vehicles[i].y);
            CHECK(p1.vehicles[i].idm.v0 == p2.vehicles[i].idm.v0);
        }
    }

    // Different seeds must not produce the same scenario (sanity check that
    // the seed is actually used, not ignored).
    {
        sim::ScenarioState s1, s2;
        sim::ScenarioParams p1, p2;
        sim::make_scenario(1, s1, p1);
        sim::make_scenario(2, s2, p2);
        bool any_diff = false;
        for (int i = 0; i < sim::kVehiclesPerScenario; ++i) {
            if (p1.vehicles[i].idm.v0 != p2.vehicles[i].idm.v0) any_diff = true;
        }
        CHECK(any_diff);
    }

    // Running 600 ticks on the CPU reference must never produce a NaN or an
    // escaped vehicle (radius wildly outside the three lanes), which would
    // indicate the steering controller or IDM/MOBIL interaction diverged.
    {
        const int n = 50;
        sim::ScenarioState states[n];
        sim::ScenarioParams params[n];
        for (int i = 0; i < n; ++i) sim::make_scenario(9000 + i, states[i], params[i]);
        sim::cpu_run_ticks(states, params, n, 600, sim::kDt);
        for (int i = 0; i < n; ++i) {
            for (int v = 0; v < sim::kVehiclesPerScenario; ++v) {
                const sim::VehicleState& vs = states[i].vehicles[v];
                CHECK(std::isfinite(vs.x) && std::isfinite(vs.y) && std::isfinite(vs.yaw) && std::isfinite(vs.v));
                double r = sim::radius_of(vs.x, vs.y);
                CHECK(r > sim::kBaseRadius - 5.0 && r < sim::lane_radius(sim::kLanes - 1) + 5.0);
                CHECK(vs.v >= 0.0);
            }
        }
    }

    CHECK_SUITE_END();
}
