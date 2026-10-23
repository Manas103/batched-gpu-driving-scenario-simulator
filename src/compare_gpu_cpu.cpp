// Runs the same N scenarios through gpu_run_ticks and cpu_run_ticks for the
// same number of ticks and reports the worst-case relative difference
// across every (x, y, yaw, v) component of every vehicle of every scenario.
// This is the answer to "how do you know the GPU path is the same physics":
// both call sim::step_scenario, and this is the number that proves it.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include "sim/cpu_reference.hpp"
#include "sim/kernel_api.hpp"
#include "sim/scenario_gen.hpp"

static double rel_diff(double a, double b) {
    double denom = std::max(std::fabs(a), std::fabs(b));
    if (denom < 1e-9) return std::fabs(a - b);
    return std::fabs(a - b) / denom;
}

int main(int argc, char** argv) {
    int n = argc > 1 ? atoi(argv[1]) : 4096;
    int ticks = argc > 2 ? atoi(argv[2]) : 600;

    std::vector<sim::ScenarioState> gpu_states(n), cpu_states(n);
    std::vector<sim::ScenarioParams> params(n);
    for (int i = 0; i < n; ++i) {
        sim::make_scenario(3000000ULL + i, gpu_states[i], params[i]);
        cpu_states[i] = gpu_states[i];
    }

    sim::gpu_run_ticks(gpu_states.data(), params.data(), n, ticks, sim::kDt);
    sim::cpu_run_ticks(cpu_states.data(), params.data(), n, ticks, sim::kDt);

    double max_rel = 0.0;
    int worst_scenario = -1, worst_vehicle = -1;
    const char* worst_field = "";
    for (int i = 0; i < n; ++i) {
        for (int v = 0; v < sim::kVehiclesPerScenario; ++v) {
            const sim::VehicleState& g = gpu_states[i].vehicles[v];
            const sim::VehicleState& c = cpu_states[i].vehicles[v];
            struct Field { const char* name; double g, c; };
            Field fields[4] = {{"x", g.x, c.x}, {"y", g.y, c.y}, {"yaw", g.yaw, c.yaw}, {"v", g.v, c.v}};
            for (auto& f : fields) {
                double r = rel_diff(f.g, f.c);
                if (r > max_rel) {
                    max_rel = r;
                    worst_scenario = i;
                    worst_vehicle = v;
                    worst_field = f.name;
                }
            }
        }
    }

    printf("n=%d ticks=%d max_relative_diff=%.3e (scenario=%d vehicle=%d field=%s)\n",
           n, ticks, max_rel, worst_scenario, worst_vehicle, worst_field);
    return 0;
}
