#pragma once
#include "sim/step.hpp"
#include "sim/types.hpp"

namespace sim {

// Single-threaded CPU reference: one scenario fully stepped before moving
// to the next. It calls the exact same step_scenario as the GPU kernel; the
// only difference between this function and gpu_run_ticks is which
// processor runs the loop.
inline void cpu_run_ticks(ScenarioState* states, const ScenarioParams* params, int n, int ticks, double dt) {
    for (int i = 0; i < n; ++i) {
        for (int t = 0; t < ticks; ++t) {
            step_scenario(states[i], params[i], dt);
        }
    }
}

} // namespace sim
