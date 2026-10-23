#pragma once
#include <cmath>

#include "sim/portable.hpp"
#include "sim/types.hpp"

namespace sim {

// Intelligent Driver Model longitudinal acceleration. `gap` is the net
// bumper-to-bumper distance to the leader (already length-corrected,
// clamped to a small positive floor so a momentarily negative gap from a
// just-completed lane change cannot divide by zero); `dv` is own speed
// minus leader speed (positive = closing). `has_leader` false means free
// road: the interaction term drops out entirely.
SIM_HD inline double idm_accel(const IdmParams& p, double v, double gap, double dv, bool has_leader) {
    double free_term = p.a_max * (1.0 - pow(v / p.v0, p.delta));
    if (!has_leader) {
        return free_term;
    }
    double s_star = p.s0 + v * p.T + (v * dv) / (2.0 * sqrt(p.a_max * p.b));
    if (s_star < p.s0) s_star = p.s0;
    double g = gap;
    if (g < 0.1) g = 0.1;
    double interaction = (s_star / g);
    interaction = interaction * interaction;
    return p.a_max * (1.0 - pow(v / p.v0, p.delta) - interaction);
}

} // namespace sim
