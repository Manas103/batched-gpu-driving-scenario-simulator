#pragma once
#include "sim/portable.hpp"
#include "sim/types.hpp"

namespace sim {

// MOBIL (Kesting, Treiber & Helbing 2007), split into the two pure
// comparisons the model makes once the four accelerations (own current,
// own hypothetical, candidate-lane follower before and after) have been
// evaluated with idm_accel. Finding those accelerations requires walking
// the scenario's other vehicles, which is scenario-level logic (see
// step.hpp); these two functions are the part that is just arithmetic on
// four numbers, which is what makes them unit-testable in isolation.

// True when switching lanes is worth it: the ego vehicle's own gain, minus
// politeness times the new follower's loss, clears the hysteresis
// threshold. A negative `a_new_follower_after - a_new_follower_before`
// (the follower is worse off) subtracts from the incentive.
SIM_HD inline bool mobil_incentive(const MobilParams& p, double a_old, double a_new,
                                    double a_new_follower_before, double a_new_follower_after) {
    double gain = (a_new - a_old) + p.politeness * (a_new_follower_after - a_new_follower_before);
    return gain > p.change_threshold;
}

// True when the move does not force the new follower to brake harder than
// the model's safety bound. This is checked independently of the
// incentive: a lane change can be incentivized and still unsafe.
SIM_HD inline bool mobil_safe(const MobilParams& p, double a_new_follower_after) {
    return a_new_follower_after > -p.safe_decel;
}

} // namespace sim
