#include "check.hpp"
#include "sim/idm.hpp"

int main() {
    sim::IdmParams p;
    p.v0 = 20.0; p.T = 1.5; p.a_max = 1.0; p.b = 1.5; p.s0 = 2.0; p.delta = 4.0;

    // Free road, stationary: full max acceleration.
    CHECK_NEAR(sim::idm_accel(p, 0.0, 0.0, 0.0, false), p.a_max, 1e-9);

    // Free road, at desired speed: zero acceleration.
    CHECK_NEAR(sim::idm_accel(p, 20.0, 0.0, 0.0, false), 0.0, 1e-9);

    // A huge gap with no closing speed should read almost like free road.
    double free_term = p.a_max * (1.0 - pow(10.0 / p.v0, p.delta));
    double with_far_leader = sim::idm_accel(p, 10.0, 100000.0, 0.0, true);
    CHECK_NEAR(with_far_leader, free_term, 1e-3);

    // A near-zero gap must brake hard, well past comfortable deceleration.
    double with_near_leader = sim::idm_accel(p, 10.0, 0.5, 2.0, true);
    CHECK(with_near_leader < -5.0);

    // Closing on a slower leader at a moderate gap should decelerate, not
    // accelerate, relative to the far-leader case above.
    double with_closing_leader = sim::idm_accel(p, 15.0, 20.0, 5.0, true);
    CHECK(with_closing_leader < free_term);

    CHECK_SUITE_END();
}
