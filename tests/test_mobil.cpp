#include "check.hpp"
#include "sim/mobil.hpp"

int main() {
    sim::MobilParams p;
    p.politeness = 0.2;
    p.change_threshold = 0.2;
    p.safe_decel = 3.0;

    // A clear personal gain with no cost to the new follower clears the
    // threshold.
    CHECK(sim::mobil_incentive(p, 0.0, 1.0, 0.0, 0.0));

    // A gain smaller than the threshold does not.
    CHECK(!sim::mobil_incentive(p, 0.0, 0.1, 0.0, 0.0));

    // A gain that only clears the threshold before politeness is applied,
    // but not after, must be rejected: gain=0.5, follower loses 2.0 m/s^2,
    // net = 0.5 + 0.2*(-2.0) = 0.1 < 0.2.
    CHECK(!sim::mobil_incentive(p, 0.0, 0.5, 0.0, -2.0));

    // Safety: a new follower left above -safe_decel is fine; at or below it
    // is not.
    CHECK(sim::mobil_safe(p, -2.9));
    CHECK(!sim::mobil_safe(p, -3.1));

    CHECK_SUITE_END();
}
