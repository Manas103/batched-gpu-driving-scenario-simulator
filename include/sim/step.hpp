#pragma once
#include <cmath>

#include "sim/bicycle.hpp"
#include "sim/idm.hpp"
#include "sim/mobil.hpp"
#include "sim/portable.hpp"
#include "sim/ring.hpp"
#include "sim/types.hpp"

namespace sim {

constexpr double kAccelClampLo = -10.0;
constexpr double kAccelClampHi = 5.0;
constexpr double kSteerHeadingGain = 2.0;
constexpr double kSteerLateralGain = 0.01; // 1/m

SIM_HD inline double clampd(double v, double lo, double hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

// IDM acceleration of a vehicle at (v, theta) in lane `lane`, against
// whichever of the scenario's other vehicles is its nearest leader in that
// lane. Used both for a vehicle's own candidate accelerations and for
// evaluating a MOBIL candidate's new/old follower.
SIM_HD inline double idm_accel_in_lane(const ScenarioState& s, const ScenarioParams& p,
                                        int self_idx, int lane, double v, double theta) {
    double radius = lane_radius(lane);
    double best_gap = 1.0e18;
    int leader = -1;
    for (int j = 0; j < kVehiclesPerScenario; ++j) {
        if (j == self_idx) continue;
        int lane_j = nearest_lane(radius_of(s.vehicles[j].x, s.vehicles[j].y));
        if (lane_j != lane) continue;
        double theta_j = theta_of(s.vehicles[j].x, s.vehicles[j].y);
        double g = arc_gap(radius, theta, theta_j);
        if (g < best_gap) {
            best_gap = g;
            leader = j;
        }
    }
    if (leader < 0) {
        return idm_accel(p.vehicles[self_idx].idm, v, 0.0, 0.0, false);
    }
    double gap = best_gap - p.vehicles[leader].length;
    double dv = v - s.vehicles[leader].v;
    return idm_accel(p.vehicles[self_idx].idm, v, gap, dv, true);
}

// Nearest vehicle behind `theta` in `lane` other than `self_idx` and
// `exclude_idx` (the vehicle hypothetically inserting ahead of it). Returns
// -1 when the lane is empty of other traffic.
SIM_HD inline int find_follower_in_lane(const ScenarioState& s, int self_idx, int exclude_idx,
                                         int lane, double theta) {
    double radius = lane_radius(lane);
    double best_gap = 1.0e18;
    int follower = -1;
    for (int j = 0; j < kVehiclesPerScenario; ++j) {
        if (j == self_idx || j == exclude_idx) continue;
        int lane_j = nearest_lane(radius_of(s.vehicles[j].x, s.vehicles[j].y));
        if (lane_j != lane) continue;
        double theta_j = theta_of(s.vehicles[j].x, s.vehicles[j].y);
        double g = arc_gap(radius, theta_j, theta); // gap from j to theta, i.e. j is behind
        if (g < best_gap) {
            best_gap = g;
            follower = j;
        }
    }
    return follower;
}

// Advances one scenario by one tick. This single function is called once
// per scenario by the CUDA kernel (one GPU thread per scenario) and once
// per scenario, in a serial loop, by the CPU reference; it is the only
// place the IDM, MOBIL and bicycle-model math are combined, so there is no
// way for the two execution paths to compute a different answer.
SIM_HD inline void step_scenario(ScenarioState& s, const ScenarioParams& p, double dt) {
    double theta[kVehiclesPerScenario];
    for (int i = 0; i < kVehiclesPerScenario; ++i) {
        theta[i] = theta_of(s.vehicles[i].x, s.vehicles[i].y);
    }

    double accel[kVehiclesPerScenario];
    for (int i = 0; i < kVehiclesPerScenario; ++i) {
        VehicleState& v = s.vehicles[i];
        accel[i] = idm_accel_in_lane(s, p, i, v.target_lane, v.v, theta[i]);

        if (v.since_change >= kMinChangeInterval) {
            double best_gain_accel = accel[i];
            int best_lane = v.target_lane;
            for (int cand = v.target_lane - 1; cand <= v.target_lane + 1; cand += 2) {
                if (cand < 0 || cand >= kLanes) continue;
                double a_new = idm_accel_in_lane(s, p, i, cand, v.v, theta[i]);

                int follower = find_follower_in_lane(s, i, -1, cand, theta[i]);
                double a_follower_before, a_follower_after;
                if (follower < 0) {
                    a_follower_before = 0.0;
                    a_follower_after = 0.0;
                } else {
                    VehicleState& f = s.vehicles[follower];
                    a_follower_before = idm_accel_in_lane(s, p, follower, cand, f.v, theta[follower]);
                    double radius_cand = lane_radius(cand);
                    double gap = arc_gap(radius_cand, theta[follower], theta[i]) - p.vehicles[i].length;
                    double dv = f.v - v.v;
                    a_follower_after = idm_accel(p.vehicles[follower].idm, f.v, gap, dv, true);
                }

                if (mobil_incentive(p.mobil, accel[i], a_new, a_follower_before, a_follower_after) &&
                    mobil_safe(p.mobil, a_follower_after) && a_new > best_gain_accel) {
                    best_gain_accel = a_new;
                    best_lane = cand;
                }
            }
            if (best_lane != v.target_lane) {
                v.target_lane = best_lane;
                v.since_change = 0;
                accel[i] = idm_accel_in_lane(s, p, i, v.target_lane, v.v, theta[i]);
            }
        }
        v.since_change += 1;
    }

    for (int i = 0; i < kVehiclesPerScenario; ++i) {
        VehicleState& v = s.vehicles[i];
        double r_cur = radius_of(v.x, v.y);
        double r_target = lane_radius(v.target_lane);
        double desired_curvature = 1.0 / r_target - kSteerLateralGain * (r_target - r_cur);
        double tangent_heading = wrap_pi(theta[i] + 1.5707963267948966);
        double heading_error = wrap_pi(tangent_heading - v.yaw);
        double steer = atan(kWheelbase * desired_curvature) + kSteerHeadingGain * heading_error;
        steer = clampd(steer, -kMaxSteer, kMaxSteer);
        double a = clampd(accel[i], kAccelClampLo, kAccelClampHi);
        bicycle_rk4_step(v.x, v.y, v.yaw, v.v, a, steer, dt);
    }
}

} // namespace sim
