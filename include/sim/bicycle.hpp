#pragma once
#include <cmath>

#include "sim/portable.hpp"
#include "sim/types.hpp"

namespace sim {

struct BicycleDeriv {
    double dx, dy, dyaw, dv;
};

// x' = v*cos(yaw), y' = v*sin(yaw), yaw' = v*tan(steer)/L, v' = accel.
// Kinematic bicycle about the rear axle; steer and accel are held constant
// across the four RK4 stages (they are the commanded inputs for this tick,
// not functions of the state being integrated).
SIM_HD inline BicycleDeriv bicycle_deriv(double yaw, double v, double accel, double steer) {
    BicycleDeriv d;
    d.dx = v * cos(yaw);
    d.dy = v * sin(yaw);
    d.dyaw = v * tan(steer) / kWheelbase;
    d.dv = accel;
    return d;
}

SIM_HD inline void bicycle_rk4_step(double& x, double& y, double& yaw, double& v,
                                     double accel, double steer, double dt) {
    BicycleDeriv k1 = bicycle_deriv(yaw, v, accel, steer);
    BicycleDeriv k2 = bicycle_deriv(yaw + 0.5 * dt * k1.dyaw, v + 0.5 * dt * k1.dv, accel, steer);
    BicycleDeriv k3 = bicycle_deriv(yaw + 0.5 * dt * k2.dyaw, v + 0.5 * dt * k2.dv, accel, steer);
    BicycleDeriv k4 = bicycle_deriv(yaw + dt * k3.dyaw, v + dt * k3.dv, accel, steer);

    x += (dt / 6.0) * (k1.dx + 2.0 * k2.dx + 2.0 * k3.dx + k4.dx);
    y += (dt / 6.0) * (k1.dy + 2.0 * k2.dy + 2.0 * k3.dy + k4.dy);
    yaw += (dt / 6.0) * (k1.dyaw + 2.0 * k2.dyaw + 2.0 * k3.dyaw + k4.dyaw);
    v += (dt / 6.0) * (k1.dv + 2.0 * k2.dv + 2.0 * k3.dv + k4.dv);
    if (v < 0.0) v = 0.0; // a vehicle braking to a stop does not reverse
}

} // namespace sim
