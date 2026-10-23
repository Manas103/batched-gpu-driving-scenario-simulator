#include "check.hpp"
#include "sim/bicycle.hpp"

int main() {
    // Straight line, zero steer, zero accel: x advances by v*dt, y and yaw
    // and v are unchanged (RK4 of a constant-velocity straight line is
    // exact, so the tolerance here is tight).
    {
        double x = 0, y = 0, yaw = 0, v = 10.0;
        sim::bicycle_rk4_step(x, y, yaw, v, 0.0, 0.0, 0.05);
        CHECK_NEAR(x, 0.5, 1e-9);
        CHECK_NEAR(y, 0.0, 1e-9);
        CHECK_NEAR(yaw, 0.0, 1e-9);
        CHECK_NEAR(v, 10.0, 1e-9);
    }

    // Constant acceleration for one step should match v0 + a*dt.
    {
        double x = 0, y = 0, yaw = 0, v = 5.0;
        sim::bicycle_rk4_step(x, y, yaw, v, 2.0, 0.0, 0.1);
        CHECK_NEAR(v, 5.2, 1e-9);
    }

    // A fixed nonzero steer at constant speed should turn at yaw_rate =
    // v*tan(steer)/L, matching the analytic rate over a short step.
    {
        double L = sim::kWheelbase;
        double steer = 0.1;
        double v0 = 10.0;
        double x = 0, y = 0, yaw = 0, v = v0;
        double dt = 0.001; // small enough that RK4 error is negligible
        sim::bicycle_rk4_step(x, y, yaw, v, 0.0, steer, dt);
        double expected_yaw_rate = v0 * tan(steer) / L;
        CHECK_NEAR(yaw / dt, expected_yaw_rate, 1e-4);
    }

    // Braking to a stop must not reverse: a large negative acceleration
    // held for long enough has to clamp v at exactly zero.
    {
        double x = 0, y = 0, yaw = 0, v = 1.0;
        for (int i = 0; i < 1000; ++i) {
            sim::bicycle_rk4_step(x, y, yaw, v, -5.0, 0.0, 0.05);
        }
        CHECK(v == 0.0);
    }

    CHECK_SUITE_END();
}
