// Measures environment-steps/sec (scenarios * ticks / wall time) for the
// GPU kernel and for the single-threaded CPU reference, on the same N
// scenarios and tick count, so the two numbers are directly comparable.
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include "sim/cpu_reference.hpp"
#include "sim/kernel_api.hpp"
#include "sim/scenario_gen.hpp"

int main(int argc, char** argv) {
    int n_gpu = argc > 1 ? atoi(argv[1]) : 4096;
    int ticks_gpu = argc > 2 ? atoi(argv[2]) : 2000;
    int n_cpu = argc > 3 ? atoi(argv[3]) : 4096;
    int ticks_cpu = argc > 4 ? atoi(argv[4]) : 20;

    printf("device: %s\n", sim::gpu_device_info().c_str());

    {
        std::vector<sim::ScenarioState> states(n_gpu);
        std::vector<sim::ScenarioParams> params(n_gpu);
        for (int i = 0; i < n_gpu; ++i) sim::make_scenario(1000000ULL + i, states[i], params[i]);

        // One warmup launch so the timed run excludes CUDA context/JIT setup.
        sim::gpu_run_ticks(states.data(), params.data(), n_gpu, 1, sim::kDt);
        for (int i = 0; i < n_gpu; ++i) sim::make_scenario(1000000ULL + i, states[i], params[i]);

        auto t0 = std::chrono::steady_clock::now();
        sim::gpu_run_ticks(states.data(), params.data(), n_gpu, ticks_gpu, sim::kDt);
        auto t1 = std::chrono::steady_clock::now();
        double secs = std::chrono::duration<double>(t1 - t0).count();
        double steps = (double)n_gpu * (double)ticks_gpu;
        printf("gpu: n=%d ticks=%d secs=%.6f env_steps_per_sec=%.1f\n", n_gpu, ticks_gpu, secs, steps / secs);
    }

    {
        std::vector<sim::ScenarioState> states(n_cpu);
        std::vector<sim::ScenarioParams> params(n_cpu);
        for (int i = 0; i < n_cpu; ++i) sim::make_scenario(2000000ULL + i, states[i], params[i]);

        auto t0 = std::chrono::steady_clock::now();
        sim::cpu_run_ticks(states.data(), params.data(), n_cpu, ticks_cpu, sim::kDt);
        auto t1 = std::chrono::steady_clock::now();
        double secs = std::chrono::duration<double>(t1 - t0).count();
        double steps = (double)n_cpu * (double)ticks_cpu;
        printf("cpu: n=%d ticks=%d secs=%.6f env_steps_per_sec=%.1f\n", n_cpu, ticks_cpu, secs, steps / secs);
    }
    return 0;
}
