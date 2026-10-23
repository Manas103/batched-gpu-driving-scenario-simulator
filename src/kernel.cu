#include <cuda_runtime.h>

#include <sstream>
#include <stdexcept>

#include "sim/kernel_api.hpp"
#include "sim/step.hpp"

namespace {

#define CUDA_CHECK(expr)                                                         \
    do {                                                                         \
        cudaError_t _err = (expr);                                               \
        if (_err != cudaSuccess) {                                               \
            std::ostringstream _oss;                                             \
            _oss << "CUDA error at " << __FILE__ << ":" << __LINE__ << ": "      \
                 << cudaGetErrorString(_err);                                    \
            throw std::runtime_error(_oss.str());                                \
        }                                                                        \
    } while (0)

// One thread, one scenario, `ticks` fixed steps looped entirely on the
// device. There is exactly one CUDA kernel in this repository: the IDM,
// MOBIL and bicycle-model math it calls is sim::step_scenario, the same
// function the CPU reference calls in its own serial loop.
__global__ void step_kernel(sim::ScenarioState* states, const sim::ScenarioParams* params,
                             int n, int ticks, double dt) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= n) return;
    sim::ScenarioState s = states[idx];
    sim::ScenarioParams p = params[idx];
    for (int t = 0; t < ticks; ++t) {
        sim::step_scenario(s, p, dt);
    }
    states[idx] = s;
}

} // namespace

namespace sim {

void gpu_run_ticks(ScenarioState* states, const ScenarioParams* params, int n, int ticks, double dt) {
    ScenarioState* d_states = nullptr;
    ScenarioParams* d_params = nullptr;
    CUDA_CHECK(cudaMalloc(&d_states, sizeof(ScenarioState) * (size_t)n));
    CUDA_CHECK(cudaMalloc(&d_params, sizeof(ScenarioParams) * (size_t)n));
    CUDA_CHECK(cudaMemcpy(d_states, states, sizeof(ScenarioState) * (size_t)n, cudaMemcpyHostToDevice));
    CUDA_CHECK(cudaMemcpy(d_params, params, sizeof(ScenarioParams) * (size_t)n, cudaMemcpyHostToDevice));

    int block = 128;
    int grid = (n + block - 1) / block;
    step_kernel<<<grid, block>>>(d_states, d_params, n, ticks, dt);
    CUDA_CHECK(cudaGetLastError());
    CUDA_CHECK(cudaDeviceSynchronize());

    CUDA_CHECK(cudaMemcpy(states, d_states, sizeof(ScenarioState) * (size_t)n, cudaMemcpyDeviceToHost));
    CUDA_CHECK(cudaFree(d_states));
    CUDA_CHECK(cudaFree(d_params));
}

std::string gpu_device_info() {
    int device = 0;
    cudaError_t err = cudaGetDevice(&device);
    if (err != cudaSuccess) return "no CUDA device";
    cudaDeviceProp prop;
    err = cudaGetDeviceProperties(&prop, device);
    if (err != cudaSuccess) return "no CUDA device";
    std::ostringstream oss;
    oss << prop.name << " (sm_" << prop.major << prop.minor << "), "
        << prop.multiProcessorCount << " SMs";
    return oss.str();
}

} // namespace sim
