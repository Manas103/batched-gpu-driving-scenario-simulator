#pragma once
#include <string>

#include "sim/types.hpp"

// The boundary between CUDA and everything else. Only kernel.cu (compiled
// by nvcc) sees a CUDA type in this repository; every other translation
// unit, including the CLI tools built by plain g++, calls through these
// three plain-C++ functions and never includes <cuda_runtime.h>.
namespace sim {

// Advances all `n` scenarios by `ticks` fixed steps entirely on the GPU: one
// kernel launch, one thread per scenario, no host round trip until the very
// end, when the final state is copied back into `states`. This is the
// function the throughput benchmark and the GPU/CPU agreement check both
// call; only the tick count and what is measured around the call differ.
void gpu_run_ticks(ScenarioState* states, const ScenarioParams* params, int n, int ticks, double dt);

// Name and compute capability of the device gpu_run_ticks will use, for the
// README and benchmark output.
std::string gpu_device_info();

} // namespace sim
