// Runs one full session from a seed manifest and prints the SHA-256 digest
// of the final state of every scenario. Run the same manifest through this
// binary N times (tools/run_replay_digests.py does 100) and every digest
// should be identical: that is the replay-determinism claim.
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "sim/kernel_api.hpp"
#include "sim/manifest.hpp"
#include "sim/scenario_gen.hpp"
#include "sim/sha256.hpp"
#include "sim/cpu_reference.hpp"

int main(int argc, char** argv) {
    if (argc < 3) {
        fprintf(stderr, "usage: run_session <manifest> <ticks> [--cpu]\n");
        return 1;
    }
    std::string manifest_path = argv[1];
    int ticks = atoi(argv[2]);
    bool use_cpu = (argc > 3 && std::string(argv[3]) == "--cpu");

    std::vector<uint64_t> seeds = sim::read_manifest(manifest_path);
    int n = (int)seeds.size();

    std::vector<sim::ScenarioState> states(n);
    std::vector<sim::ScenarioParams> params(n);
    for (int i = 0; i < n; ++i) {
        sim::make_scenario(seeds[i], states[i], params[i]);
    }

    if (use_cpu) {
        sim::cpu_run_ticks(states.data(), params.data(), n, ticks, sim::kDt);
    } else {
        sim::gpu_run_ticks(states.data(), params.data(), n, ticks, sim::kDt);
    }

    sim::Sha256 hasher;
    for (int i = 0; i < n; ++i) {
        hasher.update(&states[i], sizeof(sim::ScenarioState));
    }
    printf("%s\n", hasher.hexdigest().c_str());
    return 0;
}
