#pragma once
#include <cstdint>
#include <fstream>
#include <stdexcept>
#include <vector>

namespace sim {

// Plain-text manifest: first line is the scenario count, one seed per line
// after that. A manifest is the only input a replay session needs; the
// same file read twice must produce the same list of seeds, which is the
// whole determinism claim resting on splitmix64 having no hidden state.
inline std::vector<uint64_t> read_manifest(const std::string& path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("cannot open manifest: " + path);
    size_t n;
    in >> n;
    std::vector<uint64_t> seeds(n);
    for (size_t i = 0; i < n; ++i) {
        if (!(in >> seeds[i])) throw std::runtime_error("truncated manifest: " + path);
    }
    return seeds;
}

} // namespace sim
