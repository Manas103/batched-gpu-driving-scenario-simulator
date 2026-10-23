#pragma once

// A single header turns on when the same source is compiled by nvcc (the GPU
// kernel) and when it is compiled by plain g++ (the single-threaded CPU
// reference). Using the identical function bodies on both sides is the whole
// basis for the GPU/CPU agreement claim: there is only one copy of the IDM,
// MOBIL and bicycle-model math in this repository, not a GPU version and a
// CPU version that could silently drift apart.
#if defined(__CUDACC__)
#define SIM_HD __host__ __device__
#else
#define SIM_HD
#endif
