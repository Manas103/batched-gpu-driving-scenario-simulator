cmake_minimum_required(VERSION 3.18)
project(batched_gpu_driving_scenario_simulator CXX CUDA)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_STANDARD 17)
# Real SASS for sm_89 only, no embedded PTX: this machine's driver
# (CUDA UMD 13.3) cannot JIT-compile PTX emitted by a 13.4 nvcc, so a
# virtual-architecture build fails at kernel launch with "the provided PTX
# was compiled with an unsupported toolchain" even though compilation
# succeeds. Emitting sm_89 SASS directly skips that JIT step entirely.
set(CMAKE_CUDA_ARCHITECTURES OFF)
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} --generate-code arch=compute_89,code=sm_89")

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS_RELEASE "-O2")
set(CMAKE_CUDA_FLAGS_RELEASE "-O2")

# ASan/UBSan cover every CXX target (the IDM/MOBIL/bicycle/SHA-256 math and
# the host-side CLI tools); they do not instrument src/kernel.cu, which is
# nvcc-compiled device code and is checked separately with compute-sanitizer
# (see README "Validation"). TSan is not offered: nothing in this repo
# shares mutable state across threads (each GPU thread owns one disjoint
# scenario; the CPU reference is single-threaded), so there is no race to
# find.
option(ENABLE_ASAN "Enable AddressSanitizer + UBSan on CXX targets" OFF)
if(ENABLE_ASAN)
  add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-fsanitize=address,undefined> $<$<COMPILE_LANGUAGE:CXX>:-fno-omit-frame-pointer> $<$<COMPILE_LANGUAGE:CXX>:-g> $<$<COMPILE_LANGUAGE:CXX>:-O1>)
  add_link_options(-fsanitize=address,undefined)
endif()

add_library(sim_cuda src/kernel.cu)
target_include_directories(sim_cuda PUBLIC include)

add_executable(bench_throughput src/bench_throughput.cpp)
target_include_directories(bench_throughput PRIVATE include)
target_link_libraries(bench_throughput PRIVATE sim_cuda)

add_executable(run_session src/run_session.cpp)
target_include_directories(run_session PRIVATE include)
target_link_libraries(run_session PRIVATE sim_cuda)

add_executable(compare_gpu_cpu src/compare_gpu_cpu.cpp)
target_include_directories(compare_gpu_cpu PRIVATE include)
target_link_libraries(compare_gpu_cpu PRIVATE sim_cuda)

enable_testing()

add_executable(check_idm tests/test_idm.cpp)
target_include_directories(check_idm PRIVATE include tests)
add_test(NAME check_idm COMMAND check_idm)

add_executable(check_mobil tests/test_mobil.cpp)
target_include_directories(check_mobil PRIVATE include tests)
add_test(NAME check_mobil COMMAND check_mobil)

add_executable(check_bicycle tests/test_bicycle.cpp)
target_include_directories(check_bicycle PRIVATE include tests)
add_test(NAME check_bicycle COMMAND check_bicycle)

add_executable(check_sha256 tests/test_sha256.cpp)
target_include_directories(check_sha256 PRIVATE include tests)
add_test(NAME check_sha256 COMMAND check_sha256)

add_executable(check_scenario tests/test_scenario.cpp)
target_include_directories(check_scenario PRIVATE include tests)
add_test(NAME check_scenario COMMAND check_scenario)
