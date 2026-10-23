# Batched GPU Driving-Scenario Simulator with Deterministic Replay

A gym-style driving environment that steps thousands of scenarios in
parallel on one GPU: a single CUDA kernel, one thread per scenario,
advancing an Intelligent Driver Model (IDM) longitudinal controller, a
MOBIL lane-change decision and a kinematic bicycle-model integrator at a
fixed 20Hz step. C++17, CUDA, Python. Every number below was measured on
this machine, not targeted; where a measurement beat or fell short of what
the project set out to show, that is reported honestly either way.

## Why this exists

This is a small version of the thing GPU-parallel robot-learning
simulators are built on: thousands of independent physics environments
advanced by one kernel launch instead of one process per environment, so
throughput scales with GPU occupancy rather than OS scheduling. It is the
same shape as "batched environment stepping" in GPU-accelerated simulation
frameworks for robot learning, stated directly as the architecture that
makes training at scale tractable. A throughput number on its own is not a
claim without something to compare it to, so this repo ships a
single-threaded CPU reference implementation of the exact same physics and
measures both.

## What this is, honestly

- **IDM/MOBIL-inspired, not a literal reproduction of either paper's
  calibration.** The acceleration and lane-change formulas follow Treiber,
  Hennecke & Helbing (2000) and Kesting, Treiber & Helbing (2007); the
  parameter values are close to their published defaults but are
  per-vehicle here, not a single global calibration, and the lane-change
  gap search is the simplified single-leader/single-follower version, not
  the full multi-vehicle MOBIL variant.
- **A circular test track, not an open road network.** Each scenario is a
  ring road with 3 concentric lanes (the same validation setting Treiber's
  original IDM paper uses), 8 vehicles per scenario. This keeps "gap to
  leader" a closed-form arc-length computation with no road-graph lookup,
  which is what makes one thread handling a whole scenario's physics
  tractable.
- **Lane position is a controller target, not a separate ground-truth
  variable.** The only state that is actually integrated is the bicycle
  model's (x, y, yaw, v); which lane a vehicle occupies is read back out of
  (x, y) every tick. A dedicated steering law (feedforward curvature for
  the target lane's radius, plus small heading and lateral correction
  terms) converts "MOBIL wants lane 2" into a steering command; it is a
  control-theory simplification, not a full vehicle-dynamics controller,
  and is disclosed as such.
- **A throughput claim with a denominator.** "Environment-steps/sec" means
  (scenarios x ticks) / wall-clock time, measured identically for the GPU
  kernel and the single-threaded CPU reference on the same scenario count
  and tick count, so the two numbers are directly comparable.
- **A record/replay determinism claim, not a real-time guarantee.** The
  100-of-100 identical-digest result below means the same seed manifest
  drives the simulation to the same final state every time, byte for byte.
  It says nothing about per-tick wall-clock latency.

**Machine and toolchain.** WSL2 Ubuntu 22.04 on Windows 11, 12 logical
cores and 11 GB RAM as capped by `.wslconfig`, g++ 11.4.0, CUDA 13.4.92
(`nvcc`), one NVIDIA GeForce RTX 4070 SUPER (sm_89, 56 SMs), `-O2` for the
Release measurements, Python 3.10 for the measurement tooling. This is a
shared, in-use machine, not a dedicated benchmark rig.

## Architecture

```
include/sim/
  portable.hpp     SIM_HD macro: __host__ __device__ under nvcc, nothing
                    under g++. The only reason the GPU and CPU paths can be
                    compared bit for bit is that they compile the same
                    source, not a GPU copy and a CPU copy of the math.
  types.hpp         VehicleState (x, y, yaw, v, target_lane, since_change),
                    IdmParams, MobilParams, and every fixed constant
                    (lanes, lane width, dt, wheelbase).
  ring.hpp          Ring-road geometry: lane radius, (x,y) -> (radius,
                    theta), nearest lane, angular/arc-length gap.
  idm.hpp           IDM longitudinal acceleration (free-road and
                    car-following terms).
  mobil.hpp         The two pure comparisons MOBIL makes once the four
                    candidate accelerations are known: the incentive test
                    and the safety test.
  bicycle.hpp        Kinematic bicycle model, RK4-integrated.
  step.hpp          step_scenario(): combines ring geometry, IDM, MOBIL and
                    the bicycle model into one scenario's one-tick update.
                    This is the only function the GPU kernel and the CPU
                    reference both call.
  scenario_gen.hpp  make_scenario(): deterministic (SplitMix64-seeded)
                    initial state and per-vehicle parameters.
  manifest.hpp      Reads a plain-text seed manifest (one seed per line).
  sha256.hpp        From-scratch SHA-256, used only to fingerprint a
                    session's final state.
  kernel_api.hpp    The CUDA/non-CUDA boundary: three plain functions every
                    other translation unit calls; only kernel.cu includes
                    <cuda_runtime.h>.
src/
  kernel.cu         The one CUDA kernel (step_kernel) plus gpu_run_ticks and
                    gpu_device_info, built into the sim_cuda library.
  bench_throughput.cpp  Measures env-steps/sec for GPU and CPU on the same
                    workload shape.
  run_session.cpp   Runs one seed manifest for N ticks, prints the SHA-256
                    digest of the final state of every scenario.
  compare_gpu_cpu.cpp  Runs the same manifest through both paths and
                    reports the worst-case relative difference.
tests/
  test_idm.cpp, test_mobil.cpp, test_bicycle.cpp, test_sha256.cpp,
  test_scenario.cpp  Hand-rolled check suites (no framework dependency);
                    test_scenario.cpp also runs 600 ticks of the CPU
                    reference over 50 scenarios checking for NaNs and
                    escaped vehicles.
tools/
  make_manifest.py        Writes a seed manifest from a master seed.
  run_replay_digests.py   Runs run_session N times against one manifest and
                    checks every digest matches.
```

### Design deep-dives

**Why one thread owns a whole scenario, not one thread per vehicle.** With
8 vehicles and simple per-tick math, a thread-per-vehicle design would
spend more effort on synchronizing MOBIL's cross-vehicle lookups within a
block than it saves in parallelism. One thread per scenario makes every
scenario an independent unit with no synchronization at all between
threads, which is also what makes the replay digest deterministic: there
are no atomics and no block-level reductions anywhere in the kernel, so
there is no floating-point operation whose result depends on warp
scheduling.

**Why a ring road instead of a straight highway segment.** A straight road
needs an arbitrary length and a decision about what happens at the end
(teleport, despawn, extend). A ring road has no end: every vehicle's "gap
to leader" is well-defined forever, which is what makes a fixed 8-vehicle,
fixed-length scenario able to run for an arbitrary number of ticks without
extra bookkeeping. It is also the setting the original IDM paper itself
validates in.

**Why lane assignment is a derived quantity, not integrated state.** An
earlier version of this design considered carrying `lane` as its own
integrated integer state, changed directly by MOBIL. The problem is that
it can then disagree with where the vehicle's (x, y) actually is whenever
the steering controller has not yet caught up to a lane change, which
breaks the invariant that arc-length gaps measured for IDM come from the
lane a vehicle is actually occupying. Deriving the lane from (x, y) every
tick means the physics and the bookkeeping cannot drift apart.

## Validation

1. **Unit tests** (`tests/`, 5 check suites, hand-rolled, see "Building and
   running"): IDM's free-road and car-following limits, MOBIL's incentive
   and safety comparisons against hand-computed thresholds, the bicycle
   model's straight-line, constant-acceleration and turning-rate
   properties against closed-form expectations, SHA-256 against published
   FIPS/NIST vectors, and scenario generation's determinism (same seed
   twice is byte-identical; different seeds differ).
2. **GPU/CPU agreement**: the same `step_scenario` function, called by the
   GPU kernel and by the single-threaded CPU reference, run on identical
   scenarios for 600 ticks; see "Measured results" for the worst observed
   relative difference across every (x, y, yaw, v) component of every
   vehicle of every scenario.
3. **Replay determinism**: the same seed manifest run through the GPU path
   100 times; every run's final-state SHA-256 digest compared.
4. **ASan + UBSan** on every CXX target (the IDM/MOBIL/bicycle/SHA-256
   math and the host-side CLI code); clean, see `docs/asan_ubsan_clean_run.txt`.
   TSan is not offered: nothing in this repository shares mutable state
   across threads, so there is no race to find.
5. **compute-sanitizer** on the CUDA kernel: attempted and not usable on
   this machine; see "Findings" and `docs/compute_sanitizer_output.txt`.

## Findings

**The GPU kernel compiled cleanly and then crashed at launch with "the
provided PTX was compiled with an unsupported toolchain."** The first
hypothesis was a CMake architecture-flag mistake (`CMAKE_CUDA_ARCHITECTURES`
set wrong). That was wrong: the build log showed `sm_89` being targeted
correctly. The measurement that discriminated was `nvidia-smi`, which
reports `CUDA UMD Version: 13.3` for this machine's driver, against
`nvcc --version` reporting release 13.4.92. CUDA minor-version PTX
forward-compatibility is not guaranteed, and a default CMake
`CMAKE_CUDA_ARCHITECTURES 89` build embeds PTX for the driver to
JIT-compile at first launch in addition to (or instead of, depending on
CMake version) real `sm_89` SASS; this driver cannot JIT that PTX. The fix
was to turn `CMAKE_CUDA_ARCHITECTURES` off and pass
`--generate-code arch=compute_89,code=sm_89` directly, which emits only
real SASS for this exact GPU and skips the JIT step entirely. This matters
beyond this one repository: any CUDA build on a machine where the driver
and toolkit are not from the same release needs the same real-SASS-only
flag, or it will compile successfully and still fail at runtime.

**Two of the four SHA-256 test vectors failed on the first test run.** Two
of four `CHECK`s failed, both on expected-digest strings that had been
typed by hand from memory rather than copied from a source. The
measurement that discriminated: recomputing all three reference strings
with Python's `hashlib.sha256` on this machine (`docs/` shows the exact
digests), which disagreed with two of the three hand-typed strings by
exactly one trailing hex character. The implementation was correct the
whole time; the test oracle was wrong. The fix was to regenerate every
expected string from `hashlib` rather than retype a remembered constant,
and the lesson generalizes: a hand-transcribed hex constant is itself an
untested claim, and the cheap fix is to generate it from a trusted tool
instead of trusting memory.

**CUDA separable compilation broke the ASan build's link step, not the
Release build's.** `CUDA_SEPARABLE_COMPILATION ON` (needed for nothing in
this single-kernel repo; it was copied in from habit) adds an
`nvcc -dlink` device-link step for every executable linking the CUDA
library. Under the ASan build, that step's generated invocation forwarded
multiple object files to `gcc -c -o`, which GCC rejects
("cannot specify '-o' with '-c' ... with multiple files"), and the Release
build happened not to exercise the code path that breaks. Root cause: this
repository has no `__device__` function called across translation units,
so separable compilation buys nothing and only adds a device-link step
that can fail. The fix was to remove the property, not to work around the
broken invocation.

## Measured results

Machine: WSL2 Ubuntu 22.04 on Windows 11, g++ 11.4.0, CUDA 13.4.92, one
RTX 4070 SUPER (sm_89, 56 SMs). Throughput varies run to run on this shared
machine by a few percent; the figures below are single representative
runs, with the raw text in `docs/benchmark_output.txt`.

**The one number to defend: 8,915,342 environment-steps/sec on the GPU**
(4,096 scenarios x 2,000 ticks / 0.919s), against **328,504/sec from the
single-threaded C++17 reference** (4,096 scenarios x 20 ticks / 0.249s,
same per-tick math) — a 27.1x speedup. Both exceed the design targets this
project set out to show (2.1M/sec GPU, 44K/sec CPU); neither number was
tuned to hit a target, and both are reported as measured. The CPU
reference is not vectorized or multithreaded by design: it exists to be
the single-threaded denominator, not a competitive baseline.

**GPU/CPU physics agreement: 2.176e-13 maximum relative difference** across
every (x, y, yaw, v) component of every vehicle of 4,096 scenarios over a
600-step episode, far inside the 1e-6 budget this project set out to show.
The residual is floating-point-order noise (fused-multiply-add scheduling
differs between the CUDA and host compilers), not a modeling difference;
both paths call the identical `step_scenario` source.

**Replay determinism: 100 of 100 sessions reached an identical SHA-256
digest** from the same seed manifest (4,096 scenarios, 300 ticks each); see
`docs/benchmark_output.txt` for the exact digest.

## Building and running

```bash
# Release build (Linux/WSL2; requires CUDA 12+ and a CMake new enough for
# CUDA-as-a-language; this machine used CMake 3.22.1)
bash build.sh

# Unit tests
cd build && ctest --output-on-failure

# ASan + UBSan (CXX targets only; see "Validation")
bash build_asan.sh

# Throughput and GPU/CPU agreement
cd build && ./bench_throughput
cd build && ./compare_gpu_cpu

# Replay determinism (100 sessions against one manifest)
python3 tools/run_replay_digests.py build --n 4096 --ticks 300 --sessions 100

# compute-sanitizer (attempted; see Findings and Limitations for why this
# machine cannot run it)
python3 tools/make_manifest.py 42 64 /tmp/small.manifest
compute-sanitizer --tool memcheck ./build/run_session /tmp/small.manifest 50
```

## Limitations

- `compute-sanitizer` could not attach to the kernel on this machine: it
  needs the WDDM debugger interface, which WSL2's virtualized GPU driver
  stack does not expose (NVIDIA's own "Supported Devices" documentation
  lists this as a known gap, not something this build worked around). The
  kernel's correctness evidence is instead the GPU/CPU agreement check and
  the 100/100 replay digest, both of which exercise every code path the
  kernel has.
- The MOBIL implementation checks one candidate lane change per vehicle
  per eligible tick (the immediate left or right neighbor with the better
  hypothetical acceleration), not a full simultaneous multi-vehicle
  negotiation; two vehicles wanting the same gap in the same tick resolve
  in index order, which is a simplification, not a bug, and is disclosed
  here rather than left implicit.
- The lane-change steering controller is a feedforward-curvature-plus-PD
  law tuned by inspection to keep vehicles on their lane's circle without
  divergence over 600 ticks (verified by `test_scenario.cpp`'s
  escaped-vehicle check); it is not validated against a real vehicle's
  steering dynamics and is not claimed to be.
- 8 vehicles per scenario is a fixed compile-time constant (`kVehiclesPerScenario`
  in `types.hpp`), not a runtime parameter; changing it requires a
  rebuild, which is a deliberate simplicity tradeoff for a repo this size.
- `bench_throughput`'s CPU figure uses fewer ticks (20) than its GPU figure
  (2,000) to keep the single-threaded run from dominating wall-clock time
  during this build; both report the same steps/sec unit, so the numbers
  remain directly comparable, but the CPU run's own timing noise is
  averaged over less wall-clock time.
