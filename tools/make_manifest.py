#!/usr/bin/env python3
"""Writes a plain-text seed manifest: N lines of distinct 64-bit seeds
derived from one master seed, in the same splitmix64 family the C++ side
uses for per-scenario scenario generation. The manifest file is the only
thing a replay session reads; regenerate it with:
  python tools/make_manifest.py <master_seed> <n> <out_path>
"""
import sys

MASK64 = (1 << 64) - 1


def splitmix64(x: int) -> int:
    x = (x + 0x9E3779B97F4A7C15) & MASK64
    z = x
    z = ((z ^ (z >> 30)) * 0xBF58476D1CE4E5B9) & MASK64
    z = ((z ^ (z >> 27)) * 0x94D049BB133111EB) & MASK64
    z = z ^ (z >> 31)
    return z


def main():
    if len(sys.argv) != 4:
        print(__doc__)
        sys.exit(1)
    master = int(sys.argv[1])
    n = int(sys.argv[2])
    out_path = sys.argv[3]
    seeds = []
    state = master
    for i in range(n):
        state = splitmix64(state ^ (i * 0xD1B54A32D192ED03 & MASK64))
        seeds.append(state)
    with open(out_path, "w") as f:
        f.write(f"{n}\n")
        for s in seeds:
            f.write(f"{s}\n")
    print(f"wrote {n} seeds to {out_path}")


if __name__ == "__main__":
    main()
