#!/usr/bin/env python3
"""Generates one seed manifest and runs run_session against it `--sessions`
times, checking that every run produces the identical SHA-256 digest. This
is the replay-determinism measurement reported in the README.

Usage: python tools/run_replay_digests.py <build_dir> [--n N] [--ticks T] [--sessions S]
"""
import argparse
import subprocess
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from make_manifest import splitmix64  # noqa: E402


def write_manifest(path: Path, master: int, n: int) -> None:
    state = master
    seeds = []
    for i in range(n):
        state = splitmix64(state ^ (i * 0xD1B54A32D192ED03 & ((1 << 64) - 1)))
        seeds.append(state)
    with open(path, "w") as f:
        f.write(f"{n}\n")
        for s in seeds:
            f.write(f"{s}\n")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("build_dir")
    ap.add_argument("--n", type=int, default=4096)
    ap.add_argument("--ticks", type=int, default=300)
    ap.add_argument("--sessions", type=int, default=100)
    ap.add_argument("--master-seed", type=int, default=777)
    args = ap.parse_args()

    run_session = Path(args.build_dir) / "run_session"
    if not run_session.exists():
        run_session = Path(args.build_dir) / "run_session.exe"
    if not run_session.exists():
        print(f"run_session binary not found under {args.build_dir}", file=sys.stderr)
        sys.exit(1)

    with tempfile.TemporaryDirectory() as td:
        manifest_path = Path(td) / "session.manifest"
        write_manifest(manifest_path, args.master_seed, args.n)

        digests = []
        for i in range(args.sessions):
            out = subprocess.run(
                [str(run_session), str(manifest_path), str(args.ticks)],
                capture_output=True, text=True, check=True,
            )
            digests.append(out.stdout.strip())

        unique = set(digests)
        matches = sum(1 for d in digests if d == digests[0])
        print(f"sessions={args.sessions} n_scenarios={args.n} ticks={args.ticks}")
        print(f"unique digests: {len(unique)}")
        print(f"digest: {digests[0]}")
        print(f"{matches} of {args.sessions} sessions matched the first session's digest")
        if len(unique) != 1:
            sys.exit(1)


if __name__ == "__main__":
    main()
