#!/usr/bin/env python3
"""Compare current solver state and timings with an immutable hash-cache commit.

Builds isolated copies; only Solver access specifiers change for introspection.
Requires Linux, Python 3, Git, GCC and SFML development libraries. No NumPy.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import random
import shlex
import shutil
import statistics
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
BASELINE = "7aa9581"
FLAGS = ["-std=c++20", "-O3", "-DNDEBUG", "-flto=auto"]


def run(command, **kwargs):
    return subprocess.run(command, check=True, **kwargs)


def build(destination, baseline, compiler):
    if baseline:
        files = subprocess.check_output(
            ["git", "ls-tree", "-r", "--name-only", baseline, "--", "src", "include"],
            cwd=ROOT, text=True).splitlines()
        for name in files:
            target = destination / name
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(subprocess.check_output(
                ["git", "show", f"{baseline}:{name}"], cwd=ROOT))
    else:
        for name in ["src", "include"]:
            shutil.copytree(ROOT / name, destination / name)
    header = destination / "include/solver/solver.hpp"
    header.write_text(header.read_text().replace("private:", "public:"))
    binary = destination / "probe"
    command = [*compiler, *FLAGS, "-I" + str(destination / "include"),
               str(ROOT / "tests/solver_cache_probe.cpp"),
               str(destination / "src/solver.cpp"),
               str(destination / "src/environment.cpp"),
               "-o", str(binary), "-lsfml-graphics", "-lsfml-window", "-lsfml-system"]
    if baseline:
        command.insert(len(compiler), "-DVBM_LEGACY_CACHE")
    run(command)
    return binary


def performance_cases(quick):
    dimensions = [(64, 64), (256, 256), (512, 512), (1024, 1024), (1024, 256)]
    if quick:
        dimensions = [(64, 64), (256, 256)]
    cases = [(kind, w, h, 17) for w, h in dimensions
             for kind in ["empty", "sparse", "dense", "maze", "multi"]]
    if not quick:
        cases += [(kind, 512, 512, seed) for seed in [91, 503]
                  for kind in ["sparse", "dense"]]
    cases += [("noise", n, n, 17) for n in ([64] if quick else [64, 256, 512])]
    if not quick:
        cases += [("image:" + str(ROOT / "images" / (name + ".png")), w, h, 17)
                  for name, w, h in [("maze_1", 242, 322), ("maze_2", 690, 402),
                                    ("lab_image_edited", 893, 646),
                                    ("huge_maze", 800, 800)]]
    return [(*case, "vbm", 0.5, 0, 1, 1) for case in cases]


def compatibility_cases():
    cases = [("empty", w, h, 17, "vbm", 0.5, 0, 1, 1)
             for w, h in [(1, 1), (1, 64), (64, 1), (128, 33)]]
    cases += [("diagonal", 2, 2, 17, "vbm", 0.5, 0, 1, 1)]
    cases += [(kind, 40, 28, 91, method, 0.5, 0, 1, greedy)
              for kind in ["empty", "single", "dense", "noise"]
              for method, greedy in [("vstar", 1), ("vstar", 0), ("astar", 1),
                                     ("astar", 0), ("distance", 1), ("sequence", 1)]]
    cases += [("dense", 40, 28, 503, "vbm", threshold, expand, 5, 1)
              for threshold in [0, 0.25, 0.5, 1] for expand in [0, 1]]
    return cases


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", default=BASELINE)
    parser.add_argument("--rounds", type=int, default=3)
    parser.add_argument("--repeats", type=int, default=7)
    parser.add_argument("--quick", action="store_true")
    parser.add_argument("--output", type=Path, default=ROOT / "build/cache-comparison")
    args = parser.parse_args()
    if args.rounds < 1 or args.repeats < 1:
        parser.error("rounds and repeats must be positive")
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    baseline = subprocess.check_output(
        ["git", "rev-parse", "--verify", args.baseline + "^{commit}"],
        cwd=ROOT, text=True).strip()
    compiler = shlex.split(os.environ.get("CXX", "g++"))
    cpu = min(os.sched_getaffinity(0))
    performance = performance_cases(args.quick)
    compatibility = compatibility_cases()
    cases = performance + compatibility
    rows = []
    parity = []
    metadata = {"baseline": baseline, "compiler": subprocess.check_output(
        [*compiler, "--version"], text=True).splitlines()[0], "flags": FLAGS,
        "cpu": cpu, "rounds": args.rounds, "repeats": args.repeats,
        "performance_cases": len(performance), "compatibility_cases": len(compatibility),
        "current_source_sha256": {str(f.relative_to(ROOT)): hashlib.sha256(f.read_bytes()).hexdigest()
                                  for folder in ["src", "include", "tests"]
                                  for f in sorted((ROOT / folder).rglob("*"))
                                  if f.is_file() and f.suffix in [".cpp", ".hpp", ".py"]}}
    (output / "metadata.json").write_text(json.dumps(metadata, indent=2) + "\n")
    with tempfile.TemporaryDirectory(prefix="vbm-cache-compare-") as directory:
        scratch = Path(directory)
        binaries = {"baseline": build(scratch / "baseline", baseline, compiler),
                    "compact": build(scratch / "compact", None, compiler)}
        schedule = [(r, i) for i in range(len(performance)) for r in range(args.rounds)]
        schedule += [(0, i) for i in range(len(performance), len(cases))]
        random.Random(4863).shuffle(schedule)
        with (output / "runs.jsonl").open("w") as log:
            for step, (round_id, index) in enumerate(schedule):
                case = cases[index]
                kind, w, h, seed, method, threshold, expand, speed, greedy = case
                repeats = args.repeats if index < len(performance) else 2
                order = ["baseline", "compact"]
                random.Random(710 + step).shuffle(order)
                states = {}
                counts = {}
                for variant in order:
                    state = scratch / (variant + ".bin")
                    command = ["taskset", "-c", str(cpu), str(binaries[variant]),
                               kind, str(w), str(h), str(seed), str(repeats), str(state),
                               method, str(threshold), str(expand), str(speed), str(greedy)]
                    result = run(command, capture_output=True, text=True, timeout=120)
                    row = json.loads(result.stdout)
                    row.update(case_id=index, case=case, round=round_id, variant=variant)
                    log.write(json.dumps(row) + "\n")
                    log.flush()
                    rows.append(row)
                    states[variant] = state.read_bytes()
                    counts[variant] = row["cache_entries"]
                equal = states["baseline"] == states["compact"]
                equal_counts = counts["baseline"] == counts["compact"]
                parity.append({"case_id": index, "round": round_id, "equal": equal,
                               "equal_cache_entries": equal_counts,
                               "bytes": len(states["baseline"])})
                if not equal or not equal_counts:
                    for variant, state in states.items():
                        (output / f"failure-{index}-{variant}.bin").write_bytes(state)
                    raise RuntimeError(f"Solver parity failed for {case}")
                if (step + 1) % 12 == 0:
                    print(f"Compared {step + 1}/{len(schedule)} paired runs", flush=True)
    (output / "parity.json").write_text(json.dumps(parity, indent=2) + "\n")
    comparisons = []
    for index, case in enumerate(performance):
        matched = [row for row in rows if row["case_id"] == index]
        def ratio(field):
            values = []
            for round_id in range(args.rounds):
                pair = {row["variant"]: row for row in matched if row["round"] == round_id}
                a, b = pair["baseline"][field], pair["compact"][field]
                values.append((statistics.median(b) / statistics.median(a))
                              if isinstance(a, list) else b / a)
            return statistics.median(values)
        comparisons.append({"case": case, "warm_ratio": ratio("times_ms"),
                            "first_solve_ratio": ratio("cold_ms"),
                            "rss_ratio": ratio("rss_kib")})
    summary = {"all_parity_passed": True, "distinct_cases": len(cases),
               "paired_runs": len(parity), "performance_cases": len(performance),
               "median_warm_ratio": statistics.median(r["warm_ratio"] for r in comparisons),
               "worst_warm_ratio": max(r["warm_ratio"] for r in comparisons),
               "median_first_solve_ratio": statistics.median(r["first_solve_ratio"] for r in comparisons),
               "cases": comparisons}
    (output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps({k: v for k, v in summary.items() if k != "cases"}, indent=2))


if __name__ == "__main__":
    main()
