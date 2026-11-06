#!/usr/bin/env python3
"""Run identical workloads in isolated processes and retain raw measurements."""
import argparse
import csv
import json
import math
import statistics
import subprocess
from pathlib import Path


def percentile(values, percent):
    return sorted(values)[max(0, math.ceil(len(values) * percent / 100) - 1)]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--solver", action="append", required=True, help="label=/absolute/path/to/ik_benchmark")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--performance-samples", type=int, default=20000)
    parser.add_argument("--correctness-samples", type=int, default=200)
    parser.add_argument("--periodic-samples", type=int, default=10000)
    parser.add_argument("--suite", choices=("all", "correctness", "performance", "periodic"), default="all")
    args = parser.parse_args()
    solvers = dict(item.split("=", 1) for item in args.solver)
    args.output.mkdir(parents=True, exist_ok=True)
    summary = []
    workloads = []
    for robot in ("cartesian", "arm6"):
        for mode in range(5):
            workloads.append(("correctness", robot, "smooth", mode, args.correctness_samples, .02, 0))
        for scenario in ("jump", "limits", "singular", "unreachable"):
            if robot == "cartesian" and scenario == "singular":
                continue  # Its pitch limits exclude the Euler singularity.
            workloads.append(("correctness", robot, scenario, 0, args.correctness_samples, .02, 0))
        for scenario in ("smooth", "exact"):
            workloads.append(("performance", robot, scenario, 0, args.performance_samples, .00055, 0))
    if args.periodic_samples:
        workloads.append(("periodic", "arm6", "smooth", 0, args.periodic_samples, .00055, 1000))

    for group, robot, scenario, mode, samples, timeout, period in workloads:
        if args.suite != "all" and group != args.suite:
            continue
        measurements = {}
        for label, executable in solvers.items():
            name = f"{group}_{robot}_{scenario}_mode{mode}_{label}"
            path = args.output / f"{name}.csv"
            command = [executable, str(path), str(samples), str(timeout), str(mode), scenario, robot, str(period)]
            result = subprocess.run(command, check=True, capture_output=True, text=True, timeout=180)
            print(name, result.stdout.strip(), flush=True)
            rows = list(csv.DictReader(path.open()))
            assert len(rows) == samples
            measurements[label] = rows
            times = [float(row["us"]) for row in rows]
            successful = [row for row in rows if int(row["rc"]) >= 0]
            record = dict(group=group, robot=robot, scenario=scenario, mode=mode, solver=label,
                          samples=samples, timeout_seconds=timeout, period_us=period,
                          success=len(successful), mean_us=statistics.mean(times), max_us=max(times),
                          p50_us=percentile(times, 50), p95_us=percentile(times, 95), p99_us=percentile(times, 99),
                          p999_us=percentile(times, 99.9), over_1ms=sum(t > 1000 for t in times),
                          over_budget=sum(t > timeout * 1e6 for t in times),
                          max_position_error=max((float(row["position_error"]) for row in successful), default=0),
                          max_rotation_error=max((float(row["rotation_error"]) for row in successful), default=0),
                          release_deadline_misses=sum(float(row["release_to_finish_us"]) > 1000 for row in rows))
            if group == "correctness" and scenario != "unreachable" and len(successful) != samples:
                raise AssertionError(f"Correctness workload did not solve all targets: {name}")
            summary.append(record)
        labels = list(measurements)
        reference = measurements[labels[0]]
        input_fields = [key for key in reference[0] if key.startswith(("seed", "target"))]
        for label in labels[1:]:
            both = 0
            max_joint_difference = 0.0
            for expected, actual in zip(reference, measurements[label]):
                assert all(expected[key] == actual[key] for key in input_fields), "Inputs differ across backends"
                if int(expected["rc"]) < 0 or int(actual["rc"]) < 0:
                    continue
                both += 1
                for j in range(6):
                    difference = float(actual[f"q{j}"]) - float(expected[f"q{j}"])
                    if robot != "cartesian" or j >= 3:
                        difference = math.remainder(difference, 2 * math.pi)
                    max_joint_difference = max(max_joint_difference, abs(difference))
            print(f"  paired {labels[0]} vs {label}: both_solved={both} max_joint_difference={max_joint_difference:.9g}", flush=True)
            # The nonsingular 3P+3R fixture has a locally unique solution at these seeds.
            # The 6R chain may admit different valid branches, validated independently by FK.
            if robot == "cartesian" and max_joint_difference > 5e-5:
                raise AssertionError("Locally unique solutions disagree")
            for record in summary:
                if (record["group"], record["robot"], record["scenario"], record["mode"], record["solver"]) == (group, robot, scenario, mode, label):
                    record["paired_reference"] = labels[0]
                    record["paired_solved"] = both
                    record["max_joint_difference"] = max_joint_difference
        (args.output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")


if __name__ == "__main__":
    main()
