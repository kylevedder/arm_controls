#!/usr/bin/env python3
"""Collect pytest tests and run one deterministic, duration-balanced shard."""

from __future__ import annotations

import argparse
import json
import math
import statistics
import subprocess
import sys
from pathlib import Path


def parse_node_ids(collection_output: str) -> list[str]:
    """Extract pytest node IDs from quiet collection output."""
    return [line.strip() for line in collection_output.splitlines() if "::" in line]


def load_durations(path: Path | None) -> dict[str, float]:
    """Load measured test durations, rejecting malformed or unsafe weights."""
    if path is None or not path.exists():
        return {}

    raw = json.loads(path.read_text())
    if not isinstance(raw, dict):
        raise ValueError(f"duration data in {path} must be a JSON object")

    durations: dict[str, float] = {}
    for node_id, duration in raw.items():
        if not isinstance(node_id, str) or not isinstance(duration, (int, float)):
            raise ValueError(f"invalid duration entry in {path}: {node_id!r}: {duration!r}")
        duration = float(duration)
        if not math.isfinite(duration) or duration < 0:
            raise ValueError(f"invalid duration entry in {path}: {node_id!r}: {duration!r}")
        durations[node_id] = duration
    return durations


def assign_shards(
    node_ids: list[str], shard_count: int, durations: dict[str, float]
) -> tuple[list[list[str]], list[float]]:
    """Balance tests with longest-processing-time-first scheduling.

    Unknown tests use the median measured duration. With no measurements every
    test has equal weight, which degenerates to deterministic round-robin
    assignment while the first instrumented run gathers real timings.
    """
    measured = [durations[node_id] for node_id in node_ids if node_id in durations]
    default_duration = statistics.median(measured) if measured else 1.0
    indexed_node_ids = list(enumerate(node_ids))
    indexed_node_ids.sort(
        key=lambda item: (-durations.get(item[1], default_duration), item[0])
    )

    shards: list[list[str]] = [[] for _ in range(shard_count)]
    totals = [0.0] * shard_count
    for _collection_index, node_id in indexed_node_ids:
        shard_index = min(
            range(shard_count),
            key=lambda index: (totals[index], len(shards[index]), index),
        )
        shards[shard_index].append(node_id)
        totals[shard_index] += durations.get(node_id, default_duration)
    return shards, totals


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--shard-index", required=True, type=int)
    parser.add_argument("--shard-count", required=True, type=int)
    parser.add_argument("--durations-file", type=Path)
    parser.add_argument("--junit-xml", type=Path)
    parser.add_argument("paths", nargs="+")
    args = parser.parse_args()

    if args.shard_count < 1:
        parser.error("--shard-count must be positive")
    if not 0 <= args.shard_index < args.shard_count:
        parser.error("--shard-index must be between zero and shard-count minus one")

    collect = subprocess.run(
        [sys.executable, "-m", "pytest", "-o", "addopts=", "--collect-only", "-q", *args.paths],
        check=False,
        capture_output=True,
        text=True,
    )
    if collect.returncode != 0:
        sys.stdout.write(collect.stdout)
        sys.stderr.write(collect.stderr)
        return collect.returncode

    node_ids = parse_node_ids(collect.stdout)
    try:
        durations = load_durations(args.durations_file)
    except (OSError, ValueError, json.JSONDecodeError) as error:
        parser.error(str(error))
    shards, totals = assign_shards(node_ids, args.shard_count, durations)
    selected = shards[args.shard_index]
    if not selected:
        print("pytest shard selected no tests", file=sys.stderr)
        return 1

    print(
        f"Running pytest shard {args.shard_index + 1}/{args.shard_count}: "
        f"{len(selected)} of {len(node_ids)} tests "
        f"(predicted {totals[args.shard_index]:.2f}s)",
        flush=True,
    )
    for node_id in selected:
        print(f"  {node_id} ({durations.get(node_id, 0.0):.2f}s measured)", flush=True)

    pytest_command = [
        sys.executable,
        "-m",
        "pytest",
        "-o",
        "addopts=",
        "-vv",
        "--durations=0",
        "--durations-min=0",
    ]
    if args.junit_xml is not None:
        pytest_command.extend(["--junitxml", str(args.junit_xml)])
    pytest_command.extend(selected)
    return subprocess.run(pytest_command, check=False).returncode


if __name__ == "__main__":
    raise SystemExit(main())
