from __future__ import annotations

import json
import sys
from importlib import import_module
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

run_pytest_shard = import_module("scripts.run_pytest_shard")
assign_shards = run_pytest_shard.assign_shards
load_durations = run_pytest_shard.load_durations
parse_node_ids = run_pytest_shard.parse_node_ids


def test_parse_node_ids_ignores_pytest_summary() -> None:
    output = """tests/test_example.py::test_one
tests/test_example.py::test_two[value]

2 tests collected in 0.01s
"""
    assert parse_node_ids(output) == [
        "tests/test_example.py::test_one",
        "tests/test_example.py::test_two[value]",
    ]


def test_assign_shards_balances_longest_tests_first() -> None:
    node_ids = ["a", "b", "c", "d", "e", "f"]
    durations = {"a": 10.0, "b": 9.0, "c": 5.0, "d": 4.0, "e": 2.0, "f": 1.0}

    shards, totals = assign_shards(node_ids, shard_count=2, durations=durations)

    assert shards == [["a", "d", "e"], ["b", "c", "f"]]
    assert totals == [16.0, 15.0]


def test_assign_shards_uses_median_for_unmeasured_tests() -> None:
    shards, totals = assign_shards(
        ["slow", "new", "fast"],
        shard_count=2,
        durations={"slow": 9.0, "fast": 1.0},
    )

    assert shards == [["slow"], ["new", "fast"]]
    assert totals == [9.0, 6.0]


def test_load_durations_validates_values(tmp_path) -> None:
    duration_file = tmp_path / "durations.json"
    duration_file.write_text(json.dumps({"test_one": 1.25}))
    assert load_durations(duration_file) == {"test_one": 1.25}

    duration_file.write_text(json.dumps({"test_one": -1}))
    with pytest.raises(ValueError, match="invalid duration entry"):
        load_durations(duration_file)
