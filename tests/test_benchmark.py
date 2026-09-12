"""Benchmark validation test: verifies zero collisions and >= 20% speedup."""

import pytest
from run_benchmark import run_benchmark_scenario


def test_benchmark_standard_scenario():
    """
    Automated test validating that Decentralized Solution:
    1. Achieves exactly 0 collisions.
    2. Achieves >= 20% task completion time reduction vs. Stop-and-Wait Baseline.
    """
    res = run_benchmark_scenario("Test Standard Fleet", num_robots=4, num_tasks=12, seed=42)

    sol = res["solution"]
    comp = res["comparison"]

    # Criteria 1: Zero Collisions
    assert sol["collisions"] == 0, f"Expected 0 collisions, got {sol['collisions']}"

    # Criteria 2: >= 20% Speedup
    time_reduction = comp["time_reduction_pct"]
    assert time_reduction >= 20.0, f"Expected >= 20% time reduction, got {time_reduction}%"
