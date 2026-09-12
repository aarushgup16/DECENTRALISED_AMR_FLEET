"""Automated Benchmark Suite: Baseline vs. Decentralized AMR Solution."""

import json
import os
import sys
import time
from dataclasses import asdict
from typing import Any, Dict, List, Tuple

from decentralized_amr.simulation.baseline_runner import BaselineSimulator
from decentralized_amr.simulation.fleet_coordinator import FleetCoordinator
from decentralized_amr.simulation.warehouse_map import WarehouseMap
from decentralized_amr.task_allocation.task import WarehouseTask


def run_benchmark_scenario(
    scenario_name: str,
    num_robots: int = 4,
    num_tasks: int = 12,
    seed: int = 42,
    block_choke: bool = False
) -> Dict[str, Any]:
    """
    Run identical scenario across Baseline (Stop-and-Wait) and Solution (NH-ORCA + CBBA).
    """
    print(f"\n>>> Running Scenario: [{scenario_name}] (Robots: {num_robots}, Tasks: {num_tasks})")
    
    # Generate identical tasks
    ref_map = WarehouseMap()
    tasks_baseline = ref_map.generate_random_tasks(num_tasks=num_tasks, seed=seed)
    tasks_solution = ref_map.generate_random_tasks(num_tasks=num_tasks, seed=seed)

    # 1. Run Baseline Simulation (Stop-and-Wait)
    print("  [1/2] Executing Baseline (Stop-and-Wait)...")
    base_sim = BaselineSimulator(num_robots=num_robots)
    base_sim.inject_tasks(tasks_baseline)
    base_metrics = base_sim.run_until_complete(max_seconds=250.0)

    # 2. Run Solution Simulation (NH-ORCA + CBBA + Lamport + Priority)
    print("  [2/2] Executing Solution (NH-ORCA + CBBA + Lamport Clocks)...")
    sol_coord = FleetCoordinator(num_robots=num_robots)
    sol_coord.inject_tasks(tasks_solution)

    if block_choke:
        # Dynamically inject obstacle in center intersection after 5 seconds
        choke = sol_coord.map.choke_regions[1]
        sol_coord.inject_blocked_aisle("BENCHMARK-CHOKE-BLOCK", choke)

    while sol_coord.sim_time < 250.0:
        still_running = sol_coord.step()
        if not still_running:
            break

    sol_metrics = sol_coord.get_metrics_summary()

    # 3. Compute Comparative Metrics
    t_base = max(0.1, base_metrics.total_time_seconds)
    t_sol = max(0.1, sol_metrics.total_time_seconds)
    time_reduction_pct = round(((t_base - t_sol) / t_base) * 100.0, 2)
    speedup_ratio = round(t_base / t_sol, 2)

    passed_collision = (sol_metrics.collision_count == 0)
    passed_speedup = (time_reduction_pct >= 20.0)

    result = {
        "scenario": scenario_name,
        "robots": num_robots,
        "tasks": num_tasks,
        "baseline": {
            "time_seconds": base_metrics.total_time_seconds,
            "collisions": base_metrics.collision_count,
            "near_misses": base_metrics.near_miss_count,
            "throughput_tasks_min": base_metrics.throughput_tasks_per_min,
            "distance_m": base_metrics.total_fleet_distance_m,
            "battery_consumed_pct": base_metrics.total_battery_consumed_pct
        },
        "solution": {
            "time_seconds": sol_metrics.total_time_seconds,
            "collisions": sol_metrics.collision_count,
            "near_misses": sol_metrics.near_miss_count,
            "throughput_tasks_min": sol_metrics.throughput_tasks_per_min,
            "distance_m": sol_metrics.total_fleet_distance_m,
            "battery_consumed_pct": sol_metrics.total_battery_consumed_pct,
            "priority_tie_breaks": sol_metrics.priority_tie_breaks
        },
        "comparison": {
            "time_reduction_pct": time_reduction_pct,
            "speedup_ratio": f"{speedup_ratio}x",
            "zero_collision_criteria": "PASSED ✓" if passed_collision else "FAILED ✗",
            "speedup_criteria_20pct": "PASSED ✓" if passed_speedup else "FAILED ✗",
            "overall_status": "PASSED ALL CRITERIA ✓" if (passed_collision and passed_speedup) else "FAILED ✗"
        }
    }
    return result


def main():
    print("=" * 80)
    print("      DECENTRALIZED AMR FLEET — BENCHMARK VALIDATION SUITE")
    print("  Comparing: Baseline (Stop-and-Wait) vs. Solution (NH-ORCA + CBBA + Lamport)")
    print("=" * 80)

    scenarios = [
        ("Standard Warehouse Fleet", 4, 12, 42, False),
        ("High Congestion Fleet", 6, 18, 101, False),
        ("Dynamic Obstacle Re-Routing", 4, 10, 77, True)
    ]

    all_results = []

    for (name, robots, tasks, seed, block) in scenarios:
        res = run_benchmark_scenario(name, num_robots=robots, num_tasks=tasks, seed=seed, block_choke=block)
        all_results.append(res)

        b = res["baseline"]
        s = res["solution"]
        c = res["comparison"]

        print("\n" + "-" * 75)
        print(f"  SCENARIO RESULT: {name}")
        print("-" * 75)
        print(f"  {'Metric':<28} | {'Baseline (Stop-Wait)':<20} | {'Solution (NH-ORCA)':<20}")
        print(f"  {'-'*28}-+-{'-'*20}-+-{'-'*20}")
        print(f"  {'Task Completion Time':<28} | {b['time_seconds']:>17.2f} s | {s['time_seconds']:>17.2f} s")
        print(f"  {'Time Reduction (Target ≥20%)':<28} | {'-':>20} | {c['time_reduction_pct']:>16.2f} %")
        print(f"  {'Inter-Robot Collisions':<28} | {b['collisions']:>20} | {s['collisions']:>20}")
        print(f"  {'Fleet Throughput':<28} | {b['throughput_tasks_min']:>11.2f} tasks/m | {s['throughput_tasks_min']:>11.2f} tasks/m")
        print(f"  {'Fleet Total Distance':<28} | {b['distance_m']:>18.2f} m | {s['distance_m']:>18.2f} m")
        print(f"  {'Priority Tie-Breaks':<28} | {'N/A (Stopped)':>20} | {s['priority_tie_breaks']:>20}")
        print("-" * 75)
        print(f"  Status: {c['overall_status']} (Zero Collisions: {c['zero_collision_criteria']}, Speedup ≥20%: {c['speedup_criteria_20pct']})\n")

    # Save benchmark report to JSON
    report_file = os.path.join(os.path.dirname(__file__), "benchmark_report.json")
    with open(report_file, "w") as f:
        json.dump(all_results, f, indent=2)
    print(f"Detailed benchmark metrics report saved to: {report_file}\n")


if __name__ == "__main__":
    main()

