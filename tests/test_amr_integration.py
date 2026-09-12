"""Integration test for full decentralized multi-AMR warehouse simulation."""

import pytest
from decentralized_amr.simulation.fleet_coordinator import FleetCoordinator


def test_multi_amr_collision_free_task_completion():
    """
    Verify a fleet of 4 decentralized AMRs complete tasks with zero collisions.
    """
    coordinator = FleetCoordinator(num_robots=4)
    tasks = coordinator.map.generate_random_tasks(num_tasks=8, seed=123)
    coordinator.inject_tasks(tasks)

    max_steps = 4000 # 200 seconds of simulated time
    step_count = 0

    while step_count < max_steps:
        still_running = coordinator.step()
        step_count += 1
        if not still_running:
            break

    summary = coordinator.get_metrics_summary()

    # Success criteria verification
    assert summary.collision_count == 0, f"Expected 0 collisions, got {summary.collision_count}"
    assert summary.tasks_completed == 8, f"Expected all 8 tasks completed, got {summary.tasks_completed}"
    assert summary.total_time_seconds > 0.0
