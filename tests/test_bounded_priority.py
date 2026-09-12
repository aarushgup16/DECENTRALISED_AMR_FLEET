"""Unit tests for Bounded Priority tie-breaker and anti-starvation guarantees."""

import pytest
from decentralized_amr.collision_avoidance.bounded_priority import BoundedPriorityEvaluator
from decentralized_amr.config import PriorityConfig


def test_priority_score_components():
    """Verify individual components of the bounded priority score."""
    evaluator = BoundedPriorityEvaluator()

    # Base priority for idle robot with 0 yields
    p_idle = evaluator.compute_priority(node_id=1, task_urgency=0, consecutive_yields=0, dist_to_goal=10.0, battery_pct=100.0)

    # Urgent task increases priority by ~50,000 * urgency
    p_urgent = evaluator.compute_priority(node_id=1, task_urgency=2, consecutive_yields=0, dist_to_goal=10.0, battery_pct=100.0)
    assert p_urgent > p_idle + 99000.0


def test_starvation_freedom_capped_yield():
    """
    Verify anti-starvation property:
    A robot that keeps yielding increases its priority up to the cap of 20,000,
    guaranteeing it will eventually outrank standard peers with identical urgency.
    """
    evaluator = BoundedPriorityEvaluator()

    # Robot 1 yielding 1 time
    p_yield_1 = evaluator.compute_priority(node_id=1, task_urgency=1, consecutive_yields=1)
    # Robot 1 yielding 2 times
    p_yield_2 = evaluator.compute_priority(node_id=1, task_urgency=1, consecutive_yields=2)
    # Robot 1 yielding 10 times (should hit cap of 20,000)
    p_yield_10 = evaluator.compute_priority(node_id=1, task_urgency=1, consecutive_yields=10)

    assert p_yield_2 > p_yield_1
    assert p_yield_10 == p_yield_2, "Yield bonus must be strictly capped at 20,000 to prevent unbounded inflation"

    # A yielding robot (2 yields) must outrank a fresh robot with identical urgency
    p_fresh = evaluator.compute_priority(node_id=2, task_urgency=1, consecutive_yields=0)
    assert p_yield_2 > p_fresh


def test_choke_right_of_way_resolution():
    """Verify deterministic right-of-way resolution at choke points."""
    evaluator = BoundedPriorityEvaluator()

    # Robot 1 higher priority -> Winner
    winner, yielder = evaluator.resolve_choke_right_of_way(
        robot_a_id=1, robot_a_priority=65000.0,
        robot_b_id=2, robot_b_priority=52000.0
    )
    assert winner == 1
    assert yielder == 2

    # Tied priority -> Higher robot ID wins deterministically
    winner_tie, yielder_tie = evaluator.resolve_choke_right_of_way(
        robot_a_id=1, robot_a_priority=50000.0,
        robot_b_id=2, robot_b_priority=50000.0
    )
    assert winner_tie == 2
    assert yielder_tie == 1

