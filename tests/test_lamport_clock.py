"""Unit tests for Lamport Logical Clock and causal happens-before ordering."""

import pytest
from decentralized_amr.network.lamport_clock import LamportClock


def test_monotonic_tick():
    """Verify local tick increments clock monotonically."""
    clock = LamportClock(initial_value=0, node_id=1)
    assert clock.get_time() == 0
    assert clock.tick() == 1
    assert clock.tick() == 2
    assert clock.get_time() == 2


def test_causal_witness_rule():
    """Verify witness rule: L_local = max(L_local, L_msg) + 1."""
    clock = LamportClock(initial_value=5, node_id=1)
    # Receiving message with lower clock
    res = clock.witness(received_clock=3)
    assert res == 6
    assert clock.get_time() == 6

    # Receiving message with higher clock (e.g. from fast peer)
    res2 = clock.witness(received_clock=20)
    assert res2 == 21
    assert clock.get_time() == 21


def test_stale_message_detection():
    """Verify stale message identification from delayed/dead-zone nodes."""
    clock = LamportClock(initial_value=10, node_id=1)
    # Recorded clock for a task state is 15
    recorded_clock = 15
    # Incoming stale bid with clock 12
    assert clock.is_stale(incoming_clock=12, recorded_clock=recorded_clock) is True
    # Incoming fresh bid with clock 16
    assert clock.is_stale(incoming_clock=16, recorded_clock=recorded_clock) is False


def test_deterministic_total_ordering():
    """Verify deterministic tie-breaking by clock and then node_id."""
    clock = LamportClock()
    # A has higher clock than B -> A wins
    assert clock.compare(clock_a=10, node_a=1, clock_b=5, node_b=2) == 1
    assert clock.compare(clock_a=5, node_a=1, clock_b=10, node_b=2) == -1

    # Equal clocks -> higher node_id wins
    assert clock.compare(clock_a=10, node_a=2, clock_b=10, node_b=1) == 1
    assert clock.compare(clock_a=10, node_a=1, clock_b=10, node_b=2) == -1
    assert clock.compare(clock_a=10, node_a=1, clock_b=10, node_b=1) == 0

