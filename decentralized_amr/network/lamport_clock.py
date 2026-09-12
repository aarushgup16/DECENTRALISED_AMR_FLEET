"""Lamport Logical Clock implementation for causal message ordering."""

import threading
from typing import Optional


class LamportClock:
    """
    Monotonic Lamport Logical Clock.
    
    Provides strict causal 'happens-before' ordering across all decentralized AMRs.
    Guarantees that an AMR returning from a Wi-Fi dead zone with stale bids or outdated
    state cannot overwrite newer consensus state.
    """

    def __init__(self, initial_value: int = 0, node_id: Optional[int] = None):
        self._lock = threading.Lock()
        self._clock: int = initial_value
        self._node_id: Optional[int] = node_id

    def tick(self) -> int:
        """
        Increment the clock on a local event (e.g., preparing to send a message).
        Returns the updated clock value.
        """
        with self._lock:
            self._clock += 1
            return self._clock

    def witness(self, received_clock: int) -> int:
        """
        Update the clock upon receiving a message with a remote Lamport timestamp.
        Rule: L_local = max(L_local, L_msg) + 1.
        Returns the updated clock value.
        """
        with self._lock:
            self._clock = max(self._clock, received_clock) + 1
            return self._clock

    def get_time(self) -> int:
        """Return the current logical clock value."""
        with self._lock:
            return self._clock

    def is_stale(self, incoming_clock: int, recorded_clock: int) -> bool:
        """
        Check if an incoming message's Lamport clock is causally older or equal
        to the recorded clock for that state item.
        """
        return incoming_clock <= recorded_clock

    def compare(self, clock_a: int, node_a: int, clock_b: int, node_b: int) -> int:
        """
        Deterministic total ordering of two events (clock_a, node_a) vs (clock_b, node_b).
        Returns:
            > 0 if A is causally after B (A wins)
            < 0 if A is causally before B (B wins)
            0 if identical
        """
        if clock_a != clock_b:
            return 1 if clock_a > clock_b else -1
        # Deterministic tie-breaker using node_id
        if node_a != node_b:
            return 1 if node_a > node_b else -1
        return 0

    def __repr__(self) -> str:
        return f"LamportClock(node_id={self._node_id}, clock={self.get_time()})"

