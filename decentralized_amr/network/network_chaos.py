"""Network degradation and dead-zone chaos simulator."""

import random
from typing import List, Optional, Tuple


class NetworkChaosSimulator:
    """
    Simulates real-world wireless network conditions:
    - Packet loss
    - Random latency jitter
    - Wi-Fi dead zones (spatial blind spots where packets are dropped or delayed)
    """

    def __init__(
        self,
        packet_loss_rate: float = 0.0,
        min_latency_ms: float = 0.0,
        max_latency_ms: float = 0.0,
        dead_zones: Optional[List[Tuple[float, float, float, float]]] = None
    ):
        self.packet_loss_rate = packet_loss_rate
        self.min_latency_ms = min_latency_ms
        self.max_latency_ms = max_latency_ms
        self.dead_zones = dead_zones or []

    def is_in_dead_zone(self, x: float, y: float) -> bool:
        """Check if coordinates fall inside any designated Wi-Fi dead zone."""
        for (x_min, y_min, x_max, y_max) in self.dead_zones:
            if x_min <= x <= x_max and y_min <= y <= y_max:
                return True
        return False

    def should_drop(self, sender_pos: Optional[Tuple[float, float]] = None,
                    receiver_pos: Optional[Tuple[float, float]] = None) -> bool:
        """Determine if a packet should be dropped due to loss rate or dead zones."""
        if sender_pos and self.is_in_dead_zone(sender_pos[0], sender_pos[1]):
            return True
        if receiver_pos and self.is_in_dead_zone(receiver_pos[0], receiver_pos[1]):
            return True
        if self.packet_loss_rate > 0.0:
            return random.random() < self.packet_loss_rate
        return False

    def get_latency(self) -> float:
        """Return simulated packet delay in seconds."""
        if self.max_latency_ms <= 0.0:
            return 0.0
        delay_ms = random.uniform(self.min_latency_ms, self.max_latency_ms)
        return delay_ms / 1000.0

