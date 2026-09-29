"""Network degradation, live fault injection, and dead-zone chaos simulator."""

import random
from typing import List, Optional, Set, Tuple


class NetworkChaosSimulator:
    """
    Simulates real-world wireless network conditions and live edge chaos injection:
    - Packet loss spikes (e.g. 50% global loss)
    - Dynamic link severing / network partitions (Node A <-> Node B)
    - Node killing (dead node hardware failure)
    - Wi-Fi dead zones (spatial blind spots)
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
        self.dropped_packets_total = 0
        self.transmitted_packets_total = 0

        # Dynamic fault injection state
        self.severed_links: Set[Tuple[int, int]] = set() # Undirected pair (min(a,b), max(a,b))
        self.killed_nodes: Set[int] = set()

    def sever_link(self, node_a: int, node_b: int):
        """Sever bidirectional communication link between two AMR nodes."""
        pair = (min(node_a, node_b), max(node_a, node_b))
        self.severed_links.add(pair)

    def heal_link(self, node_a: int, node_b: int):
        """Restore severed link between two AMR nodes."""
        pair = (min(node_a, node_b), max(node_a, node_b))
        self.severed_links.discard(pair)

    def heal_all_links(self):
        """Heal all active network partitions."""
        self.severed_links.clear()

    def is_link_severed(self, node_a: int, node_b: int) -> bool:
        """Check if mesh link between node_a and node_b is severed."""
        pair = (min(node_a, node_b), max(node_a, node_b))
        return pair in self.severed_links

    def kill_node(self, node_id: int):
        """Simulate total node hardware / radio crash."""
        self.killed_nodes.add(node_id)

    def revive_node(self, node_id: int):
        """Revive previously killed AMR node."""
        self.killed_nodes.discard(node_id)

    def set_packet_loss_spike(self, rate: float):
        """Set global packet loss spike (0.0 to 1.0)."""
        self.packet_loss_rate = max(0.0, min(1.0, rate))

    def is_in_dead_zone(self, x: float, y: float) -> bool:
        """Check if coordinates fall inside any designated Wi-Fi dead zone."""
        for (x_min, y_min, x_max, y_max) in self.dead_zones:
            if x_min <= x <= x_max and y_min <= y <= y_max:
                return True
        return False

    def should_drop(
        self,
        sender_pos: Optional[Tuple[float, float]] = None,
        receiver_pos: Optional[Tuple[float, float]] = None,
        sender_id: Optional[int] = None,
        receiver_id: Optional[int] = None
    ) -> bool:
        """Determine if a packet should be dropped due to partition, dead-node, dead-zone, or loss rate."""
        self.transmitted_packets_total += 1

        # 1. Check killed nodes
        if sender_id is not None and sender_id in self.killed_nodes:
            self.dropped_packets_total += 1
            return True
        if receiver_id is not None and receiver_id in self.killed_nodes:
            self.dropped_packets_total += 1
            return True

        # 2. Check severed topological link
        if sender_id is not None and receiver_id is not None:
            if self.is_link_severed(sender_id, receiver_id):
                self.dropped_packets_total += 1
                return True

        # 3. Check spatial dead zones
        if sender_pos and self.is_in_dead_zone(sender_pos[0], sender_pos[1]):
            self.dropped_packets_total += 1
            return True
        if receiver_pos and self.is_in_dead_zone(receiver_pos[0], receiver_pos[1]):
            self.dropped_packets_total += 1
            return True

        # 4. Check global packet loss spike
        if self.packet_loss_rate > 0.0:
            if random.random() < self.packet_loss_rate:
                self.dropped_packets_total += 1
                return True

        return False

    def get_latency(self) -> float:
        """Return simulated packet delay in seconds."""
        if self.max_latency_ms <= 0.0:
            return 0.0
        delay_ms = random.uniform(self.min_latency_ms, self.max_latency_ms)
        return delay_ms / 1000.0
