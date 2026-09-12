"""Decentralized Peer-to-Peer messaging mesh with Lamport clock integration."""

import threading
import time
from typing import Callable, Dict, List, Optional, Set, Tuple
from decentralized_amr.network.lamport_clock import LamportClock
from decentralized_amr.network.messages import BaseMessage, MessageType
from decentralized_amr.network.network_chaos import NetworkChaosSimulator


class P2PNode:
    """
    Decentralized network endpoint on each AMR edge device.
    Maintains its own Lamport clock and direct peer links.
    """

    def __init__(
        self,
        node_id: int,
        lamport_clock: LamportClock,
        mesh: "P2PMesh",
        comm_range: float = 15.0
    ):
        self.node_id = node_id
        self.clock = lamport_clock
        self.mesh = mesh
        self.comm_range = comm_range
        self.position: Tuple[float, float] = (0.0, 0.0)
        self._handlers: Dict[MessageType, List[Callable[[BaseMessage], None]]] = {}
        self._lock = threading.Lock()
        
        # Register with the decentralized mesh
        self.mesh.register_node(self)

    def update_position(self, x: float, y: float):
        """Update current known coordinates for spatial broadcast range checks."""
        self.position = (x, y)

    def register_handler(self, msg_type: MessageType, handler: Callable[[BaseMessage], None]):
        """Register a callback handler for a specific message type."""
        with self._lock:
            if msg_type not in self._handlers:
                self._handlers[msg_type] = []
            self._handlers[msg_type].append(handler)

    def broadcast(self, message: BaseMessage):
        """
        Broadcast message to all reachable peers.
        Advances local Lamport clock and stamps message before dispatch.
        """
        message.sender_id = self.node_id
        message.lamport_clock = self.clock.tick()
        message.wall_time = time.time()
        message.target_id = None
        self.mesh.route_broadcast(self, message)

    def send_direct(self, target_id: int, message: BaseMessage):
        """
        Send point-to-point message directly to a specific peer.
        Advances local Lamport clock and stamps message.
        """
        message.sender_id = self.node_id
        message.lamport_clock = self.clock.tick()
        message.wall_time = time.time()
        message.target_id = target_id
        self.mesh.route_direct(self, target_id, message)

    def receive(self, message: BaseMessage):
        """
        Process incoming message from a peer.
        Updates local Lamport clock via causal witness rule:
        L_local = max(L_local, L_msg) + 1.
        """
        self.clock.witness(message.lamport_clock)
        
        with self._lock:
            handlers = list(self._handlers.get(message.msg_type, []))
            
        for handler in handlers:
            try:
                handler(message)
            except Exception as e:
                print(f"[P2PNode-{self.node_id}] Handler error for {message.msg_type}: {e}")


class P2PMesh:
    """
    Decentralized communication medium connecting P2P nodes.
    Supports peer discovery, spatial range filtering, network chaos injection,
    and passive read-only monitoring listeners.
    """

    def __init__(self, chaos_simulator: Optional[NetworkChaosSimulator] = None):
        self._nodes: Dict[int, P2PNode] = {}
        self._passive_listeners: List[Callable[[BaseMessage], None]] = []
        self._lock = threading.Lock()
        self.chaos = chaos_simulator or NetworkChaosSimulator()

    def register_node(self, node: P2PNode):
        """Register an active AMR node."""
        with self._lock:
            self._nodes[node.node_id] = node

    def unregister_node(self, node_id: int):
        """Unregister an AMR node."""
        with self._lock:
            self._nodes.pop(node_id, None)

    def add_passive_listener(self, listener: Callable[[BaseMessage], None]):
        """
        Add a read-only passive subscriber (e.g. Dashboard WebSocket).
        Passive listeners receive telemetry with zero command authority and
        do not affect mesh consensus or Lamport clocks.
        """
        with self._lock:
            self._passive_listeners.append(listener)

    def remove_passive_listener(self, listener: Callable[[BaseMessage], None]):
        """Remove a passive listener."""
        with self._lock:
            if listener in self._passive_listeners:
                self._passive_listeners.remove(listener)

    def route_broadcast(self, sender: P2PNode, message: BaseMessage):
        """Deliver broadcast message to all peers within communication range."""
        with self._lock:
            active_nodes = list(self._nodes.values())
            listeners = list(self._passive_listeners)

        # Notify passive monitoring listeners (Dashboard)
        for listener in listeners:
            try:
                listener(message)
            except Exception:
                pass

        sender_pos = sender.position
        for peer in active_nodes:
            if peer.node_id == sender.node_id:
                continue
            
            # Check spatial distance for communication range
            peer_pos = peer.position
            dist = ((sender_pos[0] - peer_pos[0]) ** 2 + (sender_pos[1] - peer_pos[1]) ** 2) ** 0.5
            if dist > sender.comm_range:
                continue

            # Check network degradation / dead zones
            if self.chaos.should_drop(sender_pos, peer_pos):
                continue

            peer.receive(message)

    def route_direct(self, sender: P2PNode, target_id: int, message: BaseMessage):
        """Deliver direct message to target node."""
        with self._lock:
            target_node = self._nodes.get(target_id)
            listeners = list(self._passive_listeners)

        for listener in listeners:
            try:
                listener(message)
            except Exception:
                pass

        if not target_node:
            return

        if self.chaos.should_drop(sender.position, target_node.position):
            return

        target_node.receive(message)

