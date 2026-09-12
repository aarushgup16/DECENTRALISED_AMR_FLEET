"""Typed P2P message definitions for the decentralized AMR fleet."""

from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Optional, Tuple
import time


class MessageType(str, Enum):
    """Enumeration of message types exchanged over the P2P mesh."""
    HEARTBEAT = "HEARTBEAT"
    TELEMETRY = "TELEMETRY"
    CBBA_BID = "CBBA_BID"
    TASK_RELEASE = "TASK_RELEASE"
    CHOKE_YIELD_REQUEST = "CHOKE_YIELD_REQUEST"
    CHOKE_YIELD_RESPONSE = "CHOKE_YIELD_RESPONSE"
    OBSTACLE_ALERT = "OBSTACLE_ALERT"


@dataclass
class BaseMessage:
    """Base message format with sender ID, Lamport clock, and wall-clock timestamp."""
    sender_id: int
    lamport_clock: int
    msg_type: MessageType = MessageType.HEARTBEAT
    wall_time: float = field(default_factory=time.time)
    target_id: Optional[int] = None   # None indicates broadcast


@dataclass
class TelemetryMessage(BaseMessage):
    """Periodic state broadcast from each AMR."""
    x: float = 0.0
    y: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    heading: float = 0.0
    linear_v: float = 0.0
    angular_w: float = 0.0
    battery: float = 100.0
    current_task_id: Optional[str] = None
    status: str = "IDLE"
    priority_score: float = 0.0
    intended_path: List[Tuple[float, float]] = field(default_factory=list)
    msg_type: MessageType = MessageType.TELEMETRY


@dataclass
class CBBABidMessage(BaseMessage):
    """CBBA bid & consensus state broadcast."""
    winning_bids: Dict[str, float] = field(default_factory=dict)
    winning_agents: Dict[str, int] = field(default_factory=dict)
    bid_clocks: Dict[str, int] = field(default_factory=dict)
    msg_type: MessageType = MessageType.CBBA_BID


@dataclass
class TaskReleaseMessage(BaseMessage):
    """Broadcast when a robot releases its task bundle due to blocked aisle/obstacle."""
    task_id: str = ""
    releasing_agent_id: int = 0
    reason: str = "blocked_aisle"
    obstacle_pos: Optional[Tuple[float, float]] = None
    msg_type: MessageType = MessageType.TASK_RELEASE


@dataclass
class ChokeYieldMessage(BaseMessage):
    """Choke point right-of-way negotiation message."""
    choke_id: str = ""
    candidate_priority: float = 0.0
    is_yielding: bool = False
    action: str = "PROPOSE"  # "PROPOSE", "YIELD", "PASS"
    msg_type: MessageType = MessageType.CHOKE_YIELD_REQUEST


@dataclass
class ObstacleAlertMessage(BaseMessage):
    """Broadcast alert when a robot discovers an unmapped dynamic obstacle."""
    obstacle_id: str = ""
    x_min: float = 0.0
    y_min: float = 0.0
    x_max: float = 0.0
    y_max: float = 0.0
    detected_by: int = 0
    msg_type: MessageType = MessageType.OBSTACLE_ALERT

