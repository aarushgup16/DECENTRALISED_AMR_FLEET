"""Task definitions and status tracking for decentralized warehouse operations."""

from dataclasses import dataclass, field
from enum import Enum
from typing import Optional, Tuple
import time


class TaskStatus(str, Enum):
    """Lifecycle status of a warehouse transportation task."""
    UNASSIGNED = "UNASSIGNED"
    BIDDING = "BIDDING"
    ASSIGNED = "ASSIGNED"
    IN_PROGRESS = "IN_PROGRESS"
    COMPLETED = "COMPLETED"
    FAILED = "FAILED"
    RELEASED = "RELEASED"


@dataclass
class WarehouseTask:
    """Represents a discrete warehouse item transport job."""
    task_id: str
    pickup_pos: Tuple[float, float]
    dropoff_pos: Tuple[float, float]
    urgency: int = 1               # 1=normal, 2=priority, 3=high, 4=emergency
    base_reward: float = 100.0     # Base completion value
    status: TaskStatus = TaskStatus.UNASSIGNED
    assigned_robot_id: Optional[int] = None
    creation_time: float = field(default_factory=time.time)
    start_time: Optional[float] = None
    completion_time: Optional[float] = None
    demand_weight: float = 1.0     # Payload weight in kg
    
    @property
    def total_value(self) -> float:
        """Total priority-weighted economic value of completing the task."""
        return self.base_reward + (self.urgency * 50.0)

    def mark_assigned(self, robot_id: int):
        self.status = TaskStatus.ASSIGNED
        self.assigned_robot_id = robot_id

    def mark_in_progress(self):
        self.status = TaskStatus.IN_PROGRESS
        if self.start_time is None:
            self.start_time = time.time()

    def mark_completed(self):
        self.status = TaskStatus.COMPLETED
        self.completion_time = time.time()

    def mark_released(self):
        self.status = TaskStatus.RELEASED
        self.assigned_robot_id = None

