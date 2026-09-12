"""Metrics collector for validating zero collisions, speedup, and decentralized performance."""

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple


@dataclass
class CollisionEvent:
    """Recorded collision event between two robots or a robot and an obstacle."""
    timestamp: float
    robot_a: int
    robot_b: Optional[int]
    distance: float
    location: Tuple[float, float]
    event_type: str = "INTER_ROBOT" # "INTER_ROBOT" or "WALL"


@dataclass
class SimulationMetrics:
    """Aggregated run metrics."""
    total_time_seconds: float = 0.0
    tasks_completed: int = 0
    total_tasks: int = 0
    collision_count: int = 0
    near_miss_count: int = 0
    deadlock_count: int = 0
    total_fleet_distance_m: float = 0.0
    total_battery_consumed_pct: float = 0.0
    throughput_tasks_per_min: float = 0.0
    priority_tie_breaks: int = 0
    cbba_consensus_events: int = 0
    collision_log: List[CollisionEvent] = field(default_factory=list)


class MetricsCollector:
    """
    Real-time telemetry and validation monitor.
    Tracks collisions, task durations, battery, and throughput.
    """

    def __init__(self, physical_radius: float = 0.35):
        self.physical_radius = physical_radius
        self.collision_threshold = 2.0 * physical_radius # 0.70m
        self.near_miss_threshold = self.collision_threshold + 0.15 # 0.85m

        self.collision_events: List[CollisionEvent] = []
        self.near_miss_events: List[CollisionEvent] = []
        self.deadlock_events: List[str] = []

        self.sim_time = 0.0
        self.task_start_times: Dict[str, float] = {}
        self.task_end_times: Dict[str, float] = {}
        self.total_tasks_count = 0

    def start_scenario(self, total_tasks: int):
        """Reset and initialize scenario metrics."""
        self.sim_time = 0.0
        self.total_tasks_count = total_tasks
        self.collision_events.clear()
        self.near_miss_events.clear()
        self.deadlock_events.clear()
        self.task_start_times.clear()
        self.task_end_times.clear()

    def update(self, current_sim_time: float, robot_positions: List[Tuple[int, float, float]]):
        """
        Check for inter-robot collisions and near misses at each physics tick.
        robot_positions: list of (robot_id, x, y)
        """
        self.sim_time = current_sim_time
        num_robots = len(robot_positions)

        for i in range(num_robots):
            id_a, xa, ya = robot_positions[i]
            for j in range(i + 1, num_robots):
                id_b, xb, yb = robot_positions[j]
                dist = math.sqrt((xa - xb)**2 + (ya - yb)**2)

                if dist < self.collision_threshold:
                    event = CollisionEvent(
                        timestamp=current_sim_time,
                        robot_a=id_a,
                        robot_b=id_b,
                        distance=dist,
                        location=((xa + xb) / 2.0, (ya + yb) / 2.0),
                        event_type="INTER_ROBOT"
                    )
                    self.collision_events.append(event)
                elif dist < self.near_miss_threshold:
                    event = CollisionEvent(
                        timestamp=current_sim_time,
                        robot_a=id_a,
                        robot_b=id_b,
                        distance=dist,
                        location=((xa + xb) / 2.0, (ya + yb) / 2.0),
                        event_type="NEAR_MISS"
                    )
                    self.near_miss_events.append(event)

    def record_task_started(self, task_id: str, sim_time: float):
        if task_id not in self.task_start_times:
            self.task_start_times[task_id] = sim_time

    def record_task_completed(self, task_id: str, sim_time: float):
        self.task_end_times[task_id] = sim_time

    def get_summary(
        self,
        total_fleet_distance: float,
        total_battery_used: float,
        priority_tie_breaks: int = 0,
        cbba_events: int = 0
    ) -> SimulationMetrics:
        """Compute aggregated run metrics."""
        tasks_done = len(self.task_end_times)
        time_min = max(0.01, self.sim_time / 60.0)
        throughput = tasks_done / time_min

        return SimulationMetrics(
            total_time_seconds=round(self.sim_time, 2),
            tasks_completed=tasks_done,
            total_tasks=self.total_tasks_count,
            collision_count=len(self.collision_events),
            near_miss_count=len(self.near_miss_events),
            deadlock_count=len(self.deadlock_events),
            total_fleet_distance_m=round(total_fleet_distance, 2),
            total_battery_consumed_pct=round(total_battery_used, 2),
            throughput_tasks_per_min=round(throughput, 2),
            priority_tie_breaks=priority_tie_breaks,
            cbba_consensus_events=cbba_events,
            collision_log=list(self.collision_events)
        )

