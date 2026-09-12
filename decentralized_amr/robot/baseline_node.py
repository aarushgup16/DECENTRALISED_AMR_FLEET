"""Baseline AMR Node with Stop-and-Wait collision handling for comparative benchmarking."""

import math
import time
from enum import Enum
from typing import Dict, List, Optional, Tuple

from decentralized_amr.config import DEFAULT_ROBOT_CONFIG, RobotConfig
from decentralized_amr.task_allocation.routing import GridPlanner
from decentralized_amr.task_allocation.task import TaskStatus, WarehouseTask


class BaselineState(str, Enum):
    IDLE = "IDLE"
    MOVING_TO_PICKUP = "MOVING_TO_PICKUP"
    PICKING = "PICKING"
    MOVING_TO_DROPOFF = "MOVING_TO_DROPOFF"
    DROPPING = "DROPPING"
    STOPPED_WAITING = "STOPPED_WAITING"


class BaselineAMRNode:
    """
    Baseline robot node implementing standard industry Stop-and-Wait logic.
    - Naive greedy nearest-task assignment.
    - Full-stop upon detecting any obstacle or robot in safety radius / forward path.
    - Waits until path is clear before resuming.
    """

    def __init__(
        self,
        robot_id: int,
        initial_pos: Tuple[float, float],
        initial_heading: float = 0.0,
        config: Optional[RobotConfig] = None
    ):
        self.robot_id = robot_id
        self.cfg = config or DEFAULT_ROBOT_CONFIG

        self.x = initial_pos[0]
        self.y = initial_pos[1]
        self.heading = initial_heading
        self.linear_v = 0.0
        self.angular_w = 0.0

        self.battery = self.cfg.battery_capacity
        self.state = BaselineState.IDLE
        self.current_task: Optional[WarehouseTask] = None
        self.path: List[Tuple[float, float]] = []
        self.active_waypoint_idx = 0
        self.dwell_timer = 0.0
        self.stop_wait_timer = 0.0
        self.total_distance_traveled = 0.0
        self.tasks_completed_count = 0
        self.total_time_stopped = 0.0

        self.planner = GridPlanner(inflation_radius=self.cfg.effective_radius)

    def assign_task(self, task: WarehouseTask):
        """Assign task directly from centralized/naive queue."""
        self.current_task = task
        self.current_task.mark_assigned(self.robot_id)
        self.current_task.mark_in_progress()
        self.state = BaselineState.MOVING_TO_PICKUP
        self._plan_path_to(task.pickup_pos)

    def _plan_path_to(self, goal: Tuple[float, float]):
        waypoints = self.planner.plan_path((self.x, self.y), goal)
        if waypoints:
            self.path = waypoints
            self.active_waypoint_idx = 0
        else:
            self.path.clear()
            self.active_waypoint_idx = 0

    def step(self, dt: float, peer_positions: List[Tuple[int, float, float, float]]):
        """
        Execute one control step with Stop-and-Wait logic.
        peer_positions: list of (peer_id, x, y, heading)
        """
        # Battery drain
        if self.linear_v > 0.05:
            self.battery = max(0.0, self.battery - (self.cfg.drain_moving_per_sec * dt))
        else:
            self.battery = max(0.0, self.battery - (self.cfg.drain_idle_per_sec * dt))

        if not self.current_task or self.state == BaselineState.IDLE:
            self.linear_v = 0.0
            self.angular_w = 0.0
            return

        # Dwell handling
        if self.state in (BaselineState.PICKING, BaselineState.DROPPING):
            self.dwell_timer -= dt
            self.linear_v = 0.0
            self.angular_w = 0.0
            if self.dwell_timer <= 0.0:
                if self.state == BaselineState.PICKING:
                    self.state = BaselineState.MOVING_TO_DROPOFF
                    self._plan_path_to(self.current_task.dropoff_pos)
                elif self.state == BaselineState.DROPPING:
                    self.current_task.mark_completed()
                    self.tasks_completed_count += 1
                    self.current_task = None
                    self.state = BaselineState.IDLE
            return

        # Stop-and-Wait proximity check
        # Check if another peer robot is within safety braking distance (1.6m) in front
        must_stop = False
        for (peer_id, px, py, p_heading) in peer_positions:
            if peer_id == self.robot_id:
                continue
            dist = math.sqrt((px - self.x)**2 + (py - self.y)**2)
            if dist < 1.6:
                # Check if in forward sector
                dx = px - self.x
                dy = py - self.y
                forward_x = math.cos(self.heading)
                forward_y = math.sin(self.heading)
                dot = (dx * forward_x + dy * forward_y) / max(1e-4, dist)
                if dot > 0.3 or dist < 0.9:
                    # Lower ID yields/waits to prevent symmetric deadlock in baseline
                    if self.robot_id < peer_id or dist < 0.9:
                        must_stop = True
                        break

        if must_stop:
            self.linear_v = 0.0
            self.angular_w = 0.0
            self.total_time_stopped += dt
            return

        # Waypoint navigation
        if not self.path or self.active_waypoint_idx >= len(self.path):
            self.linear_v = 0.0
            self.angular_w = 0.0
            return

        target_wp = self.path[self.active_waypoint_idx]
        dist_to_wp = math.sqrt((target_wp[0] - self.x)**2 + (target_wp[1] - self.y)**2)

        if dist_to_wp < 0.35 and self.active_waypoint_idx < len(self.path) - 1:
            self.active_waypoint_idx += 1
            target_wp = self.path[self.active_waypoint_idx]
            dist_to_wp = math.sqrt((target_wp[0] - self.x)**2 + (target_wp[1] - self.y)**2)

        # Destination arrival
        if self.active_waypoint_idx == len(self.path) - 1 and dist_to_wp < 0.4:
            if self.state == BaselineState.MOVING_TO_PICKUP:
                self.state = BaselineState.PICKING
                self.dwell_timer = 0.5
                self.linear_v = 0.0
                self.angular_w = 0.0
                return
            elif self.state == BaselineState.MOVING_TO_DROPOFF:
                self.state = BaselineState.DROPPING
                self.dwell_timer = 0.5
                self.linear_v = 0.0
                self.angular_w = 0.0
                return

        # Simple proportional heading and speed control
        target_angle = math.atan2(target_wp[1] - self.y, target_wp[0] - self.x)
        angle_diff = (target_angle - self.heading + math.pi) % (2.0 * math.pi) - math.pi

        self.angular_w = max(-self.cfg.max_angular_speed, min(self.cfg.max_angular_speed, angle_diff * 3.0))
        
        if abs(angle_diff) > 0.6:
            # Turn in place if heading is off
            self.linear_v = 0.0
        else:
            self.linear_v = min(self.cfg.max_linear_speed, max(0.2, dist_to_wp * 1.2))

        # Integrate kinematics
        dx = self.linear_v * math.cos(self.heading) * dt
        dy = self.linear_v * math.sin(self.heading) * dt
        self.x += dx
        self.y += dy
        self.heading = (self.heading + self.angular_w * dt) % (2.0 * math.pi)
        self.total_distance_traveled += math.sqrt(dx * dx + dy * dy)

