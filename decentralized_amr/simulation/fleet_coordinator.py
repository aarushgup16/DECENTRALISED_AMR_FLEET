"""Multi-AMR Simulation Coordinator for decentralized and baseline fleet runs."""

import math
import time
from typing import Any, Callable, Dict, List, Optional, Tuple

from decentralized_amr.config import DEFAULT_ROBOT_CONFIG, DEFAULT_WAREHOUSE_CONFIG, RobotConfig, WarehouseConfig
from decentralized_amr.network.messages import ObstacleAlertMessage
from decentralized_amr.network.p2p_mesh import P2PMesh
from decentralized_amr.robot.amr_node import AMRNode, AMRState
from decentralized_amr.robot.baseline_node import BaselineAMRNode, BaselineState
from decentralized_amr.simulation.metrics_collector import MetricsCollector, SimulationMetrics
from decentralized_amr.simulation.warehouse_map import WarehouseMap
from decentralized_amr.task_allocation.task import TaskStatus, WarehouseTask


class FleetCoordinator:
    """
    Simulates the physical environment and multi-agent coordination.
    Coordinates physics steps, mesh telemetry, task injection, and metric tracking.
    """

    def __init__(
        self,
        num_robots: int = 4,
        warehouse_config: Optional[WarehouseConfig] = None,
        robot_config: Optional[RobotConfig] = None
    ):
        self.num_robots = num_robots
        self.w_cfg = warehouse_config or DEFAULT_WAREHOUSE_CONFIG
        self.r_cfg = robot_config or DEFAULT_ROBOT_CONFIG

        self.map = WarehouseMap(self.w_cfg)
        self.mesh = P2PMesh()
        self.metrics = MetricsCollector(physical_radius=self.r_cfg.radius)

        self.robots: List[AMRNode] = []
        self.task_pool: Dict[str, WarehouseTask] = {}
        self.sim_time = 0.0
        self.is_running = False
        self.dt = self.r_cfg.control_dt

        self._init_fleet()

    def _init_fleet(self):
        """Spawn AMR nodes at designated start positions with sufficient spacing."""
        self.robots.clear()
        start_locations = [
            (2.0, 5.0, 0.0),
            (8.5, 5.0, 0.0),
            (1.0, 10.0, 0.0),
            (21.5, 5.0, 0.0),
            (28.0, 5.0, 0.0),
            (29.0, 10.0, math.pi)
        ]

        for i in range(self.num_robots):
            init_x, init_y, init_th = start_locations[i % len(start_locations)]
            init_x += (i // len(start_locations)) * 1.5
            robot = AMRNode(
                robot_id=i + 1,
                initial_pos=(init_x, init_y),
                initial_heading=init_th,
                mesh=self.mesh,
                config=self.r_cfg
            )
            self.robots.append(robot)

    def add_task(self, task: WarehouseTask):
        """Inject a new task into the decentralized CBBA task pool across all robots."""
        self.task_pool[task.task_id] = task
        for robot in self.robots:
            robot.cbba.add_known_task(task)

    def inject_tasks(self, tasks: List[WarehouseTask]):
        """Inject a batch of tasks and run initial CBBA consensus."""
        self.metrics.start_scenario(len(tasks))
        for task in tasks:
            self.add_task(task)

        # Run initial decentralized auction consensus rounds
        for _ in range(5):
            for robot in self.robots:
                changed = robot.cbba.build_bundle((robot.x, robot.y))
                if changed:
                    robot.p2p_node.broadcast(robot.cbba.create_bid_message())
            for robot in self.robots:
                robot._update_task_execution()

    def inject_blocked_aisle(self, obs_id: str, bounds: Tuple[float, float, float, float]):
        """
        Dynamically block an aisle and broadcast alert across P2P mesh.
        """
        self.map.add_dynamic_obstacle(obs_id, bounds)
        alert = ObstacleAlertMessage(
            sender_id=0,
            lamport_clock=1,
            obstacle_id=obs_id,
            x_min=bounds[0],
            y_min=bounds[1],
            x_max=bounds[2],
            y_max=bounds[3],
            detected_by=0
        )
        for robot in self.robots:
            robot.p2p_node.receive(alert)

    def step(self) -> bool:
        """
        Advance simulation by one physics/control tick (dt = 0.05s).
        Returns True if tasks remain, False if all tasks completed.
        """
        self.sim_time += self.dt
        static_segs = self.map.get_all_obstacle_segments()

        # Gather live perceived peers for full decentralized sensing
        live_peers = [
            (
                r.robot_id,
                (r.x, r.y),
                (r.linear_v * math.cos(r.heading), r.linear_v * math.sin(r.heading)),
                r.heading,
                r.cfg.radius,
                r.get_current_priority()
            )
            for r in self.robots
        ]

        # Step each autonomous robot
        for robot in self.robots:
            robot.step(self.dt, static_obstacle_segments=static_segs, live_peers=live_peers)

        # Broadcast telemetry periodically (every 2 ticks = 10 Hz)
        if int(self.sim_time / self.dt) % 2 == 0:
            for robot in self.robots:
                robot.broadcast_telemetry()

        # Track robot positions and check for collisions
        positions = [(r.robot_id, r.x, r.y) for r in self.robots]
        self.metrics.update(self.sim_time, positions)

        # Check completed tasks
        for r in self.robots:
            for t_id in list(r.cbba.completed_tasks):
                if t_id in self.task_pool and t_id not in self.metrics.task_end_times:
                    self.task_pool[t_id].status = TaskStatus.COMPLETED
                    self.metrics.record_task_completed(t_id, self.sim_time)

        # Check termination (all tasks completed)
        all_completed = len(self.task_pool) > 0 and all(
            t.status == TaskStatus.COMPLETED for t in self.task_pool.values()
        )
        return not all_completed

    def get_snapshot(self) -> Dict[str, Any]:
        """Generate full state JSON snapshot for passive Dashboard WebSocket."""
        return {
            "sim_time": round(self.sim_time, 2),
            "robots": [
                {
                    "id": r.robot_id,
                    "x": round(r.x, 3),
                    "y": round(r.y, 3),
                    "heading": round(r.heading, 3),
                    "linear_v": round(r.linear_v, 2),
                    "angular_w": round(r.angular_w, 2),
                    "battery": round(r.battery, 1),
                    "state": r.state.value,
                    "current_task": r.current_task.task_id if r.current_task else None,
                    "priority_score": round(r.get_current_priority(), 1),
                    "lamport_clock": r.clock.get_time(),
                    "consecutive_yields": r.consecutive_yields,
                    "bundle": list(r.cbba.bundle),
                    "path": [(round(px, 2), round(py, 2)) for px, py in r.path[r.active_waypoint_idx:r.active_waypoint_idx + 6]]
                }
                for r in self.robots
            ],
            "tasks": [
                {
                    "id": t.task_id,
                    "pickup": [round(t.pickup_pos[0], 1), round(t.pickup_pos[1], 1)],
                    "dropoff": [round(t.dropoff_pos[0], 1), round(t.dropoff_pos[1], 1)],
                    "urgency": t.urgency,
                    "status": t.status.value,
                    "assigned_to": t.assigned_robot_id
                }
                for t in self.task_pool.values()
            ],
            "dynamic_obstacles": [
                {"id": k, "bounds": [round(v, 2) for v in val]}
                for k, val in self.map.dynamic_obstacles.items()
            ],
            "metrics": {
                "collision_count": len(self.metrics.collision_events),
                "near_miss_count": len(self.metrics.near_miss_events),
                "completed_tasks": len(self.metrics.task_end_times),
                "total_tasks": len(self.task_pool),
                "fleet_distance": round(sum(r.total_distance_traveled for r in self.robots), 1),
                "priority_tie_breaks": sum(r.priority_tie_breaks_won + r.priority_tie_breaks_lost for r in self.robots)
            }
        }

    def get_metrics_summary(self) -> SimulationMetrics:
        """Return final run metrics summary."""
        total_dist = sum(r.total_distance_traveled for r in self.robots)
        total_battery_used = sum(100.0 - r.battery for r in self.robots)
        tie_breaks = sum(r.priority_tie_breaks_won + r.priority_tie_breaks_lost for r in self.robots)
        return self.metrics.get_summary(total_dist, total_battery_used, priority_tie_breaks=tie_breaks)
