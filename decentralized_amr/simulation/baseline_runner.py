"""Baseline simulation runner for Stop-and-Wait comparative benchmarking."""

import math
from typing import Dict, List, Optional, Tuple

from decentralized_amr.config import DEFAULT_ROBOT_CONFIG, DEFAULT_WAREHOUSE_CONFIG, RobotConfig, WarehouseConfig
from decentralized_amr.robot.baseline_node import BaselineAMRNode, BaselineState
from decentralized_amr.simulation.metrics_collector import MetricsCollector, SimulationMetrics
from decentralized_amr.simulation.warehouse_map import WarehouseMap
from decentralized_amr.task_allocation.task import TaskStatus, WarehouseTask


class BaselineSimulator:
    """
    Runs the baseline Stop-and-Wait simulation on the identical warehouse map and task set.
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
        self.metrics = MetricsCollector(physical_radius=self.r_cfg.radius)
        self.robots: List[BaselineAMRNode] = []
        self.task_pool: Dict[str, WarehouseTask] = {}
        self.unassigned_tasks: List[WarehouseTask] = []
        self.sim_time = 0.0
        self.dt = self.r_cfg.control_dt

        self._init_fleet()

    def _init_fleet(self):
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
            init_x += (i // len(start_locations)) * 0.5
            robot = BaselineAMRNode(
                robot_id=i + 1,
                initial_pos=(init_x, init_y),
                initial_heading=init_th,
                config=self.r_cfg
            )
            self.robots.append(robot)

    def inject_tasks(self, tasks: List[WarehouseTask]):
        self.metrics.start_scenario(len(tasks))
        self.task_pool = {t.task_id: t for t in tasks}
        self.unassigned_tasks = list(tasks)

    def step(self) -> bool:
        """
        Advance baseline simulation by one tick.
        Returns True if tasks remain, False if all completed.
        """
        self.sim_time += self.dt

        # 1. Greedy Centralized Assignment for any idle robots
        for robot in self.robots:
            if robot.state == BaselineState.IDLE and not robot.current_task and self.unassigned_tasks:
                # Find nearest pickup task
                best_task_idx = 0
                min_dist = 1e9
                for idx, t in enumerate(self.unassigned_tasks):
                    d = math.sqrt((t.pickup_pos[0] - robot.x)**2 + (t.pickup_pos[1] - robot.y)**2)
                    if d < min_dist:
                        min_dist = d
                        best_task_idx = idx

                chosen_task = self.unassigned_tasks.pop(best_task_idx)
                robot.assign_task(chosen_task)

        # 2. Step each baseline robot with Stop-and-Wait checks
        peer_positions = [(r.robot_id, r.x, r.y, r.heading) for r in self.robots]
        for robot in self.robots:
            robot.step(self.dt, peer_positions)

        # 3. Collision tracking
        positions = [(r.robot_id, r.x, r.y) for r in self.robots]
        self.metrics.update(self.sim_time, positions)

        # 4. Check task completions
        for t in self.task_pool.values():
            if t.status == TaskStatus.COMPLETED and t.task_id not in self.metrics.task_end_times:
                self.metrics.record_task_completed(t.task_id, self.sim_time)

        # 5. Check termination
        all_completed = len(self.task_pool) > 0 and all(
            t.status == TaskStatus.COMPLETED for t in self.task_pool.values()
        )
        return not all_completed

    def run_until_complete(self, max_seconds: float = 300.0) -> SimulationMetrics:
        """Run simulation until completion or timeout."""
        while self.sim_time < max_seconds:
            still_running = self.step()
            if not still_running:
                break

        total_dist = sum(r.total_distance_traveled for r in self.robots)
        total_battery_used = sum(100.0 - r.battery for r in self.robots)
        return self.metrics.get_summary(total_dist, total_battery_used)
