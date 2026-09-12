"""Warehouse Map geometry, obstacle segments, and task generation."""

import random
from typing import Dict, List, Optional, Tuple
from decentralized_amr.config import DEFAULT_WAREHOUSE_CONFIG, WarehouseConfig
from decentralized_amr.task_allocation.task import WarehouseTask


class WarehouseMap:
    """
    Continuous 2D Warehouse Environment.
    Defines storage racks, 1-lane choke points, pickup docks, dropoff conveyors,
    charging bays, and dynamic obstacle management.
    """

    def __init__(self, config: Optional[WarehouseConfig] = None):
        self.cfg = config or DEFAULT_WAREHOUSE_CONFIG
        self.width = self.cfg.width
        self.height = self.cfg.height
        self.racks = self.cfg.rack_obstacles
        self.choke_regions = self.cfg.choke_points
        self.pickups = self.cfg.pickup_locations
        self.dropoffs = self.cfg.dropoff_locations
        self.charging_bays = self.cfg.charging_locations

        # Dynamic obstacles: dict[obs_id, (x_min, y_min, x_max, y_max)]
        self.dynamic_obstacles: Dict[str, Tuple[float, float, float, float]] = {}

        # Cached static line segments for NH-ORCA solver
        self.static_segments = self._generate_obstacle_segments()

    def _generate_obstacle_segments(self) -> List[Tuple[Tuple[float, float], Tuple[float, float]]]:
        """Convert bounding boxes of walls and racks into line segments."""
        segs: List[Tuple[Tuple[float, float], Tuple[float, float]]] = []

        # 1. Warehouse border walls
        w, h = self.width, self.height
        segs.append(((0.0, 0.0), (w, 0.0)))
        segs.append(((w, 0.0), (w, h)))
        segs.append(((w, h), (0.0, h)))
        segs.append(((0.0, h), (0.0, 0.0)))

        # 2. Storage racks
        for (x1, y1, x2, y2) in self.racks:
            segs.append(((x1, y1), (x2, y1)))
            segs.append(((x2, y1), (x2, y2)))
            segs.append(((x2, y2), (x1, y2)))
            segs.append(((x1, y2), (x1, y1)))

        return segs

    def get_all_obstacle_segments(self) -> List[Tuple[Tuple[float, float], Tuple[float, float]]]:
        """Return combined static + dynamic line segments for NH-ORCA."""
        segs = list(self.static_segments)
        for (x1, y1, x2, y2) in self.dynamic_obstacles.values():
            segs.append(((x1, y1), (x2, y1)))
            segs.append(((x2, y1), (x2, y2)))
            segs.append(((x2, y2), (x1, y2)))
            segs.append(((x1, y2), (x1, y1)))
        return segs

    def add_dynamic_obstacle(self, obs_id: str, bounds: Tuple[float, float, float, float]):
        """Add dynamic obstacle (e.g. blocked aisle)."""
        self.dynamic_obstacles[obs_id] = bounds

    def remove_dynamic_obstacle(self, obs_id: str):
        """Remove dynamic obstacle."""
        self.dynamic_obstacles.pop(obs_id, None)

    def is_in_choke_point(self, x: float, y: float) -> bool:
        """Check if (x, y) is inside any narrow single-lane choke region."""
        for (x1, y1, x2, y2) in self.choke_regions:
            if x1 <= x <= x2 and y1 <= y <= y2:
                return True
        return False

    def generate_random_tasks(self, num_tasks: int = 10, seed: Optional[int] = 42) -> List[WarehouseTask]:
        """Generate a deterministic or randomized batch of warehouse transport tasks."""
        if seed is not None:
            rng = random.Random(seed)
        else:
            rng = random.Random()

        tasks: List[WarehouseTask] = []
        for i in range(num_tasks):
            pickup = rng.choice(self.pickups)
            dropoff = rng.choice(self.dropoffs)
            # Ensure pickup != dropoff
            while dropoff == pickup:
                dropoff = rng.choice(self.dropoffs)

            urgency = rng.choice([1, 1, 1, 2, 2, 3, 4])  # Weighted distribution
            task = WarehouseTask(
                task_id=f"TASK-{i+1:03d}",
                pickup_pos=pickup,
                dropoff_pos=dropoff,
                urgency=urgency,
                base_reward=100.0 + urgency * 25.0
            )
            tasks.append(task)
        return tasks

