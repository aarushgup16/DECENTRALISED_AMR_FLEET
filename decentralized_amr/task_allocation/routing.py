"""Grid-based path planning with A*, dynamic obstacle avoidance, and path smoothing."""

import heapq
import math
from typing import Dict, List, Optional, Set, Tuple
import numpy as np

from decentralized_amr.config import DEFAULT_WAREHOUSE_CONFIG, WarehouseConfig


class GridPlanner:
    """
    Local A* path planner running on edge AMR nodes.
    Supports dynamic obstacle injection, blocked-aisle re-routing, and waypoint smoothing.
    """

    def __init__(self, config: Optional[WarehouseConfig] = None, inflation_radius: float = 0.4):
        self.cfg = config or DEFAULT_WAREHOUSE_CONFIG
        self.resolution = self.cfg.grid_resolution
        self.width_m = self.cfg.width
        self.height_m = self.cfg.height
        self.cols = int(self.width_m / self.resolution)
        self.rows = int(self.height_m / self.resolution)
        self.inflation_radius = inflation_radius
        self.inflation_cells = max(1, int(self.inflation_radius / self.resolution))

        # Static occupancy grid: 0 = free, 1 = obstacle
        self.static_grid = np.zeros((self.rows, self.cols), dtype=np.uint8)
        self._init_static_obstacles()

        # Dynamic obstacles: dict[obstacle_id, (x_min, y_min, x_max, y_max)]
        self.dynamic_obstacles: Dict[str, Tuple[float, float, float, float]] = {}
        self._recompute_occupancy_grid()

    def _init_static_obstacles(self):
        """Mark warehouse borders and storage racks on the grid."""
        # 1. Borders
        self.static_grid[0, :] = 1
        self.static_grid[self.rows - 1, :] = 1
        self.static_grid[:, 0] = 1
        self.static_grid[:, self.cols - 1] = 1

        # 2. Racks
        for (x1, y1, x2, y2) in self.cfg.rack_obstacles:
            c1 = max(0, int(x1 / self.resolution))
            c2 = min(self.cols - 1, int(x2 / self.resolution))
            r1 = max(0, int(y1 / self.resolution))
            r2 = min(self.rows - 1, int(y2 / self.resolution))
            self.static_grid[r1:r2 + 1, c1:c2 + 1] = 1

    def _recompute_occupancy_grid(self):
        """Generate combined inflated occupancy grid."""
        raw_grid = np.copy(self.static_grid)

        # Overlay dynamic obstacles
        for (x1, y1, x2, y2) in self.dynamic_obstacles.values():
            c1 = max(0, int(x1 / self.resolution))
            c2 = min(self.cols - 1, int(x2 / self.resolution))
            r1 = max(0, int(y1 / self.resolution))
            r2 = min(self.rows - 1, int(y2 / self.resolution))
            raw_grid[r1:r2 + 1, c1:c2 + 1] = 1

        # Inflate obstacles by inflation_radius
        self.grid = np.copy(raw_grid)
        obs_r, obs_c = np.where(raw_grid == 1)
        for r, c in zip(obs_r, obs_c):
            r_min = max(0, r - self.inflation_cells)
            r_max = min(self.rows - 1, r + self.inflation_cells)
            c_min = max(0, c - self.inflation_cells)
            c_max = min(self.cols - 1, c + self.inflation_cells)
            self.grid[r_min:r_max + 1, c_min:c_max + 1] = 1

    def add_dynamic_obstacle(self, obstacle_id: str, bounds: Tuple[float, float, float, float]):
        """Add or update a dynamic obstacle (e.g. blocked aisle)."""
        self.dynamic_obstacles[obstacle_id] = bounds
        self._recompute_occupancy_grid()

    def remove_dynamic_obstacle(self, obstacle_id: str):
        """Remove a dynamic obstacle."""
        if obstacle_id in self.dynamic_obstacles:
            del self.dynamic_obstacles[obstacle_id]
            self._recompute_occupancy_grid()

    def clear_dynamic_obstacles(self):
        """Clear all dynamic obstacles."""
        self.dynamic_obstacles.clear()
        self._recompute_occupancy_grid()

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """Convert continuous (x, y) to (row, col)."""
        col = max(0, min(self.cols - 1, int(x / self.resolution)))
        row = max(0, min(self.rows - 1, int(y / self.resolution)))
        return (row, col)

    def grid_to_world(self, row: int, col: int) -> Tuple[float, float]:
        """Convert (row, col) to continuous center (x, y)."""
        x = (col + 0.5) * self.resolution
        y = (row + 0.5) * self.resolution
        return (x, y)

    def is_free(self, row: int, col: int) -> bool:
        """Check if grid cell is walkable."""
        if 0 <= row < self.rows and 0 <= col < self.cols:
            return self.grid[row, col] == 0
        return False

    def is_world_free(self, x: float, y: float) -> bool:
        """Check if world coordinate is walkable."""
        r, c = self.world_to_grid(x, y)
        return self.is_free(r, c)

    def plan_path(
        self,
        start: Tuple[float, float],
        goal: Tuple[float, float]
    ) -> Optional[List[Tuple[float, float]]]:
        """
        Compute an optimal collision-free path using A* and line-of-sight smoothing.
        Returns a list of continuous waypoints [(x1, y1), (x2, y2), ...] or None if unreachable.
        """
        start_rc = self.world_to_grid(start[0], start[1])
        goal_rc = self.world_to_grid(goal[0], goal[1])

        # If start or goal is in an obstacle cell, find closest free cell
        if not self.is_free(start_rc[0], start_rc[1]):
            start_rc = self._find_nearest_free(start_rc)
            if not start_rc:
                return None

        if not self.is_free(goal_rc[0], goal_rc[1]):
            goal_rc = self._find_nearest_free(goal_rc)
            if not goal_rc:
                return None

        if start_rc == goal_rc:
            return [start, goal]

        # A* Search
        # Priority queue item: (f_score, g_score, (r, c))
        frontier = []
        heapq.heappush(frontier, (0.0, 0.0, start_rc))
        came_from: Dict[Tuple[int, int], Tuple[int, int]] = {}
        g_scores: Dict[Tuple[int, int], float] = {start_rc: 0.0}

        # 8-connected neighbor offsets: (dr, dc, cost)
        neighbors = [
            (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
            (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)),
            (1, -1, math.sqrt(2)), (1, 1, math.sqrt(2))
        ]

        found = False
        while frontier:
            f, current_g, current = heapq.heappop(frontier)

            if current == goal_rc:
                found = True
                break

            if current_g > g_scores.get(current, float('inf')):
                continue

            cr, cc = current
            for dr, dc, cost in neighbors:
                nr, nc = cr + dr, cc + dc
                if not self.is_free(nr, nc):
                    continue

                # Prevent diagonal corner cutting
                if dr != 0 and dc != 0:
                    if not self.is_free(cr + dr, cc) or not self.is_free(cr, cc + dc):
                        continue

                tentative_g = current_g + cost
                next_cell = (nr, nc)

                if tentative_g < g_scores.get(next_cell, float('inf')):
                    g_scores[next_cell] = tentative_g
                    # Euclidean distance heuristic
                    h = math.sqrt((nr - goal_rc[0])**2 + (nc - goal_rc[1])**2)
                    heapq.heappush(frontier, (tentative_g + h, tentative_g, next_cell))
                    came_from[next_cell] = current

        if not found:
            return None

        # Reconstruct path
        path_cells = []
        curr = goal_rc
        while curr in came_from:
            path_cells.append(curr)
            curr = came_from[curr]
        path_cells.append(start_rc)
        path_cells.reverse()

        # Convert to world coordinates
        world_pts = [self.grid_to_world(r, c) for (r, c) in path_cells]
        world_pts[0] = start
        world_pts[-1] = goal

        # Smooth path with line-of-sight shortcutting
        smoothed = self._smooth_path(world_pts)
        return smoothed

    def _find_nearest_free(self, center_rc: Tuple[int, int], max_radius: int = 5) -> Optional[Tuple[int, int]]:
        """Find nearest walkable cell within search radius."""
        cr, cc = center_rc
        for rad in range(1, max_radius + 1):
            for dr in range(-rad, rad + 1):
                for dc in range(-rad, rad + 1):
                    nr, nc = cr + dr, cc + dc
                    if self.is_free(nr, nc):
                        return (nr, nc)
        return None

    def _has_line_of_sight(self, p1: Tuple[float, float], p2: Tuple[float, float]) -> bool:
        """Raycast check between p1 and p2 to verify unhindered visibility."""
        dist = math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)
        if dist < 1e-4:
            return True

        steps = max(2, int(dist / (self.resolution * 0.5)))
        for i in range(steps + 1):
            t = i / float(steps)
            x = p1[0] + t * (p2[0] - p1[0])
            y = p1[1] + t * (p2[1] - p1[1])
            if not self.is_world_free(x, y):
                return False
        return True

    def _smooth_path(self, points: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """Prune redundant collinear/visible waypoints for smooth motion."""
        if len(points) <= 2:
            return points

        smoothed = [points[0]]
        curr_idx = 0

        while curr_idx < len(points) - 1:
            furthest_idx = curr_idx + 1
            for next_idx in range(len(points) - 1, curr_idx, -1):
                if self._has_line_of_sight(points[curr_idx], points[next_idx]):
                    furthest_idx = next_idx
                    break
            smoothed.append(points[furthest_idx])
            curr_idx = furthest_idx

        return smoothed

    def compute_path_length(self, path: List[Tuple[float, float]]) -> float:
        """Compute total Euclidean distance of path."""
        if not path or len(path) < 2:
            return 0.0
        total = 0.0
        for i in range(len(path) - 1):
            p1 = path[i]
            p2 = path[i + 1]
            total += math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)
        return total

