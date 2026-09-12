"""O(1) Bitset Reservation Table for spatial-temporal conflict checks on edge devices."""

import numpy as np
from typing import List, Optional, Tuple


class BitsetReservationTable:
    """
    Spatiotemporal grid reservation table using flat 64-bit integer bitmasks.
    
    Provides O(1) bitwise AND conflict checking across short lookahead horizons.
    Ideal for memory-constrained edge hardware (Jetson / RPi) with small L1/L2 caches.
    """

    def __init__(
        self,
        grid_width: int = 60,
        grid_height: int = 40,
        resolution: float = 0.5,
        horizon_steps: int = 10,
        time_step_duration: float = 0.5
    ):
        self.grid_width = grid_width
        self.grid_height = grid_height
        self.resolution = resolution
        self.horizon_steps = horizon_steps
        self.time_step_duration = time_step_duration
        self.total_cells = grid_width * grid_height
        self.words_per_step = (self.total_cells + 63) // 64

        # Array of shape (horizon_steps, words_per_step) of type uint64
        self.bitset = np.zeros((self.horizon_steps, self.words_per_step), dtype=np.uint64)

    def clear(self):
        """Reset all reservations."""
        self.bitset.fill(0)

    def coords_to_cell(self, x: float, y: float) -> Optional[int]:
        """Convert continuous coordinates (x, y) to flat grid cell index."""
        col = int(x / self.resolution)
        row = int(y / self.resolution)
        if 0 <= col < self.grid_width and 0 <= row < self.grid_height:
            return row * self.grid_width + col
        return None

    def cell_to_coords(self, cell_idx: int) -> Tuple[float, float]:
        """Convert flat grid cell index back to continuous center coordinates."""
        row = cell_idx // self.grid_width
        col = cell_idx % self.grid_width
        x = (col + 0.5) * self.resolution
        y = (row + 0.5) * self.resolution
        return (x, y)

    def set_cell(self, step: int, x: float, y: float, footprint_radius: float = 0.4):
        """
        Mark a spatial cell (and adjacent cells within robot footprint radius)
        as reserved at discrete time step `step`.
        """
        if not (0 <= step < self.horizon_steps):
            return

        min_col = max(0, int((x - footprint_radius) / self.resolution))
        max_col = min(self.grid_width - 1, int((x + footprint_radius) / self.resolution))
        min_row = max(0, int((y - footprint_radius) / self.resolution))
        max_row = min(self.grid_height - 1, int((y + footprint_radius) / self.resolution))

        for r in range(min_row, max_row + 1):
            for c in range(min_col, max_col + 1):
                idx = r * self.grid_width + c
                word_idx = idx // 64
                bit_idx = idx % 64
                self.bitset[step, word_idx] |= (np.uint64(1) << np.uint64(bit_idx))

    def set_trajectory(
        self,
        waypoints: List[Tuple[float, float]],
        nominal_speed: float = 1.0,
        footprint_radius: float = 0.4
    ):
        """
        Project intended trajectory waypoints across future time horizon steps.
        """
        self.clear()
        if not waypoints:
            return

        curr_x, curr_y = waypoints[0]
        wp_idx = 0
        total_dist_covered = 0.0

        for step in range(self.horizon_steps):
            t_future = step * self.time_step_duration
            target_dist = t_future * nominal_speed

            # Interpolate position along waypoint polyline
            while wp_idx < len(waypoints) - 1:
                p1 = waypoints[wp_idx]
                p2 = waypoints[wp_idx + 1]
                seg_len = ((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)**0.5

                if seg_len < 1e-4:
                    wp_idx += 1
                    continue

                if total_dist_covered + seg_len >= target_dist:
                    remain = target_dist - total_dist_covered
                    ratio = remain / seg_len
                    curr_x = p1[0] + ratio * (p2[0] - p1[0])
                    curr_y = p1[1] + ratio * (p2[1] - p1[1])
                    break
                else:
                    total_dist_covered += seg_len
                    wp_idx += 1
                    curr_x, curr_y = p2

            self.set_cell(step, curr_x, curr_y, footprint_radius)

    def has_conflict(self, other: "BitsetReservationTable") -> bool:
        """
        Perform O(1) bitwise AND check against another robot's reservation table.
        Returns True if any spatiotemporal overlap is detected.
        """
        # Element-wise bitwise AND across the entire bitset array
        overlap = np.bitwise_and(self.bitset, other.bitset)
        return bool(np.any(overlap != 0))

    def get_conflict_steps(self, other: "BitsetReservationTable") -> List[int]:
        """Return list of time step indices where conflicts occur."""
        overlap = np.bitwise_and(self.bitset, other.bitset)
        conflicts = []
        for step in range(self.horizon_steps):
            if np.any(overlap[step] != 0):
                conflicts.append(step)
        return conflicts

