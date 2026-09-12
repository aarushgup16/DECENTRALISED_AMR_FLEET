"""System configuration and physical constants for the Decentralized AMR Fleet."""

from dataclasses import dataclass, field
from typing import List, Tuple


@dataclass(frozen=True)
class RobotConfig:
    """Kinematic and physical constraints of each AMR (differential-drive)."""
    radius: float = 0.35              # Physical robot radius in meters (0.7m diameter)
    inflation_epsilon: float = 0.20   # Generous safety margin epsilon for non-holonomic tracking
    
    @property
    def effective_radius(self) -> float:
        """Effective inflated radius R = r + epsilon = 0.55m."""
        return self.radius + self.inflation_epsilon

    max_linear_speed: float = 1.2      # m/s
    max_linear_accel: float = 1.5      # m/s^2
    max_angular_speed: float = 2.5     # rad/s
    max_angular_accel: float = 4.0     # rad/s^2
    
    sensing_range: float = 8.0         # LiDAR / neighbor perception range (meters)
    comm_range: float = 25.0           # Direct P2P wireless broadcast radius (meters)
    time_horizon_orca: float = 3.0     # Lookahead horizon tau for NH-ORCA in seconds
    control_dt: float = 0.05           # Control loop step (20 Hz)

    # Battery model
    battery_capacity: float = 100.0    # 100%
    drain_moving_per_sec: float = 0.03 # % per sec
    drain_idle_per_sec: float = 0.005  # % per sec
    charge_per_sec: float = 0.3        # % per sec
    battery_low_threshold: float = 20.0# Low battery return-to-charge


@dataclass(frozen=True)
class PriorityConfig:
    """Weights for the Bounded Priority Tie-Breaker."""
    weight_urgency: float = 50000.0
    weight_yield: float = 10000.0
    max_yield_bonus: float = 20000.0   # Hard cap on yield bonus to prevent starvation
    weight_goal: float = 100.0
    weight_battery: float = 10.0


@dataclass
class WarehouseConfig:
    """Warehouse dimensions and layout configuration."""
    width: float = 30.0                # meters (X dimension)
    height: float = 20.0               # meters (Y dimension)
    grid_resolution: float = 0.5       # grid cell size in meters (60 x 40 grid)

    # Storage racks as list of [x_min, y_min, x_max, y_max]
    rack_obstacles: List[Tuple[float, float, float, float]] = field(default_factory=lambda: [
        # Left storage bay
        (4.0, 3.5, 7.0, 16.5),
        (10.0, 3.5, 13.0, 16.5),
        
        # Right storage bay
        (17.0, 3.5, 20.0, 16.5),
        (23.0, 3.5, 26.0, 16.5),
    ])

    # Narrow choke points (1-lane passages where tie-breaking occurs)
    choke_points: List[Tuple[float, float, float, float]] = field(default_factory=lambda: [
        (7.0, 9.0, 10.0, 11.0),    # Central cross-aisle choke 1
        (13.0, 9.0, 17.0, 11.0),   # Central main intersection choke 2
        (20.0, 9.0, 23.0, 11.0),   # Central cross-aisle choke 3
    ])

    # Pickup workstations (bottom docks)
    pickup_locations: List[Tuple[float, float]] = field(default_factory=lambda: [
        (2.0, 1.5),
        (8.5, 1.5),
        (15.0, 1.5),
        (21.5, 1.5),
        (28.0, 1.5),
    ])

    # Dropoff conveyor docks (top docks)
    dropoff_locations: List[Tuple[float, float]] = field(default_factory=lambda: [
        (2.0, 18.5),
        (8.5, 18.5),
        (15.0, 18.5),
        (21.5, 18.5),
        (28.0, 18.5),
    ])

    # Charging bays
    charging_locations: List[Tuple[float, float]] = field(default_factory=lambda: [
        (1.0, 10.0),
        (29.0, 10.0),
    ])


DEFAULT_ROBOT_CONFIG = RobotConfig()
DEFAULT_PRIORITY_CONFIG = PriorityConfig()
DEFAULT_WAREHOUSE_CONFIG = WarehouseConfig()

