"""Unit and kinematic tests for Non-Holonomic ORCA collision avoidance."""

import math
import pytest
import numpy as np

from decentralized_amr.config import RobotConfig
from decentralized_amr.collision_avoidance.nh_orca import NHORCASolver


def test_nh_orca_head_on_avoidance():
    """Verify two AMRs in head-on trajectory avoid collision without stopping."""
    cfg = RobotConfig(radius=0.35, inflation_epsilon=0.15) # Effective radius = 0.50m
    solver = NHORCASolver(cfg)

    # Robot 1 at (0, 0) moving towards (10, 0)
    p1 = (0.0, 0.0)
    v1 = (1.0, 0.0)
    th1 = 0.0

    # Robot 2 at (4.0, 0.0) moving towards (-6.0, 0.0)
    p2 = (4.0, 0.0)
    v2 = (-1.0, 0.0)

    # Preferred velocity for Robot 1 is straight towards goal (1.0, 0.0)
    v_pref = (1.0, 0.0)
    neighbors = [(p2, v2, cfg.radius)]

    v_cmd, w_cmd, v_safe = solver.compute_nh_control(
        pos_i=p1,
        vel_i=v1,
        heading_i=th1,
        v_current=1.0,
        w_current=0.0,
        v_pref_vec=v_pref,
        neighbors=neighbors,
        dt=0.05
    )

    # Robot 1 should not come to a complete stop; it should deflect lateral velocity
    assert v_cmd > 0.1, "NH-ORCA should maintain continuous forward motion"
    # Safe velocity vector should steer away from the direct head-on line (vy != 0 or w_cmd != 0)
    assert abs(v_safe[1]) > 0.01 or abs(w_cmd) > 0.01, "ORCA must produce lateral steering to avoid collision"


def test_nh_orca_kinematic_limits():
    """Verify resulting control commands strictly adhere to differential-drive limits."""
    cfg = RobotConfig(max_linear_speed=1.2, max_angular_speed=2.0, max_linear_accel=1.2, max_angular_accel=3.0)
    solver = NHORCASolver(cfg)

    # Demanding high velocity
    v_cmd, w_cmd, v_safe = solver.compute_nh_control(
        pos_i=(0.0, 0.0),
        vel_i=(0.0, 0.0),
        heading_i=0.0,
        v_current=0.0,
        w_current=0.0,
        v_pref_vec=(5.0, 5.0), # Extreme request
        neighbors=[],
        dt=0.05
    )

    # Linear speed limit & acceleration limit
    assert 0.0 <= v_cmd <= cfg.max_linear_speed
    assert abs(v_cmd - 0.0) <= cfg.max_linear_accel * 0.05 + 1e-4
    assert abs(w_cmd) <= cfg.max_angular_speed
    assert abs(w_cmd - 0.0) <= cfg.max_angular_accel * 0.05 + 1e-4


def test_nh_orca_static_obstacle_avoidance():
    """Verify ORCA deflects away from static obstacle segment."""
    cfg = RobotConfig()
    solver = NHORCASolver(cfg)

    p_robot = (5.0, 5.0)
    v_robot = (1.0, 0.0)
    th_robot = 0.0

    # Obstacle segment blocking path at x=6.0, y in [4.0, 6.0]
    obs_segments = [((6.0, 4.0), (6.0, 6.0))]

    v_cmd, w_cmd, v_safe = solver.compute_nh_control(
        pos_i=p_robot,
        vel_i=v_robot,
        heading_i=th_robot,
        v_current=1.0,
        w_current=0.0,
        v_pref_vec=(1.0, 0.0),
        neighbors=[],
        obstacles=obs_segments,
        dt=0.05
    )

    # Velocity in X must be reduced or deflected in Y to avoid penetrating wall
    assert v_safe[0] < 0.95 or abs(v_safe[1]) > 0.05

