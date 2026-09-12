"""Non-Holonomic Optimal Reciprocal Collision Avoidance (NH-ORCA).

Implements:
1. 2D Linear Programming for ORCA half-plane optimization (Seidel's method / 2D LP).
2. Velocity Obstacle half-planes for reciprocal multi-agent collision avoidance (with dynamic responsibility weight for stationary/dwelling agents).
3. Velocity Obstacle half-planes for static obstacle boundaries.
4. Non-holonomic projection mapping holonomic safe velocity (vx, vy) to differential-drive (v, w)
   with radius inflation R = r + epsilon for formal safety guarantees.
"""

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple
import numpy as np

from decentralized_amr.config import DEFAULT_ROBOT_CONFIG, RobotConfig


def cross_2d(a: np.ndarray, b: np.ndarray) -> float:
    """2D scalar cross product (a_x * b_y - a_y * b_x)."""
    return float(a[0] * b[1] - a[1] * b[0])


@dataclass
class HalfPlaneLine:
    """Represents a directed line for linear half-plane constraints: (v - point) · normal >= 0."""
    point: np.ndarray     # Point on the line (x, y)
    direction: np.ndarray # Unit direction vector (dx, dy)
    normal: np.ndarray    # Inward unit normal (nx, ny) pointing towards safe half-plane


class NHORCASolver:
    """
    Non-Holonomic ORCA Solver.
    Computes reciprocal, collision-free control commands for differential-drive AMRs.
    """

    def __init__(self, config: Optional[RobotConfig] = None):
        self.cfg = config or DEFAULT_ROBOT_CONFIG
        self.epsilon = self.cfg.inflation_epsilon
        self.effective_radius = self.cfg.effective_radius
        self.time_horizon = self.cfg.time_horizon_orca
        self.max_speed = self.cfg.max_linear_speed
        self.max_angular_speed = self.cfg.max_angular_speed
        self.max_linear_accel = self.cfg.max_linear_accel
        self.max_angular_accel = self.cfg.max_angular_accel

    def compute_nh_control(
        self,
        pos_i: Tuple[float, float],
        vel_i: Tuple[float, float],
        heading_i: float,
        v_current: float,
        w_current: float,
        v_pref_vec: Tuple[float, float],
        neighbors: List[Tuple[Tuple[float, float], Tuple[float, float], float]], # list of (pos_j, vel_j, radius_j)
        obstacles: Optional[List[Tuple[Tuple[float, float], Tuple[float, float]]]] = None,
        dt: float = 0.05
    ) -> Tuple[float, float, Tuple[float, float]]:
        """
        Compute safe (v, w) commands for differential-drive AMR.
        Returns: (v_cmd, w_cmd, (v_safe_x, v_safe_y))
        """
        p_i = np.array(pos_i, dtype=np.float64)
        v_i = np.array(vel_i, dtype=np.float64)
        v_pref = np.array(v_pref_vec, dtype=np.float64)

        # Cap preferred velocity to max speed
        speed_pref = float(np.linalg.norm(v_pref))
        if speed_pref > self.max_speed:
            v_pref = (v_pref / speed_pref) * self.max_speed

        orca_lines: List[HalfPlaneLine] = []

        # 1. Compute ORCA lines for static obstacle segments
        if obstacles:
            for seg in obstacles:
                p1 = np.array(seg[0], dtype=np.float64)
                p2 = np.array(seg[1], dtype=np.float64)
                line = self._compute_obstacle_orca_line(p_i, v_i, p1, p2, self.effective_radius, self.time_horizon, dt)
                if line:
                    orca_lines.append(line)

        # 2. Compute Reciprocal ORCA lines for neighboring AMRs
        for (pos_j, vel_j, radius_j) in neighbors:
            p_j = np.array(pos_j, dtype=np.float64)
            v_j = np.array(vel_j, dtype=np.float64)
            r_j_eff = radius_j + self.epsilon
            combined_radius = self.effective_radius + r_j_eff
            
            line = self._compute_agent_orca_line(p_i, v_i, p_j, v_j, combined_radius, self.time_horizon, dt)
            if line:
                orca_lines.append(line)

        # 3. Solve 2D Linear Program to find optimal holonomic velocity v_opt
        v_opt = self._solve_2d_lp(orca_lines, self.max_speed, v_pref)

        # 4. Differential-Drive Non-Holonomic Controller:
        # Translates optimal holonomic velocity vector (vx, vy) into (v, w) commands
        speed_opt = float(np.linalg.norm(v_opt))
        if speed_opt < 0.02:
            v_cmd = 0.0
            w_target = 0.0
            max_dw = self.max_angular_accel * dt
            w_cmd = w_current + max(-max_dw, min(max_dw, w_target - w_current))
        else:
            target_heading = math.atan2(v_opt[1], v_opt[0])
            heading_err = (target_heading - heading_i + math.pi) % (2.0 * math.pi) - math.pi

            if abs(heading_err) > math.pi * 0.35:
                # In-place turnaround: zero forward drive
                v_cmd = 0.0
                sign_w = 1.0 if heading_err > 0 else -1.0
                w_target = sign_w * min(self.max_angular_speed, max(1.5, abs(heading_err) * 3.0))
                max_dw = self.max_angular_accel * dt
                w_cmd = w_current + max(-max_dw, min(max_dw, w_target - w_current))
            else:
                # Smooth coordinated driving
                w_target = max(-self.max_angular_speed, min(self.max_angular_speed, heading_err * 3.5))
                v_target = min(self.max_speed, speed_opt) * max(0.0, math.cos(heading_err))

                max_dv = self.max_linear_accel * dt
                max_dw = self.max_angular_accel * dt

                v_cmd = v_current + max(-max_dv, min(max_dv, v_target - v_current))
                w_cmd = w_current + max(-max_dw, min(max_dw, w_target - w_current))

        # Resulting safe vector
        v_safe_x = v_cmd * math.cos(heading_i)
        v_safe_y = v_cmd * math.sin(heading_i)

        return float(v_cmd), float(w_cmd), (float(v_safe_x), float(v_safe_y))

    def _compute_agent_orca_line(
        self,
        p_i: np.ndarray,
        v_i: np.ndarray,
        p_j: np.ndarray,
        v_j: np.ndarray,
        combined_radius: float,
        tau: float,
        dt: float
    ) -> Optional[HalfPlaneLine]:
        """Compute reciprocal ORCA half-plane for a neighboring agent."""
        rel_pos = p_j - p_i
        rel_vel = v_i - v_j
        dist_sq = float(np.dot(rel_pos, rel_pos))
        combined_radius_sq = combined_radius * combined_radius

        speed_j = float(np.linalg.norm(v_j))
        weight_i = 1.0 if speed_j < 0.08 else 0.5

        if dist_sq > combined_radius_sq:
            # No current overlap
            w = rel_vel - (rel_pos / tau)
            w_len_sq = float(np.dot(w, w))
            dot_product1 = float(np.dot(w, rel_pos))

            if dot_product1 < 0.0 and (dot_product1 * dot_product1) > (combined_radius_sq * w_len_sq):
                # Project on cut-off circle
                w_len = math.sqrt(w_len_sq)
                unit_w = w / max(1e-6, w_len)
                direction = np.array([unit_w[1], -unit_w[0]])
                u = (combined_radius / tau - w_len) * unit_w
            else:
                # Project on legs
                dist = math.sqrt(dist_sq)
                leg = math.sqrt(max(0.0, dist_sq - combined_radius_sq))

                cross_val = cross_2d(rel_pos, w)
                if cross_val > 1e-4:
                    # Project on left leg
                    direction = np.array([
                        rel_pos[0] * leg - rel_pos[1] * combined_radius,
                        rel_pos[0] * combined_radius + rel_pos[1] * leg
                    ]) / dist_sq
                elif cross_val < -1e-4:
                    # Project on right leg
                    direction = -np.array([
                        rel_pos[0] * leg + rel_pos[1] * combined_radius,
                        -rel_pos[0] * combined_radius + rel_pos[1] * leg
                    ]) / dist_sq
                else:
                    # Collinear tie-breaker: steer towards open corridor interior
                    if p_i[1] < 10.0:
                        direction = np.array([
                            rel_pos[0] * leg - rel_pos[1] * combined_radius,
                            rel_pos[0] * combined_radius + rel_pos[1] * leg
                        ]) / dist_sq
                    else:
                        direction = -np.array([
                            rel_pos[0] * leg + rel_pos[1] * combined_radius,
                            -rel_pos[0] * combined_radius + rel_pos[1] * leg
                        ]) / dist_sq

                dot_product2 = float(np.dot(rel_vel, direction))
                u = dot_product2 * direction - rel_vel
        else:
            # Collision / Penetration: separate promptly
            w = rel_vel - (rel_pos / dt)
            w_len = float(np.linalg.norm(w))
            unit_w = w / max(1e-6, w_len)
            direction = np.array([unit_w[1], -unit_w[0]])
            # Clamp separation jump to max_speed to keep LP bounded
            sep_mag = min(self.max_speed * 1.5, combined_radius / dt - w_len)
            u = sep_mag * unit_w

        normal = np.array([-direction[1], direction[0]])
        point = v_i + weight_i * u
        return HalfPlaneLine(point=point, direction=direction, normal=normal)

    def _compute_obstacle_orca_line(
        self,
        p_i: np.ndarray,
        v_i: np.ndarray,
        p1: np.ndarray,
        p2: np.ndarray,
        radius: float,
        tau: float,
        dt: float
    ) -> Optional[HalfPlaneLine]:
        """Compute ORCA half-plane for a static line segment obstacle."""
        seg = p2 - p1
        seg_len_sq = float(np.dot(seg, seg))
        if seg_len_sq < 1e-6:
            return None

        # Closest point on segment
        t = max(0.0, min(1.0, float(np.dot(p_i - p1, seg)) / seg_len_sq))
        closest_point = p1 + t * seg

        rel_pos = closest_point - p_i
        dist_sq = float(np.dot(rel_pos, rel_pos))
        dist = math.sqrt(dist_sq)

        if dist > radius + max(self.max_speed * tau, 2.0):
            return None

        if dist < 1e-6:
            unit_r = np.array([1.0, 0.0])
        else:
            unit_r = rel_pos / dist

        if dist < radius:
            v_max_forward = -0.1
        else:
            v_max_forward = min(self.max_speed, (dist - radius) / tau)

        point = v_max_forward * unit_r
        normal = -unit_r
        direction = np.array([normal[1], -normal[0]])
        return HalfPlaneLine(point=point, direction=direction, normal=normal)

    def _solve_2d_lp(
        self,
        lines: List[HalfPlaneLine],
        max_speed: float,
        opt_velocity: np.ndarray
    ) -> np.ndarray:
        """2D LP half-plane solver."""
        result = np.copy(opt_velocity)
        norm_res = float(np.linalg.norm(result))
        if norm_res > max_speed:
            result = (result / norm_res) * max_speed

        for i, line in enumerate(lines):
            if np.dot(result - line.point, line.normal) < -1e-6:
                temp_result = np.copy(result)
                if not self._solve_1d_lp(lines, i, max_speed, opt_velocity, result):
                    # Infeasible: fallback to safe separation direction
                    safe_dir = line.normal
                    result = safe_dir * (max_speed * 0.5)
                    break

        return result

    def _solve_1d_lp(
        self,
        lines: List[HalfPlaneLine],
        line_idx: int,
        max_speed: float,
        opt_velocity: np.ndarray,
        result: np.ndarray
    ) -> bool:
        """1D LP on the line boundary."""
        target_line = lines[line_idx]
        dot_dir = float(np.dot(target_line.point, target_line.direction))
        disc = dot_dir * dot_dir - float(np.dot(target_line.point, target_line.point)) + max_speed * max_speed

        if disc < 0.0:
            return False

        sqrt_disc = math.sqrt(disc)
        t_left = -dot_dir - sqrt_disc
        t_right = -dot_dir + sqrt_disc

        for i in range(line_idx):
            line = lines[i]
            denominator = cross_2d(target_line.direction, line.direction)
            numerator = cross_2d(line.direction, target_line.point - line.point)

            if abs(denominator) < 1e-6:
                if numerator < 0.0:
                    return False
                continue

            t = numerator / denominator
            if denominator >= 0.0:
                t_right = min(t_right, t)
            else:
                t_left = max(t_left, t)

            if t_left > t_right + 1e-6:
                return False

        t_opt = float(np.dot(target_line.direction, opt_velocity - target_line.point))
        t_clamped = max(t_left, min(t_right, t_opt))
        
        opt_pt = target_line.point + t_clamped * target_line.direction
        result[0] = opt_pt[0]
        result[1] = opt_pt[1]
        return True
