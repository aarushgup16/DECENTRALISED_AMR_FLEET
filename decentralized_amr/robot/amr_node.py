"""Autonomous edge AMR node with NH-ORCA, CBBA, Lamport clocks, and Bounded Priority."""

import math
import time
from enum import Enum
from typing import Dict, List, Optional, Tuple
import numpy as np

from decentralized_amr.config import DEFAULT_ROBOT_CONFIG, RobotConfig
from decentralized_amr.network.lamport_clock import LamportClock
from decentralized_amr.network.messages import (
    BaseMessage,
    CBBABidMessage,
    MessageType,
    ObstacleAlertMessage,
    TaskReleaseMessage,
    TelemetryMessage,
)
from decentralized_amr.network.p2p_mesh import P2PMesh, P2PNode
from decentralized_amr.collision_avoidance.bounded_priority import BoundedPriorityEvaluator
from decentralized_amr.collision_avoidance.bitset_reservation import BitsetReservationTable
from decentralized_amr.collision_avoidance.nh_orca import NHORCASolver
from decentralized_amr.task_allocation.cbba_agent import CBBAAgent
from decentralized_amr.task_allocation.routing import GridPlanner
from decentralized_amr.task_allocation.task import TaskStatus, WarehouseTask


class AMRState(str, Enum):
    """Operational state of the autonomous robot."""
    IDLE = "IDLE"
    BIDDING = "BIDDING"
    MOVING_TO_PICKUP = "MOVING_TO_PICKUP"
    PICKING = "PICKING"
    MOVING_TO_DROPOFF = "MOVING_TO_DROPOFF"
    DROPPING = "DROPPING"
    YIELDING = "YIELDING"
    CHARGING = "CHARGING"


class AMRNode:
    """
    Decentralized Autonomous Mobile Robot edge agent.
    Runs entirely on onboard compute (e.g. Jetson / RPi) with zero reliance on a central server.
    """

    def __init__(
        self,
        robot_id: int,
        initial_pos: Tuple[float, float],
        initial_heading: float = 0.0,
        mesh: Optional[P2PMesh] = None,
        config: Optional[RobotConfig] = None
    ):
        self.robot_id = robot_id
        self.cfg = config or DEFAULT_ROBOT_CONFIG
        
        # Kinematics
        self.home_pos = initial_pos
        self.x = initial_pos[0]
        self.y = initial_pos[1]
        self.heading = initial_heading
        self.linear_v = 0.0
        self.angular_w = 0.0
        self.target_v_vec: Tuple[float, float] = (0.0, 0.0)

        # Battery & State
        self.battery = self.cfg.battery_capacity
        self.state = AMRState.IDLE
        self.current_task: Optional[WarehouseTask] = None
        self.active_waypoint_idx = 0
        self.path: List[Tuple[float, float]] = []
        self.dwell_timer = 0.0
        self.consecutive_yields = 0
        self.total_distance_traveled = 0.0
        self.tasks_completed_count = 0
        self.priority_tie_breaks_won = 0
        self.priority_tie_breaks_lost = 0
        self.active_yielding_peers: set = set()

        # Subsystems
        self.clock = LamportClock(node_id=robot_id)
        self.mesh = mesh or P2PMesh()
        self.p2p_node = P2PNode(robot_id, self.clock, self.mesh, comm_range=self.cfg.comm_range)
        self.p2p_node.update_position(self.x, self.y)

        self.planner = GridPlanner(inflation_radius=self.cfg.effective_radius)
        self.cbba = CBBAAgent(robot_id, self.clock, self.planner)
        self.orca = NHORCASolver(self.cfg)
        self.priority_evaluator = BoundedPriorityEvaluator()
        self.reservation_table = BitsetReservationTable()

        # Peer telemetry cache: dict[peer_id, TelemetryMessage]
        self.peer_telemetry: Dict[int, TelemetryMessage] = {}

        # Register P2P handlers
        self.p2p_node.register_handler(MessageType.TELEMETRY, self._handle_peer_telemetry)
        self.p2p_node.register_handler(MessageType.CBBA_BID, self._handle_cbba_bid)
        self.p2p_node.register_handler(MessageType.TASK_RELEASE, self._handle_task_release)
        self.p2p_node.register_handler(MessageType.OBSTACLE_ALERT, self._handle_obstacle_alert)

    # -------------------------------------------------------------------------
    # P2P Message Handlers
    # -------------------------------------------------------------------------
    def _handle_peer_telemetry(self, msg: BaseMessage):
        if isinstance(msg, TelemetryMessage) and msg.sender_id != self.robot_id:
            self.peer_telemetry[msg.sender_id] = msg

    def _handle_cbba_bid(self, msg: BaseMessage):
        if isinstance(msg, CBBABidMessage) and msg.sender_id != self.robot_id:
            altered = self.cbba.process_peer_bid_message(msg, (self.x, self.y))
            if altered:
                self._update_task_execution()

    def _handle_task_release(self, msg: BaseMessage):
        if isinstance(msg, TaskReleaseMessage) and msg.sender_id != self.robot_id:
            self.cbba.handle_peer_task_release(msg)

    def _handle_obstacle_alert(self, msg: BaseMessage):
        if isinstance(msg, ObstacleAlertMessage):
            self.planner.add_dynamic_obstacle(msg.obstacle_id, (msg.x_min, msg.y_min, msg.x_max, msg.y_max))
            if self.current_task:
                self._replan_current_path()

    # -------------------------------------------------------------------------
    # Main Edge Control Loop Step
    # -------------------------------------------------------------------------
    def step(
        self,
        dt: float,
        static_obstacle_segments: Optional[List[Tuple[Tuple[float, float], Tuple[float, float]]]] = None,
        live_peers: Optional[List[Tuple[int, Tuple[float, float], Tuple[float, float], float, float, float]]] = None
    ):
        """
        Execute one control cycle (dt = 0.05s / 20 Hz).
        live_peers: list of (peer_id, (x, y), (vx, vy), heading, radius, priority_score)
        """
        # 1. Update Battery Model
        if self.linear_v > 0.05 or abs(self.angular_w) > 0.05:
            self.battery = max(0.0, self.battery - (self.cfg.drain_moving_per_sec * dt))
        else:
            self.battery = max(0.0, self.battery - (self.cfg.drain_idle_per_sec * dt))

        # 2. Run CBBA Task Allocation Check
        if self.state == AMRState.IDLE or not self.current_task:
            bundle_changed = self.cbba.build_bundle((self.x, self.y))
            if bundle_changed:
                self.p2p_node.broadcast(self.cbba.create_bid_message())
                self._update_task_execution()

        # 3. State Machine & Navigation Logic
        v_pref_vec = self._update_navigation_state(dt)

        # If in dwelling state (picking or dropping), stay stationary
        if self.state in (AMRState.PICKING, AMRState.DROPPING):
            self.linear_v = 0.0
            self.angular_w = 0.0
            self.target_v_vec = (0.0, 0.0)
            self.p2p_node.update_position(self.x, self.y)
            return

        # 4. Check for Choke-Point / Aisle Tie-Breaks with Opposing Neighbors
        v_pref_vec = self._check_choke_point_priority(v_pref_vec, live_peers)

        # 5. Gather Neighbor Info for NH-ORCA
        neighbors_orca = []
        if live_peers is not None:
            for peer_id, (px, py), (pvx, pvy), pth, prad, pprio in live_peers:
                if peer_id != self.robot_id:
                    neighbors_orca.append(((px, py), (pvx, pvy), prad))
        else:
            for peer_id, tlm in self.peer_telemetry.items():
                dist = math.sqrt((tlm.x - self.x)**2 + (tlm.y - self.y)**2)
                if dist <= self.cfg.sensing_range:
                    neighbors_orca.append(((tlm.x, tlm.y), (tlm.vx, tlm.vy), self.cfg.radius))

        # 6. Run NH-ORCA Solver
        v_cmd, w_cmd, v_safe = self.orca.compute_nh_control(
            pos_i=(self.x, self.y),
            vel_i=(self.linear_v * math.cos(self.heading), self.linear_v * math.sin(self.heading)),
            heading_i=self.heading,
            v_current=self.linear_v,
            w_current=self.angular_w,
            v_pref_vec=v_pref_vec,
            neighbors=neighbors_orca,
            obstacles=static_obstacle_segments,
            dt=dt
        )

        self.linear_v = v_cmd
        self.angular_w = w_cmd
        self.target_v_vec = v_safe

        # 7. Integrate Differential-Drive Kinematics
        dx = self.linear_v * math.cos(self.heading) * dt
        dy = self.linear_v * math.sin(self.heading) * dt
        self.x = max(self.cfg.radius, min(30.0 - self.cfg.radius, self.x + dx))
        self.y = max(self.cfg.radius, min(20.0 - self.cfg.radius, self.y + dy))
        self.heading = (self.heading + self.angular_w * dt) % (2.0 * math.pi)
        
        step_dist = math.sqrt(dx * dx + dy * dy)
        self.total_distance_traveled += step_dist

        # 8. Update Bitset Reservation & Network Node
        self.p2p_node.update_position(self.x, self.y)
        if self.path and self.active_waypoint_idx < len(self.path):
            lookahead_pts = [(self.x, self.y)] + self.path[self.active_waypoint_idx:self.active_waypoint_idx + 8]
            self.reservation_table.set_trajectory(lookahead_pts, nominal_speed=self.cfg.max_linear_speed)

    def broadcast_telemetry(self):
        """Broadcast state telemetry over P2P mesh."""
        priority = self.get_current_priority()
        tlm = TelemetryMessage(
            sender_id=self.robot_id,
            lamport_clock=self.clock.get_time(),
            msg_type=MessageType.TELEMETRY,
            x=self.x,
            y=self.y,
            vx=self.linear_v * math.cos(self.heading),
            vy=self.linear_v * math.sin(self.heading),
            heading=self.heading,
            linear_v=self.linear_v,
            angular_w=self.angular_w,
            battery=self.battery,
            current_task_id=self.current_task.task_id if self.current_task else None,
            status=self.state.value,
            priority_score=priority,
            intended_path=self.path[self.active_waypoint_idx:self.active_waypoint_idx + 5] if self.path else []
        )
        self.p2p_node.broadcast(tlm)

    # -------------------------------------------------------------------------
    # State Machine Helpers
    # -------------------------------------------------------------------------
    def _update_task_execution(self):
        """Update active task from CBBA bundle."""
        active = self.cbba.get_current_task()
        if active is not None:
            if active != self.current_task:
                self.current_task = active
                self.current_task.mark_in_progress()
                self.state = AMRState.MOVING_TO_PICKUP
                self._plan_path_to(self.current_task.pickup_pos)
        else:
            self.current_task = None
            self.state = AMRState.IDLE
            # If not near home station, return home to keep aisles and docks clear
            dist_to_home = math.sqrt((self.home_pos[0] - self.x)**2 + (self.home_pos[1] - self.y)**2)
            if dist_to_home > 1.2:
                self._plan_path_to(self.home_pos)
            else:
                self.path.clear()
                self.active_waypoint_idx = 0

    def _plan_path_to(self, goal: Tuple[float, float]):
        """Compute path using local A* grid planner."""
        waypoints = self.planner.plan_path((self.x, self.y), goal)
        if waypoints:
            self.path = waypoints
            self.active_waypoint_idx = 0
        else:
            if self.current_task:
                release_msg = self.cbba.release_task(self.current_task.task_id, (self.x, self.y))
                if release_msg:
                    self.p2p_node.broadcast(release_msg)
                self.current_task = None
                self.state = AMRState.IDLE

    def _replan_current_path(self):
        """Recompute path to current target destination."""
        if self.state == AMRState.MOVING_TO_PICKUP and self.current_task:
            self._plan_path_to(self.current_task.pickup_pos)
        elif self.state == AMRState.MOVING_TO_DROPOFF and self.current_task:
            self._plan_path_to(self.current_task.dropoff_pos)

    def _get_dock_staging_pos(self, dock_pos: Tuple[float, float]) -> Tuple[float, float]:
        """Compute safe waiting staging spot 1.4m before dock entrance."""
        dx, dy = dock_pos
        if dy < 5.0:
            return (dx, dy + 1.4)
        elif dy > 15.0:
            return (dx, dy - 1.4)
        return (dx, dy)

    def _is_dock_occupied_by_peer(self, dock_pos: Tuple[float, float]) -> bool:
        """Check if another robot is currently docked or loading at this dock."""
        for peer_id, tlm in self.peer_telemetry.items():
            dist_to_dock = math.sqrt((tlm.x - dock_pos[0])**2 + (tlm.y - dock_pos[1])**2)
            if dist_to_dock < 0.95:
                return True
        return False

    def _update_navigation_state(self, dt: float) -> Tuple[float, float]:
        """Update waypoint following and state transitions. Returns preferred velocity vector."""
        if self.state == AMRState.IDLE and not self.path:
            return (0.0, 0.0)

        # Handling Dwell Times (Loading / Unloading)
        if self.state in (AMRState.PICKING, AMRState.DROPPING):
            self.dwell_timer -= dt
            self.linear_v = 0.0
            self.angular_w = 0.0
            if self.dwell_timer <= 0.0:
                if self.state == AMRState.PICKING:
                    self.state = AMRState.MOVING_TO_DROPOFF
                    self._plan_path_to(self.current_task.dropoff_pos)
                elif self.state == AMRState.DROPPING:
                    self.current_task.mark_completed()
                    self.cbba.mark_task_completed(self.current_task.task_id)
                    self.tasks_completed_count += 1
                    self.current_task = None
                    self.state = AMRState.IDLE
                    self._update_task_execution()
            return (0.0, 0.0)

        # Check path waypoints
        if not self.path or self.active_waypoint_idx >= len(self.path):
            return (0.0, 0.0)

        # Final destination check
        final_goal = self.path[-1]
        is_final_dock = (final_goal[1] < 3.0 or final_goal[1] > 17.0)

        # If approaching dock and it is occupied by peer, hold at staging spot
        if is_final_dock and self.state != AMRState.IDLE and self._is_dock_occupied_by_peer(final_goal):
            staging_pos = self._get_dock_staging_pos(final_goal)
            dist_to_staging = math.sqrt((staging_pos[0] - self.x)**2 + (staging_pos[1] - self.y)**2)
            if dist_to_staging < 0.6:
                return (0.0, 0.0)

        target_wp = self.path[self.active_waypoint_idx]
        dist_to_wp = math.sqrt((target_wp[0] - self.x)**2 + (target_wp[1] - self.y)**2)

        # Advance to next waypoint if close for smooth continuous cornering
        if dist_to_wp < 0.75 and self.active_waypoint_idx < len(self.path) - 1:
            self.active_waypoint_idx += 1
            target_wp = self.path[self.active_waypoint_idx]
            dist_to_wp = math.sqrt((target_wp[0] - self.x)**2 + (target_wp[1] - self.y)**2)

        # Arrival at final destination
        if self.active_waypoint_idx == len(self.path) - 1 and dist_to_wp < 0.6:
            if self.state == AMRState.MOVING_TO_PICKUP:
                self.state = AMRState.PICKING
                self.dwell_timer = 0.3
                self.linear_v = 0.0
                self.angular_w = 0.0
                return (0.0, 0.0)
            elif self.state == AMRState.MOVING_TO_DROPOFF:
                self.state = AMRState.DROPPING
                self.dwell_timer = 0.3
                self.linear_v = 0.0
                self.angular_w = 0.0
                return (0.0, 0.0)
            elif self.state == AMRState.IDLE:
                self.path.clear()
                self.active_waypoint_idx = 0
                self.linear_v = 0.0
                self.angular_w = 0.0
                return (0.0, 0.0)

        # Preferred direction towards active waypoint with right-hand traffic bias
        dx = target_wp[0] - self.x
        dy = target_wp[1] - self.y
        norm = math.sqrt(dx * dx + dy * dy)
        if norm > 1e-4:
            desired_speed = min(self.cfg.max_linear_speed, max(0.4, norm * 1.5))
            u_fwd_x = dx / norm
            u_fwd_y = dy / norm
            # Perpendicular vector to the right of heading: (u_y, -u_x)
            u_right_x = u_fwd_y
            u_right_y = -u_fwd_x
            
            vx = (u_fwd_x + 0.15 * u_right_x) * desired_speed
            vy = (u_fwd_y + 0.15 * u_right_y) * desired_speed
            return (vx, vy)
        return (0.0, 0.0)

    # -------------------------------------------------------------------------
    # Bounded Priority Choke-Point Check
    # -------------------------------------------------------------------------
    def get_current_priority(self) -> float:
        """Compute current priority score."""
        urgency = self.current_task.urgency if self.current_task else 0
        dist_to_goal = 10.0
        if self.current_task and self.path and self.active_waypoint_idx < len(self.path):
            goal_pt = self.path[-1]
            dist_to_goal = math.sqrt((goal_pt[0] - self.x)**2 + (goal_pt[1] - self.y)**2)

        return self.priority_evaluator.compute_priority(
            node_id=self.robot_id,
            task_urgency=urgency,
            consecutive_yields=self.consecutive_yields,
            dist_to_goal=dist_to_goal,
            battery_pct=self.battery
        )

    def _check_choke_point_priority(
        self,
        v_pref_vec: Tuple[float, float],
        live_peers: Optional[List[Tuple[int, Tuple[float, float], Tuple[float, float], float, float, float]]] = None
    ) -> Tuple[float, float]:
        """
        Evaluate right-of-way when encountering a peer in a narrow aisle or choke point.
        """
        my_priority = self.get_current_priority()

        peer_candidates = []
        if live_peers is not None:
            for pid, (px, py), (pvx, pvy), pth, prad, pprio in live_peers:
                if pid != self.robot_id:
                    peer_candidates.append((pid, px, py, pth, pprio))
        else:
            for pid, tlm in self.peer_telemetry.items():
                peer_candidates.append((pid, tlm.x, tlm.y, tlm.heading, tlm.priority_score))

        # Cleanup resolved yielding encounters
        for active_pid in list(self.active_yielding_peers):
            found_candidate = any(p[0] == active_pid for p in peer_candidates)
            if not found_candidate:
                self.active_yielding_peers.remove(active_pid)
            else:
                for cand in peer_candidates:
                    if cand[0] == active_pid:
                        dist = math.sqrt((cand[1] - self.x)**2 + (cand[2] - self.y)**2)
                        if dist > 3.5:
                            self.active_yielding_peers.remove(active_pid)
                            self.consecutive_yields = max(0, self.consecutive_yields - 1)

        is_yielding = False

        for peer_id, px, py, p_heading, p_prio in peer_candidates:
            dist = math.sqrt((px - self.x)**2 + (py - self.y)**2)
            if dist < 2.5:
                my_heading_vec = (math.cos(self.heading), math.sin(self.heading))
                peer_heading_vec = (math.cos(p_heading), math.sin(p_heading))
                rel_pos = (px - self.x, py - self.y)
                
                dot_head = my_heading_vec[0] * peer_heading_vec[0] + my_heading_vec[1] * peer_heading_vec[1]
                dot_rel = (rel_pos[0] * my_heading_vec[0] + rel_pos[1] * my_heading_vec[1]) / max(1e-4, dist)

                # Head-on or converging encounter ahead
                if dot_head < -0.2 and dot_rel > 0.2:
                    winner, yielder = self.priority_evaluator.resolve_choke_right_of_way(
                        self.robot_id, my_priority, peer_id, p_prio
                    )
                    if yielder == self.robot_id:
                        if peer_id not in self.active_yielding_peers:
                            self.active_yielding_peers.add(peer_id)
                            self.consecutive_yields += 1
                            self.priority_tie_breaks_lost += 1
                        # Actively back away from oncoming winner
                        ux = (self.x - px) / max(1e-4, dist)
                        uy = (self.y - py) / max(1e-4, dist)
                        return (ux * 0.5, uy * 0.5)
                    else:
                        if peer_id in self.active_yielding_peers:
                            self.active_yielding_peers.remove(peer_id)
                        self.priority_tie_breaks_won += 1

        return v_pref_vec
