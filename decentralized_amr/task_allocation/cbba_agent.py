"""Consensus-Based Bundle Algorithm (CBBA) with Lamport-ordered bid tie-breaking."""

import math
import threading
from typing import Dict, List, Optional, Set, Tuple
from decentralized_amr.network.lamport_clock import LamportClock
from decentralized_amr.network.messages import CBBABidMessage, TaskReleaseMessage
from decentralized_amr.task_allocation.routing import GridPlanner
from decentralized_amr.task_allocation.task import TaskStatus, WarehouseTask


class CBBAAgent:
    """
    Decentralized task allocation agent running on each AMR node.
    Implements two-phase CBBA (Bundle Construction + Consensus) with
    Lamport logical clock tie-breaking and dynamic re-auctioning upon blocked aisles.
    """

    def __init__(
        self,
        robot_id: int,
        lamport_clock: LamportClock,
        planner: GridPlanner,
        max_bundle_size: int = 3
    ):
        self.robot_id = robot_id
        self.clock = lamport_clock
        self.planner = planner
        self.max_bundle_size = max_bundle_size

        self.bundle: List[str] = []                       # Ordered list of task IDs in current bundle
        self.path_waypoints: List[Tuple[float, float]] = [] # Ordered route [(x, y), ...] for bundle
        self.winning_bids: Dict[str, float] = {}          # y_i(j): highest known bid value for task j
        self.winning_agents: Dict[str, int] = {}          # z_i(j): ID of winning agent for task j
        self.bid_clocks: Dict[str, int] = {}              # s_i(j): Lamport timestamp when winning bid was placed

        self.known_tasks: Dict[str, WarehouseTask] = {}   # Pool of available tasks known to this agent
        self.completed_tasks: Set[str] = set()
        self._lock = threading.Lock()

    def add_known_task(self, task: WarehouseTask):
        """Add a task to the local pool if not already completed."""
        with self._lock:
            if task.task_id not in self.completed_tasks:
                self.known_tasks[task.task_id] = task
                if task.task_id not in self.winning_bids:
                    self.winning_bids[task.task_id] = 0.0
                    self.winning_agents[task.task_id] = -1
                    self.bid_clocks[task.task_id] = 0

    def mark_task_completed(self, task_id: str):
        """Mark task as completed and remove from active bidding."""
        with self._lock:
            self.completed_tasks.add(task_id)
            if task_id in self.bundle:
                self.bundle.remove(task_id)
            self.known_tasks.pop(task_id, None)

    # -------------------------------------------------------------------------
    # PHASE 1: Bundle Construction (Local Greedy Phase)
    # -------------------------------------------------------------------------
    def build_bundle(self, current_robot_pos: Tuple[float, float]) -> bool:
        """
        Greedily insert best-scoring candidate tasks into bundle until capacity
        or no further improvements exist. Returns True if bundle changed.
        """
        with self._lock:
            changed = False
            while len(self.bundle) < self.max_bundle_size:
                best_task_id: Optional[str] = None
                best_score = -1e9
                best_marginal_gain = 0.0

                for task_id, task in self.known_tasks.items():
                    if task_id in self.completed_tasks or task_id in self.bundle:
                        continue
                    
                    if task.status in (TaskStatus.COMPLETED, TaskStatus.IN_PROGRESS) and task.assigned_robot_id != self.robot_id and task.assigned_robot_id is not None:
                        continue

                    # If another robot already holds a confirmed winning bid in consensus
                    winner = self.winning_agents.get(task_id, -1)
                    if winner != -1 and winner != self.robot_id:
                        if task.status in (TaskStatus.IN_PROGRESS, TaskStatus.COMPLETED):
                            continue

                    score = self._compute_task_score(task, current_robot_pos)
                    current_high_bid = self.winning_bids.get(task_id, 0.0)
                    is_my_win = (self.winning_agents.get(task_id) == self.robot_id)

                    # Bid must strictly exceed current highest known bid, or be owned by this agent
                    if is_my_win or score > current_high_bid + 1e-4:
                        gain = score if is_my_win else (score - current_high_bid)
                        if gain > best_marginal_gain:
                            best_marginal_gain = gain
                            best_score = score
                            best_task_id = task_id

                if best_task_id is not None:
                    # Win task locally
                    self.bundle.append(best_task_id)
                    lamport_ts = self.clock.tick()
                    self.winning_bids[best_task_id] = best_score
                    self.winning_agents[best_task_id] = self.robot_id
                    self.bid_clocks[best_task_id] = lamport_ts
                    self.known_tasks[best_task_id].mark_assigned(self.robot_id)
                    changed = True
                else:
                    break

            if changed:
                self._reconstruct_path(current_robot_pos)
            return changed

    def _compute_task_score(self, task: WarehouseTask, current_robot_pos: Tuple[float, float]) -> float:
        """
        Compute score = Total Value - Path Insertion Distance Cost.
        """
        start_pt = current_robot_pos if not self.path_waypoints else self.path_waypoints[-1]
        
        # Path distance: current -> pickup -> dropoff
        dist_to_pickup = math.sqrt((task.pickup_pos[0] - start_pt[0])**2 + (task.pickup_pos[1] - start_pt[1])**2)
        dist_pickup_to_drop = math.sqrt((task.dropoff_pos[0] - task.pickup_pos[0])**2 + (task.dropoff_pos[1] - task.pickup_pos[1])**2)
        total_dist_cost = (dist_to_pickup + dist_pickup_to_drop) * 2.0

        score = task.total_value - total_dist_cost
        return max(1.0, score)

    def _reconstruct_path(self, current_pos: Tuple[float, float]):
        """Rebuild sequence of waypoints for all tasks currently in bundle."""
        self.path_waypoints.clear()
        curr = current_pos

        for task_id in self.bundle:
            task = self.known_tasks.get(task_id)
            if not task:
                continue
            # Plan path: curr -> pickup
            seg1 = self.planner.plan_path(curr, task.pickup_pos)
            if seg1:
                self.path_waypoints.extend(seg1)
                curr = task.pickup_pos
            # Plan path: pickup -> dropoff
            seg2 = self.planner.plan_path(curr, task.dropoff_pos)
            if seg2:
                self.path_waypoints.extend(seg2)
                curr = task.dropoff_pos

    # -------------------------------------------------------------------------
    # PHASE 2: Consensus Phase (Conflict Resolution & Lamport Ordering)
    # -------------------------------------------------------------------------
    def process_peer_bid_message(self, msg: CBBABidMessage, current_robot_pos: Tuple[float, float]) -> bool:
        """
        Process a peer's CBBA bid message and resolve conflicts.
        Uses Lamport logical clock causal ordering for deterministic tie-breaking.
        Returns True if local bundle was pruned/altered.
        """
        with self._lock:
            bundle_altered = False
            first_outbid_idx = -1

            for task_id, remote_bid in msg.winning_bids.items():
                if task_id in self.completed_tasks:
                    continue

                # If we are actively in progress carrying or executing this task, do not relinquish
                active_task = self.known_tasks.get(task_id)
                if active_task and active_task.status == TaskStatus.IN_PROGRESS and active_task.assigned_robot_id == self.robot_id:
                    continue

                remote_winner = msg.winning_agents.get(task_id, -1)
                remote_clock = msg.bid_clocks.get(task_id, 0)

                local_bid = self.winning_bids.get(task_id, 0.0)
                local_winner = self.winning_agents.get(task_id, -1)
                local_clock = self.bid_clocks.get(task_id, 0)

                remote_wins = False

                if remote_bid > local_bid + 1e-4:
                    remote_wins = True
                elif abs(remote_bid - local_bid) <= 1e-4 and remote_winner != -1:
                    # Bids are equal: deterministic tie-break
                    # 1. Higher Lamport clock (causally more recent event) wins
                    if remote_clock > local_clock:
                        remote_wins = True
                    elif remote_clock == local_clock:
                        # 2. Higher robot ID wins
                        if remote_winner > local_winner:
                            remote_wins = True

                if remote_wins:
                    self.winning_bids[task_id] = remote_bid
                    self.winning_agents[task_id] = remote_winner
                    self.bid_clocks[task_id] = remote_clock

                    # Check if our own task in bundle was outbid
                    if task_id in self.bundle and remote_winner != self.robot_id:
                        idx = self.bundle.index(task_id)
                        if first_outbid_idx == -1 or idx < first_outbid_idx:
                            first_outbid_idx = idx

            # If an item in our bundle was outbid, release it and all downstream tasks
            if first_outbid_idx != -1:
                pruned_tasks = self.bundle[first_outbid_idx:]
                self.bundle = self.bundle[:first_outbid_idx]
                for pruned_id in pruned_tasks:
                    if self.winning_agents.get(pruned_id) == self.robot_id:
                        self.winning_agents[pruned_id] = -1
                        self.winning_bids[pruned_id] = 0.0
                self._reconstruct_path(current_robot_pos)
                bundle_altered = True

            return bundle_altered

    # -------------------------------------------------------------------------
    # Dynamic Re-routing & Task Release (Blocked Aisle Handler)
    # -------------------------------------------------------------------------
    def release_task(self, task_id: str, current_robot_pos: Tuple[float, float]) -> Optional[TaskReleaseMessage]:
        """
        Release a task due to an impassable blocked aisle or dynamic obstacle.
        Resets local bid so peers can immediately re-auction it.
        """
        with self._lock:
            if task_id in self.bundle:
                idx = self.bundle.index(task_id)
                pruned_tasks = self.bundle[idx:]
                self.bundle = self.bundle[:idx]

                for p_id in pruned_tasks:
                    self.winning_bids[p_id] = 0.0
                    self.winning_agents[p_id] = -1
                    self.bid_clocks[p_id] = self.clock.tick()
                    if p_id in self.known_tasks:
                        self.known_tasks[p_id].mark_released()

                self._reconstruct_path(current_robot_pos)

                release_msg = TaskReleaseMessage(
                    sender_id=self.robot_id,
                    lamport_clock=self.clock.get_time(),
                    task_id=task_id,
                    releasing_agent_id=self.robot_id,
                    reason="blocked_aisle"
                )
                return release_msg
        return None

    def handle_peer_task_release(self, msg: TaskReleaseMessage):
        """Process peer task release: clear winning bids for released task."""
        with self._lock:
            task_id = msg.task_id
            if task_id in self.known_tasks and task_id not in self.completed_tasks:
                self.winning_bids[task_id] = 0.0
                self.winning_agents[task_id] = -1
                self.bid_clocks[task_id] = msg.lamport_clock
                self.known_tasks[task_id].mark_released()

    def create_bid_message(self) -> CBBABidMessage:
        """Construct CBBABidMessage representing current local consensus state."""
        with self._lock:
            return CBBABidMessage(
                sender_id=self.robot_id,
                lamport_clock=self.clock.get_time(),
                winning_bids=dict(self.winning_bids),
                winning_agents=dict(self.winning_agents),
                bid_clocks=dict(self.bid_clocks)
            )

    def get_current_task(self) -> Optional[WarehouseTask]:
        """Return currently active task in bundle (first item)."""
        with self._lock:
            if self.bundle:
                return self.known_tasks.get(self.bundle[0])
            return None
