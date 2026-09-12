"""Bounded Priority Tie-Breaker for deterministic, starvation-free choke-point resolution."""

from typing import Any, Dict, Optional, Tuple
from decentralized_amr.config import DEFAULT_PRIORITY_CONFIG, PriorityConfig


class BoundedPriorityEvaluator:
    """
    Computes deterministic priority score P_i:
      P_i = 50000 * U_task(i)
          + min(10000 * T_yield(i), 20000)   # Capped yield bonus (anti-starvation guarantee)
          + 100 / (D_goal(i) + 1)            # Closer-to-goal bonus
          + 10 / (B_i + 1)                   # Low-battery urgency
          + ID_i                             # Deterministic tie-breaker
    """

    def __init__(self, config: Optional[PriorityConfig] = None):
        self.cfg = config or DEFAULT_PRIORITY_CONFIG

    def compute_priority(
        self,
        node_id: int,
        task_urgency: int = 1,        # 0=idle, 1=normal, 2=priority, 3=high, 4=emergency
        consecutive_yields: float = 0.0, # number of yields or seconds spent yielding
        dist_to_goal: float = 10.0,
        battery_pct: float = 100.0
    ) -> float:
        """Compute the bounded priority score for a given AMR state."""
        # 1. Task urgency component
        urgency_term = self.cfg.weight_urgency * float(task_urgency)

        # 2. Capped yield bonus (guarantees a yielding robot eventually out-ranks peers)
        raw_yield_bonus = self.cfg.weight_yield * float(consecutive_yields)
        yield_term = min(raw_yield_bonus, self.cfg.max_yield_bonus)

        # 3. Proximity to goal
        goal_term = self.cfg.weight_goal / (max(0.0, dist_to_goal) + 1.0)

        # 4. Battery urgency (lower battery gets slight precedence to finish task)
        battery_term = self.cfg.weight_battery / (max(0.0, battery_pct) + 1.0)

        # 5. Deterministic ID tiebreaker
        id_term = float(node_id)

        score = urgency_term + yield_term + goal_term + battery_term + id_term
        return score

    def resolve_choke_right_of_way(
        self,
        robot_a_id: int,
        robot_a_priority: float,
        robot_b_id: int,
        robot_b_priority: float
    ) -> Tuple[int, int]:
        """
        Determine which robot has right-of-way at a narrow choke point.
        Returns: (winner_robot_id, yielding_robot_id)
        """
        if robot_a_priority > robot_b_priority:
            return (robot_a_id, robot_b_id)
        elif robot_b_priority > robot_a_priority:
            return (robot_b_id, robot_a_id)
        else:
            # Deterministic ID tie-breaker
            if robot_a_id > robot_b_id:
                return (robot_a_id, robot_b_id)
            return (robot_b_id, robot_a_id)

